"""mission_bt_node — 行为树驱动的巡检任务调度节点.

职责:
  1. 接收 StartMission / StopMission / PauseMission Service。
  2. 从 waypoint_manager 获取航点详情。
  3. 动态构建行为树，按顺序对每个航点执行 "导航 → 到达后任务"。
  4. 发布 MissionStatus 话题反馈进度。

行为树结构 (每个航点):
  Sequence
  ├── CheckBattery (Condition)
  └── Selector [路线类型分支]
      ├── Sequence [普通导航/到达后任务]
      │   ├── IsNotFollowPathPoint
      │   ├── NavigateToGoal (Action)
      │   └── Selector [waypoint_type 分支]
      │       ├── Sequence [TYPE_VISION]
      │       │   ├── IsVisionPoint
      │       │   ├── WaitAfterNavArrived
      │       │   ├── PtzAbsoluteMove
      │       │   ├── WaitPtzStable
      │       │   ├── WaitAfterPtzArrived
      │       │   ├── CaptureImage
      │       │   ├── UploadInspectionAlbum
      │       │   └── RunModelTask
      │       ├── Sequence [TYPE_CHARGE]
      │       │   ├── IsChargePoint
      │       │   └── Recharge
      │       └── IsNormalPoint (SUCCESS = 到达即完成)
      └── Sequence [TYPE_FOLLOW_PATH]
          ├── IsFollowPathPoint
          └── FollowPathAction
"""
from __future__ import annotations

import threading
import time
import uuid
from pathlib import Path
from typing import Any

import py_trees
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

try:
    from py_trees_ros.visitors import TreeToMsgVisitor
    from py_trees_ros_interfaces.msg import BehaviourTree as BehaviourTreeMsg
    _HAS_PY_TREES_ROS = True
except ImportError:
    _HAS_PY_TREES_ROS = False

from rhw_msgs.msg import InspectionAlbumReport, MissionStatus, WaypointTask
from rhw_msgs.srv import GetWaypoints, PauseMission, StartMission, StopMission

from rhw_task_scheduler.bt_actions.charge_action import Recharge
from rhw_task_scheduler.bt_actions.condition_nodes import (
    CheckBattery,
    IsChargePoint,
    IsFollowPathPoint,
    IsNotFollowPathPoint,
    IsNormalPoint,
    IsVisionPoint,
)
from rhw_task_scheduler.bt_actions.follow_path_action import FollowPathAction
from rhw_task_scheduler.bt_actions.navigate_action import NavigateToGoal
from rhw_task_scheduler.bt_actions.ptz_actions import (
    CaptureImage,
    PtzAbsoluteMove,
    WaitForDuration,
    WaitPtzStable,
)
from rhw_task_scheduler.bt_actions.vision_actions import (
    RunModelTask,
    UploadInspectionAlbum,
)
from rhw_task_scheduler.bt_utils import safe_slug
from rhw_task_scheduler.service_audit import ServiceAuditPublisher


class MissionBtNode(Node):
    """行为树驱动的巡检任务调度主节点."""

    def __init__(self) -> None:
        super().__init__('mission_bt_node')
        self._declare_parameters()
        self._read_parameters()

        self._callback_group = ReentrantCallbackGroup()

        # ---- 状态 ----
        self._mission_running = False
        self._mission_paused = False
        self._waypoint_queue: list[dict[str, Any]] = []
        self._current_index = 0
        self._completed_count = 0
        self._current_task_id = ''
        self._bt: py_trees.trees.BehaviourTree | None = None
        self._bt_lock = threading.Lock()
        self._tick_count = 0
        self._last_root_status: py_trees.common.Status | None = None

        # ---- 发布器 ----
        self._status_pub = self.create_publisher(
            MissionStatus, self._mission_status_topic, 10
        )
        self._album_report_pub = self.create_publisher(
            InspectionAlbumReport, self._inspection_album_report_topic, 10
        )
        self._service_audit = ServiceAuditPublisher(self)

        # ---- py_trees_ros 实时可视化 ----
        self._tree_msg_visitor: TreeToMsgVisitor | None = None
        self._tree_snapshot_pub = None
        if _HAS_PY_TREES_ROS:
            self._tree_msg_visitor = TreeToMsgVisitor()
            self._tree_snapshot_pub = self.create_publisher(
                BehaviourTreeMsg, '~/snapshots', 2
            )
            self.get_logger().info(
                'py_trees_ros snapshot publisher enabled on '
                f'{self.get_name()}/snapshots'
            )

        # ---- Service Clients ----
        self._get_waypoints_client = self.create_client(
            GetWaypoints,
            self._get_waypoints_service,
            callback_group=self._callback_group,
        )

        # ---- Service Servers ----
        self.create_service(
            StartMission,
            '/mission/start',
            self._handle_start,
            callback_group=self._callback_group,
        )
        self.create_service(
            StopMission,
            '/mission/stop',
            self._handle_stop,
            callback_group=self._callback_group,
        )
        self.create_service(
            PauseMission,
            '/mission/pause',
            self._handle_pause,
            callback_group=self._callback_group,
        )

        # ---- Blackboard 初始化 ----
        self._bb = py_trees.blackboard.Client(name='MissionBtNode')
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/nav_result', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/nav_retry_max', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/battery_low', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/last_capture_path', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/last_capture_url', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/last_capture_file_size', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/last_model_result_json_path', access=py_trees.common.Access.WRITE)
        self._bb.set('/nav_retry_max', self._nav_retry_max)
        self._bb.set('/battery_low', False)
        self._bb.set('/last_capture_path', '')
        self._bb.set('/last_capture_url', '')
        self._bb.set('/last_capture_file_size', 0)
        self._bb.set('/last_model_result_json_path', '')

        # ---- 行为树 tick 定时器 ----
        self._tick_timer = self.create_timer(
            1.0 / self._bt_tick_rate_hz,
            self._tick_bt,
            callback_group=self._callback_group,
        )

        # ---- 状态发布定时器 ----
        self._status_timer = self.create_timer(
            1.0,
            self._publish_status,
            callback_group=self._callback_group,
        )

        self.get_logger().info('mission_bt_node started')
        self.get_logger().info(
            f'service audit publisher enabled on {self._service_audit.topic}'
        )

    # ================================================================
    #  参数声明与读取
    # ================================================================

    def _declare_parameters(self) -> None:
        self.declare_parameter('bt_tick_rate_hz', 10.0)
        self.declare_parameter('nav_retry_max', 3)
        self.declare_parameter('navigation_frame_id', 'map')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('nav2_behavior_tree', '')
        self.declare_parameter('follow_path_action', '/follow_path')
        self.declare_parameter('follow_path_controller_id', '')
        self.declare_parameter('follow_path_goal_checker_id', '')
        self.declare_parameter('follow_path_progress_checker_id', '')
        self.declare_parameter('ptz_absolute_move_service', '/ptz/absolute_move')
        self.declare_parameter('ptz_capture_service', '/ptz/capture_image')
        self.declare_parameter('ptz_status_topic', '/ptz/status')
        self.declare_parameter('nav_arrived_to_ptz_delay_sec', 0.0)
        self.declare_parameter('ptz_arrived_to_capture_delay_sec', 0.0)
        self.declare_parameter('ptz_stable_timeout_sec', 5.0)
        self.declare_parameter('default_ptz_channel', 1)
        self.declare_parameter('recharge_service', '/recharge')
        self.declare_parameter('battery_topic', '/robot/battery_status')
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('waypoint_task_timeout_sec', 120.0)
        self.declare_parameter('mission_status_topic', '/mission/status')
        self.declare_parameter('inspection_album_report_topic', '/inspection/album_reports')
        self.declare_parameter('inspection_album_upload_service', '/inspection/album_report/upload')
        self.declare_parameter('album_upload_timeout_sec', 30.0)
        self.declare_parameter('model_task_run_service', '/rhw/model/task/run')
        self.declare_parameter('model_task_timeout_sec', 60.0)
        self.declare_parameter('get_waypoints_service', '/waypoint_manager/get_waypoints')
        self.declare_parameter('debug_print_tree_on_build', True)
        self.declare_parameter('debug_print_tree_on_tick', False)
        self.declare_parameter('debug_tree_show_status', True)
        self.declare_parameter('debug_tree_log_every_n_ticks', 1)
        self.declare_parameter('debug_export_tree_dot', False)
        self.declare_parameter('debug_tree_output_dir', '/tmp/rhw_task_scheduler_bt')

    def _read_parameters(self) -> None:
        self._bt_tick_rate_hz = float(self.get_parameter('bt_tick_rate_hz').value)
        self._nav_retry_max = int(self.get_parameter('nav_retry_max').value)
        self._mission_status_topic = str(self.get_parameter('mission_status_topic').value)
        self._inspection_album_report_topic = str(
            self.get_parameter('inspection_album_report_topic').value
        )
        self._get_waypoints_service = str(self.get_parameter('get_waypoints_service').value)
        self._debug_print_tree_on_build = bool(self.get_parameter('debug_print_tree_on_build').value)
        self._debug_print_tree_on_tick = bool(self.get_parameter('debug_print_tree_on_tick').value)
        self._debug_tree_show_status = bool(self.get_parameter('debug_tree_show_status').value)
        self._debug_tree_log_every_n_ticks = max(int(self.get_parameter('debug_tree_log_every_n_ticks').value), 1)
        self._debug_export_tree_dot = bool(self.get_parameter('debug_export_tree_dot').value)
        self._debug_tree_output_dir = Path(
            str(self.get_parameter('debug_tree_output_dir').value)
        ).expanduser()

    # ================================================================
    #  Service Handlers
    # ================================================================

    def _handle_start(
        self, request: StartMission.Request, response: StartMission.Response
    ) -> StartMission.Response:
        started_at = time.monotonic()
        self._service_audit.publish(
            service='/mission/start',
            role='server',
            phase='request',
            request=request,
        )
        if self._mission_running:
            response.result = 0
            response.message = 'Mission already running, stop first'
            self._service_audit.publish(
                service='/mission/start',
                role='server',
                phase='response',
                request=request,
                response=response,
                success=False,
                duration_ms=(time.monotonic() - started_at) * 1000.0,
            )
            return response

        map_name = request.map_name
        waypoint_ids = list(request.waypoint_ids)
        task_id = str(getattr(request, 'task_id', '') or '').strip()

        if not map_name or not waypoint_ids:
            response.result = 0
            response.message = 'map_name and waypoint_ids are required'
            self._service_audit.publish(
                service='/mission/start',
                role='server',
                phase='response',
                request=request,
                response=response,
                success=False,
                duration_ms=(time.monotonic() - started_at) * 1000.0,
            )
            return response

        ok, msg = self._start_mission(map_name, waypoint_ids, task_id=task_id)
        response.result = 1 if ok else 0
        response.message = msg
        self._service_audit.publish(
            service='/mission/start',
            role='server',
            phase='response',
            request=request,
            response=response,
            success=ok,
            duration_ms=(time.monotonic() - started_at) * 1000.0,
        )
        return response

    def _handle_stop(
        self, request: StopMission.Request, response: StopMission.Response
    ) -> StopMission.Response:
        started_at = time.monotonic()
        self._service_audit.publish(
            service='/mission/stop',
            role='server',
            phase='request',
            request=request,
        )
        self._stop_mission()
        response.result = 1
        response.message = 'Mission stopped'
        self._service_audit.publish(
            service='/mission/stop',
            role='server',
            phase='response',
            request=request,
            response=response,
            success=True,
            duration_ms=(time.monotonic() - started_at) * 1000.0,
        )
        return response

    def _handle_pause(
        self, request: PauseMission.Request, response: PauseMission.Response
    ) -> PauseMission.Response:
        started_at = time.monotonic()
        self._service_audit.publish(
            service='/mission/pause',
            role='server',
            phase='request',
            request=request,
        )
        if not self._mission_running:
            response.result = 0
            response.message = 'No mission running'
            self._service_audit.publish(
                service='/mission/pause',
                role='server',
                phase='response',
                request=request,
                response=response,
                success=False,
                duration_ms=(time.monotonic() - started_at) * 1000.0,
            )
            return response

        self._mission_paused = request.pause
        state = 'paused' if request.pause else 'resumed'
        self.get_logger().info(f'Mission {state}')
        response.result = 1
        response.message = f'Mission {state}'
        self._service_audit.publish(
            service='/mission/pause',
            role='server',
            phase='response',
            request=request,
            response=response,
            success=True,
            duration_ms=(time.monotonic() - started_at) * 1000.0,
        )
        return response

    # ================================================================
    #  任务管理核心
    # ================================================================

    def _start_mission(
        self,
        map_name: str,
        waypoint_ids: list[str],
        *,
        task_id: str = '',
    ) -> tuple[bool, str]:
        """获取航点详情并启动行为树."""
        # 同步调用 GetWaypoints
        if not self._get_waypoints_client.service_is_ready():
            self._service_audit.publish(
                service=self._get_waypoints_service,
                role='client',
                phase='response',
                request={'map_name': map_name},
                success=False,
                details={'reason': 'service_not_ready'},
            )
            return False, 'GetWaypoints service not ready'

        req = GetWaypoints.Request()
        req.map_name = map_name
        started_at = time.monotonic()
        self._service_audit.publish(
            service=self._get_waypoints_service,
            role='client',
            phase='request',
            request=req,
            details={'waypoint_ids': waypoint_ids},
        )
        future = self._get_waypoints_client.call_async(req)

        # 使用非阻塞轮询等待，避免 spin_until_future_complete 阻塞 executor
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() > deadline:
                self._service_audit.publish(
                    service=self._get_waypoints_service,
                    role='client',
                    phase='response',
                    request=req,
                    success=False,
                    duration_ms=(time.monotonic() - started_at) * 1000.0,
                    details={'reason': 'timeout', 'waypoint_ids': waypoint_ids},
                )
                return False, 'GetWaypoints service timeout'
            time.sleep(0.05)

        if future.exception() is not None:
            self._service_audit.publish(
                service=self._get_waypoints_service,
                role='client',
                phase='response',
                request=req,
                success=False,
                duration_ms=(time.monotonic() - started_at) * 1000.0,
                details={'reason': 'exception', 'error': str(future.exception())},
            )
            return False, f'GetWaypoints call error: {future.exception()}'

        result = future.result()
        self._service_audit.publish(
            service=self._get_waypoints_service,
            role='client',
            phase='response',
            request=req,
            response=result,
            success=(result.result == 1),
            duration_ms=(time.monotonic() - started_at) * 1000.0,
            details={'waypoint_ids': waypoint_ids},
        )
        if result.result != 1:
            return False, f'GetWaypoints failed: {result.message}'

        # 按 waypoint_ids 顺序构建队列
        all_wps = {wp.waypoint_id: self._wp_msg_to_dict(wp) for wp in result.waypoints}
        queue = []
        for wid in waypoint_ids:
            if wid in all_wps:
                queue.append(all_wps[wid])
            else:
                self.get_logger().warning(f'Waypoint {wid} not found, skipped')

        if not queue:
            return False, 'No valid waypoints found'

        self._current_task_id = task_id or self._generate_local_task_id()
        self._waypoint_queue = queue
        self._current_index = 0
        self._completed_count = 0
        self._mission_running = True
        self._mission_paused = False

        # 设置第一个航点并构建行为树
        self._setup_current_waypoint()

        self.get_logger().info(
            f'Mission started: task_id={self._current_task_id} map={map_name} '
            f'waypoints={len(queue)}'
        )
        return True, f'Mission started with {len(queue)} waypoints, task_id={self._current_task_id}'

    def _stop_mission(self) -> None:
        """停止当前任务."""
        with self._bt_lock:
            self._bt = None
        self._mission_running = False
        self._mission_paused = False
        self._waypoint_queue.clear()
        self._current_index = 0
        self._completed_count = 0
        self._current_task_id = ''
        self._tick_count = 0
        self._last_root_status = None
        self.get_logger().info('Mission stopped')

    def publish_inspection_album_report(self, wp: dict[str, Any], capture_result: Any) -> None:
        """发布视觉点抓拍结果，供 HTTPS 上报节点处理."""
        msg = InspectionAlbumReport()
        msg.task_id = self._current_task_id or self._generate_local_task_id()
        msg.point_id = str(wp.get('waypoint_id', ''))
        msg.point_name = str(wp.get('label') or msg.point_id)
        msg.image_path = str(getattr(capture_result, 'file_path', '') or '')
        msg.capture_url = str(getattr(capture_result, 'capture_url', '') or '')
        file_size = int(getattr(capture_result, 'file_size', 0) or 0)
        msg.file_size = max(0, min(file_size, 4294967295))

        if not msg.point_id or not msg.image_path:
            self.get_logger().warning(
                'Skip inspection album report: point_id or image_path is empty'
            )
            return

        self._album_report_pub.publish(msg)
        self.get_logger().info(
            'Inspection album report event published: '
            f'task_id={msg.task_id} point_id={msg.point_id} image={msg.image_path}'
        )

    def _setup_current_waypoint(self) -> None:
        """将当前航点写入 Blackboard 并构建单航点行为树."""
        if self._current_index >= len(self._waypoint_queue):
            return
        wp = self._waypoint_queue[self._current_index]
        self._bb.set('/current_waypoint', wp)
        self._bb.set('/nav_result', '')
        self._bb.set('/last_capture_path', '')
        self._bb.set('/last_capture_url', '')
        self._bb.set('/last_capture_file_size', 0)
        self._bb.set('/last_model_result_json_path', '')

        tree = self._build_waypoint_tree()
        with self._bt_lock:
            self._bt = py_trees.trees.BehaviourTree(root=tree)

        self._maybe_print_tree(tree, reason='build')
        self._maybe_export_tree(tree)

    def _advance_to_next_waypoint(self) -> bool:
        """前进到下一个航点，返回 False 表示任务完成."""
        self._completed_count += 1
        self._current_index += 1

        if self._current_index >= len(self._waypoint_queue):
            return False

        self._setup_current_waypoint()
        return True

    # ================================================================
    #  行为树构建
    # ================================================================

    def _build_waypoint_tree(self) -> py_trees.behaviour.Behaviour:
        """为当前航点构建行为树.

        Sequence
        ├── CheckBattery
        └── Selector [任务类型分支]
            ├── Sequence [NAVIGATE: navigate_to_pose → 到达后任务]
            │   ├── NavigateToGoal
            │   └── Selector
            │       ├── Sequence [VISION: absolute_move → wait → capture]
            │       ├── Sequence [CHARGE: recharge]
            │       └── IsNormalPoint [NORMAL: 到达即完成]
            └── Sequence [FOLLOW_PATH: follow_path]
        """
        root = py_trees.composites.Sequence(name='WaypointHandler', memory=True)

        # 1) 电量检查
        root.add_child(CheckBattery('CheckBattery', node=self))

        route_selector = py_trees.composites.Selector(
            name='RouteSelector', memory=False
        )

        # 2a) 普通/视觉/充电：先导航到目标点，再执行到达后任务
        nav_seq = py_trees.composites.Sequence(name='NavigateAndTask', memory=True)
        nav_seq.add_child(IsNotFollowPathPoint('IsNotFollowPathPoint?'))
        nav_seq.add_child(NavigateToGoal('NavigateToGoal', node=self))

        # 3) 到达后任务分支
        task_selector = py_trees.composites.Selector(
            name='TaskSelector', memory=False
        )

        # 3a) 视觉识别任务
        vision_seq = py_trees.composites.Sequence(name='VisionTask', memory=True)
        vision_seq.add_child(IsVisionPoint('IsVisionPoint?'))
        vision_seq.add_child(
            WaitForDuration(
                'WaitAfterNavArrived',
                node=self,
                parameter_name='nav_arrived_to_ptz_delay_sec',
            )
        )
        vision_seq.add_child(PtzAbsoluteMove('PtzAbsoluteMove', node=self))
        vision_seq.add_child(WaitPtzStable('WaitPtzStable', node=self))
        vision_seq.add_child(
            WaitForDuration(
                'WaitAfterPtzArrived',
                node=self,
                parameter_name='ptz_arrived_to_capture_delay_sec',
            )
        )
        vision_seq.add_child(CaptureImage('CaptureImage', node=self))
        vision_seq.add_child(UploadInspectionAlbum('UploadInspectionAlbum', node=self))
        vision_seq.add_child(RunModelTask('RunModelTask', node=self))
        task_selector.add_child(vision_seq)

        # 3b) 充电任务
        charge_seq = py_trees.composites.Sequence(name='ChargeTask', memory=True)
        charge_seq.add_child(IsChargePoint('IsChargePoint?'))
        charge_seq.add_child(Recharge('Recharge', node=self))
        task_selector.add_child(charge_seq)

        # 3c) 普通导航点（到达即完成）
        task_selector.add_child(IsNormalPoint('IsNormalPoint?'))

        nav_seq.add_child(task_selector)
        route_selector.add_child(nav_seq)

        # 2b) 巡线任务：直接执行 FollowPath
        follow_path_seq = py_trees.composites.Sequence(name='FollowPathTask', memory=True)
        follow_path_seq.add_child(IsFollowPathPoint('IsFollowPathPoint?'))
        follow_path_seq.add_child(FollowPathAction('FollowPathAction', node=self))
        route_selector.add_child(follow_path_seq)

        root.add_child(route_selector)
        return root

    # ================================================================
    #  行为树 tick 驱动
    # ================================================================

    def _tick_bt(self) -> None:
        """定时器回调：驱动行为树执行."""
        if not self._mission_running or self._mission_paused:
            return

        with self._bt_lock:
            bt = self._bt

        if bt is None:
            self.get_logger().warning('_tick_bt: bt is None but mission_running=True', throttle_duration_sec=5.0)
            return

        try:
            bt.tick()
        except Exception as exc:
            self.get_logger().error(f'BT tick error: {exc}')
            self._stop_mission()
            return

        self._tick_count += 1
        root_status = bt.root.status

        # 发布树快照供 py_trees_ros_viewer 实时可视化
        self._publish_tree_snapshot(bt.root)

        if root_status != self._last_root_status:
            self.get_logger().info(f'BT root status -> {root_status.name}')
            self._last_root_status = root_status

        if self._debug_print_tree_on_tick and (self._tick_count % self._debug_tree_log_every_n_ticks == 0):
            self._maybe_print_tree(bt.root, reason=f'tick#{self._tick_count}')

        if root_status == py_trees.common.Status.SUCCESS:
            wp = self._waypoint_queue[self._current_index]
            self.get_logger().info(
                f'Waypoint completed: {wp.get("waypoint_id", "?")} '
                f'({self._completed_count + 1}/{len(self._waypoint_queue)})'
            )
            if not self._advance_to_next_waypoint():
                self._publish_tree_snapshot(bt.root)  # 最终快照
                self._mission_running = False
                self.get_logger().info('Mission completed — all waypoints done')

        elif root_status == py_trees.common.Status.FAILURE:
            wp = self._waypoint_queue[self._current_index]
            self.get_logger().warning(
                f'Waypoint failed: {wp.get("waypoint_id", "?")}, skipping'
            )
            if not self._advance_to_next_waypoint():
                self._publish_tree_snapshot(bt.root)  # 最终快照
                self._mission_running = False
                self.get_logger().info('Mission completed (with failures)')

    # ================================================================
    #  树快照发布 (py_trees_ros_viewer)
    # ================================================================

    def _publish_tree_snapshot(self, root: py_trees.behaviour.Behaviour) -> None:
        """将行为树状态序列化为 BehaviourTree 消息发布，供 viewer 实时展示."""
        if self._tree_msg_visitor is None or self._tree_snapshot_pub is None:
            return
        try:
            self._tree_msg_visitor.initialise()
            for node in root.iterate():
                self._tree_msg_visitor.run(node)
            self._tree_snapshot_pub.publish(self._tree_msg_visitor.tree)
        except Exception as exc:
            self.get_logger().debug(f'Snapshot publish error: {exc}')

    # ================================================================
    #  状态发布
    # ================================================================

    def _publish_status(self) -> None:
        """定时发布 MissionStatus."""
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()

        if not self._mission_running:
            if self._completed_count > 0 and self._completed_count >= len(self._waypoint_queue):
                msg.status = MissionStatus.COMPLETED
            else:
                msg.status = MissionStatus.IDLE
        elif self._mission_paused:
            msg.status = MissionStatus.PAUSED
        else:
            msg.status = MissionStatus.RUNNING

        if self._waypoint_queue and self._current_index < len(self._waypoint_queue):
            msg.current_waypoint_id = self._waypoint_queue[self._current_index].get(
                'waypoint_id', ''
            )
        msg.total_waypoints = len(self._waypoint_queue)
        msg.completed_waypoints = self._completed_count
        msg.message = ''

        self._status_pub.publish(msg)

    # ================================================================
    #  工具方法
    # ================================================================

    @staticmethod
    def _wp_msg_to_dict(wp: WaypointTask) -> dict[str, Any]:
        return {
            'waypoint_id': wp.waypoint_id,
            'map_name': wp.map_name,
            'pose': {'x': wp.pose.x, 'y': wp.pose.y, 'theta': wp.pose.theta},
            'waypoint_type': int(wp.waypoint_type),
            'label': wp.label,
            'task_params': wp.task_params,
        }

    @staticmethod
    def _generate_local_task_id() -> str:
        return f'local-{int(time.time() * 1000)}-{uuid.uuid4().hex[:8]}'

    def _maybe_print_tree(self, tree_root: py_trees.behaviour.Behaviour, *, reason: str) -> None:
        if not self._debug_print_tree_on_build and not self._debug_print_tree_on_tick:
            return
        try:
            tree_text = py_trees.display.unicode_tree(
                root=tree_root,
                show_status=self._debug_tree_show_status,
            )
            self.get_logger().info(f'BT tree ({reason}):\n{tree_text}')
        except Exception as exc:
            self.get_logger().warning(f'Failed to print BT tree: {exc}')

    def _maybe_export_tree(self, tree_root: py_trees.behaviour.Behaviour) -> None:
        if not self._debug_export_tree_dot:
            return
        try:
            self._debug_tree_output_dir.mkdir(parents=True, exist_ok=True)
            waypoint_id = ''
            if self._waypoint_queue and self._current_index < len(self._waypoint_queue):
                waypoint_id = self._waypoint_queue[self._current_index].get('waypoint_id', '')
            base_name = safe_slug(f'waypoint_{self._current_index}_{waypoint_id}', fallback='waypoint_tree')
            outputs = py_trees.display.render_dot_tree(
                root=tree_root,
                name=base_name,
                target_directory=str(self._debug_tree_output_dir),
                with_blackboard_variables=True,
            )
            self.get_logger().info(f'BT dot exported: {outputs}')
        except Exception as exc:
            self.get_logger().warning(f'Failed to export BT dot tree: {exc}')

def main() -> None:
    rclpy.init()
    node = MissionBtNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
