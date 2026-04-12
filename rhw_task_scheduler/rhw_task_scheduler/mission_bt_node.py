"""mission_bt_node — 行为树驱动的巡检任务调度节点.

职责:
  1. 接收 StartMission / StopMission / PauseMission Service 或 MQTT 消息。
  2. 从 waypoint_manager 获取航点详情。
  3. 动态构建行为树，按顺序对每个航点执行 "导航 → 到达后任务"。
  4. 发布 MissionStatus 话题反馈进度。

行为树结构 (每个航点):
  Sequence
  ├── CheckBattery (Condition)
  ├── NavigateToGoal (Action)
  └── Selector [按 waypoint_type 分支]
      ├── Sequence [TYPE_VISION]
      │   ├── IsVisionPoint
      │   ├── PtzGotoPreset
      │   ├── WaitPtzStable
      │   ├── CaptureImage
      │   └── TriggerInference
      ├── Sequence [TYPE_CHARGE]
      │   ├── IsChargePoint
      │   └── Recharge
      └── IsNormalPoint (SUCCESS = 到达即完成)
"""
from __future__ import annotations

import json
import threading
from typing import Any

import py_trees
import rclpy
from rclpy.node import Node

from rhw_msgs.msg import MissionStatus, WaypointTask
from rhw_msgs.srv import GetWaypoints, PauseMission, StartMission, StopMission

from rhw_task_scheduler.bt_actions.charge_action import Recharge
from rhw_task_scheduler.bt_actions.condition_nodes import (
    CheckBattery,
    IsChargePoint,
    IsNormalPoint,
    IsVisionPoint,
)
from rhw_task_scheduler.bt_actions.inference_action import TriggerInference
from rhw_task_scheduler.bt_actions.navigate_action import CancelNavigation, NavigateToGoal
from rhw_task_scheduler.bt_actions.ptz_actions import CaptureImage, PtzGotoPreset, WaitPtzStable


class MissionBtNode(Node):
    """行为树驱动的巡检任务调度主节点."""

    def __init__(self) -> None:
        super().__init__('mission_bt_node')
        self._declare_parameters()
        self._read_parameters()

        # ---- 状态 ----
        self._mission_running = False
        self._mission_paused = False
        self._waypoint_queue: list[dict[str, Any]] = []
        self._current_index = 0
        self._completed_count = 0
        self._bt: py_trees.trees.BehaviourTree | None = None
        self._bt_lock = threading.Lock()

        # ---- 发布器 ----
        self._status_pub = self.create_publisher(
            MissionStatus, self._mission_status_topic, 10
        )

        # ---- Service Clients ----
        self._get_waypoints_client = self.create_client(
            GetWaypoints, self._get_waypoints_service
        )

        # ---- Service Servers ----
        self.create_service(StartMission, '/mission/start', self._handle_start)
        self.create_service(StopMission, '/mission/stop', self._handle_stop)
        self.create_service(PauseMission, '/mission/pause', self._handle_pause)

        # ---- Blackboard 初始化 ----
        self._bb = py_trees.blackboard.Client(name='MissionBtNode')
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/nav_result', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/nav_retry_max', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/battery_low', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/last_capture_path', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/inference_result', access=py_trees.common.Access.WRITE)
        self._bb.set('/nav_retry_max', self._nav_retry_max)
        self._bb.set('/battery_low', False)
        self._bb.set('/last_capture_path', '')
        self._bb.set('/inference_result', {})

        # ---- MQTT (可选) ----
        self._mqtt_client = None
        if self._mqtt_enabled:
            self._setup_mqtt()

        # ---- 行为树 tick 定时器 ----
        self._tick_timer = self.create_timer(
            1.0 / self._bt_tick_rate_hz, self._tick_bt
        )

        # ---- 状态发布定时器 ----
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info('mission_bt_node started')

    # ================================================================
    #  参数声明与读取
    # ================================================================

    def _declare_parameters(self) -> None:
        self.declare_parameter('bt_tick_rate_hz', 10.0)
        self.declare_parameter('goal_service', '/move_base_simple/goal')
        self.declare_parameter('cancel_service', '/move_base/cancel')
        self.declare_parameter('nav_status_topic', '/navigation_status')
        self.declare_parameter('nav_retry_max', 3)
        self.declare_parameter('ptz_goto_preset_service', '/ptz/goto_preset')
        self.declare_parameter('ptz_capture_service', '/ptz/capture_image')
        self.declare_parameter('ptz_status_topic', '/ptz/status')
        self.declare_parameter('ptz_stable_timeout_sec', 5.0)
        self.declare_parameter('default_ptz_channel', 1)
        self.declare_parameter('recharge_service', '/recharge')
        self.declare_parameter('battery_topic', '/robot/battery_status')
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('waypoint_task_timeout_sec', 120.0)
        self.declare_parameter('mqtt_enabled', False)
        self.declare_parameter('mqtt_broker_host', '127.0.0.1')
        self.declare_parameter('mqtt_broker_port', 1883)
        self.declare_parameter('mqtt_client_id', 'rhw_mission_bt')
        self.declare_parameter('mqtt_mission_start_topic', 'rhw/mission/start')
        self.declare_parameter('mqtt_mission_status_topic', 'rhw/mission/status')
        self.declare_parameter('mission_status_topic', '/mission/status')
        self.declare_parameter('get_waypoints_service', '/waypoint_manager/get_waypoints')

    def _read_parameters(self) -> None:
        self._bt_tick_rate_hz = float(self.get_parameter('bt_tick_rate_hz').value)
        self._nav_retry_max = int(self.get_parameter('nav_retry_max').value)
        self._mqtt_enabled = bool(self.get_parameter('mqtt_enabled').value)
        self._mqtt_broker_host = str(self.get_parameter('mqtt_broker_host').value)
        self._mqtt_broker_port = int(self.get_parameter('mqtt_broker_port').value)
        self._mqtt_client_id = str(self.get_parameter('mqtt_client_id').value)
        self._mqtt_start_topic = str(self.get_parameter('mqtt_mission_start_topic').value)
        self._mqtt_status_topic = str(self.get_parameter('mqtt_mission_status_topic').value)
        self._mission_status_topic = str(self.get_parameter('mission_status_topic').value)
        self._get_waypoints_service = str(self.get_parameter('get_waypoints_service').value)

    # ================================================================
    #  MQTT
    # ================================================================

    def _setup_mqtt(self) -> None:
        try:
            import paho.mqtt.client as mqtt

            client = mqtt.Client(
                client_id=self._mqtt_client_id,
                protocol=mqtt.MQTTv311,
            )
            client.on_connect = self._on_mqtt_connect
            client.on_message = self._on_mqtt_message
            client.connect_async(self._mqtt_broker_host, self._mqtt_broker_port)
            client.loop_start()
            self._mqtt_client = client
            self.get_logger().info(
                f'MQTT connected to {self._mqtt_broker_host}:{self._mqtt_broker_port}'
            )
        except Exception as exc:
            self.get_logger().warning(f'MQTT setup failed: {exc}')

    def _on_mqtt_connect(self, client, userdata, flags, rc) -> None:
        if rc == 0:
            client.subscribe(self._mqtt_start_topic)
            self.get_logger().info(f'MQTT subscribed: {self._mqtt_start_topic}')
        else:
            self.get_logger().warning(f'MQTT connect failed: rc={rc}')

    def _on_mqtt_message(self, client, userdata, msg) -> None:
        if msg.topic == self._mqtt_start_topic:
            try:
                payload = json.loads(msg.payload.decode('utf-8'))
                map_name = payload.get('map_name', '')
                waypoint_ids = payload.get('waypoint_ids', [])
                if map_name and waypoint_ids:
                    self.get_logger().info(
                        f'MQTT mission start: map={map_name} wps={len(waypoint_ids)}'
                    )
                    self._start_mission(map_name, list(waypoint_ids))
            except (json.JSONDecodeError, TypeError) as exc:
                self.get_logger().warning(f'MQTT message parse error: {exc}')

    def _mqtt_publish_status(self, status_dict: dict) -> None:
        if self._mqtt_client is not None:
            try:
                self._mqtt_client.publish(
                    self._mqtt_status_topic,
                    json.dumps(status_dict, ensure_ascii=False),
                    qos=0,
                )
            except Exception:
                pass

    # ================================================================
    #  Service Handlers
    # ================================================================

    def _handle_start(
        self, request: StartMission.Request, response: StartMission.Response
    ) -> StartMission.Response:
        if self._mission_running:
            response.result = 0
            response.message = 'Mission already running, stop first'
            return response

        map_name = request.map_name
        waypoint_ids = list(request.waypoint_ids)

        if not map_name or not waypoint_ids:
            response.result = 0
            response.message = 'map_name and waypoint_ids are required'
            return response

        ok, msg = self._start_mission(map_name, waypoint_ids)
        response.result = 1 if ok else 0
        response.message = msg
        return response

    def _handle_stop(
        self, request: StopMission.Request, response: StopMission.Response
    ) -> StopMission.Response:
        self._stop_mission()
        response.result = 1
        response.message = 'Mission stopped'
        return response

    def _handle_pause(
        self, request: PauseMission.Request, response: PauseMission.Response
    ) -> PauseMission.Response:
        if not self._mission_running:
            response.result = 0
            response.message = 'No mission running'
            return response

        self._mission_paused = request.pause
        state = 'paused' if request.pause else 'resumed'
        self.get_logger().info(f'Mission {state}')
        response.result = 1
        response.message = f'Mission {state}'
        return response

    # ================================================================
    #  任务管理核心
    # ================================================================

    def _start_mission(self, map_name: str, waypoint_ids: list[str]) -> tuple[bool, str]:
        """获取航点详情并启动行为树."""
        # 同步调用 GetWaypoints
        if not self._get_waypoints_client.service_is_ready():
            return False, 'GetWaypoints service not ready'

        req = GetWaypoints.Request()
        req.map_name = map_name
        future = self._get_waypoints_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            return False, 'GetWaypoints service timeout'

        result = future.result()
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

        self._waypoint_queue = queue
        self._current_index = 0
        self._completed_count = 0
        self._mission_running = True
        self._mission_paused = False

        # 设置第一个航点并构建行为树
        self._setup_current_waypoint()

        self.get_logger().info(
            f'Mission started: map={map_name} waypoints={len(queue)}'
        )
        return True, f'Mission started with {len(queue)} waypoints'

    def _stop_mission(self) -> None:
        """停止当前任务."""
        with self._bt_lock:
            self._bt = None
        self._mission_running = False
        self._mission_paused = False
        self._waypoint_queue.clear()
        self._current_index = 0
        self._completed_count = 0
        self.get_logger().info('Mission stopped')

    def _setup_current_waypoint(self) -> None:
        """将当前航点写入 Blackboard 并构建单航点行为树."""
        if self._current_index >= len(self._waypoint_queue):
            return
        wp = self._waypoint_queue[self._current_index]
        self._bb.set('/current_waypoint', wp)
        self._bb.set('/nav_result', '')
        self._bb.set('/last_capture_path', '')

        tree = self._build_waypoint_tree()
        with self._bt_lock:
            self._bt = py_trees.trees.BehaviourTree(root=tree)

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
        ├── NavigateToGoal
        └── Selector [任务类型分支]
            ├── Sequence [VISION: preset → wait → capture → inference]
            ├── Sequence [CHARGE: recharge]
            └── IsNormalPoint [NORMAL: 到达即完成]
        """
        root = py_trees.composites.Sequence(name='WaypointHandler', memory=True)

        # 1) 电量检查
        root.add_child(CheckBattery('CheckBattery', node=self))

        # 2) 导航到目标
        root.add_child(NavigateToGoal('NavigateToGoal', node=self))

        # 3) 到达后任务分支
        task_selector = py_trees.composites.Selector(
            name='TaskSelector', memory=False
        )

        # 3a) 视觉识别任务
        vision_seq = py_trees.composites.Sequence(name='VisionTask', memory=True)
        vision_seq.add_child(IsVisionPoint('IsVisionPoint?'))
        vision_seq.add_child(PtzGotoPreset('PtzGotoPreset', node=self))
        vision_seq.add_child(WaitPtzStable('WaitPtzStable', node=self))
        vision_seq.add_child(CaptureImage('CaptureImage', node=self))
        vision_seq.add_child(TriggerInference('TriggerInference', node=self))
        task_selector.add_child(vision_seq)

        # 3b) 充电任务
        charge_seq = py_trees.composites.Sequence(name='ChargeTask', memory=True)
        charge_seq.add_child(IsChargePoint('IsChargePoint?'))
        charge_seq.add_child(Recharge('Recharge', node=self))
        task_selector.add_child(charge_seq)

        # 3c) 普通导航点（到达即完成）
        task_selector.add_child(IsNormalPoint('IsNormalPoint?'))

        root.add_child(task_selector)
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
            return

        try:
            bt.tick()
        except Exception as exc:
            self.get_logger().error(f'BT tick error: {exc}')
            self._stop_mission()
            return

        root_status = bt.root.status

        if root_status == py_trees.common.Status.SUCCESS:
            wp = self._waypoint_queue[self._current_index]
            self.get_logger().info(
                f'Waypoint completed: {wp.get("waypoint_id", "?")} '
                f'({self._completed_count + 1}/{len(self._waypoint_queue)})'
            )
            if not self._advance_to_next_waypoint():
                self._mission_running = False
                self.get_logger().info('Mission completed — all waypoints done')

        elif root_status == py_trees.common.Status.FAILURE:
            wp = self._waypoint_queue[self._current_index]
            self.get_logger().warning(
                f'Waypoint failed: {wp.get("waypoint_id", "?")}, skipping'
            )
            if not self._advance_to_next_waypoint():
                self._mission_running = False
                self.get_logger().info('Mission completed (with failures)')

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

        # MQTT 转发
        if self._mqtt_client is not None:
            self._mqtt_publish_status({
                'status': int(msg.status),
                'current_waypoint_id': msg.current_waypoint_id,
                'total_waypoints': int(msg.total_waypoints),
                'completed_waypoints': int(msg.completed_waypoints),
            })

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

    def destroy_node(self) -> bool:
        if self._mqtt_client is not None:
            try:
                self._mqtt_client.loop_stop()
                self._mqtt_client.disconnect()
            except Exception:
                pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MissionBtNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
