"""Standalone mock services for testing the mission flow.

This node provides mocked waypoint, navigation, battery, recharge and optional
PTZ services/topics so ``mission_bt_node`` can be driven from ``/mission/start``
without a full robot stack.
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rhw_msgs.msg import PtzStatus, UdpBatteryStatus, WaypointTask
from rhw_msgs.srv import (
    CaptureImage,
    GetWaypoints,
    InspectionAlbumUpload,
    ModelTaskRun,
    PtzAbsoluteMove,
    Recharge,
)
from rhw_task_scheduler.service_audit import ServiceAuditPublisher

try:
    from nav2_msgs.action import FollowPath, NavigateToPose
    _HAS_NAV2_MSGS = True
except ImportError:
    FollowPath = None
    NavigateToPose = None
    _HAS_NAV2_MSGS = False


class MissionTestMocks(Node):
    """Mock external services for the mission workflow."""

    def __init__(self) -> None:
        super().__init__('mission_test_mocks')
        self._callback_group = ReentrantCallbackGroup()
        self._one_shot_timers: set[Any] = set()
        self._timer_lock = threading.Lock()
        self._audit = ServiceAuditPublisher(self)

        self._declare_parameters()
        self._read_parameters()
        self._waypoints_by_map = self._load_waypoints()

        self._last_ptz_pose = {
            'channel': int(self._default_ptz_channel),
            'azimuth': 0.0,
            'elevation': 0.0,
            'zoom': 0.0,
        }
        self._ptz_online = True

        self._waypoints_srv = None
        self._navigate_to_pose_server = None
        self._follow_path_server = None
        self._ptz_move_srv = None
        self._ptz_capture_srv = None
        self._album_upload_srv = None
        self._model_task_srv = None
        self._recharge_srv = None
        self._battery_timer = None

        self._ptz_pub = None
        self._battery_pub = None

        self._setup_interfaces()
        self.get_logger().info('mission_test_mocks started')

    def _declare_parameters(self) -> None:
        self.declare_parameter('use_real_waypoints', False)
        self.declare_parameter('use_real_navigation', False)
        self.declare_parameter('use_real_ptz', True)
        self.declare_parameter('use_real_recharge', False)
        self.declare_parameter('use_real_battery', False)
        self.declare_parameter('use_real_album_upload', False)
        self.declare_parameter('use_real_model_task', False)

        self.declare_parameter('map_name', 'factory_map')
        self.declare_parameter('waypoints_json', '')

        self.declare_parameter('waypoints_service', '/test/waypoint_manager/get_waypoints')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('follow_path_action', '/follow_path')
        self.declare_parameter('ptz_absolute_move_service', '/test/ptz/absolute_move')
        self.declare_parameter('ptz_capture_service', '/test/ptz/capture_image')
        self.declare_parameter('ptz_status_topic', '/test/ptz/status')
        self.declare_parameter('recharge_service', '/test/recharge')
        self.declare_parameter('battery_topic', '/test/robot/battery_status')
        self.declare_parameter(
            'album_upload_service',
            '/test/inspection/album_report/upload',
        )
        self.declare_parameter('model_task_run_service', '/test/rhw/model/task/run')

        self.declare_parameter('default_ptz_channel', 1)
        self.declare_parameter('navigation_delay_sec', 1.2)
        self.declare_parameter('ptz_delay_sec', 1.0)
        self.declare_parameter('battery_publish_period_sec', 1.0)
        self.declare_parameter('battery_level', 95.0)
        self.declare_parameter('navigation_result', 'reached')
        self.declare_parameter('recharge_result', 1)
        self.declare_parameter('capture_save_dir', '/tmp/rhw_task_scheduler_mock')
        self.declare_parameter('album_upload_result', True)
        self.declare_parameter('model_task_result', True)

    def _read_parameters(self) -> None:
        self._use_real_waypoints = bool(self.get_parameter('use_real_waypoints').value)
        self._use_real_navigation = bool(self.get_parameter('use_real_navigation').value)
        self._use_real_ptz = bool(self.get_parameter('use_real_ptz').value)
        self._use_real_recharge = bool(self.get_parameter('use_real_recharge').value)
        self._use_real_battery = bool(self.get_parameter('use_real_battery').value)
        self._use_real_album_upload = bool(
            self.get_parameter('use_real_album_upload').value
        )
        self._use_real_model_task = bool(self.get_parameter('use_real_model_task').value)

        self._map_name = str(self.get_parameter('map_name').value)
        self._waypoints_json = str(self.get_parameter('waypoints_json').value)

        self._waypoints_service = str(self.get_parameter('waypoints_service').value)
        self._navigate_to_pose_action = str(
            self.get_parameter('navigate_to_pose_action').value
        )
        self._follow_path_action = str(self.get_parameter('follow_path_action').value)
        self._ptz_absolute_move_service = str(
            self.get_parameter('ptz_absolute_move_service').value
        )
        self._ptz_capture_service = str(self.get_parameter('ptz_capture_service').value)
        self._ptz_status_topic = str(self.get_parameter('ptz_status_topic').value)
        self._recharge_service = str(self.get_parameter('recharge_service').value)
        self._battery_topic = str(self.get_parameter('battery_topic').value)
        self._album_upload_service = str(
            self.get_parameter('album_upload_service').value
        )
        self._model_task_run_service = str(
            self.get_parameter('model_task_run_service').value
        )

        self._default_ptz_channel = int(self.get_parameter('default_ptz_channel').value)
        self._navigation_delay_sec = max(
            float(self.get_parameter('navigation_delay_sec').value), 0.0
        )
        self._ptz_delay_sec = max(float(self.get_parameter('ptz_delay_sec').value), 0.0)
        self._battery_publish_period_sec = max(
            float(self.get_parameter('battery_publish_period_sec').value), 0.2
        )
        self._battery_level = float(self.get_parameter('battery_level').value)
        self._navigation_result = (
            str(self.get_parameter('navigation_result').value).strip().lower()
        )
        self._recharge_result = int(self.get_parameter('recharge_result').value)
        self._capture_save_dir = Path(
            str(self.get_parameter('capture_save_dir').value)
        ).expanduser()
        self._album_upload_result = bool(
            self.get_parameter('album_upload_result').value
        )
        self._model_task_result = bool(self.get_parameter('model_task_result').value)

    def _setup_interfaces(self) -> None:
        if not self._use_real_waypoints:
            self._waypoints_srv = self.create_service(
                GetWaypoints,
                self._waypoints_service,
                self._handle_get_waypoints,
                callback_group=self._callback_group,
            )

        if not self._use_real_navigation:
            if _HAS_NAV2_MSGS:
                self._navigate_to_pose_server = ActionServer(
                    self,
                    NavigateToPose,
                    self._navigate_to_pose_action,
                    execute_callback=self._execute_navigate_to_pose,
                    goal_callback=self._handle_action_goal,
                    cancel_callback=self._handle_action_cancel,
                    callback_group=self._callback_group,
                )
                self._follow_path_server = ActionServer(
                    self,
                    FollowPath,
                    self._follow_path_action,
                    execute_callback=self._execute_follow_path,
                    goal_callback=self._handle_action_goal,
                    cancel_callback=self._handle_action_cancel,
                    callback_group=self._callback_group,
                )
            else:
                self.get_logger().warning(
                    'nav2_msgs is not installed; mock /navigate_to_pose and '
                    '/follow_path action servers are disabled'
                )

        if not self._use_real_ptz:
            self._ptz_pub = self.create_publisher(PtzStatus, self._ptz_status_topic, 10)
            self._ptz_move_srv = self.create_service(
                PtzAbsoluteMove,
                self._ptz_absolute_move_service,
                self._handle_ptz_move,
                callback_group=self._callback_group,
            )
            self._ptz_capture_srv = self.create_service(
                CaptureImage,
                self._ptz_capture_service,
                self._handle_capture_image,
                callback_group=self._callback_group,
            )
            self._publish_ptz_status(active_action='idle', message='mock ptz ready')

        if not self._use_real_album_upload:
            self._album_upload_srv = self.create_service(
                InspectionAlbumUpload,
                self._album_upload_service,
                self._handle_album_upload,
                callback_group=self._callback_group,
            )

        if not self._use_real_model_task:
            self._model_task_srv = self.create_service(
                ModelTaskRun,
                self._model_task_run_service,
                self._handle_model_task_run,
                callback_group=self._callback_group,
            )

        if not self._use_real_recharge:
            self._recharge_srv = self.create_service(
                Recharge,
                self._recharge_service,
                self._handle_recharge,
                callback_group=self._callback_group,
            )

        if not self._use_real_battery:
            self._battery_pub = self.create_publisher(UdpBatteryStatus, self._battery_topic, 10)
            self._battery_timer = self.create_timer(
                self._battery_publish_period_sec,
                self._publish_battery_status,
                callback_group=self._callback_group,
            )
            self._publish_battery_status()

        self.get_logger().info(
            'mock configuration: '
            f'waypoints={not self._use_real_waypoints} '
            f'nav={not self._use_real_navigation} '
            f'ptz={not self._use_real_ptz} '
            f'album_upload={not self._use_real_album_upload} '
            f'model_task={not self._use_real_model_task} '
            f'recharge={not self._use_real_recharge} '
            f'battery={not self._use_real_battery}'
        )

    def _load_waypoints(self) -> dict[str, list[WaypointTask]]:
        raw = self._waypoints_json.strip()
        if not raw:
            return {self._map_name: self._default_waypoints(self._map_name)}

        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(
                f'waypoints_json parse failed, fallback to defaults: {exc}'
            )
            return {self._map_name: self._default_waypoints(self._map_name)}

        if isinstance(data, list):
            return {self._map_name: [self._spec_to_waypoint(item, self._map_name) for item in data]}

        if isinstance(data, dict):
            if 'waypoints' in data and isinstance(data['waypoints'], list):
                map_name = str(data.get('map_name', self._map_name))
                return {
                    map_name: [self._spec_to_waypoint(item, map_name) for item in data['waypoints']]
                }

            mapped: dict[str, list[WaypointTask]] = {}
            for map_name, waypoints in data.items():
                if isinstance(waypoints, list):
                    mapped[str(map_name)] = [
                        self._spec_to_waypoint(item, str(map_name)) for item in waypoints
                    ]
            if mapped:
                return mapped

        self.get_logger().warning('Unsupported waypoints_json format, fallback to defaults')
        return {self._map_name: self._default_waypoints(self._map_name)}

    def _default_waypoints(self, map_name: str) -> list[WaypointTask]:
        return [
            self._spec_to_waypoint(
                {
                    'waypoint_id': 'normal_001',
                    'pose': {'x': 1.0, 'y': 0.0, 'theta': 0.0},
                    'waypoint_type': WaypointTask.TYPE_NORMAL,
                    'label': 'mock navigation',
                    'task_params': '',
                },
                map_name,
            ),
            self._spec_to_waypoint(
                {
                    'waypoint_id': 'vision_001',
                    'pose': {'x': 2.0, 'y': 0.0, 'theta': 0.0},
                    'waypoint_type': WaypointTask.TYPE_VISION,
                    'label': 'mock vision',
                    'task_params': json.dumps(
                        {
                            'azimuth': 180.0,
                            'elevation': 0.0,
                            'zoom': 0.0,
                            'channel': self._default_ptz_channel,
                            'azimuth_speed': 50,
                            'elevation_speed': 50,
                            'inference_type': 'fire_equipment_detection',
                        },
                        ensure_ascii=False,
                    ),
                },
                map_name,
            ),
            self._spec_to_waypoint(
                {
                    'waypoint_id': 'charge_001',
                    'pose': {'x': 3.0, 'y': 0.0, 'theta': 0.0},
                    'waypoint_type': WaypointTask.TYPE_CHARGE,
                    'label': 'mock recharge',
                    'task_params': json.dumps({'timeout_sec': 300}, ensure_ascii=False),
                },
                map_name,
            ),
            self._spec_to_waypoint(
                {
                    'waypoint_id': 'follow_path_001',
                    'pose': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
                    'waypoint_type': WaypointTask.TYPE_FOLLOW_PATH,
                    'label': 'mock follow path',
                    'task_params': json.dumps(
                        {
                            'path': [
                                {'x': 0.0, 'y': 0.0, 'theta': 0.0},
                                {'x': 1.0, 'y': 0.4, 'theta': 0.0},
                                {'x': 2.0, 'y': 0.0, 'theta': 0.0},
                            ],
                        },
                        ensure_ascii=False,
                    ),
                },
                map_name,
            ),
        ]

    def _spec_to_waypoint(self, spec: Any, default_map_name: str) -> WaypointTask:
        if not isinstance(spec, dict):
            spec = {}

        wp = WaypointTask()
        wp.waypoint_id = str(spec.get('waypoint_id', 'waypoint'))
        wp.map_name = str(spec.get('map_name', default_map_name))
        pose = spec.get('pose', {})
        if not isinstance(pose, dict):
            pose = {}
        wp.pose.x = float(pose.get('x', 0.0))
        wp.pose.y = float(pose.get('y', 0.0))
        wp.pose.theta = float(pose.get('theta', 0.0))
        wp.waypoint_type = int(spec.get('waypoint_type', WaypointTask.TYPE_NORMAL))
        wp.label = str(spec.get('label', wp.waypoint_id))
        task_params = spec.get('task_params', '')
        if isinstance(task_params, (dict, list)):
            wp.task_params = json.dumps(task_params, ensure_ascii=False)
        else:
            wp.task_params = str(task_params)
        return wp

    def _handle_action_goal(self, goal_request) -> GoalResponse:
        del goal_request
        return GoalResponse.ACCEPT

    def _handle_action_cancel(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def _schedule_once(self, delay_sec: float, callback) -> None:
        delay_sec = max(float(delay_sec), 0.0)
        if delay_sec == 0.0:
            callback()
            return

        holder: dict[str, Any] = {}

        def _wrapped() -> None:
            timer = holder.get('timer')
            if timer is not None:
                timer.cancel()
                with self._timer_lock:
                    self._one_shot_timers.discard(timer)
            callback()

        timer = self.create_timer(delay_sec, _wrapped, callback_group=self._callback_group)
        holder['timer'] = timer
        with self._timer_lock:
            self._one_shot_timers.add(timer)

    def _handle_get_waypoints(
        self, request: GetWaypoints.Request, response: GetWaypoints.Response
    ) -> GetWaypoints.Response:
        started_at = time.monotonic()
        self._audit.publish(
            service=self._waypoints_service,
            role='server',
            phase='request',
            request=request,
        )

        map_name = request.map_name.strip()
        if map_name:
            waypoints = self._waypoints_by_map.get(map_name)
            if waypoints is None:
                response.result = 0
                response.waypoints = []
                response.message = f'map not found: {map_name}'
                self._audit.publish(
                    service=self._waypoints_service,
                    role='server',
                    phase='response',
                    request=request,
                    response=response,
                    success=False,
                    duration_ms=(time.monotonic() - started_at) * 1000.0,
                )
                return response
        else:
            waypoints = [wp for items in self._waypoints_by_map.values() for wp in items]

        response.result = 1
        response.waypoints = waypoints
        response.message = f'mock waypoints returned: {len(waypoints)}'
        self._audit.publish(
            service=self._waypoints_service,
            role='server',
            phase='response',
            request=request,
            response=response,
            success=True,
            duration_ms=(time.monotonic() - started_at) * 1000.0,
            details={'map_name': map_name or '*', 'count': len(waypoints)},
        )
        self.get_logger().info(
            f'GetWaypoints -> map={map_name or "*"} count={len(waypoints)}'
        )
        return response

    def _execute_navigate_to_pose(self, goal_handle):
        request = goal_handle.request
        goal = request.pose
        self.get_logger().info(
            'NavigateToPose accepted: '
            f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
        )

        result = NavigateToPose.Result()
        if not self._publish_nav2_feedback(goal_handle, goal):
            return result

        if self._navigation_result == 'failed':
            goal_handle.abort()
        elif self._navigation_result == 'cancelled':
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return result

    def _execute_follow_path(self, goal_handle):
        request = goal_handle.request
        path_points = len(request.path.poses)
        self.get_logger().info(f'FollowPath accepted: points={path_points}')

        result = FollowPath.Result()
        if not self._publish_follow_path_feedback(goal_handle, path_points):
            return result

        if self._navigation_result == 'failed':
            if hasattr(result, 'error_code'):
                result.error_code = 1
            if hasattr(result, 'error_msg'):
                result.error_msg = 'mock FollowPath failed'
            goal_handle.abort()
        elif self._navigation_result == 'cancelled':
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return result

    def _publish_nav2_feedback(self, goal_handle, goal: PoseStamped) -> bool:
        steps = max(int(self._navigation_delay_sec / 0.2), 1)
        step_sec = self._navigation_delay_sec / float(steps)
        for index in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False

            feedback = NavigateToPose.Feedback()
            if hasattr(feedback, 'current_pose'):
                feedback.current_pose = goal
            if hasattr(feedback, 'distance_remaining'):
                feedback.distance_remaining = float(steps - index) / float(steps)
            if hasattr(feedback, 'estimated_time_remaining'):
                feedback.estimated_time_remaining = self._duration_from_sec(
                    (steps - index) * step_sec
                )
            if hasattr(feedback, 'navigation_time'):
                feedback.navigation_time = self._duration_from_sec(index * step_sec)
            if hasattr(feedback, 'number_of_recoveries'):
                feedback.number_of_recoveries = 0
            goal_handle.publish_feedback(feedback)
            time.sleep(step_sec)
        return True

    def _publish_follow_path_feedback(self, goal_handle, path_points: int) -> bool:
        steps = max(int(self._navigation_delay_sec / 0.2), 1)
        for index in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False

            feedback = FollowPath.Feedback()
            if hasattr(feedback, 'distance_to_goal'):
                feedback.distance_to_goal = float(steps - index) / float(steps)
            if hasattr(feedback, 'speed'):
                feedback.speed = 0.3
            if hasattr(feedback, 'current_pose') and path_points > 0:
                feedback.current_pose = goal_handle.request.path.poses[
                    min(index, path_points - 1)
                ]
            goal_handle.publish_feedback(feedback)
            time.sleep(self._navigation_delay_sec / float(steps))
        return True

    @staticmethod
    def _duration_from_sec(seconds: float) -> Duration:
        duration = Duration()
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - duration.sec) * 1_000_000_000)
        return duration

    def _handle_ptz_move(
        self, request: PtzAbsoluteMove.Request, response: PtzAbsoluteMove.Response
    ) -> PtzAbsoluteMove.Response:
        started_at = time.monotonic()
        self._audit.publish(
            service=self._ptz_absolute_move_service,
            role='server',
            phase='request',
            request=request,
        )

        self._last_ptz_pose = {
            'channel': int(request.channel) if request.channel else self._default_ptz_channel,
            'azimuth': float(request.azimuth),
            'elevation': float(request.elevation),
            'zoom': float(request.zoom),
        }
        self._publish_ptz_status(active_action='moving', message='mock ptz moving')
        self._schedule_once(self._ptz_delay_sec, self._finish_ptz_move)

        response.result = 1
        response.message = 'mock ptz move accepted'
        self._audit.publish(
            service=self._ptz_absolute_move_service,
            role='server',
            phase='response',
            request=request,
            response=response,
            success=True,
            duration_ms=(time.monotonic() - started_at) * 1000.0,
        )
        self.get_logger().info(
            'PTZ absolute move accepted: '
            f'ch={self._last_ptz_pose["channel"]} '
            f'az={self._last_ptz_pose["azimuth"]:.2f} '
            f'el={self._last_ptz_pose["elevation"]:.2f} '
            f'zoom={self._last_ptz_pose["zoom"]:.2f}'
        )
        return response

    def _finish_ptz_move(self) -> None:
        self._publish_ptz_status(active_action='idle', message='mock ptz idle')

    def _publish_ptz_status(self, *, active_action: str, message: str) -> None:
        if self._ptz_pub is None:
            return

        msg = PtzStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.online = self._ptz_online
        msg.channel = int(self._last_ptz_pose['channel'])
        msg.azimuth = float(self._last_ptz_pose['azimuth'])
        msg.elevation = float(self._last_ptz_pose['elevation'])
        msg.zoom = float(self._last_ptz_pose['zoom'])
        msg.active_action = str(active_action)
        msg.message = str(message)
        self._ptz_pub.publish(msg)

    def _handle_capture_image(
        self, request: CaptureImage.Request, response: CaptureImage.Response
    ) -> CaptureImage.Response:
        started_at = time.monotonic()
        self._audit.publish(
            service=self._ptz_capture_service,
            role='server',
            phase='request',
            request=request,
        )

        channel = int(request.channel) if request.channel else self._default_ptz_channel
        save_path = request.save_path.strip() if request.save_path else ''
        if not save_path:
            ts = time.strftime('%Y%m%d_%H%M%S')
            save_path = str(self._capture_save_dir / f'capture_{channel}_{ts}.jpg')

        payload = f'mock capture channel={channel}\n'.encode('utf-8')
        try:
            Path(save_path).parent.mkdir(parents=True, exist_ok=True)
            with open(save_path, 'wb') as f:
                f.write(payload)
        except OSError as exc:
            response.result = 0
            response.capture_url = ''
            response.file_path = ''
            response.file_size = 0
            response.saved = False
            response.message = f'mock capture failed: {exc}'
            self._audit.publish(
                service=self._ptz_capture_service,
                role='server',
                phase='response',
                request=request,
                response=response,
                success=False,
                duration_ms=(time.monotonic() - started_at) * 1000.0,
                details={'error': str(exc)},
            )
            return response

        response.result = 1
        response.capture_url = f'mock://capture/{channel}'
        response.file_path = save_path
        response.file_size = len(payload)
        response.saved = True
        response.message = 'mock capture saved'
        self._audit.publish(
            service=self._ptz_capture_service,
            role='server',
            phase='response',
            request=request,
            response=response,
            success=True,
            duration_ms=(time.monotonic() - started_at) * 1000.0,
            details={'file_path': save_path, 'file_size': len(payload)},
        )
        self.get_logger().info(f'CaptureImage saved mock file: {save_path}')
        return response

    def _handle_album_upload(
        self,
        request: InspectionAlbumUpload.Request,
        response: InspectionAlbumUpload.Response,
    ) -> InspectionAlbumUpload.Response:
        started_at = time.monotonic()
        self._audit.publish(
            service=self._album_upload_service,
            role='server',
            phase='request',
            request=request,
        )

        response.ok = bool(self._album_upload_result)
        response.code = 'OK' if response.ok else 'MOCK_UPLOAD_FAILED'
        response.message = (
            'mock album upload ok' if response.ok else 'mock album upload failed'
        )
        response.trace_id = f'mock-{time.time_ns()}'
        response.http_status = 200 if response.ok else 500
        response.response_body = '{"code":0}' if response.ok else '{"code":500}'

        self._audit.publish(
            service=self._album_upload_service,
            role='server',
            phase='response',
            request=request,
            response=response,
            success=response.ok,
            duration_ms=(time.monotonic() - started_at) * 1000.0,
        )
        self.get_logger().info(
            f'InspectionAlbumUpload result={response.ok} image={request.image_path}'
        )
        return response

    def _handle_model_task_run(
        self,
        request: ModelTaskRun.Request,
        response: ModelTaskRun.Response,
    ) -> ModelTaskRun.Response:
        started_at = time.monotonic()
        self._audit.publish(
            service=self._model_task_run_service,
            role='server',
            phase='request',
            request=request,
        )

        response.ok = bool(self._model_task_result)
        response.code = 'OK' if response.ok else 'MOCK_MODEL_FAILED'
        response.message = 'mock model task ok' if response.ok else 'mock model task failed'
        response.request_id = str(request.request_id)
        response.task_name = str(request.task_name)
        response.task_type = 'mock'
        response.model_path = '/tmp/mock_model.engine'
        response.backend = 'mock'
        response.frame_path = '/tmp/mock_frame.jpg'
        response.result_json_path = f'/tmp/mock_model_result_{request.request_id}.json'
        response.item_count = 1 if response.ok else 0
        response.error_count = 0 if response.ok else 1
        response.latency_ms = 12.3
        response.error_category = '' if response.ok else 'mock_error'
        response.detail_json = (
            '{"items":[{"label":"mock","score":0.99}]}' if response.ok else '{}'
        )

        self._audit.publish(
            service=self._model_task_run_service,
            role='server',
            phase='response',
            request=request,
            response=response,
            success=response.ok,
            duration_ms=(time.monotonic() - started_at) * 1000.0,
        )
        self.get_logger().info(
            f'ModelTaskRun result={response.ok} task_name={request.task_name}'
        )
        return response

    def _handle_recharge(
        self, request: Recharge.Request, response: Recharge.Response
    ) -> Recharge.Response:
        started_at = time.monotonic()
        self._audit.publish(
            service=self._recharge_service,
            role='server',
            phase='request',
            request=request,
        )

        response.result = int(self._recharge_result)
        response_message = 'mock recharge ok' if response.result >= 0 else 'mock recharge failed'
        self._audit.publish(
            service=self._recharge_service,
            role='server',
            phase='response',
            request=request,
            response=response,
            success=(response.result >= 0),
            duration_ms=(time.monotonic() - started_at) * 1000.0,
            details={'result': response.result},
        )
        self.get_logger().info(f'Recharge result: {response_message}')
        return response

    def _publish_battery_status(self) -> None:
        if self._battery_pub is None:
            return

        msg = UdpBatteryStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage_left = 24.0
        msg.voltage_right = 24.0
        msg.battery_level_left = float(self._battery_level)
        msg.battery_level_right = float(self._battery_level)
        msg.battery_temperature_left = 28.0
        msg.battery_temperature_right = 28.0
        msg.charge_left = False
        msg.charge_right = False
        self._battery_pub.publish(msg)

    def destroy_node(self) -> bool:
        for action_server in (self._navigate_to_pose_server, self._follow_path_server):
            if action_server is not None:
                try:
                    action_server.destroy()
                except Exception:
                    pass
        with self._timer_lock:
            timers = list(self._one_shot_timers)
            self._one_shot_timers.clear()
        for timer in timers:
            try:
                timer.cancel()
            except Exception:
                pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MissionTestMocks()
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
