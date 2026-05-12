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
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rhw_msgs.msg import NavigationStatus, PtzStatus, UdpBatteryStatus, WaypointTask
from rhw_msgs.srv import CaptureImage, Goal, GetWaypoints, PtzAbsoluteMove, Recharge
from rhw_task_scheduler.service_audit import ServiceAuditPublisher


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

        self._nav_goal: PoseStamped | None = None
        self._last_ptz_pose = {
            'channel': int(self._default_ptz_channel),
            'azimuth': 0.0,
            'elevation': 0.0,
            'zoom': 0.0,
        }
        self._ptz_online = True

        self._waypoints_srv = None
        self._goal_srv = None
        self._ptz_move_srv = None
        self._ptz_capture_srv = None
        self._recharge_srv = None
        self._battery_timer = None

        self._nav_pub = None
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

        self.declare_parameter('map_name', 'factory_map')
        self.declare_parameter('waypoints_json', '')

        self.declare_parameter('waypoints_service', '/test/waypoint_manager/get_waypoints')
        self.declare_parameter('goal_service', '/test/move_base_simple/goal')
        self.declare_parameter('nav_status_topic', '/test/navigation_status')
        self.declare_parameter('ptz_absolute_move_service', '/test/ptz/absolute_move')
        self.declare_parameter('ptz_capture_service', '/test/ptz/capture_image')
        self.declare_parameter('ptz_status_topic', '/test/ptz/status')
        self.declare_parameter('recharge_service', '/test/recharge')
        self.declare_parameter('battery_topic', '/test/robot/battery_status')

        self.declare_parameter('default_ptz_channel', 1)
        self.declare_parameter('navigation_delay_sec', 1.2)
        self.declare_parameter('ptz_delay_sec', 1.0)
        self.declare_parameter('battery_publish_period_sec', 1.0)
        self.declare_parameter('battery_level', 95.0)
        self.declare_parameter('navigation_result', 'reached')
        self.declare_parameter('recharge_result', 1)
        self.declare_parameter('capture_save_dir', '/tmp/rhw_task_scheduler_mock')

    def _read_parameters(self) -> None:
        self._use_real_waypoints = bool(self.get_parameter('use_real_waypoints').value)
        self._use_real_navigation = bool(self.get_parameter('use_real_navigation').value)
        self._use_real_ptz = bool(self.get_parameter('use_real_ptz').value)
        self._use_real_recharge = bool(self.get_parameter('use_real_recharge').value)
        self._use_real_battery = bool(self.get_parameter('use_real_battery').value)

        self._map_name = str(self.get_parameter('map_name').value)
        self._waypoints_json = str(self.get_parameter('waypoints_json').value)

        self._waypoints_service = str(self.get_parameter('waypoints_service').value)
        self._goal_service = str(self.get_parameter('goal_service').value)
        self._nav_status_topic = str(self.get_parameter('nav_status_topic').value)
        self._ptz_absolute_move_service = str(
            self.get_parameter('ptz_absolute_move_service').value
        )
        self._ptz_capture_service = str(self.get_parameter('ptz_capture_service').value)
        self._ptz_status_topic = str(self.get_parameter('ptz_status_topic').value)
        self._recharge_service = str(self.get_parameter('recharge_service').value)
        self._battery_topic = str(self.get_parameter('battery_topic').value)

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

    def _setup_interfaces(self) -> None:
        if not self._use_real_waypoints:
            self._waypoints_srv = self.create_service(
                GetWaypoints,
                self._waypoints_service,
                self._handle_get_waypoints,
                callback_group=self._callback_group,
            )

        if not self._use_real_navigation:
            self._nav_pub = self.create_publisher(NavigationStatus, self._nav_status_topic, 10)
            self._goal_srv = self.create_service(
                Goal,
                self._goal_service,
                self._handle_goal,
                callback_group=self._callback_group,
            )
            self._publish_navigation_status(
                NavigationStatus.STATUS_IDLE,
                message='mock navigation ready',
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
        wp.task_params = str(spec.get('task_params', ''))
        return wp

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

    def _handle_goal(self, request: Goal.Request, response: Goal.Response) -> Goal.Response:
        started_at = time.monotonic()
        self._audit.publish(
            service=self._goal_service,
            role='server',
            phase='request',
            request=request,
        )

        self._nav_goal = request.goal
        self._publish_navigation_status(
            NavigationStatus.STATUS_NAVIGATING,
            message='mock navigating',
            goal=request.goal,
            remaining_distance=12.0,
            estimated_time=max(self._navigation_delay_sec, 0.1) * 2.0,
        )
        self._schedule_once(self._navigation_delay_sec, self._finish_navigation)

        response.result = 1
        self._audit.publish(
            service=self._goal_service,
            role='server',
            phase='response',
            request=request,
            response=response,
            success=True,
            duration_ms=(time.monotonic() - started_at) * 1000.0,
            details={'result': 'accepted', 'final_state': self._navigation_result},
        )
        self.get_logger().info(
            'Goal accepted: '
            f'({request.goal.pose.position.x:.2f}, {request.goal.pose.position.y:.2f})'
        )
        return response

    def _finish_navigation(self) -> None:
        if self._navigation_result == 'failed':
            status = NavigationStatus.STATUS_FAILED
            message = 'mock navigation failed'
        elif self._navigation_result == 'cancelled':
            status = NavigationStatus.STATUS_CANCELLED
            message = 'mock navigation cancelled'
        else:
            status = NavigationStatus.STATUS_REACHED
            message = 'mock reached'

        self._publish_navigation_status(
            status,
            message=message,
            goal=self._nav_goal,
            remaining_distance=0.0,
            estimated_time=0.0,
        )

    def _publish_navigation_status(
        self,
        status: int,
        *,
        message: str,
        goal: PoseStamped | None = None,
        remaining_distance: float = -1.0,
        estimated_time: float = -1.0,
    ) -> None:
        if self._nav_pub is None:
            return

        msg = NavigationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = int(status)
        if goal is None:
            goal = PoseStamped()
            goal.header.stamp = msg.header.stamp
            goal.header.frame_id = 'map'
        msg.current_goal = goal
        msg.remaining_distance = float(remaining_distance)
        msg.estimated_time = float(estimated_time)
        msg.message = str(message)
        self._nav_pub.publish(msg)

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
