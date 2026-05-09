"""mock_mission_runner — 一键灌入测试点位并启动 mock 任务。

用途：
1. 可选地打开 `mission_bt_node` 的 mock 调试参数。
2. 删除同名旧测试点位。
3. 写入主地图测试点位，并可额外写入一张虚拟地图的点位快照。
4. 调用 `/mission/start` 启动任务。

运行方式：
    ros2 run rhw_task_scheduler mock_mission_runner
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import rclpy
from geometry_msgs.msg import Pose2D
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node

from rhw_msgs.msg import WaypointTask
from rhw_msgs.srv import AddWaypoint, DeleteWaypoint, StartMission


@dataclass(frozen=True)
class WaypointSeed:
    waypoint_id: str
    x: float
    y: float
    theta: float
    waypoint_type: int
    label: str
    task_params: str = ''


DEFAULT_SEEDS: tuple[WaypointSeed, ...] = (
    WaypointSeed(
        waypoint_id='normal_001',
        x=1.0,
        y=1.0,
        theta=0.0,
        waypoint_type=WaypointTask.TYPE_NORMAL,
        label='普通导航点A',
    ),
    WaypointSeed(
        waypoint_id='vision_001',
        x=2.0,
        y=1.5,
        theta=0.0,
        waypoint_type=WaypointTask.TYPE_VISION,
        label='视觉识别点A',
        task_params='{"preset_id":1,"channel":1,"inference_type":"det"}',
    ),
    WaypointSeed(
        waypoint_id='normal_002',
        x=3.0,
        y=2.0,
        theta=1.57,
        waypoint_type=WaypointTask.TYPE_NORMAL,
        label='普通导航点B',
    ),
    WaypointSeed(
        waypoint_id='vision_002',
        x=4.0,
        y=2.5,
        theta=0.0,
        waypoint_type=WaypointTask.TYPE_VISION,
        label='视觉识别点B',
        task_params='{"preset_id":2,"channel":1,"inference_type":"det"}',
    ),
    WaypointSeed(
        waypoint_id='normal_003',
        x=5.0,
        y=1.0,
        theta=3.14,
        waypoint_type=WaypointTask.TYPE_NORMAL,
        label='普通导航点C',
    ),
    WaypointSeed(
        waypoint_id='charge_001',
        x=6.0,
        y=0.8,
        theta=3.14,
        waypoint_type=WaypointTask.TYPE_CHARGE,
        label='充电点',
    ),
)

SECONDARY_SEEDS: tuple[WaypointSeed, ...] = (
    WaypointSeed(
        waypoint_id='room_normal_001',
        x=0.5,
        y=0.5,
        theta=0.0,
        waypoint_type=WaypointTask.TYPE_NORMAL,
        label='房间导航点A',
    ),
    WaypointSeed(
        waypoint_id='room_vision_001',
        x=1.5,
        y=0.8,
        theta=0.0,
        waypoint_type=WaypointTask.TYPE_VISION,
        label='房间视觉点A',
        task_params='{"preset_id":3,"channel":1,"inference_type":"det"}',
    ),
    WaypointSeed(
        waypoint_id='room_vision_002',
        x=2.2,
        y=1.1,
        theta=1.57,
        waypoint_type=WaypointTask.TYPE_VISION,
        label='房间视觉点B',
        task_params='{"preset_id":4,"channel":1,"inference_type":"det"}',
    ),
)


class MockMissionRunnerNode(Node):
    """一键创建测试点位并启动 mock 巡检任务。"""

    def __init__(self) -> None:
        super().__init__('mock_mission_runner')

        self.declare_parameter('map_name', 'factory_map')
        self.declare_parameter('seed_secondary_map', True)
        self.declare_parameter('secondary_map_name', 'room_map')
        self.declare_parameter('mission_node_name', '/mission_bt_node')
        self.declare_parameter('add_waypoint_service', '/waypoint_manager/add_waypoint')
        self.declare_parameter('delete_waypoint_service', '/waypoint_manager/delete_waypoint')
        self.declare_parameter('start_mission_service', '/mission/start')
        self.declare_parameter('enable_mock_params', True)
        self.declare_parameter('mock_delay_sec', 5.0)
        self.declare_parameter('mock_nav_result', 'success')
        self.declare_parameter('mock_ptz_result', 'success')
        self.declare_parameter('mock_capture_result', 'success')
        self.declare_parameter('mock_charge_result', 'success')
        self.declare_parameter('mock_inference_result', 'success')
        self.declare_parameter('mock_battery_level', 100.0)
        self.declare_parameter('configure_tree_debug_params', False)
        self.declare_parameter('print_tree_on_build', True)
        self.declare_parameter('print_tree_on_tick', False)
        self.declare_parameter('export_tree_dot', False)

        self._map_name = str(self.get_parameter('map_name').value)
        self._seed_secondary_map = bool(self.get_parameter('seed_secondary_map').value)
        self._secondary_map_name = str(self.get_parameter('secondary_map_name').value)
        self._mission_node_name = str(self.get_parameter('mission_node_name').value)
        self._enable_mock_params = bool(self.get_parameter('enable_mock_params').value)

        self._add_client = self.create_client(
            AddWaypoint,
            str(self.get_parameter('add_waypoint_service').value),
        )
        self._delete_client = self.create_client(
            DeleteWaypoint,
            str(self.get_parameter('delete_waypoint_service').value),
        )
        self._start_client = self.create_client(
            StartMission,
            str(self.get_parameter('start_mission_service').value),
        )
        self._set_params_client = self.create_client(
            SetParameters,
            f'{self._mission_node_name}/set_parameters',
        )

    def run(self) -> int:
        if self._seed_secondary_map and self._secondary_map_name == self._map_name:
            self.get_logger().error(
                'secondary_map_name must differ from map_name when seed_secondary_map=true'
            )
            return 1

        if not self._wait_for_core_services():
            return 1

        if self._enable_mock_params and not self._configure_mock_params():
            return 1

        if not self._delete_existing(self._map_name, DEFAULT_SEEDS):
            return 1

        if self._seed_secondary_map and not self._delete_existing(
            self._secondary_map_name, SECONDARY_SEEDS
        ):
            return 1

        if not self._add_waypoints(self._map_name, DEFAULT_SEEDS):
            return 1

        if self._seed_secondary_map and not self._add_waypoints(
            self._secondary_map_name, SECONDARY_SEEDS
        ):
            return 1

        if not self._start_mission([seed.waypoint_id for seed in DEFAULT_SEEDS]):
            return 1

        if self._seed_secondary_map:
            self.get_logger().info(
                f'Seeded secondary mock map: {self._secondary_map_name} '
                f'waypoints={len(SECONDARY_SEEDS)}'
            )

        self.get_logger().info('Mock mission runner finished successfully')
        return 0

    def _wait_for_core_services(self) -> bool:
        required = [
            ('add_waypoint', self._add_client),
            ('delete_waypoint', self._delete_client),
            ('start_mission', self._start_client),
        ]

        if self._enable_mock_params:
            required.append(('set_parameters', self._set_params_client))

        for label, client in required:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Service not ready: {label}')
                return False
        return True

    def _configure_mock_params(self) -> bool:
        req = SetParameters.Request()
        req.parameters = [
            self._bool_param('debug_mock_enabled', True),
            self._double_param('debug_mock_delay_sec', float(self.get_parameter('mock_delay_sec').value)),
            self._string_param('debug_mock_nav_result', str(self.get_parameter('mock_nav_result').value)),
            self._string_param('debug_mock_ptz_result', str(self.get_parameter('mock_ptz_result').value)),
            self._string_param('debug_mock_capture_result', str(self.get_parameter('mock_capture_result').value)),
            self._string_param('debug_mock_charge_result', str(self.get_parameter('mock_charge_result').value)),
            self._string_param('debug_mock_inference_result', str(self.get_parameter('mock_inference_result').value)),
            self._double_param('debug_mock_battery_level', float(self.get_parameter('mock_battery_level').value)),
        ]

        if bool(self.get_parameter('configure_tree_debug_params').value):
            req.parameters.extend([
                self._bool_param('debug_print_tree_on_build', bool(self.get_parameter('print_tree_on_build').value)),
                self._bool_param('debug_print_tree_on_tick', bool(self.get_parameter('print_tree_on_tick').value)),
                self._bool_param('debug_export_tree_dot', bool(self.get_parameter('export_tree_dot').value)),
            ])

        future = self._set_params_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().error('SetParameters timeout')
            return False
        result = future.result()
        if result is None or not all(item.successful for item in result.results):
            self.get_logger().error('Failed to enable mock parameters on mission_bt_node')
            return False
        self.get_logger().info('mission_bt_node mock parameters configured')
        return True

    def _delete_existing(self, map_name: str, seeds: Iterable[WaypointSeed]) -> bool:
        for seed in seeds:
            req = DeleteWaypoint.Request()
            req.map_name = map_name
            req.waypoint_id = seed.waypoint_id
            future = self._delete_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if not future.done():
                self.get_logger().warning(
                    f'Delete timeout for {map_name}/{seed.waypoint_id}, continue'
                )
                continue
            result = future.result()
            if result is not None and result.result == 1:
                self.get_logger().info(
                    f'Deleted old seed waypoint: {map_name}/{seed.waypoint_id}'
                )
        return True

    def _add_waypoints(self, map_name: str, seeds: Iterable[WaypointSeed]) -> bool:
        for seed in seeds:
            req = AddWaypoint.Request()
            req.waypoint = self._seed_to_msg(map_name, seed)
            future = self._add_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if not future.done():
                self.get_logger().error(f'Add waypoint timeout: {map_name}/{seed.waypoint_id}')
                return False
            result = future.result()
            if result is None or result.result != 1:
                message = result.message if result is not None else 'no response'
                self.get_logger().error(
                    f'Add waypoint failed: {map_name}/{seed.waypoint_id} -> {message}'
                )
                return False
            self.get_logger().info(f'Added waypoint: {map_name}/{seed.waypoint_id}')
        return True

    def _start_mission(self, waypoint_ids: list[str]) -> bool:
        req = StartMission.Request()
        req.map_name = self._map_name
        req.waypoint_ids = waypoint_ids
        future = self._start_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().error('StartMission timeout')
            return False
        result = future.result()
        if result is None or result.result != 1:
            message = result.message if result is not None else 'no response'
            self.get_logger().error(f'StartMission failed: {message}')
            return False
        self.get_logger().info(
            f'Mock mission started: map={self._map_name}, waypoint_ids={waypoint_ids}'
        )
        return True

    def _seed_to_msg(self, map_name: str, seed: WaypointSeed) -> WaypointTask:
        msg = WaypointTask()
        msg.waypoint_id = seed.waypoint_id
        msg.map_name = map_name
        msg.pose = Pose2D(x=seed.x, y=seed.y, theta=seed.theta)
        msg.waypoint_type = seed.waypoint_type
        msg.label = seed.label
        msg.task_params = seed.task_params
        return msg

    @staticmethod
    def _bool_param(name: str, value: bool) -> Parameter:
        return Parameter(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value),
        )

    @staticmethod
    def _double_param(name: str, value: float) -> Parameter:
        return Parameter(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value),
        )

    @staticmethod
    def _string_param(name: str, value: str) -> Parameter:
        return Parameter(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value),
        )


def main() -> None:
    rclpy.init()
    node = MockMissionRunnerNode()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if exit_code != 0:
        raise SystemExit(exit_code)