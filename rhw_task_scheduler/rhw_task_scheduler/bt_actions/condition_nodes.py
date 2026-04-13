"""condition_nodes — 条件判断行为树叶节点.

CheckBattery:   检查电量是否低于阈值。
IsVisionPoint:  判断当前航点是否为视觉识别点。
IsChargePoint:  判断当前航点是否为充电点。
IsNormalPoint:  判断当前航点是否为普通导航点。
"""
from __future__ import annotations

import py_trees
from rclpy.node import Node

from rhw_msgs.msg import UdpBatteryStatus, WaypointTask
from rhw_task_scheduler.debug_tools import is_debug_mock_enabled


class CheckBattery(py_trees.behaviour.Behaviour):
    """电量检查：电量充足返回 SUCCESS，低于阈值返回 FAILURE.

    写入 Blackboard:
        /battery_low  — bool
    """

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/battery_low', access=py_trees.common.Access.WRITE)

        battery_topic = self._node.get_parameter('battery_topic').value
        self._threshold = float(self._node.get_parameter('low_battery_threshold').value)
        self._battery_level: float = 100.0
        self._sub = self._node.create_subscription(
            UdpBatteryStatus, battery_topic, self._on_battery, 10
        )

    def _on_battery(self, msg: UdpBatteryStatus) -> None:
        # 取左右电池中较低的电量
        self._battery_level = min(msg.battery_level_left, msg.battery_level_right)

    def update(self) -> py_trees.common.Status:
        if is_debug_mock_enabled(self._node):
            self._battery_level = float(self._node.get_parameter('debug_mock_battery_level').value)

        low = self._battery_level < self._threshold
        self._bb.set('/battery_low', low)
        if low:
            self._node.get_logger().warning(
                f'Battery low: {self._battery_level:.1f}% < {self._threshold}%'
            )
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class IsVisionPoint(py_trees.behaviour.Behaviour):
    """当前航点是否为 TYPE_VISION，是则 SUCCESS，否则 FAILURE."""

    def __init__(self, name: str, **kwargs):
        super().__init__(name, **kwargs)
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        wp = self._bb.get('/current_waypoint')
        if wp and int(wp.get('waypoint_type', -1)) == WaypointTask.TYPE_VISION:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsChargePoint(py_trees.behaviour.Behaviour):
    """当前航点是否为 TYPE_CHARGE，是则 SUCCESS，否则 FAILURE."""

    def __init__(self, name: str, **kwargs):
        super().__init__(name, **kwargs)
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        wp = self._bb.get('/current_waypoint')
        if wp and int(wp.get('waypoint_type', -1)) == WaypointTask.TYPE_CHARGE:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class IsNormalPoint(py_trees.behaviour.Behaviour):
    """当前航点是否为 TYPE_NORMAL，是则 SUCCESS，否则 FAILURE."""

    def __init__(self, name: str, **kwargs):
        super().__init__(name, **kwargs)
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        wp = self._bb.get('/current_waypoint')
        if wp and int(wp.get('waypoint_type', -1)) == WaypointTask.TYPE_NORMAL:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
