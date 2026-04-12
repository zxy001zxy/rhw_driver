"""charge_action — 充电行为树叶节点.

Recharge: 调用 Recharge.srv 进行自动回充。
"""
from __future__ import annotations

import math

import py_trees
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from rhw_msgs.srv import Recharge as RechargeSrv


class Recharge(py_trees.behaviour.Behaviour):
    """调用 Recharge.srv 自动回充.

    从 Blackboard 读取:
        /current_waypoint  — dict (使用 pose 作为充电桩位姿)
    """

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)

        srv_name = self._node.get_parameter('recharge_service').value
        self._client = self._node.create_client(RechargeSrv, srv_name)
        self._future = None

    def initialise(self) -> None:
        self._future = None

    def update(self) -> py_trees.common.Status:
        if self._future is not None:
            if not self._future.done():
                return py_trees.common.Status.RUNNING
            try:
                result = self._future.result()
                if result.result >= 0:
                    self._node.get_logger().info('Recharge succeeded')
                    return py_trees.common.Status.SUCCESS
                self._node.get_logger().warning(f'Recharge failed: result={result.result}')
                return py_trees.common.Status.FAILURE
            except Exception as exc:
                self._node.get_logger().error(f'Recharge exception: {exc}')
                return py_trees.common.Status.FAILURE

        if not self._client.service_is_ready():
            self._node.get_logger().warning('Recharge service not ready')
            return py_trees.common.Status.RUNNING

        wp = self._bb.get('/current_waypoint')
        pose = wp.get('pose', {}) if wp else {}

        req = RechargeSrv.Request()
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.position.x = float(pose.get('x', 0.0))
        goal.pose.position.y = float(pose.get('y', 0.0))
        theta = float(pose.get('theta', 0.0))
        goal.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.orientation.w = math.cos(theta / 2.0)
        req.recharge_goal = goal

        self._node.get_logger().info('Sending recharge request')
        self._future = self._client.call_async(req)
        return py_trees.common.Status.RUNNING
