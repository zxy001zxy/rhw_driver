"""navigate_action — 导航行为树叶节点.

NavigateToGoal: 调用 Goal.srv 发送导航目标，轮询 NavigationStatus 判断结果。
"""
from __future__ import annotations

import math
import time

import py_trees
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from rhw_msgs.msg import NavigationStatus
from rhw_msgs.srv import Goal
from rhw_task_scheduler.service_audit import ServiceAuditPublisher


class NavigateToGoal(py_trees.behaviour.Behaviour):
    """发送导航目标并等待到达.

    从 Blackboard 读取:
        /current_waypoint  — dict (包含 pose: {x, y, theta})
    写入 Blackboard:
        /nav_result        — str ("reached" | "failed" | "cancelled")
    """

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)
        self._bb.register_key(key='/nav_result', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/nav_retry_max', access=py_trees.common.Access.READ)

        # 审计发布 — 复用主节点的或创建新的
        if hasattr(self._node, '_service_audit'):
            self._audit = self._node._service_audit
        else:
            self._audit = ServiceAuditPublisher(self._node)

        goal_srv = self._node.get_parameter('goal_service').value
        self._goal_client = self._node.create_client(Goal, goal_srv)

        nav_topic = self._node.get_parameter('nav_status_topic').value
        self._latest_status: int = NavigationStatus.STATUS_IDLE
        self._nav_sub = self._node.create_subscription(
            NavigationStatus, nav_topic, self._on_nav_status, 10
        )

        self._sent = False
        self._retry_count = 0

    def _on_nav_status(self, msg: NavigationStatus) -> None:
        self._latest_status = msg.status

    def initialise(self) -> None:
        self._sent = False
        self._retry_count = 0
        self._latest_status = NavigationStatus.STATUS_IDLE

    def update(self) -> py_trees.common.Status:
        wp = self._bb.get('/current_waypoint')
        if wp is None:
            self._bb.set('/nav_result', 'failed')
            return py_trees.common.Status.FAILURE

        # 首次 tick: 发送导航目标
        if not self._sent:
            return self._send_goal(wp)

        # 后续 tick: 检查导航状态
        status = self._latest_status

        if status == NavigationStatus.STATUS_REACHED:
            self._bb.set('/nav_result', 'reached')
            self._node.get_logger().info(f'Navigation reached: {wp.get("waypoint_id", "?")}')
            return py_trees.common.Status.SUCCESS

        if status == NavigationStatus.STATUS_FAILED:
            retry_max = int(self._bb.get('/nav_retry_max') or 3)
            self._retry_count += 1
            if self._retry_count <= retry_max:
                self._node.get_logger().warning(
                    f'Navigation failed, retry {self._retry_count}/{retry_max}'
                )
                self._sent = False
                return py_trees.common.Status.RUNNING
            self._bb.set('/nav_result', 'failed')
            self._node.get_logger().error('Navigation failed after all retries')
            return py_trees.common.Status.FAILURE

        if status == NavigationStatus.STATUS_CANCELLED:
            self._bb.set('/nav_result', 'cancelled')
            return py_trees.common.Status.FAILURE

        # NAVIGATING / PAUSED / WAITING → 继续等待
        return py_trees.common.Status.RUNNING

    def _send_goal(self, wp: dict) -> py_trees.common.Status:
        if not self._goal_client.service_is_ready():
            self._node.get_logger().warning('Goal service not ready, waiting...')
            return py_trees.common.Status.RUNNING

        pose = wp.get('pose', {})
        req = Goal.Request()
        req.type = 0  # 自由导航-前进

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(pose.get('x', 0.0))
        goal_pose.pose.position.y = float(pose.get('y', 0.0))
        goal_pose.pose.position.z = 0.0
        theta = float(pose.get('theta', 0.0))
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)
        req.goal = goal_pose

        self._goal_req_time = time.time()
        self._audit.publish(
            service=self._node.get_parameter('goal_service').value,
            role='client',
            phase='request',
            request={
                'type': int(req.type),
                'type_desc': '自由导航-前进',
                'goal': {
                    'x': goal_pose.pose.position.x,
                    'y': goal_pose.pose.position.y,
                    'theta': theta,
                },
            },
            details={'waypoint_id': wp.get('waypoint_id', '?')},
        )

        future = self._goal_client.call_async(req)
        future.add_done_callback(self._on_goal_response)
        self._sent = True
        self._node.get_logger().info(
            f'Sent nav goal: ({pose.get("x", 0):.2f}, {pose.get("y", 0):.2f}, '
            f'θ={pose.get("theta", 0):.2f}) wp={wp.get("waypoint_id", "?")}'
        )
        return py_trees.common.Status.RUNNING

    def _on_goal_response(self, future) -> None:
        duration = (time.time() - self._goal_req_time) * 1000 if hasattr(self, '_goal_req_time') else None
        try:
            result = future.result()
            self._audit.publish(
                service=self._node.get_parameter('goal_service').value,
                role='client',
                phase='response',
                response={'result': int(result.result)},
                success=(result.result in (1, 3)),
                duration_ms=duration,
            )
            self._node.get_logger().debug(f'Goal service returned: {result.result}')
        except Exception as exc:
            self._audit.publish(
                service=self._node.get_parameter('goal_service').value,
                role='client',
                phase='response',
                success=False,
                duration_ms=duration,
                details={'error': str(exc)},
            )
            self._node.get_logger().error(f'Goal service call failed: {exc}')

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass
