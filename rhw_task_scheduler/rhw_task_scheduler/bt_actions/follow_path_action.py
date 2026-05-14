"""FollowPathAction — Nav2 FollowPath 行为树叶节点。"""
from __future__ import annotations

import math
import time
from typing import Any

import py_trees
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node

from rhw_task_scheduler.bt_utils import parse_task_params
from rhw_task_scheduler.service_audit import ServiceAuditPublisher

try:
    from nav2_msgs.action import FollowPath
    _HAS_NAV2_MSGS = True
except ImportError:
    FollowPath = None
    _HAS_NAV2_MSGS = False


class FollowPathAction(py_trees.behaviour.Behaviour):
    """调用 Nav2 /follow_path action 执行巡线路径。"""

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)

        if hasattr(self._node, '_service_audit'):
            self._audit = self._node._service_audit
        else:
            self._audit = ServiceAuditPublisher(self._node)

        self._action_name = str(self._node.get_parameter('follow_path_action').value)
        self._frame_id = str(self._node.get_parameter('navigation_frame_id').value)
        self._timeout_sec = max(
            float(self._node.get_parameter('waypoint_task_timeout_sec').value), 0.1
        )
        self._default_controller_id = str(
            self._node.get_parameter('follow_path_controller_id').value
        )
        self._default_goal_checker_id = str(
            self._node.get_parameter('follow_path_goal_checker_id').value
        )
        self._default_progress_checker_id = str(
            self._node.get_parameter('follow_path_progress_checker_id').value
        )

        if _HAS_NAV2_MSGS:
            self._client = ActionClient(self._node, FollowPath, self._action_name)
        else:
            self._client = None

        self._sent = False
        self._start_time: float | None = None
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._result_logged = False

    def initialise(self) -> None:
        self._sent = False
        self._start_time = None
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._result_logged = False

    def update(self) -> py_trees.common.Status:
        wp = self._bb.get('/current_waypoint')
        if wp is None:
            return py_trees.common.Status.FAILURE

        if not _HAS_NAV2_MSGS or self._client is None:
            self._node.get_logger().error(
                'nav2_msgs is not installed, cannot use FollowPathAction'
            )
            return py_trees.common.Status.FAILURE

        if not self._sent:
            return self._send_goal(wp)

        if self._is_timeout():
            self._cancel_goal()
            self._node.get_logger().error('Nav2 FollowPath timeout')
            return py_trees.common.Status.FAILURE

        if self._goal_future is not None and not self._goal_future.done():
            return py_trees.common.Status.RUNNING

        if self._goal_handle is None and self._goal_future is not None:
            self._goal_handle = self._goal_future.result()
            if not self._goal_handle.accepted:
                self._node.get_logger().warning('Nav2 FollowPath goal rejected')
                self._audit.publish(
                    service=self._action_name,
                    role='client',
                    phase='response',
                    success=False,
                    details={'reason': 'goal_rejected', 'waypoint_id': wp.get('waypoint_id', '?')},
                )
                return py_trees.common.Status.FAILURE
            self._result_future = self._goal_handle.get_result_async()
            self._node.get_logger().info('Nav2 FollowPath goal accepted')
            return py_trees.common.Status.RUNNING

        if self._result_future is None or not self._result_future.done():
            return py_trees.common.Status.RUNNING

        result = self._result_future.result()
        status = int(result.status)
        if not self._result_logged:
            self._audit.publish(
                service=self._action_name,
                role='client',
                phase='response',
                success=(status == GoalStatus.STATUS_SUCCEEDED),
                duration_ms=self._elapsed_ms(),
                details={'status': status, 'waypoint_id': wp.get('waypoint_id', '?')},
            )
            self._result_logged = True

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info(f'FollowPath completed: {wp.get("waypoint_id", "?")}')
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().warning(f'FollowPath failed: status={status}')
        return py_trees.common.Status.FAILURE

    def _send_goal(self, wp: dict[str, Any]) -> py_trees.common.Status:
        if not self._client.server_is_ready():
            self._node.get_logger().warning('Nav2 FollowPath action server not ready')
            return py_trees.common.Status.RUNNING

        params = parse_task_params(wp)
        path = self._build_path(params)
        if len(path.poses) < 2:
            self._node.get_logger().warning('FollowPath requires at least 2 path points')
            return py_trees.common.Status.FAILURE

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = self._param_str(
            params, 'controller_id', self._default_controller_id
        )
        goal_msg.goal_checker_id = self._param_str(
            params, 'goal_checker_id', self._default_goal_checker_id
        )
        goal_msg.progress_checker_id = self._param_str(
            params, 'progress_checker_id', self._default_progress_checker_id
        )

        self._audit.publish(
            service=self._action_name,
            role='client',
            phase='request',
            request={
                'path_points': len(path.poses),
                'frame_id': path.header.frame_id,
                'controller_id': goal_msg.controller_id,
                'goal_checker_id': goal_msg.goal_checker_id,
                'progress_checker_id': goal_msg.progress_checker_id,
            },
            details={'waypoint_id': wp.get('waypoint_id', '?')},
        )
        self._start_time = time.monotonic()
        self._goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback,
        )
        self._sent = True
        self._node.get_logger().info(
            f'Sent FollowPath: points={len(path.poses)} wp={wp.get("waypoint_id", "?")}'
        )
        return py_trees.common.Status.RUNNING

    def _build_path(self, params: dict[str, Any]) -> Path:
        raw_points = params.get('path', params.get('poses', []))
        if not isinstance(raw_points, list):
            raw_points = []

        frame_id = self._param_str(params, 'frame_id', self._frame_id)
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self._node.get_clock().now().to_msg()

        for point in raw_points:
            pose = self._pose_stamped_from_point(point, frame_id)
            if pose is not None:
                path.poses.append(pose)
        return path

    def _pose_stamped_from_point(
        self,
        point: Any,
        frame_id: str,
    ) -> PoseStamped | None:
        if not isinstance(point, dict):
            return None

        pose_data = point.get('pose', point)
        if not isinstance(pose_data, dict):
            return None

        pose = PoseStamped()
        pose.header.frame_id = str(point.get('frame_id') or frame_id)
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = float(pose_data.get('x', 0.0))
        pose.pose.position.y = float(pose_data.get('y', 0.0))
        pose.pose.position.z = float(pose_data.get('z', 0.0))
        theta = float(pose_data.get('theta', 0.0))
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        return pose

    @staticmethod
    def _param_str(params: dict[str, Any], key: str, default: str) -> str:
        value = params.get(key, default)
        if value is None:
            return ''
        return str(value)

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        speed = getattr(feedback, 'speed', None)
        distance = getattr(feedback, 'distance_to_goal', None)
        if speed is not None or distance is not None:
            self._node.get_logger().debug(
                f'FollowPath feedback: speed={speed} distance_to_goal={distance}'
            )

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception as exc:
                self._node.get_logger().warning(f'Failed to cancel FollowPath goal: {exc}')

    def _is_timeout(self) -> bool:
        return (
            self._start_time is not None
            and (time.monotonic() - self._start_time) > self._timeout_sec
        )

    def _elapsed_ms(self) -> float | None:
        if self._start_time is None:
            return None
        return (time.monotonic() - self._start_time) * 1000.0
