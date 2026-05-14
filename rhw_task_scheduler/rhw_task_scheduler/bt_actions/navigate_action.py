"""navigate_action — Nav2 NavigateToPose 行为树叶节点。"""
from __future__ import annotations

import math
import time
from typing import Any

import py_trees
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

from rhw_task_scheduler.service_audit import ServiceAuditPublisher

try:
    from nav2_msgs.action import NavigateToPose
    _HAS_NAV2_MSGS = True
except ImportError:
    NavigateToPose = None
    _HAS_NAV2_MSGS = False


class NavigateToGoal(py_trees.behaviour.Behaviour):
    """发送 Nav2 NavigateToPose 目标并等待 action 结果。"""

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)
        self._bb.register_key(key='/nav_result', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/nav_feedback', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/nav_retry_max', access=py_trees.common.Access.READ)

        if hasattr(self._node, '_service_audit'):
            self._audit = self._node._service_audit
        else:
            self._audit = ServiceAuditPublisher(self._node)

        self._timeout_sec = max(
            float(self._node.get_parameter('waypoint_task_timeout_sec').value), 0.1
        )
        self._frame_id = str(self._node.get_parameter('navigation_frame_id').value)
        self._action_name = str(self._node.get_parameter('navigate_to_pose_action').value)
        self._behavior_tree = str(self._node.get_parameter('nav2_behavior_tree').value)

        if _HAS_NAV2_MSGS:
            self._client = ActionClient(self._node, NavigateToPose, self._action_name)
        else:
            self._client = None

        self._sent = False
        self._retry_count = 0
        self._goal_start_time: float | None = None
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._result_logged = False
        self._latest_feedback: dict[str, Any] = {}
        self._last_feedback_log_time = 0.0

    def initialise(self) -> None:
        self._sent = False
        self._retry_count = 0
        self._goal_start_time = None
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._result_logged = False
        self._latest_feedback = {}
        self._last_feedback_log_time = 0.0
        self._bb.set('/nav_feedback', {})

    def update(self) -> py_trees.common.Status:
        wp = self._bb.get('/current_waypoint')
        if wp is None:
            self._bb.set('/nav_result', 'failed')
            return py_trees.common.Status.FAILURE

        if not _HAS_NAV2_MSGS or self._client is None:
            self._node.get_logger().error(
                'nav2_msgs is not installed, cannot call /navigate_to_pose'
            )
            self._bb.set('/nav_result', 'failed')
            return py_trees.common.Status.FAILURE

        if not self._sent:
            return self._send_goal(wp)

        if self._is_timeout():
            self._cancel_goal()
            self._bb.set('/nav_result', 'failed')
            self._audit.publish(
                service=self._action_name,
                role='client',
                phase='response',
                success=False,
                duration_ms=self._elapsed_ms(),
                details={
                    'reason': 'timeout',
                    'waypoint_id': wp.get('waypoint_id', '?'),
                    'feedback': self._latest_feedback,
                },
            )
            self._node.get_logger().error('Nav2 NavigateToPose timeout')
            return py_trees.common.Status.FAILURE

        if self._goal_future is not None and not self._goal_future.done():
            return py_trees.common.Status.RUNNING

        if self._goal_handle is None and self._goal_future is not None:
            return self._handle_goal_response(wp)

        if self._result_future is None or not self._result_future.done():
            return py_trees.common.Status.RUNNING

        return self._handle_result(wp)

    def _send_goal(self, wp: dict[str, Any]) -> py_trees.common.Status:
        if not self._client.server_is_ready():
            self._node.get_logger().warning('Nav2 NavigateToPose action server not ready')
            return py_trees.common.Status.RUNNING

        pose = wp.get('pose', {})
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._pose_stamped_from_pose_dict(pose)
        if self._behavior_tree:
            goal_msg.behavior_tree = self._behavior_tree

        theta = float(pose.get('theta', 0.0))
        self._audit.publish(
            service=self._action_name,
            role='client',
            phase='request',
            request={
                'pose': {
                    'frame_id': goal_msg.pose.header.frame_id,
                    'x': goal_msg.pose.pose.position.x,
                    'y': goal_msg.pose.pose.position.y,
                    'theta': theta,
                },
                'behavior_tree': self._behavior_tree,
            },
            details={'waypoint_id': wp.get('waypoint_id', '?')},
        )
        self._goal_start_time = time.monotonic()
        self._goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback,
        )
        self._sent = True
        self._node.get_logger().info(
            f'Sent Nav2 NavigateToPose: ({pose.get("x", 0):.2f}, '
            f'{pose.get("y", 0):.2f}, θ={theta:.2f}) '
            f'wp={wp.get("waypoint_id", "?")}'
        )
        return py_trees.common.Status.RUNNING

    def _handle_goal_response(self, wp: dict[str, Any]) -> py_trees.common.Status:
        try:
            self._goal_handle = self._goal_future.result()
        except Exception as exc:
            self._bb.set('/nav_result', 'failed')
            self._audit.publish(
                service=self._action_name,
                role='client',
                phase='response',
                success=False,
                duration_ms=self._elapsed_ms(),
                details={
                    'reason': 'goal_response_error',
                    'error': str(exc),
                    'waypoint_id': wp.get('waypoint_id', '?'),
                },
            )
            self._node.get_logger().error(f'Nav2 NavigateToPose goal error: {exc}')
            return py_trees.common.Status.FAILURE

        if not self._goal_handle.accepted:
            self._bb.set('/nav_result', 'failed')
            self._audit.publish(
                service=self._action_name,
                role='client',
                phase='response',
                success=False,
                duration_ms=self._elapsed_ms(),
                details={
                    'status': 'rejected',
                    'waypoint_id': wp.get('waypoint_id', '?'),
                },
            )
            self._node.get_logger().warning('Nav2 NavigateToPose goal rejected')
            return py_trees.common.Status.FAILURE

        self._result_future = self._goal_handle.get_result_async()
        self._audit.publish(
            service=self._action_name,
            role='client',
            phase='response',
            success=True,
            duration_ms=self._elapsed_ms(),
            details={
                'status': 'accepted',
                'waypoint_id': wp.get('waypoint_id', '?'),
            },
        )
        self._node.get_logger().info('Nav2 NavigateToPose goal accepted')
        return py_trees.common.Status.RUNNING

    def _handle_result(self, wp: dict[str, Any]) -> py_trees.common.Status:
        try:
            action_result = self._result_future.result()
        except Exception as exc:
            self._bb.set('/nav_result', 'failed')
            self._node.get_logger().error(f'Nav2 NavigateToPose result error: {exc}')
            return py_trees.common.Status.FAILURE

        status = int(action_result.status)
        status_text = self._status_text(status)
        result_details = self._action_result_details(action_result)

        if not self._result_logged:
            self._audit.publish(
                service=self._action_name,
                role='client',
                phase='result',
                success=(status == GoalStatus.STATUS_SUCCEEDED),
                duration_ms=self._elapsed_ms(),
                details={
                    'status': status_text,
                    'status_code': status,
                    'waypoint_id': wp.get('waypoint_id', '?'),
                    'feedback': self._latest_feedback,
                    'result': result_details,
                },
            )
            self._result_logged = True

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._bb.set('/nav_result', 'reached')
            self._node.get_logger().info(f'Nav2 navigation succeeded: {wp.get("waypoint_id", "?")}')
            return py_trees.common.Status.SUCCESS

        if status == GoalStatus.STATUS_CANCELED:
            self._bb.set('/nav_result', 'cancelled')
            self._node.get_logger().warning('Nav2 navigation cancelled')
            return py_trees.common.Status.FAILURE

        retry_max = int(self._bb.get('/nav_retry_max') or 3)
        self._retry_count += 1
        if self._retry_count <= retry_max:
            self._node.get_logger().warning(
                f'Nav2 navigation {status_text}, retry {self._retry_count}/{retry_max}'
            )
            self._reset_goal_state_for_retry()
            return py_trees.common.Status.RUNNING

        self._bb.set('/nav_result', 'failed')
        self._node.get_logger().error(f'Nav2 navigation failed: status={status_text}')
        return py_trees.common.Status.FAILURE

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        current_pose = getattr(feedback, 'current_pose', None)
        distance_remaining = getattr(feedback, 'distance_remaining', None)
        estimated_time = getattr(feedback, 'estimated_time_remaining', None)
        navigation_time = getattr(feedback, 'navigation_time', None)
        number_of_recoveries = getattr(feedback, 'number_of_recoveries', None)

        feedback_data: dict[str, Any] = {}
        if current_pose is not None:
            feedback_data['current_pose'] = self._pose_to_dict(current_pose)
        if distance_remaining is not None:
            feedback_data['distance_remaining'] = float(distance_remaining)
        if estimated_time is not None:
            feedback_data['estimated_time_remaining_sec'] = self._duration_to_sec(
                estimated_time
            )
        if navigation_time is not None:
            feedback_data['navigation_time_sec'] = self._duration_to_sec(navigation_time)
        if number_of_recoveries is not None:
            feedback_data['number_of_recoveries'] = int(number_of_recoveries)

        self._latest_feedback = feedback_data
        self._bb.set('/nav_feedback', feedback_data)
        self._log_feedback_throttled(feedback_data)

    def _log_feedback_throttled(self, feedback: dict[str, Any]) -> None:
        now = time.monotonic()
        if now - self._last_feedback_log_time < 1.0:
            return
        self._last_feedback_log_time = now

        pose = feedback.get('current_pose', {})
        distance = feedback.get('distance_remaining')
        eta = feedback.get('estimated_time_remaining_sec')
        self._node.get_logger().debug(
            'Nav2 feedback: '
            f'pose=({pose.get("x", 0.0):.2f}, {pose.get("y", 0.0):.2f}, '
            f'θ={pose.get("theta", 0.0):.2f}) '
            f'distance_remaining={distance} eta_sec={eta}'
        )

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception as exc:
                self._node.get_logger().warning(f'Failed to cancel Nav2 goal: {exc}')

    def _reset_goal_state_for_retry(self) -> None:
        self._sent = False
        self._goal_start_time = None
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._result_logged = False
        self._latest_feedback = {}
        self._bb.set('/nav_feedback', {})

    def _pose_stamped_from_pose_dict(self, pose: dict[str, Any]) -> PoseStamped:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self._frame_id
        goal_pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(pose.get('x', 0.0))
        goal_pose.pose.position.y = float(pose.get('y', 0.0))
        goal_pose.pose.position.z = float(pose.get('z', 0.0))
        theta = float(pose.get('theta', 0.0))
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)
        return goal_pose

    @staticmethod
    def _pose_to_dict(pose_stamped: PoseStamped) -> dict[str, Any]:
        pose = pose_stamped.pose
        return {
            'frame_id': pose_stamped.header.frame_id,
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
            'theta': NavigateToGoal._yaw_from_quaternion(pose.orientation),
        }

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _duration_to_sec(duration: Any) -> float:
        sec = getattr(duration, 'sec', 0)
        nanosec = getattr(duration, 'nanosec', 0)
        return float(sec) + float(nanosec) / 1_000_000_000.0

    @staticmethod
    def _status_text(status: int) -> str:
        mapping = {
            GoalStatus.STATUS_UNKNOWN: 'unknown',
            GoalStatus.STATUS_ACCEPTED: 'accepted',
            GoalStatus.STATUS_EXECUTING: 'executing',
            GoalStatus.STATUS_CANCELING: 'canceling',
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }
        return mapping.get(status, f'unknown_{status}')

    @staticmethod
    def _action_result_details(action_result: Any) -> dict[str, Any]:
        result = getattr(action_result, 'result', None)
        if result is None:
            return {}

        details: dict[str, Any] = {}
        for field in ('error_code', 'error_msg'):
            if hasattr(result, field):
                details[field] = getattr(result, field)
        return details

    def _is_timeout(self) -> bool:
        return (
            self._goal_start_time is not None
            and (time.monotonic() - self._goal_start_time) > self._timeout_sec
        )

    def _elapsed_ms(self) -> float | None:
        if self._goal_start_time is None:
            return None
        return (time.monotonic() - self._goal_start_time) * 1000.0

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status == py_trees.common.Status.INVALID:
            self._cancel_goal()
