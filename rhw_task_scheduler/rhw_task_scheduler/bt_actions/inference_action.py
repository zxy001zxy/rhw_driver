"""inference_action — 视觉推理行为树叶节点（预留接口）.

TriggerInference: 触发 AI 推理（当前为占位实现，返回 SUCCESS）。
后续可封装为 ROS 2 Service Client 调用独立推理服务。
"""
from __future__ import annotations

import time

import py_trees
from rclpy.node import Node

from rhw_task_scheduler.debug_tools import is_debug_mock_enabled, run_mock_action


class TriggerInference(py_trees.behaviour.Behaviour):
    """触发视觉推理.

    从 Blackboard 读取:
        /current_waypoint   — dict (task_params 含 inference_type)
        /last_capture_path  — str  (抓拍图片路径)
    写入 Blackboard:
        /inference_result   — dict (推理结果，当前为空占位)

    TODO: 后续接入实际推理服务:
        - 方案 A: Service Client 调用 /inference/detect
        - 方案 B: 直接 import common.inference.Phase6InferenceRunner
    """

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)
        self._bb.register_key(key='/last_capture_path', access=py_trees.common.Access.READ)
        self._bb.register_key(key='/inference_result', access=py_trees.common.Access.WRITE)
        self._mock_start_time: float | None = None

    def initialise(self) -> None:
        self._mock_start_time = time.monotonic()

    def update(self) -> py_trees.common.Status:
        capture_path = self._bb.get('/last_capture_path')
        wp = self._bb.get('/current_waypoint')

        wp_id = wp.get('waypoint_id', '?') if wp else '?'

        if is_debug_mock_enabled(self._node):
            def _on_success() -> None:
                self._bb.set('/inference_result', {
                    'waypoint_id': wp_id,
                    'capture_path': capture_path or '',
                    'detections': [],
                    'status': 'debug_mock',
                })

            return run_mock_action(
                node=self._node,
                start_time=self._mock_start_time,
                result_parameter='debug_mock_inference_result',
                on_success=_on_success,
            )

        self._node.get_logger().info(
            f'[PLACEHOLDER] TriggerInference for wp={wp_id}, '
            f'image={capture_path or "none"}'
        )

        # 占位：实际推理结果
        self._bb.set('/inference_result', {
            'waypoint_id': wp_id,
            'capture_path': capture_path or '',
            'detections': [],
            'status': 'placeholder',
        })

        return py_trees.common.Status.SUCCESS
