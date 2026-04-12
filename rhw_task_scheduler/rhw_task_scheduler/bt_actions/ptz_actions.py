"""ptz_actions — 云台相关行为树叶节点.

PtzGotoPreset: 调用 /ptz/goto_preset 跳转预置位。
WaitPtzStable: 订阅 /ptz/status 等待云台空闲。
CaptureImage:  调用 /ptz/capture_image 抓拍。
"""
from __future__ import annotations

import time

import py_trees
from rclpy.node import Node

from rhw_msgs.msg import PtzStatus
from rhw_msgs.srv import CaptureImage as CaptureImageSrv
from rhw_msgs.srv import PtzGotoPreset as PtzGotoPresetSrv


class PtzGotoPreset(py_trees.behaviour.Behaviour):
    """调用 /ptz/goto_preset.

    从 Blackboard 读取:
        /current_waypoint.task_params  — JSON string {"preset_id": int, "channel": int}
    """

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)

        srv_name = self._node.get_parameter('ptz_goto_preset_service').value
        self._client = self._node.create_client(PtzGotoPresetSrv, srv_name)
        self._default_channel = int(self._node.get_parameter('default_ptz_channel').value)
        self._future = None

    def initialise(self) -> None:
        self._future = None

    def update(self) -> py_trees.common.Status:
        if self._future is not None:
            # 等待异步结果
            if not self._future.done():
                return py_trees.common.Status.RUNNING
            try:
                result = self._future.result()
                if result.result == 1:
                    self._node.get_logger().info('PTZ goto preset succeeded')
                    return py_trees.common.Status.SUCCESS
                self._node.get_logger().warning(f'PTZ goto preset failed: {result.message}')
                return py_trees.common.Status.FAILURE
            except Exception as exc:
                self._node.get_logger().error(f'PTZ goto preset exception: {exc}')
                return py_trees.common.Status.FAILURE

        # 解析参数
        wp = self._bb.get('/current_waypoint')
        params = self._parse_task_params(wp)

        if not self._client.service_is_ready():
            self._node.get_logger().warning('PTZ goto_preset service not ready')
            return py_trees.common.Status.RUNNING

        req = PtzGotoPresetSrv.Request()
        req.channel = int(params.get('channel', self._default_channel))
        req.preset_id = int(params.get('preset_id', 1))

        self._node.get_logger().info(
            f'PTZ goto preset: ch={req.channel} preset={req.preset_id}'
        )
        self._future = self._client.call_async(req)
        return py_trees.common.Status.RUNNING

    @staticmethod
    def _parse_task_params(wp: dict | None) -> dict:
        if wp is None:
            return {}
        raw = wp.get('task_params', '')
        if not raw:
            return {}
        import json
        try:
            return json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return {}


class WaitPtzStable(py_trees.behaviour.Behaviour):
    """等待云台动作完成（active_action == 'idle'）."""

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node

        ptz_topic = self._node.get_parameter('ptz_status_topic').value
        self._timeout = float(self._node.get_parameter('ptz_stable_timeout_sec').value)
        self._active_action = 'unknown'
        self._sub = self._node.create_subscription(
            PtzStatus, ptz_topic, self._on_status, 10
        )
        self._start_time: float | None = None

    def _on_status(self, msg: PtzStatus) -> None:
        self._active_action = msg.active_action

    def initialise(self) -> None:
        self._start_time = time.monotonic()

    def update(self) -> py_trees.common.Status:
        if self._active_action in ('idle', 'Idle', ''):
            return py_trees.common.Status.SUCCESS

        if self._start_time and (time.monotonic() - self._start_time) > self._timeout:
            self._node.get_logger().warning('PTZ stable wait timeout, proceeding')
            return py_trees.common.Status.SUCCESS  # 超时也继续

        return py_trees.common.Status.RUNNING


class CaptureImage(py_trees.behaviour.Behaviour):
    """调用 /ptz/capture_image 抓拍.

    写入 Blackboard:
        /last_capture_path  — str (抓拍文件路径)
    """

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)
        self._bb.register_key(key='/last_capture_path', access=py_trees.common.Access.WRITE)

        srv_name = self._node.get_parameter('ptz_capture_service').value
        self._client = self._node.create_client(CaptureImageSrv, srv_name)
        self._default_channel = int(self._node.get_parameter('default_ptz_channel').value)
        self._future = None

    def initialise(self) -> None:
        self._future = None

    def update(self) -> py_trees.common.Status:
        if self._future is not None:
            if not self._future.done():
                return py_trees.common.Status.RUNNING
            try:
                result = self._future.result()
                if result.result == 1:
                    self._bb.set('/last_capture_path', result.file_path)
                    self._node.get_logger().info(f'Capture saved: {result.file_path}')
                    return py_trees.common.Status.SUCCESS
                self._node.get_logger().warning(f'Capture failed: {result.message}')
                return py_trees.common.Status.FAILURE
            except Exception as exc:
                self._node.get_logger().error(f'Capture exception: {exc}')
                return py_trees.common.Status.FAILURE

        if not self._client.service_is_ready():
            self._node.get_logger().warning('capture_image service not ready')
            return py_trees.common.Status.RUNNING

        wp = self._bb.get('/current_waypoint')
        params = PtzGotoPreset._parse_task_params(wp)

        req = CaptureImageSrv.Request()
        req.channel = int(params.get('channel', self._default_channel))
        req.url_type = 'localURL'

        self._node.get_logger().info(f'Capture image: ch={req.channel}')
        self._future = self._client.call_async(req)
        return py_trees.common.Status.RUNNING
