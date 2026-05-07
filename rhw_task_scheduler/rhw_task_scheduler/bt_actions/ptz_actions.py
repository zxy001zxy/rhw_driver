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
from rhw_task_scheduler.debug_tools import (
    is_debug_mock_enabled,
    parse_task_params,
    run_mock_action,
    safe_slug,
)
from rhw_task_scheduler.service_audit import ServiceAuditPublisher


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

        if hasattr(self._node, '_service_audit'):
            self._audit = self._node._service_audit
        else:
            self._audit = ServiceAuditPublisher(self._node)

        srv_name = self._node.get_parameter('ptz_goto_preset_service').value
        self._client = self._node.create_client(PtzGotoPresetSrv, srv_name)
        self._default_channel = int(self._node.get_parameter('default_ptz_channel').value)
        self._future = None
        self._mock_start_time: float | None = None
        self._mock_audit_sent = False

    def initialise(self) -> None:
        self._future = None
        self._mock_start_time = time.monotonic()

    def update(self) -> py_trees.common.Status:
        wp = self._bb.get('/current_waypoint')

        if is_debug_mock_enabled(self._node):
            srv_name = self._node.get_parameter('ptz_goto_preset_service').value
            params = parse_task_params(wp)
            if not self._mock_audit_sent:
                self._mock_audit_sent = True
                self._req_time = time.time()
                self._audit.publish(
                    service=srv_name,
                    role='client',
                    phase='request',
                    request={
                        'channel': int(params.get('channel', self._default_channel)),
                        'preset_id': int(params.get('preset_id', 1)),
                    },
                    details={'waypoint_id': wp.get('waypoint_id', '?') if wp else '?', 'mock': True},
                )

            mock_status = run_mock_action(
                node=self._node,
                start_time=self._mock_start_time,
                result_parameter='debug_mock_ptz_result',
            )
            if mock_status != py_trees.common.Status.RUNNING:
                duration = (time.time() - self._req_time) * 1000 if self._req_time else None
                self._audit.publish(
                    service=srv_name,
                    role='client',
                    phase='response',
                    response={'result': 1 if mock_status == py_trees.common.Status.SUCCESS else 0},
                    success=(mock_status == py_trees.common.Status.SUCCESS),
                    duration_ms=duration,
                    details={'waypoint_id': wp.get('waypoint_id', '?') if wp else '?', 'mock': True},
                )
                self._node.get_logger().info(
                    f'[DEBUG MOCK] PtzGotoPreset -> {mock_status.name} wp={wp.get("waypoint_id", "?") if wp else "?"}'
                )
            return mock_status

        if self._future is not None:
            # 等待异步结果
            if not self._future.done():
                return py_trees.common.Status.RUNNING
            try:
                result = self._future.result()
                duration = (time.time() - self._req_time) * 1000 if self._req_time else None
                self._audit.publish(
                    service=self._node.get_parameter('ptz_goto_preset_service').value,
                    role='client',
                    phase='response',
                    response={'result': int(result.result), 'message': str(result.message)},
                    success=(result.result == 1),
                    duration_ms=duration,
                )
                if result.result == 1:
                    self._node.get_logger().info('PTZ goto preset succeeded')
                    return py_trees.common.Status.SUCCESS
                self._node.get_logger().warning(f'PTZ goto preset failed: {result.message}')
                return py_trees.common.Status.FAILURE
            except Exception as exc:
                duration = (time.time() - self._req_time) * 1000 if self._req_time else None
                self._audit.publish(
                    service=self._node.get_parameter('ptz_goto_preset_service').value,
                    role='client',
                    phase='response',
                    success=False,
                    duration_ms=duration,
                    details={'error': str(exc)},
                )
                self._node.get_logger().error(f'PTZ goto preset exception: {exc}')
                return py_trees.common.Status.FAILURE

        # 解析参数
        params = parse_task_params(wp)

        if not self._client.service_is_ready():
            self._node.get_logger().warning('PTZ goto_preset service not ready')
            return py_trees.common.Status.RUNNING

        req = PtzGotoPresetSrv.Request()
        req.channel = int(params.get('channel', self._default_channel))
        req.preset_id = int(params.get('preset_id', 1))

        self._node.get_logger().info(
            f'PTZ goto preset: ch={req.channel} preset={req.preset_id}'
        )
        self._req_time = time.time()
        self._audit.publish(
            service=self._node.get_parameter('ptz_goto_preset_service').value,
            role='client',
            phase='request',
            request={'channel': int(req.channel), 'preset_id': int(req.preset_id)},
        )
        self._future = self._client.call_async(req)
        return py_trees.common.Status.RUNNING

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
        if is_debug_mock_enabled(self._node):
            return run_mock_action(
                node=self._node,
                start_time=self._start_time,
                result_parameter='debug_mock_ptz_result',
            )

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

        if hasattr(self._node, '_service_audit'):
            self._audit = self._node._service_audit
        else:
            self._audit = ServiceAuditPublisher(self._node)

        srv_name = self._node.get_parameter('ptz_capture_service').value
        self._client = self._node.create_client(CaptureImageSrv, srv_name)
        self._default_channel = int(self._node.get_parameter('default_ptz_channel').value)
        self._future = None
        self._mock_start_time: float | None = None
        self._mock_audit_sent = False

    def initialise(self) -> None:
        self._future = None
        self._mock_start_time = time.monotonic()
        self._mock_audit_sent = False
        self._req_time: float | None = None

    def update(self) -> py_trees.common.Status:
        wp = self._bb.get('/current_waypoint')
        params = parse_task_params(wp)

        if is_debug_mock_enabled(self._node):
            srv_name = self._node.get_parameter('ptz_capture_service').value
            if not self._mock_audit_sent:
                self._mock_audit_sent = True
                self._req_time = time.time()
                request_payload = {
                    'channel': int(params.get('channel', self._default_channel)),
                    'url_type': str(params.get('url_type', 'localURL') or 'localURL'),
                }
                channel_format = str(params.get('channel_format', '') or '')
                save_path = str(params.get('save_path', '') or '')
                image_type = str(params.get('image_type', 'JPEG') or 'JPEG')
                if channel_format:
                    request_payload['channel_format'] = channel_format
                if save_path:
                    request_payload['save_path'] = save_path
                if image_type:
                    request_payload['image_type'] = image_type
                self._audit.publish(
                    service=srv_name,
                    role='client',
                    phase='request',
                    request=request_payload,
                    details={'waypoint_id': wp.get('waypoint_id', '?') if wp else '?', 'mock': True},
                )

            def _on_success() -> None:
                requested_path = str(params.get('save_path', '') or '')
                if requested_path:
                    file_path = requested_path
                else:
                    base_dir = str(self._node.get_parameter('debug_mock_capture_dir').value)
                    waypoint_id = wp.get('waypoint_id', 'mock_capture') if wp else 'mock_capture'
                    file_path = f'{base_dir.rstrip("/")}/{safe_slug(waypoint_id, fallback="capture")}.jpg'
                self._bb.set('/last_capture_path', file_path)

            mock_status = run_mock_action(
                node=self._node,
                start_time=self._mock_start_time,
                result_parameter='debug_mock_capture_result',
                on_success=_on_success,
            )
            if mock_status != py_trees.common.Status.RUNNING:
                duration = (time.time() - self._req_time) * 1000 if self._req_time else None
                mock_path = self._bb.get('/last_capture_path') if mock_status == py_trees.common.Status.SUCCESS else ''
                self._audit.publish(
                    service=srv_name,
                    role='client',
                    phase='response',
                    response={'result': 1 if mock_status == py_trees.common.Status.SUCCESS else 0, 'file_path': mock_path},
                    success=(mock_status == py_trees.common.Status.SUCCESS),
                    duration_ms=duration,
                    details={'waypoint_id': wp.get('waypoint_id', '?') if wp else '?', 'mock': True},
                )
                self._node.get_logger().info(
                    f'[DEBUG MOCK] CaptureImage -> {mock_status.name} wp={wp.get("waypoint_id", "?") if wp else "?"}'
                )
            return mock_status

        if self._future is not None:
            if not self._future.done():
                return py_trees.common.Status.RUNNING
            try:
                result = self._future.result()
                duration = (time.time() - self._req_time) * 1000 if self._req_time else None
                self._audit.publish(
                    service=self._node.get_parameter('ptz_capture_service').value,
                    role='client',
                    phase='response',
                    response={'result': int(result.result), 'file_path': str(result.file_path), 'message': str(result.message)},
                    success=(result.result == 1),
                    duration_ms=duration,
                )
                if result.result == 1:
                    self._bb.set('/last_capture_path', result.file_path)
                    self._node.get_logger().info(f'Capture saved: {result.file_path}')
                    return py_trees.common.Status.SUCCESS
                self._node.get_logger().warning(f'Capture failed: {result.message}')
                return py_trees.common.Status.FAILURE
            except Exception as exc:
                duration = (time.time() - self._req_time) * 1000 if self._req_time else None
                self._audit.publish(
                    service=self._node.get_parameter('ptz_capture_service').value,
                    role='client',
                    phase='response',
                    success=False,
                    duration_ms=duration,
                    details={'error': str(exc)},
                )
                self._node.get_logger().error(f'Capture exception: {exc}')
                return py_trees.common.Status.FAILURE

        if not self._client.service_is_ready():
            self._node.get_logger().warning('capture_image service not ready')
            return py_trees.common.Status.RUNNING

        req = CaptureImageSrv.Request()
        req.channel = int(params.get('channel', self._default_channel))
        req.url_type = str(params.get('url_type', 'localURL') or 'localURL')
        req.channel_format = str(params.get('channel_format', '') or '')
        req.save_path = str(params.get('save_path', '') or '')
        req.image_type = str(params.get('image_type', 'JPEG') or 'JPEG')

        self._node.get_logger().info(f'Capture image: ch={req.channel}')
        self._req_time = time.time()
        request_payload = {
            'channel': int(req.channel),
            'url_type': str(req.url_type),
        }
        if req.channel_format:
            request_payload['channel_format'] = str(req.channel_format)
        if req.save_path:
            request_payload['save_path'] = str(req.save_path)
        if req.image_type:
            request_payload['image_type'] = str(req.image_type)
        self._audit.publish(
            service=self._node.get_parameter('ptz_capture_service').value,
            role='client',
            phase='request',
            request=request_payload,
        )
        self._future = self._client.call_async(req)
        return py_trees.common.Status.RUNNING
