"""ptz_actions — 云台相关行为树叶节点.

PtzAbsoluteMove: 调用 /ptz/absolute_move 移动到绝对角度。
WaitPtzStable:  订阅 /ptz/status 等待云台空闲。
CaptureImage:   调用 /ptz/capture_image 抓拍。
"""
from __future__ import annotations

import time

import py_trees
from rclpy.node import Node

from rhw_msgs.msg import PtzStatus
from rhw_msgs.srv import CaptureImage as CaptureImageSrv
from rhw_msgs.srv import PtzAbsoluteMove as PtzAbsoluteMoveSrv
from rhw_task_scheduler.bt_utils import parse_task_params
from rhw_task_scheduler.service_audit import ServiceAuditPublisher


class PtzAbsoluteMove(py_trees.behaviour.Behaviour):
    """调用 /ptz/absolute_move.

    从 Blackboard 读取:
        /current_waypoint.task_params  — JSON string
            {"azimuth": float, "elevation": float, "channel": int,
             "zoom": float, "azimuth_speed": int, "elevation_speed": int}
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

        srv_name = self._node.get_parameter('ptz_absolute_move_service').value
        self._client = self._node.create_client(PtzAbsoluteMoveSrv, srv_name)
        self._default_channel = int(self._node.get_parameter('default_ptz_channel').value)
        self._default_move_speed = 50
        self._future = None

    def initialise(self) -> None:
        self._future = None
        self._req_time: float | None = None

    def _build_request_payload(self, params: dict) -> dict | None:
        if 'azimuth' not in params or 'elevation' not in params:
            return None

        speed = int(params.get('speed', self._default_move_speed))
        return {
            'channel': int(params.get('channel', self._default_channel)),
            'azimuth': float(params['azimuth']),
            'elevation': float(params['elevation']),
            'zoom': float(params.get('zoom', 0.0)),
            'azimuth_speed': int(params.get('azimuth_speed', speed)),
            'elevation_speed': int(params.get('elevation_speed', speed)),
        }

    def update(self) -> py_trees.common.Status:
        wp = self._bb.get('/current_waypoint')
        params = parse_task_params(wp)
        request_payload = self._build_request_payload(params)

        if request_payload is None:
            self._node.get_logger().warning(
                'Vision waypoint task_params missing azimuth/elevation for PTZ absolute move'
            )
            return py_trees.common.Status.FAILURE

        if self._future is not None:
            # 等待异步结果
            if not self._future.done():
                return py_trees.common.Status.RUNNING
            try:
                result = self._future.result()
                duration = (time.time() - self._req_time) * 1000 if self._req_time else None
                self._audit.publish(
                    service=self._node.get_parameter('ptz_absolute_move_service').value,
                    role='client',
                    phase='response',
                    response={'result': int(result.result), 'message': str(result.message)},
                    success=(result.result == 1),
                    duration_ms=duration,
                )
                if result.result == 1:
                    self._node.get_logger().info('PTZ absolute move succeeded')
                    return py_trees.common.Status.SUCCESS
                self._node.get_logger().warning(f'PTZ absolute move failed: {result.message}')
                return py_trees.common.Status.FAILURE
            except Exception as exc:
                duration = (time.time() - self._req_time) * 1000 if self._req_time else None
                self._audit.publish(
                    service=self._node.get_parameter('ptz_absolute_move_service').value,
                    role='client',
                    phase='response',
                    success=False,
                    duration_ms=duration,
                    details={'error': str(exc)},
                )
                self._node.get_logger().error(f'PTZ absolute move exception: {exc}')
                return py_trees.common.Status.FAILURE

        if not self._client.service_is_ready():
            self._node.get_logger().warning('PTZ absolute_move service not ready')
            return py_trees.common.Status.RUNNING

        req = PtzAbsoluteMoveSrv.Request()
        req.channel = int(request_payload['channel'])
        req.azimuth = float(request_payload['azimuth'])
        req.elevation = float(request_payload['elevation'])
        req.zoom = float(request_payload['zoom'])
        req.azimuth_speed = int(request_payload['azimuth_speed'])
        req.elevation_speed = int(request_payload['elevation_speed'])

        self._node.get_logger().info(
            'PTZ absolute move: '
            f'ch={req.channel} azimuth={req.azimuth:.2f} elevation={req.elevation:.2f} '
            f'zoom={req.zoom:.2f} '
            f'az_speed={req.azimuth_speed} el_speed={req.elevation_speed}'
        )
        self._req_time = time.time()
        self._audit.publish(
            service=self._node.get_parameter('ptz_absolute_move_service').value,
            role='client',
            phase='request',
            request=request_payload,
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
        if self._active_action in ('idle', 'Idle', ''):
            return py_trees.common.Status.SUCCESS

        if self._start_time and (time.monotonic() - self._start_time) > self._timeout:
            self._node.get_logger().warning('PTZ stable wait timeout, proceeding')
            return py_trees.common.Status.SUCCESS  # 超时也继续

        return py_trees.common.Status.RUNNING


class CaptureImage(py_trees.behaviour.Behaviour):
    """调用 /ptz/capture_image 抓拍.

    写入 Blackboard:
        /last_capture_path       — str (抓拍文件路径)
        /last_capture_url        — str (设备抓拍 URL)
        /last_capture_file_size  — int (图片大小，字节)
    """

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)
        self._bb.register_key(key='/last_capture_path', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/last_capture_url', access=py_trees.common.Access.WRITE)
        self._bb.register_key(key='/last_capture_file_size', access=py_trees.common.Access.WRITE)

        if hasattr(self._node, '_service_audit'):
            self._audit = self._node._service_audit
        else:
            self._audit = ServiceAuditPublisher(self._node)

        srv_name = self._node.get_parameter('ptz_capture_service').value
        self._client = self._node.create_client(CaptureImageSrv, srv_name)
        self._default_channel = int(self._node.get_parameter('default_ptz_channel').value)
        self._future = None

    def initialise(self) -> None:
        self._future = None
        self._req_time: float | None = None

    def update(self) -> py_trees.common.Status:
        wp = self._bb.get('/current_waypoint')

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
                    self._bb.set('/last_capture_path', str(result.file_path))
                    self._bb.set('/last_capture_url', str(result.capture_url))
                    self._bb.set('/last_capture_file_size', int(result.file_size))
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

        params = parse_task_params(wp)

        req = CaptureImageSrv.Request()
        req.channel = int(params.get('channel', self._default_channel))
        req.url_type = 'localURL'

        self._node.get_logger().info(f'Capture image: ch={req.channel}')
        self._req_time = time.time()
        self._audit.publish(
            service=self._node.get_parameter('ptz_capture_service').value,
            role='client',
            phase='request',
            request={'channel': int(req.channel), 'url_type': str(req.url_type)},
        )
        self._future = self._client.call_async(req)
        return py_trees.common.Status.RUNNING
