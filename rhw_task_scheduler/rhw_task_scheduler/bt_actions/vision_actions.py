"""vision_actions — 视觉点抓拍后的行为树叶节点。"""
from __future__ import annotations

import time

import py_trees
from rclpy.node import Node

from rhw_msgs.srv import InspectionAlbumUpload, ModelTaskRun
from rhw_task_scheduler.bt_utils import parse_task_params
from rhw_task_scheduler.service_audit import ServiceAuditPublisher


INSPECTION_ALBUM_UPLOAD_SERVICE = '/inspection/album_report/upload'
ALBUM_UPLOAD_TIMEOUT_SEC = 30.0
MODEL_TASK_RUN_SERVICE = '/rhw/model/task/run'
MODEL_TASK_TIMEOUT_SEC = 60.0
UINT32_MAX = 4294967295


def _get_or_declare_parameter(node: Node, name: str, default):
    try:
        if node.has_parameter(name):
            return node.get_parameter(name).value
    except Exception:
        pass

    try:
        return node.declare_parameter(name, default).value
    except Exception:
        return node.get_parameter(name).value


def _current_waypoint_or_empty(blackboard) -> dict:
    waypoint = blackboard.get('/current_waypoint') or {}
    return waypoint if isinstance(waypoint, dict) else {}


def _duration_ms(req_time: float | None) -> float | None:
    return (time.time() - req_time) * 1000 if req_time else None


def _abandon_future(client, future) -> None:
    if future is None:
        return

    remove_pending_request = getattr(client, 'remove_pending_request', None)
    if callable(remove_pending_request):
        try:
            remove_pending_request(future)
        except Exception:
            pass

    cancel = getattr(future, 'cancel', None)
    if callable(cancel):
        try:
            cancel()
        except Exception:
            pass


class UploadInspectionAlbum(py_trees.behaviour.Behaviour):
    """调用 /inspection/album_report/upload，同步判定图片上报结果。"""

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)
        self._bb.register_key(key='/last_capture_path', access=py_trees.common.Access.READ)
        self._bb.register_key(key='/last_capture_url', access=py_trees.common.Access.READ)
        self._bb.register_key(key='/last_capture_file_size', access=py_trees.common.Access.READ)

        if hasattr(self._node, '_service_audit'):
            self._audit = self._node._service_audit
        else:
            self._audit = ServiceAuditPublisher(self._node)

        self._service_name = str(
            _get_or_declare_parameter(
                self._node,
                'inspection_album_upload_service',
                INSPECTION_ALBUM_UPLOAD_SERVICE,
            )
        )
        self._timeout_sec = max(
            float(
                _get_or_declare_parameter(
                    self._node,
                    'album_upload_timeout_sec',
                    ALBUM_UPLOAD_TIMEOUT_SEC,
                )
            ),
            0.1,
        )
        self._client = self._node.create_client(
            InspectionAlbumUpload,
            self._service_name,
            callback_group=getattr(self._node, '_callback_group', None),
        )
        self._future = None
        self._req_time: float | None = None
        self._deadline: float | None = None
        self._service_not_ready_logged = False

    def initialise(self) -> None:
        self._future = None
        self._req_time = None
        self._deadline = time.monotonic() + self._timeout_sec
        self._service_not_ready_logged = False

    def _build_request(self) -> InspectionAlbumUpload.Request | None:
        wp = _current_waypoint_or_empty(self._bb)
        image_path = str(self._bb.get('/last_capture_path') or '')
        if not image_path:
            self._node.get_logger().warning('Album upload skipped: last_capture_path is empty')
            return None

        try:
            file_size = int(self._bb.get('/last_capture_file_size') or 0)
        except (TypeError, ValueError):
            self._node.get_logger().warning(
                'Album upload skipped: last_capture_file_size is not an integer'
            )
            return None

        req = InspectionAlbumUpload.Request()
        req.task_id = str(getattr(self._node, '_current_task_id', '') or '')
        req.point_id = str(wp.get('waypoint_id', ''))
        req.point_name = str(wp.get('label') or req.point_id)
        req.image_path = image_path
        req.capture_url = str(self._bb.get('/last_capture_url') or '')
        req.file_size = max(0, min(file_size, UINT32_MAX))
        return req

    def _timeout_failure(self, reason: str) -> py_trees.common.Status:
        self._audit.publish(
            service=self._service_name,
            role='client',
            phase='response',
            success=False,
            duration_ms=_duration_ms(self._req_time),
            details={'error': reason},
        )
        _abandon_future(self._client, self._future)
        self._future = None
        self._node.get_logger().warning(reason)
        return py_trees.common.Status.FAILURE

    def update(self) -> py_trees.common.Status:
        if self._future is not None:
            if not self._future.done():
                if self._deadline is not None and time.monotonic() > self._deadline:
                    return self._timeout_failure('Album upload service timeout')
                return py_trees.common.Status.RUNNING

            duration = _duration_ms(self._req_time)
            try:
                result = self._future.result()
            except Exception as exc:
                self._audit.publish(
                    service=self._service_name,
                    role='client',
                    phase='response',
                    success=False,
                    duration_ms=duration,
                    details={'error': str(exc)},
                )
                self._node.get_logger().error(f'Album upload exception: {exc}')
                return py_trees.common.Status.FAILURE

            self._audit.publish(
                service=self._service_name,
                role='client',
                phase='response',
                response=result,
                success=bool(result.ok),
                duration_ms=duration,
            )
            if bool(result.ok):
                self._node.get_logger().info(
                    f'Album upload succeeded: trace_id={result.trace_id}'
                )
                return py_trees.common.Status.SUCCESS
            self._node.get_logger().warning(
                f'Album upload failed: code={result.code} message={result.message}'
            )
            return py_trees.common.Status.FAILURE

        if not self._client.service_is_ready():
            if self._deadline is not None and time.monotonic() > self._deadline:
                return self._timeout_failure('Album upload service not ready before timeout')
            if not self._service_not_ready_logged:
                self._node.get_logger().warning('Album upload service not ready')
                self._service_not_ready_logged = True
            return py_trees.common.Status.RUNNING

        req = self._build_request()
        if req is None:
            return py_trees.common.Status.FAILURE

        self._req_time = time.time()
        self._audit.publish(
            service=self._service_name,
            role='client',
            phase='request',
            request=req,
        )
        self._future = self._client.call_async(req)
        return py_trees.common.Status.RUNNING


class RunModelTask(py_trees.behaviour.Behaviour):
    """调用 /rhw/model/task/run，根据 task_params.inference_type 执行模型任务。"""

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)
        self._bb.register_key(
            key='/last_model_result_json_path',
            access=py_trees.common.Access.WRITE,
        )

        if hasattr(self._node, '_service_audit'):
            self._audit = self._node._service_audit
        else:
            self._audit = ServiceAuditPublisher(self._node)

        self._service_name = str(
            _get_or_declare_parameter(
                self._node,
                'model_task_run_service',
                MODEL_TASK_RUN_SERVICE,
            )
        )
        self._timeout_sec = max(
            float(
                _get_or_declare_parameter(
                    self._node,
                    'model_task_timeout_sec',
                    MODEL_TASK_TIMEOUT_SEC,
                )
            ),
            0.1,
        )
        self._client = self._node.create_client(
            ModelTaskRun,
            self._service_name,
            callback_group=getattr(self._node, '_callback_group', None),
        )
        self._future = None
        self._req_time: float | None = None
        self._deadline: float | None = None
        self._service_not_ready_logged = False

    def initialise(self) -> None:
        self._future = None
        self._req_time = None
        self._deadline = time.monotonic() + self._timeout_sec
        self._service_not_ready_logged = False

    def _build_request(self) -> ModelTaskRun.Request | None:
        wp = _current_waypoint_or_empty(self._bb)
        params = parse_task_params(wp)
        task_name = str(params.get('inference_type', '')).strip()
        if not task_name:
            self._node.get_logger().warning(
                'Model task skipped: task_params.inference_type is empty'
            )
            return None

        task_id = str(getattr(self._node, '_current_task_id', '') or 'mission')
        waypoint_id = str(wp.get('waypoint_id') or 'waypoint')

        req = ModelTaskRun.Request()
        req.request_id = f'{task_id}-{waypoint_id}-{time.time_ns()}'
        req.task_name = task_name
        req.conf = 0.25
        req.iou = 0.45
        req.max_det = 100
        req.wait_for_frame_timeout_sec = 3.0
        req.max_frame_age_sec = 2.0
        req.params_json = ''
        return req

    def _timeout_failure(self, reason: str) -> py_trees.common.Status:
        self._audit.publish(
            service=self._service_name,
            role='client',
            phase='response',
            success=False,
            duration_ms=_duration_ms(self._req_time),
            details={'error': reason},
        )
        _abandon_future(self._client, self._future)
        self._future = None
        self._node.get_logger().warning(reason)
        return py_trees.common.Status.FAILURE

    def update(self) -> py_trees.common.Status:
        if self._future is not None:
            if not self._future.done():
                if self._deadline is not None and time.monotonic() > self._deadline:
                    return self._timeout_failure('Model task service timeout')
                return py_trees.common.Status.RUNNING

            duration = _duration_ms(self._req_time)
            try:
                result = self._future.result()
            except Exception as exc:
                self._audit.publish(
                    service=self._service_name,
                    role='client',
                    phase='response',
                    success=False,
                    duration_ms=duration,
                    details={'error': str(exc)},
                )
                self._node.get_logger().error(f'Model task exception: {exc}')
                return py_trees.common.Status.FAILURE

            self._audit.publish(
                service=self._service_name,
                role='client',
                phase='response',
                response=result,
                success=bool(result.ok),
                duration_ms=duration,
            )
            if bool(result.ok):
                self._bb.set('/last_model_result_json_path', str(result.result_json_path))
                self._node.get_logger().info(
                    'Model task succeeded: '
                    f'task_name={result.task_name} items={result.item_count} '
                    f'result={result.result_json_path}'
                )
                return py_trees.common.Status.SUCCESS
            self._node.get_logger().warning(
                f'Model task failed: code={result.code} message={result.message}'
            )
            return py_trees.common.Status.FAILURE

        if not self._client.service_is_ready():
            if self._deadline is not None and time.monotonic() > self._deadline:
                return self._timeout_failure('Model task service not ready before timeout')
            if not self._service_not_ready_logged:
                self._node.get_logger().warning('Model task service not ready')
                self._service_not_ready_logged = True
            return py_trees.common.Status.RUNNING

        req = self._build_request()
        if req is None:
            return py_trees.common.Status.FAILURE

        self._req_time = time.time()
        self._audit.publish(
            service=self._service_name,
            role='client',
            phase='request',
            request=req,
        )
        self._future = self._client.call_async(req)
        return py_trees.common.Status.RUNNING
