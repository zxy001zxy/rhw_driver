from __future__ import annotations

import json
from pathlib import Path
import threading
import time
from typing import Any, Optional

from rhw_model_scheduler.api_contract import CODE_DOWNSTREAM_ERROR, ensure_request_id, failure
from rhw_model_scheduler.rtsp_frame_source import RtspFrameSource, mask_stream_url, validate_rtsp_url
from rhw_model_scheduler.scheduler_core import ModelTaskSchedulerCore
from rhw_model_scheduler.workspace import find_workspace_root


def _load_ros2_runtime():
    try:
        import rclpy
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rhw_msgs.srv import ModelTaskRun
    except ImportError as exc:  # pragma: no cover - exercised only in ROS2 runtime.
        raise RuntimeError("ROS2 runtime is not available. Source ROS2 and build rhw_msgs first.") from exc
    return rclpy, Node, ReentrantCallbackGroup, MultiThreadedExecutor, ModelTaskRun


def _clean_text(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _detail_json(payload: dict[str, Any]) -> str:
    data = payload.get("data") or {}
    return json.dumps(data.get("detail") or data, ensure_ascii=False)


def fill_model_task_run_response(response: Any, payload: dict[str, Any]) -> Any:
    data = payload.get("data") or {}
    response.ok = bool(payload.get("ok"))
    response.code = str(payload.get("code") or "")
    response.message = str(payload.get("message") or "")
    response.request_id = str(payload.get("request_id") or "")
    response.task_name = str(data.get("task_name") or "")
    response.task_type = str(data.get("task_type") or "")
    response.model_path = str(data.get("model_path") or "")
    response.backend = str(data.get("backend") or "")
    response.frame_path = str(data.get("frame_path") or "")
    response.result_json_path = str(data.get("result_json_path") or "")
    response.item_count = int(data.get("item_count") or 0)
    response.error_count = int(data.get("error_count") or 0)
    response.latency_ms = float(data.get("latency_ms") or 0.0)
    response.error_category = str(data.get("error_category") or "")
    response.detail_json = _detail_json(payload)
    return response


def _write_frame_snapshot(workspace_root: Path, output_root: Path, request_id: str, frame_bgr: Any) -> Path:
    import cv2

    safe_request_id = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in request_id)
    raw_root = Path(output_root)
    frame_dir = raw_root if raw_root.is_absolute() else workspace_root / raw_root
    frame_path = frame_dir / "frames" / f"{safe_request_id}.jpg"
    frame_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(frame_path), frame_bgr):
        raise RuntimeError(f"failed to write frame snapshot: {frame_path}")
    return frame_path


class LatestFrameCache:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._seq = 0
        self._frame_bgr: Any = None
        self._received_at = 0.0
        self._last_error: str | None = None

    def current_seq(self) -> int:
        with self._condition:
            return self._seq

    def update(self, frame_bgr: Any) -> None:
        with self._condition:
            self._seq += 1
            self._frame_bgr = frame_bgr
            self._received_at = time.time()
            self._last_error = None
            self._condition.notify_all()

    def record_error(self, message: str) -> None:
        with self._condition:
            self._last_error = message

    def wait_for_fresh(self, *, after_seq: int, timeout_sec: float, max_age_sec: float) -> tuple[Any | None, str | None]:
        deadline = time.time() + max(0.0, timeout_sec)
        with self._condition:
            while self._seq <= after_seq:
                remaining = deadline - time.time()
                if remaining <= 0:
                    message = "timed out waiting for a fresh camera frame"
                    if self._last_error:
                        message = f"{message}; last image error: {self._last_error}"
                    return None, message
                self._condition.wait(timeout=remaining)
            age_sec = time.time() - self._received_at
            if max_age_sec > 0 and age_sec > max_age_sec:
                return None, f"fresh frame exceeded max age: {age_sec:.3f}s"
            return self._frame_bgr.copy(), None


def _frame_failure(request_id: str, message: str) -> dict[str, Any]:
    return failure(
        message=message,
        code=CODE_DOWNSTREAM_ERROR,
        request_id=request_id,
        data={"error_category": "frame_unavailable"},
    )


def main(args: Optional[list[str]] = None) -> None:
    rclpy, Node, ReentrantCallbackGroup, MultiThreadedExecutor, ModelTaskRun = _load_ros2_runtime()

    class RhwModelSchedulerNode(Node):
        def __init__(self) -> None:
            super().__init__("rhw_model_scheduler_node")
            self.declare_parameter("workspace_root", "")
            self.declare_parameter("service_prefix", "/rhw/model")
            self.declare_parameter("camera_stream_url", "")
            self.declare_parameter("camera_reconnect_interval_sec", 2.0)
            self.declare_parameter("camera_open_timeout_sec", 5.0)
            self.declare_parameter("camera_read_timeout_sec", 5.0)
            self.declare_parameter("manifest_path", "models/current/manifest.json")
            self.declare_parameter("output_root", "runtime/model_scheduler")
            self.declare_parameter("preload_models", True)
            self.declare_parameter("min_system_mem_available_mb", 2048.0)
            self.declare_parameter("min_gpu_mem_free_mb", 2048.0)

            configured_root = _clean_text(str(self.get_parameter("workspace_root").value))
            self.workspace_root = Path(configured_root).resolve() if configured_root else find_workspace_root()
            self.output_root = Path(str(self.get_parameter("output_root").value))
            self.callback_group = ReentrantCallbackGroup()
            self.frame_cache = LatestFrameCache()
            self.core = ModelTaskSchedulerCore(
                workspace_root=self.workspace_root,
                manifest_path=str(self.get_parameter("manifest_path").value),
                output_root=self.output_root,
            )

            camera_stream_url = validate_rtsp_url(str(self.get_parameter("camera_stream_url").value))
            service_prefix = str(self.get_parameter("service_prefix").value).rstrip("/")
            self.frame_source = RtspFrameSource(
                stream_url=camera_stream_url,
                on_frame=self.frame_cache.update,
                on_error=self._handle_camera_error,
                reconnect_interval_sec=float(self.get_parameter("camera_reconnect_interval_sec").value),
                open_timeout_sec=float(self.get_parameter("camera_open_timeout_sec").value),
                read_timeout_sec=float(self.get_parameter("camera_read_timeout_sec").value),
            )
            self.create_service(
                ModelTaskRun,
                f"{service_prefix}/task/run",
                self._handle_task_run,
                callback_group=self.callback_group,
            )

            if bool(self.get_parameter("preload_models").value):
                status = self.core.warmup_all(
                    min_system_mem_available_mb=float(self.get_parameter("min_system_mem_available_mb").value),
                    min_gpu_mem_free_mb=float(self.get_parameter("min_gpu_mem_free_mb").value),
                )
                self.get_logger().info(f"model warmup status: {json.dumps(status, ensure_ascii=False)}")
            self.frame_source.start()
            self.get_logger().info(
                f"rhw_model_scheduler ready: service={service_prefix}/task/run "
                f"camera_stream_url={mask_stream_url(camera_stream_url)} workspace_root={self.workspace_root}"
            )

        def stop(self) -> None:
            self.frame_source.stop()

        def _handle_camera_error(self, message: str) -> None:
            self.frame_cache.record_error(message)
            self.get_logger().warning(message)

        def _handle_task_run(self, request: Any, response: Any) -> Any:
            rid = _clean_text(request.request_id) or ensure_request_id()
            start_seq = self.frame_cache.current_seq()
            timeout_sec = float(request.wait_for_frame_timeout_sec) if float(request.wait_for_frame_timeout_sec) > 0 else 1.0
            max_age_sec = float(request.max_frame_age_sec) if float(request.max_frame_age_sec) > 0 else 2.0
            frame_bgr, frame_error = self.frame_cache.wait_for_fresh(
                after_seq=start_seq,
                timeout_sec=timeout_sec,
                max_age_sec=max_age_sec,
            )
            if frame_error is not None or frame_bgr is None:
                return fill_model_task_run_response(response, _frame_failure(rid, frame_error or "camera frame unavailable"))

            try:
                frame_path = _write_frame_snapshot(self.workspace_root, self.output_root, rid, frame_bgr)
            except Exception as exc:
                payload = failure(
                    message=str(exc),
                    code=CODE_DOWNSTREAM_ERROR,
                    request_id=rid,
                    data={"error_category": "frame_snapshot_failed"},
                )
                return fill_model_task_run_response(response, payload)

            payload = self.core.run_task(
                request_id=rid,
                task_name=str(request.task_name),
                frame_path=frame_path,
                conf=float(request.conf),
                iou=float(request.iou),
                max_det=int(request.max_det),
                params_json=str(request.params_json or ""),
            )
            return fill_model_task_run_response(response, payload)

    rclpy.init(args=args)
    node: Any | None = None
    executor: Any | None = None
    try:
        node = RhwModelSchedulerNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.stop()
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":  # pragma: no cover
    main()
