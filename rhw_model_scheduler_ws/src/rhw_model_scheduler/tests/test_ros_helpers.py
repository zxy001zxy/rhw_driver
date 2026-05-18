from __future__ import annotations

from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rhw_model_scheduler.model_scheduler_node import LatestFrameCache, fill_model_task_run_response
from rhw_model_scheduler.rtsp_frame_source import RtspFrameSource, mask_stream_url


class FakeCapture:
    def __init__(self, frames: list[np.ndarray] | None = None, *, opened: bool = True) -> None:
        self.frames = list(frames or [])
        self.opened = opened
        self.released = False

    def isOpened(self) -> bool:
        return self.opened

    def read(self):
        if not self.frames:
            return False, None
        return True, self.frames.pop(0)

    def release(self) -> None:
        self.released = True


class RosHelpersTest(unittest.TestCase):
    def test_rtsp_frame_source_updates_cache_with_fresh_copy(self) -> None:
        frame = np.zeros((2, 3, 3), dtype=np.uint8)
        frame[:, :, 1] = 96
        cache = LatestFrameCache()
        errors: list[str] = []
        source = RtspFrameSource(
            stream_url="rtsp://user:secret@camera.local:554/Streaming/Channels/101",
            on_frame=cache.update,
            on_error=errors.append,
            reconnect_interval_sec=0.01,
            capture_factory=lambda _url, _open_timeout, _read_timeout: FakeCapture([frame]),
        )

        source.start()
        try:
            bgr, error = cache.wait_for_fresh(after_seq=0, timeout_sec=1.0, max_age_sec=1.0)
        finally:
            source.stop()

        self.assertIsNone(error)
        self.assertIsNotNone(bgr)
        self.assertIsNot(bgr, frame)
        self.assertEqual(bgr.shape, frame.shape)

    def test_rtsp_frame_source_requires_rtsp_url(self) -> None:
        with self.assertRaisesRegex(ValueError, "camera_stream_url is required"):
            RtspFrameSource(stream_url="", on_frame=lambda _frame: None, on_error=lambda _message: None)
        with self.assertRaisesRegex(ValueError, "rtsp://"):
            RtspFrameSource(stream_url="http://camera.local/snapshot", on_frame=lambda _frame: None, on_error=lambda _message: None)

    def test_rtsp_frame_source_records_sanitized_read_error(self) -> None:
        cache = LatestFrameCache()
        errors: list[str] = []

        def on_error(message: str) -> None:
            errors.append(message)
            cache.record_error(message)

        source = RtspFrameSource(
            stream_url="rtsp://user:secret@camera.local:554/Streaming/Channels/101",
            on_frame=cache.update,
            on_error=on_error,
            reconnect_interval_sec=0.01,
            capture_factory=lambda _url, _open_timeout, _read_timeout: FakeCapture([]),
        )

        source.start()
        try:
            deadline = time.time() + 1.0
            while not errors and time.time() < deadline:
                time.sleep(0.01)
            bgr, error = cache.wait_for_fresh(after_seq=0, timeout_sec=0.05, max_age_sec=1.0)
        finally:
            source.stop()

        self.assertIsNone(bgr)
        self.assertIsNotNone(error)
        self.assertIn("timed out waiting for a fresh camera frame", error)
        self.assertIn("failed to read RTSP frame", error)
        self.assertNotIn("secret", error)

    def test_mask_stream_url_hides_credentials(self) -> None:
        self.assertEqual(
            mask_stream_url("rtsp://user:secret@camera.local:554/Streaming/Channels/101"),
            "rtsp://***:***@camera.local:554/Streaming/Channels/101",
        )

    def test_fill_response_maps_summary_fields(self) -> None:
        response = SimpleNamespace()
        payload = {
            "ok": True,
            "code": "OK",
            "message": "done",
            "request_id": "req-1",
            "data": {
                "task_name": "rust_segmentation",
                "task_type": "seg",
                "model_path": "models/current/rust_segmentation.pt",
                "backend": "pytorch",
                "frame_path": "runtime/model_scheduler/frames/req-1.jpg",
                "result_json_path": "runtime/model_scheduler/results/rust_segmentation/req-1.json",
                "item_count": 2,
                "error_count": 0,
                "latency_ms": 12.5,
                "detail": {"rust": {"rust_ratio": 0.1}},
            },
        }
        fill_model_task_run_response(response, payload)
        self.assertTrue(response.ok)
        self.assertEqual(response.task_name, "rust_segmentation")
        self.assertEqual(response.latency_ms, 12.5)
        self.assertIn("rust_ratio", response.detail_json)


if __name__ == "__main__":
    unittest.main()
