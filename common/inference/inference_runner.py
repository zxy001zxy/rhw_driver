from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any

from common.inference.contracts import parse_task_type
from common.inference.yolo_adapters import YoloTaskAdapter


class Phase6InferenceRunner:
    def __init__(
        self,
        frame_index_path: Path,
        output_root: Path,
        adapter: YoloTaskAdapter,
        default_task_type: str = "det",
        default_fps: float = 2.0,
    ) -> None:
        self.frame_index_path = Path(frame_index_path)
        self.output_root = Path(output_root)
        self.adapter = adapter
        self.default_task_type = parse_task_type(default_task_type)
        self.default_fps = float(default_fps)
        self.active_fps = self.default_fps

        self._auto_stop_event = threading.Event()
        self._auto_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._last_frame_path: str | None = None
        self._active_task_type = self.default_task_type

    def run_once(
        self,
        task_type: str,
        frame_path: str,
        conf: float = 0.25,
        iou: float = 0.45,
        max_det: int = 100,
    ) -> dict[str, Any]:
        normalized_task = parse_task_type(task_type)
        adapter_payload = self.adapter.run(
            normalized_task,
            frame_path,
            conf=conf,
            iou=iou,
            max_det=max_det,
        )
        items = list(adapter_payload.get("items", []))
        errors = list(adapter_payload.get("errors", []))
        meta = dict(adapter_payload.get("meta", {}))
        meta["request_ts"] = time.time()
        meta["partial_success"] = bool(items) and bool(errors)

        output_payload = {
            "task_type": normalized_task,
            "frame_path": frame_path,
            "items": items,
            "meta": meta,
            "errors": errors,
        }
        self.output_root.mkdir(parents=True, exist_ok=True)
        output_path = self.output_root / f"{normalized_task}_results.jsonl"
        with output_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(output_payload, ensure_ascii=False) + "\n")
        return output_payload

    def start_auto(self, task_type: str = "det", fps: float = 2.0) -> None:
        normalized_task = parse_task_type(task_type)
        normalized_fps = float(fps if fps is not None else 2.0)
        if normalized_fps <= 0:
            normalized_fps = 2.0
        interval = max(0.2, 1.0 / normalized_fps)

        with self._lock:
            if self._auto_thread is not None and self._auto_thread.is_alive():
                return
            self._auto_stop_event.clear()
            self.active_fps = normalized_fps
            self._active_task_type = normalized_task
            self._auto_thread = threading.Thread(
                target=self._auto_loop,
                args=(interval,),
                daemon=True,
                name="phase6-inference-auto",
            )
            self._auto_thread.start()

    def stop_auto(self) -> None:
        self._auto_stop_event.set()
        thread: threading.Thread | None
        with self._lock:
            thread = self._auto_thread
            self._auto_thread = None
        if thread is not None:
            thread.join(timeout=1.0)

    def _auto_loop(self, interval: float) -> None:
        while not self._auto_stop_event.is_set():
            frame_path = self._read_latest_frame_path()
            if frame_path:
                with self._lock:
                    if frame_path == self._last_frame_path:
                        pass
                    else:
                        task_type = self._active_task_type
                        self._last_frame_path = frame_path
                        self.run_once(task_type=task_type, frame_path=frame_path)
            self._auto_stop_event.wait(interval)

    def _read_latest_frame_path(self) -> str | None:
        if not self.frame_index_path.exists():
            return None
        try:
            lines = self.frame_index_path.read_text(encoding="utf-8").splitlines()
        except Exception:
            return None
        for raw_line in reversed(lines):
            line = raw_line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            frame_path = payload.get("frame_path")
            if isinstance(frame_path, str) and frame_path.strip():
                return frame_path.strip()
        return None
