from __future__ import annotations

import time
from typing import Optional


STATE_RUNNING = "running"
STATE_DEGRADED = "degraded"
STATE_RECOVERING = "recovering"
STATE_FAILED = "failed"

STATE_TEXT_CN = {
    STATE_RUNNING: "运行中",
    STATE_DEGRADED: "降级运行",
    STATE_RECOVERING: "恢复中",
    STATE_FAILED: "故障",
}


class RuntimeStateMachine:
    def __init__(self, *, default_inference_fps: float = 2.0):
        self.default_inference_fps = float(default_inference_fps)
        self.state_text_cn = dict(STATE_TEXT_CN)
        self._state = STATE_RUNNING
        self._reason = "service_ready"
        self._degraded_stage = 0
        self._preview_profile = "normal"
        self._inference_fps = self.default_inference_fps
        self._inference_enabled = False
        self._recover_attempt = 0
        self._last_error: Optional[str] = None
        self._last_error_category: Optional[str] = None
        self._updated_at = time.time()

    def mark_running(self, reason: str = "service_ready") -> None:
        self._state = STATE_RUNNING
        self._reason = reason
        self._degraded_stage = 0
        self._recover_attempt = 0
        self._preview_profile = "normal"
        if self._inference_fps <= 0:
            self._inference_fps = self.default_inference_fps
        self._updated_at = time.time()

    def mark_degraded(self, reason: str, stage: int, preview_profile: str, inference_fps: float) -> None:
        self._state = STATE_DEGRADED
        self._reason = reason
        self._degraded_stage = max(0, int(stage))
        self._preview_profile = preview_profile
        self._inference_fps = max(0.0, float(inference_fps))
        self._updated_at = time.time()

    def mark_recovering(self, reason: str, attempt: int) -> None:
        self._state = STATE_RECOVERING
        self._reason = reason
        self._recover_attempt = max(0, int(attempt))
        self._updated_at = time.time()

    def mark_failed(self, reason: str, last_error: str, category: str) -> None:
        self._state = STATE_FAILED
        self._reason = reason
        self._last_error = last_error
        self._last_error_category = category
        self._updated_at = time.time()

    def set_inference_enabled(self, enabled: bool) -> None:
        self._inference_enabled = bool(enabled)
        self._updated_at = time.time()

    def set_inference_fps(self, fps: float) -> None:
        self._inference_fps = max(0.0, float(fps))
        self._updated_at = time.time()

    def set_preview_profile(self, profile: str) -> None:
        self._preview_profile = profile
        self._updated_at = time.time()

    def snapshot(self) -> dict:
        return {
            "state": self._state,
            "state_text_cn": self.state_text_cn[self._state],
            "reason": self._reason,
            "degraded_stage": self._degraded_stage,
            "preview_profile": self._preview_profile,
            "inference_fps": self._inference_fps,
            "inference_enabled": self._inference_enabled,
            "recover_attempt": self._recover_attempt,
            "last_error": self._last_error,
            "last_error_category": self._last_error_category,
            "updated_at": self._updated_at,
        }
