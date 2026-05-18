from __future__ import annotations

import threading
import time
from typing import Any, Callable
from urllib.parse import SplitResult, urlsplit, urlunsplit


FrameCallback = Callable[[Any], None]
ErrorCallback = Callable[[str], None]
CaptureFactory = Callable[[str, float, float], Any]


def validate_rtsp_url(stream_url: str | None) -> str:
    text = str(stream_url or "").strip()
    if not text:
        raise ValueError("camera_stream_url is required")
    parsed = urlsplit(text)
    if parsed.scheme.lower() != "rtsp" or not parsed.netloc:
        raise ValueError("camera_stream_url must be a full rtsp:// URL")
    return text


def mask_stream_url(stream_url: str) -> str:
    parsed = urlsplit(stream_url)
    if "@" not in parsed.netloc:
        return stream_url
    host_part = parsed.netloc.rsplit("@", 1)[1]
    return urlunsplit(_replace_netloc(parsed, f"***:***@{host_part}"))


def mask_sensitive_message(message: str, stream_url: str) -> str:
    masked_url = mask_stream_url(stream_url)
    sanitized = str(message).replace(stream_url, masked_url)
    parsed = urlsplit(stream_url)
    if "@" in parsed.netloc:
        sanitized = sanitized.replace(parsed.netloc, urlsplit(masked_url).netloc)
    return sanitized


def _replace_netloc(parsed: SplitResult, netloc: str) -> SplitResult:
    return SplitResult(parsed.scheme, netloc, parsed.path, parsed.query, parsed.fragment)


def _default_capture_factory(stream_url: str, open_timeout_sec: float, read_timeout_sec: float) -> Any:
    import cv2

    params: list[int] = []
    open_timeout_prop = getattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC", None)
    read_timeout_prop = getattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", None)
    if open_timeout_prop is not None and open_timeout_sec > 0:
        params.extend([int(open_timeout_prop), int(open_timeout_sec * 1000)])
    if read_timeout_prop is not None and read_timeout_sec > 0:
        params.extend([int(read_timeout_prop), int(read_timeout_sec * 1000)])

    if params:
        try:
            return cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG, params)
        except Exception:
            pass

    capture = cv2.VideoCapture(stream_url)
    for prop, value_sec in ((open_timeout_prop, open_timeout_sec), (read_timeout_prop, read_timeout_sec)):
        if prop is None or value_sec <= 0:
            continue
        try:
            capture.set(int(prop), int(value_sec * 1000))
        except Exception:
            pass
    return capture


class RtspFrameSource:
    def __init__(
        self,
        *,
        stream_url: str,
        on_frame: FrameCallback,
        on_error: ErrorCallback,
        reconnect_interval_sec: float = 2.0,
        open_timeout_sec: float = 5.0,
        read_timeout_sec: float = 5.0,
        capture_factory: CaptureFactory | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.stream_url = validate_rtsp_url(stream_url)
        self.masked_stream_url = mask_stream_url(self.stream_url)
        self._on_frame = on_frame
        self._on_error = on_error
        self._reconnect_interval_sec = max(0.1, float(reconnect_interval_sec))
        self._open_timeout_sec = max(0.0, float(open_timeout_sec))
        self._read_timeout_sec = max(0.0, float(read_timeout_sec))
        self._capture_factory = capture_factory or _default_capture_factory
        self._sleep = sleep
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._capture_lock = threading.Lock()
        self._capture: Any = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="rtsp-frame-source")
        self._thread.start()

    def stop(self, timeout_sec: float = 2.0) -> None:
        self._stop_event.set()
        self._release_capture()
        if self._thread is not None:
            self._thread.join(timeout=max(0.0, timeout_sec))

    def _run(self) -> None:
        while not self._stop_event.is_set():
            capture = None
            try:
                capture = self._capture_factory(self.stream_url, self._open_timeout_sec, self._read_timeout_sec)
                with self._capture_lock:
                    self._capture = capture
                if not bool(capture.isOpened()):
                    raise RuntimeError(f"failed to open RTSP stream: {self.masked_stream_url}")

                while not self._stop_event.is_set():
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        raise RuntimeError(f"failed to read RTSP frame: {self.masked_stream_url}")
                    self._on_frame(frame)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._on_error(mask_sensitive_message(str(exc), self.stream_url))
                    self._sleep(self._reconnect_interval_sec)
            finally:
                if capture is not None:
                    self._release_capture(capture)

    def _release_capture(self, capture: Any | None = None) -> None:
        with self._capture_lock:
            target = capture if capture is not None else self._capture
            if target is self._capture:
                self._capture = None
        if target is not None:
            try:
                target.release()
            except Exception:
                pass
