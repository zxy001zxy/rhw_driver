from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import shutil
import subprocess
import threading
import time
import uuid
from typing import Optional


SUPPORTED_PREVIEW_MODES = {"hls", "latest_frame"}
PREVIEW_RESTART_BACKOFF_SEC = [1, 2, 4, 8, 16, 30]


class PreviewError(Exception):
    """Raised when preview startup or runtime fails."""

    def __init__(self, message: str, *, category: str = "unknown") -> None:
        super().__init__(message)
        self.category = category


@dataclass
class StopResult:
    session_id: str
    status: str
    was_running: bool
    forced_kill: bool
    return_code: Optional[int]
    stopped_at: Optional[float]
    error: Optional[str]


@dataclass
class PreviewSession:
    session_id: str
    rtsp_url: str
    output_dir: Path
    playlist_path: Path
    player_path: Path
    process: Optional[subprocess.Popen[str]]
    started_at: float
    preview_mode: str = "hls"
    latest_frame_path: Optional[Path] = None
    stopped_at: Optional[float] = None
    last_error: Optional[str] = None
    last_error_category: Optional[str] = None
    restart_count: int = 0
    _ffmpeg_bin: str = field(default="ffmpeg", repr=False)
    _wait_timeout_sec: float = field(default=20.0, repr=False)
    _print_ffmpeg_logs: bool = field(default=False, repr=False)
    _stop_event: threading.Event = field(default_factory=threading.Event, repr=False)
    _lock: threading.RLock = field(default_factory=threading.RLock, repr=False)
    _monitor_thread: Optional[threading.Thread] = field(default=None, repr=False)

    def is_running(self) -> bool:
        with self._lock:
            process = self.process
            return process is not None and process.poll() is None


def is_ffmpeg_installed(ffmpeg_bin: str = "ffmpeg") -> bool:
    try:
        subprocess.run(
            [ffmpeg_bin, "-version"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=True,
            timeout=2,
        )
        return True
    except Exception:
        return False


def _normalize_session_id(session_id: Optional[str]) -> str:
    if session_id and session_id.strip():
        return session_id.strip()
    return uuid.uuid4().hex


def _recreate_dir(path: Path) -> None:
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def _cleanup_preview_artifacts(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)
    for child in path.iterdir():
        if child.is_file() and (
            child.name in {"index.m3u8", "player.html", "latest.jpg"} or child.suffix in {".ts", ".m4s"}
        ):
            try:
                child.unlink()
            except FileNotFoundError:
                pass


def _player_html_content(session_id: str, *, preview_mode: str) -> str:
    if preview_mode == "latest_frame":
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Preview {session_id}</title>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 16px; background: #0f172a; color: #e2e8f0; }}
    h1 {{ font-size: 16px; margin-bottom: 10px; }}
    .stage {{ width: 100%; max-width: 1200px; min-height: 320px; background: #000; border-radius: 12px; overflow: hidden; display: grid; place-items: center; }}
    .stage.is-ready {{ min-height: 0; }}
    canvas {{ width: 100%; height: auto; display: block; background: #000; }}
    .placeholder {{ padding: 20px; color: #cbd5e1; text-align: center; }}
    code {{ background: rgba(255,255,255,0.08); padding: 2px 4px; border-radius: 4px; }}
  </style>
</head>
<body>
  <h1>Live Preview</h1>
  <p>Session: <code>{session_id}</code></p>
  <div id="stage" class="stage">
    <canvas id="frameCanvas" hidden></canvas>
    <div id="placeholder" class="placeholder">Connecting preview...</div>
  </div>
  <script>
    const stage = document.getElementById("stage");
    const canvas = document.getElementById("frameCanvas");
    const placeholder = document.getElementById("placeholder");
    const context = canvas.getContext("2d", {{ alpha: false }});
    const source = "latest.jpg";
    let timer = null;
    let requestInFlight = false;
    let loader = null;

    function nextDelay() {{
      return document.hidden ? 250 : 40;
    }}

    function refreshFrame(delay) {{
      window.clearTimeout(timer);
      timer = window.setTimeout(() => {{
        if (requestInFlight) {{
          return;
        }}
        requestInFlight = true;
        const nextImage = new Image();
        nextImage.decoding = "async";
        nextImage.addEventListener("load", () => onFrameLoad(nextImage));
        nextImage.addEventListener("error", onFrameError);
        nextImage.src = source + "?ts=" + Date.now();
        loader = nextImage;
      }}, delay);
    }}

    function onFrameLoad(image) {{
      const width = image.naturalWidth || image.width || 0;
      const height = image.naturalHeight || image.height || 0;
      loader = null;
      requestInFlight = false;
      if (!context || !width || !height) {{
        refreshFrame(120);
        return;
      }}
      if (canvas.width !== width || canvas.height !== height) {{
        canvas.width = width;
        canvas.height = height;
        stage.style.aspectRatio = String(width) + " / " + String(height);
      }}
      context.clearRect(0, 0, canvas.width, canvas.height);
      context.drawImage(image, 0, 0, canvas.width, canvas.height);
      canvas.hidden = false;
      placeholder.hidden = true;
      stage.classList.add("is-ready");
      refreshFrame(nextDelay());
    }}

    function onFrameError() {{
      loader = null;
      requestInFlight = false;
      refreshFrame(120);
    }}

    refreshFrame(0);
  </script>
</body>
</html>
"""

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Preview {session_id}</title>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 16px; }}
    h1 {{ font-size: 16px; }}
    video {{ width: 100%; max-width: 960px; background: #111; }}
    code {{ background: #f5f5f5; padding: 2px 4px; }}
  </style>
  <script src="https://cdn.jsdelivr.net/npm/hls.js@latest"></script>
</head>
<body>
  <h1>Live Preview</h1>
  <p>Session: <code>{session_id}</code></p>
  <video id="video" controls autoplay muted></video>
  <script>
    const video = document.getElementById("video");
    const source = "index.m3u8";

    if (video.canPlayType("application/vnd.apple.mpegurl")) {{
      video.src = source;
      video.play();
    }} else if (window.Hls && Hls.isSupported()) {{
      const hls = new Hls({{
        lowLatencyMode: true,
        liveSyncDurationCount: 1,
        liveMaxLatencyDurationCount: 2,
        maxLiveSyncPlaybackRate: 1.5,
        maxBufferLength: 1,
        backBufferLength: 0,
        enableWorker: true
      }});
      hls.loadSource(source);
      hls.attachMedia(video);
      hls.on(Hls.Events.MANIFEST_PARSED, () => video.play());
    }} else {{
      document.body.insertAdjacentHTML("beforeend", "<p>Browser does not support HLS.</p>");
    }}
  </script>
</body>
</html>
"""


def _pump_ffmpeg_output(stream, tag: str, print_logs: bool) -> None:
    def _run() -> None:
        for line in iter(stream.readline, ""):
            if print_logs:
                print(f"[ffmpeg][{tag}] {line.rstrip()}")
        stream.close()

    thread = threading.Thread(target=_run, daemon=True, name=f"ffmpeg-log-{tag}")
    thread.start()


def _stop_process(process: Optional[subprocess.Popen[str]]) -> tuple[str, bool, bool, Optional[int], Optional[str]]:
    if process is None:
        return ("already_stopped", False, False, None, None)
    code = process.poll()
    if code is not None:
        return ("already_stopped", False, False, code, None)

    forced_kill = False
    try:
        process.terminate()
        try:
            code = process.wait(timeout=1.5)
        except subprocess.TimeoutExpired:
            process.kill()
            forced_kill = True
            code = process.wait(timeout=1.5)
        return ("stopped", True, forced_kill, code, None)
    except Exception as exc:  # pragma: no cover - defensive branch for runtime env variance.
        return ("failed", True, forced_kill, process.poll(), str(exc))


def _build_hls_command(*, ffmpeg_bin: str, rtsp_url: str, playlist_path: Path) -> list[str]:
    return [
        ffmpeg_bin,
        "-y",
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-rtsp_transport",
        "tcp",
        "-i",
        rtsp_url,
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "ultrafast",
        "-tune",
        "zerolatency",
        "-g",
        "12",
        "-keyint_min",
        "12",
        "-sc_threshold",
        "0",
        "-f",
        "hls",
        "-hls_time",
        "0.5",
        "-hls_list_size",
        "3",
        "-hls_allow_cache",
        "0",
        "-hls_flags",
        "delete_segments+append_list+omit_endlist+split_by_time+independent_segments",
        str(playlist_path),
    ]


def _build_latest_frame_command(*, ffmpeg_bin: str, rtsp_url: str, latest_frame_path: Path) -> list[str]:
    return [
        ffmpeg_bin,
        "-y",
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-rtsp_transport",
        "tcp",
        "-i",
        rtsp_url,
        "-an",
        "-map",
        "0:v:0",
        "-fflags",
        "+discardcorrupt",
        "-q:v",
        "4",
        "-f",
        "image2",
        "-atomic_writing",
        "1",
        "-update",
        "1",
        str(latest_frame_path),
    ]


def _build_preview_command(session: PreviewSession) -> list[str]:
    if session.preview_mode == "latest_frame":
        if session.latest_frame_path is None:
            raise PreviewError("latest_frame_path is required for latest_frame preview mode", category="invalid_preview_mode")
        return _build_latest_frame_command(
            ffmpeg_bin=session._ffmpeg_bin,
            rtsp_url=session.rtsp_url,
            latest_frame_path=session.latest_frame_path,
        )
    return _build_hls_command(
        ffmpeg_bin=session._ffmpeg_bin,
        rtsp_url=session.rtsp_url,
        playlist_path=session.playlist_path,
    )


def _ready_path(session: PreviewSession) -> Path:
    if session.preview_mode == "latest_frame":
        if session.latest_frame_path is None:
            raise PreviewError("latest_frame_path is required for latest_frame preview mode", category="invalid_preview_mode")
        return session.latest_frame_path
    return session.playlist_path


def _wait_until_ready(session: PreviewSession, process: subprocess.Popen[str]) -> None:
    ready_path = _ready_path(session)
    deadline = time.monotonic() + session._wait_timeout_sec
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise PreviewError(
                f"ffmpeg exited unexpectedly with code {process.returncode}",
                category="process_exited",
            )
        if ready_path.exists() and ready_path.stat().st_size > 0:
            session.player_path.write_text(
                _player_html_content(session.session_id, preview_mode=session.preview_mode),
                encoding="utf-8",
            )
            return
        time.sleep(0.2)

    _stop_process(process)
    timeout_category = "latest_frame_publish_timeout" if session.preview_mode == "latest_frame" else "playlist_publish_timeout"
    raise PreviewError(
        f"preview output not ready within {session._wait_timeout_sec:.1f}s",
        category=timeout_category,
    )


def _spawn_preview_process(session: PreviewSession, *, cleanup_existing: bool) -> subprocess.Popen[str]:
    if cleanup_existing:
        _cleanup_preview_artifacts(session.output_dir)
    command = _build_preview_command(session)
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    if process.stdout is not None:
        _pump_ffmpeg_output(
            process.stdout,
            f"preview-{session.preview_mode}-{session.session_id}",
            session._print_ffmpeg_logs,
        )
    return process


def _monitor_preview_session(session: PreviewSession) -> None:
    while not session._stop_event.is_set():
        with session._lock:
            process = session.process
        if process is not None and process.poll() is None:
            session._stop_event.wait(0.5)
            continue
        if session._stop_event.is_set():
            return

        if process is not None:
            session.last_error = f"ffmpeg exited unexpectedly with code {process.returncode}"
            session.last_error_category = "process_exited"
            with session._lock:
                session.process = None

        delay = PREVIEW_RESTART_BACKOFF_SEC[min(session.restart_count, len(PREVIEW_RESTART_BACKOFF_SEC) - 1)]
        session.restart_count += 1
        if session._stop_event.wait(delay):
            return

        try:
            process = _spawn_preview_process(session, cleanup_existing=True)
            _wait_until_ready(session, process)
        except PreviewError as exc:
            session.last_error = str(exc)
            session.last_error_category = exc.category
            continue
        except Exception as exc:  # pragma: no cover - runtime variance.
            session.last_error = str(exc)
            session.last_error_category = "preview_restart_failed"
            continue

        with session._lock:
            session.process = process
        session.last_error = None
        session.last_error_category = None


def _ensure_monitor(session: PreviewSession) -> None:
    with session._lock:
        if session._monitor_thread is not None and session._monitor_thread.is_alive():
            return
        session._stop_event.clear()
        session._monitor_thread = threading.Thread(
            target=_monitor_preview_session,
            args=(session,),
            daemon=True,
            name=f"preview-monitor-{session.session_id}",
        )
        session._monitor_thread.start()


def stop_preview(session: PreviewSession) -> StopResult:
    session._stop_event.set()
    with session._lock:
        process = session.process
        session.process = None
    status, was_running, forced_kill, return_code, error = _stop_process(process)
    thread = session._monitor_thread
    if thread is not None and thread.is_alive():
        thread.join(timeout=1.0)
    session.stopped_at = session.stopped_at or time.time()
    if error:
        session.last_error = error
        session.last_error_category = "stop_failed"
    return StopResult(
        session_id=session.session_id,
        status=status,
        was_running=was_running,
        forced_kill=forced_kill,
        return_code=return_code,
        stopped_at=session.stopped_at,
        error=error,
    )


def start_preview(
    *,
    rtsp_url: str,
    session_id: Optional[str] = None,
    output_root: Path = Path("python_refactor/runtime/preview"),
    ffmpeg_bin: str = "ffmpeg",
    wait_timeout_sec: float = 20.0,
    print_ffmpeg_logs: bool = True,
    preview_mode: str = "hls",
) -> PreviewSession:
    if not rtsp_url or not rtsp_url.strip():
        raise PreviewError("rtsp_url is required", category="invalid_rtsp_input")
    normalized_rtsp = rtsp_url.strip()
    if not normalized_rtsp.lower().startswith("rtsp://"):
        raise PreviewError("rtsp_url must start with rtsp://", category="invalid_rtsp_input")
    if wait_timeout_sec <= 0:
        raise PreviewError("wait_timeout_sec must be greater than 0", category="invalid_wait_timeout")
    normalized_preview_mode = preview_mode.strip().lower()
    if normalized_preview_mode not in SUPPORTED_PREVIEW_MODES:
        raise PreviewError(f"unsupported preview_mode: {preview_mode}", category="invalid_preview_mode")
    if not is_ffmpeg_installed(ffmpeg_bin=ffmpeg_bin):
        raise PreviewError("ffmpeg not found, please install ffmpeg", category="ffmpeg_not_found")

    sid = _normalize_session_id(session_id)
    session_dir = output_root / sid
    playlist = session_dir / "index.m3u8"
    player = session_dir / "player.html"
    latest_frame = session_dir / "latest.jpg"

    _recreate_dir(session_dir)

    session = PreviewSession(
        session_id=sid,
        rtsp_url=normalized_rtsp,
        output_dir=session_dir,
        playlist_path=playlist,
        player_path=player,
        process=None,
        started_at=time.time(),
        preview_mode=normalized_preview_mode,
        latest_frame_path=latest_frame if normalized_preview_mode == "latest_frame" else None,
        _ffmpeg_bin=ffmpeg_bin,
        _wait_timeout_sec=wait_timeout_sec,
        _print_ffmpeg_logs=print_ffmpeg_logs,
    )

    try:
        process = _spawn_preview_process(session, cleanup_existing=False)
        with session._lock:
            session.process = process
        _wait_until_ready(session, process)
        _ensure_monitor(session)
        return session
    except PreviewError:
        stop_preview(session)
        raise
    except Exception as exc:
        stop_preview(session)
        raise PreviewError(f"failed to start preview: {exc}", category="startup_exception") from exc
