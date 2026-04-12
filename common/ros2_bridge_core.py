from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Optional
from urllib.parse import quote

from common.api_contract import (
    CODE_BUSINESS_NOOP,
    CODE_DOWNSTREAM_ERROR,
    CODE_INVALID_PARAMS,
    CODE_NOT_FOUND,
    ensure_request_id,
    failure,
    success,
)
from common.preview_manager import PreviewError, PreviewSession, start_preview, stop_preview
from common.ptz_controller import PtzController, PtzError


ControllerFactory = Callable[..., Any]
PreviewStartFn = Callable[..., PreviewSession]
PreviewStopFn = Callable[[PreviewSession], Any]


@dataclass(frozen=True)
class CameraDefaults:
    ip: str = ""
    username: str = ""
    password: str = ""
    port: int = 80
    use_https: bool = False
    verify_ssl: bool = False
    timeout_sec: float = 5.0


@dataclass(frozen=True)
class PreviewDefaults:
    output_root: Path = Path("python_refactor/runtime/preview")
    ffmpeg_bin: str = "ffmpeg"
    wait_timeout_sec: float = 30.0
    preview_mode: str = "latest_frame"
    print_ffmpeg_logs: bool = False
    rtsp_port: int = 554
    channel_stream: str = "101"


def build_default_rtsp(
    *,
    ip: str,
    username: str,
    password: str,
    rtsp_port: int,
    channel_stream: str,
) -> str:
    user = quote(username, safe="")
    pwd = quote(password, safe="")
    return f"rtsp://{user}:{pwd}@{ip}:{int(rtsp_port)}/Streaming/Channels/{channel_stream}"


class Ros2BridgeCore:
    def __init__(
        self,
        *,
        camera_defaults: CameraDefaults,
        preview_defaults: PreviewDefaults,
        controller_factory: ControllerFactory = PtzController,
        preview_start_fn: PreviewStartFn = start_preview,
        preview_stop_fn: PreviewStopFn = stop_preview,
    ) -> None:
        self.camera_defaults = camera_defaults
        self.preview_defaults = preview_defaults
        self._controller_factory = controller_factory
        self._preview_start = preview_start_fn
        self._preview_stop = preview_stop_fn
        self._sessions: Dict[str, PreviewSession] = {}
        self._lock = threading.RLock()

    @staticmethod
    def _clean_text(value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        text = str(value).strip()
        return text or None

    @staticmethod
    def _session_payload(session: PreviewSession) -> Dict[str, object]:
        process = session.process
        running = session.is_running()
        return_code = None if running or process is None else process.returncode
        now = time.time()
        effective_end = now if running else (session.stopped_at or now)
        uptime_sec = max(0.0, effective_end - session.started_at)
        inferred_exit = (not running) and (return_code is not None) and (return_code != 0)
        last_error = session.last_error or ("ffmpeg process exited unexpectedly" if inferred_exit else None)
        last_error_category = session.last_error_category or ("process_exited" if inferred_exit else None)
        if running:
            health = "running"
        elif last_error:
            health = "error"
        else:
            health = "stopped"
        return {
            "session_id": session.session_id,
            "rtsp_url": session.rtsp_url,
            "running": running,
            "pid": process.pid if process is not None else None,
            "return_code": return_code,
            "health": health,
            "started_at": session.started_at,
            "stopped_at": session.stopped_at,
            "uptime_sec": round(uptime_sec, 3),
            "last_error": last_error,
            "last_error_category": last_error_category,
            "preview_mode": session.preview_mode,
            "restart_count": int(session.restart_count),
            "output_dir": str(session.output_dir.resolve()),
            "playlist_path": str(session.playlist_path.resolve()),
            "player_path": str(session.player_path.resolve()),
            "latest_frame_path": str(session.latest_frame_path.resolve()) if session.latest_frame_path else "",
        }

    def _controller_kwargs(
        self,
        *,
        ip: Optional[str] = None,
        username: Optional[str] = None,
        password: Optional[str] = None,
        port: Optional[int] = None,
        use_https: Optional[bool] = None,
        verify_ssl: Optional[bool] = None,
        timeout_sec: Optional[float] = None,
    ) -> Dict[str, object]:
        return {
            "ip": self._clean_text(ip) or self.camera_defaults.ip,
            "username": self._clean_text(username) or self.camera_defaults.username,
            "password": self._clean_text(password) or self.camera_defaults.password,
            "port": int(port) if port is not None else int(self.camera_defaults.port),
            "use_https": bool(self.camera_defaults.use_https if use_https is None else use_https),
            "verify_ssl": bool(self.camera_defaults.verify_ssl if verify_ssl is None else verify_ssl),
            "timeout": float(self.camera_defaults.timeout_sec if timeout_sec is None else timeout_sec),
        }

    def _validate_camera_identity(self, request_id: str, *, kwargs: Dict[str, object]) -> Optional[dict]:
        for field in ("ip", "username", "password"):
            if not str(kwargs.get(field, "")).strip():
                return failure(
                    message=f"{field} is required",
                    code=CODE_INVALID_PARAMS,
                    request_id=request_id,
                    data={"field": field},
                )
        return None

    def ptz_control(
        self,
        *,
        direction: str,
        speed: int = 40,
        channel: int = 1,
        duration_ms: int = 350,
        request_id: Optional[str] = None,
        ip: Optional[str] = None,
        username: Optional[str] = None,
        password: Optional[str] = None,
        port: Optional[int] = None,
        use_https: Optional[bool] = None,
        verify_ssl: Optional[bool] = None,
        timeout_sec: Optional[float] = None,
    ) -> dict:
        rid = request_id or ensure_request_id()
        controller_kwargs = self._controller_kwargs(
            ip=ip,
            username=username,
            password=password,
            port=port,
            use_https=use_https,
            verify_ssl=verify_ssl,
            timeout_sec=timeout_sec,
        )
        invalid = self._validate_camera_identity(rid, kwargs=controller_kwargs)
        if invalid is not None:
            return invalid

        controller = self._controller_factory(**controller_kwargs)
        try:
            result = controller.control(
                direction=direction,
                speed=int(speed),
                channel=int(channel),
                duration_ms=int(duration_ms),
            )
        except PtzError as exc:
            code = CODE_INVALID_PARAMS if exc.category.startswith("invalid_") else CODE_DOWNSTREAM_ERROR
            return failure(
                message=str(exc),
                code=code,
                request_id=rid,
                data={"direction": direction, "error_category": exc.category},
            )

        if result.get("ok"):
            return success(
                message="ptz control executed",
                request_id=rid,
                data={"result": result},
            )
        return failure(
            message=str(result.get("error") or "ptz control failed"),
            code=CODE_DOWNSTREAM_ERROR,
            request_id=rid,
            data={"result": result, "error_category": result.get("error_category")},
        )

    def preview_start(
        self,
        *,
        request_id: Optional[str] = None,
        session_id: Optional[str] = None,
        rtsp_url: Optional[str] = None,
        ip: Optional[str] = None,
        username: Optional[str] = None,
        password: Optional[str] = None,
        rtsp_port: Optional[int] = None,
        channel_stream: Optional[str] = None,
        wait_timeout_sec: Optional[float] = None,
        preview_mode: Optional[str] = None,
    ) -> dict:
        rid = request_id or ensure_request_id()
        normalized_session_id = self._clean_text(session_id)
        normalized_rtsp = self._clean_text(rtsp_url)
        resolved_wait_timeout = (
            float(wait_timeout_sec) if wait_timeout_sec is not None else float(self.preview_defaults.wait_timeout_sec)
        )
        resolved_preview_mode = self._clean_text(preview_mode) or self.preview_defaults.preview_mode

        if not normalized_rtsp:
            controller_kwargs = self._controller_kwargs(ip=ip, username=username, password=password)
            invalid = self._validate_camera_identity(rid, kwargs=controller_kwargs)
            if invalid is not None:
                return invalid
            resolved_rtsp_port = int(rtsp_port) if rtsp_port is not None else int(self.preview_defaults.rtsp_port)
            resolved_channel_stream = self._clean_text(channel_stream) or self.preview_defaults.channel_stream
            normalized_rtsp = build_default_rtsp(
                ip=str(controller_kwargs["ip"]),
                username=str(controller_kwargs["username"]),
                password=str(controller_kwargs["password"]),
                rtsp_port=resolved_rtsp_port,
                channel_stream=resolved_channel_stream,
            )

        replaced_existing = False
        with self._lock:
            if normalized_session_id:
                existing = self._sessions.get(normalized_session_id)
                if existing and existing.is_running():
                    return success(
                        message="preview started",
                        request_id=rid,
                        data={
                            "created": False,
                            "replaced_existing": False,
                            "session": self._session_payload(existing),
                        },
                    )
                if existing:
                    self._preview_stop(existing)
                    self._sessions.pop(normalized_session_id, None)
                    replaced_existing = True

            try:
                session = self._preview_start(
                    rtsp_url=normalized_rtsp,
                    session_id=normalized_session_id,
                    output_root=self.preview_defaults.output_root,
                    ffmpeg_bin=self.preview_defaults.ffmpeg_bin,
                    wait_timeout_sec=resolved_wait_timeout,
                    print_ffmpeg_logs=self.preview_defaults.print_ffmpeg_logs,
                    preview_mode=resolved_preview_mode,
                )
            except PreviewError as exc:
                code = CODE_INVALID_PARAMS if exc.category.startswith("invalid_") else CODE_DOWNSTREAM_ERROR
                return failure(
                    message=str(exc),
                    code=code,
                    request_id=rid,
                    data={
                        "error_category": exc.category,
                        "session_id": normalized_session_id or "",
                    },
                )
            except Exception as exc:
                return failure(
                    message=str(exc),
                    code=CODE_DOWNSTREAM_ERROR,
                    request_id=rid,
                )

            self._sessions[session.session_id] = session
            return success(
                message="preview started",
                request_id=rid,
                data={
                    "created": True,
                    "replaced_existing": replaced_existing,
                    "session": self._session_payload(session),
                },
            )

    def preview_stop(self, *, session_id: str, request_id: Optional[str] = None) -> dict:
        rid = request_id or ensure_request_id()
        normalized_session_id = self._clean_text(session_id) or ""
        with self._lock:
            session = self._sessions.get(normalized_session_id)
            if session is None:
                return failure(
                    message=f"session not found: {normalized_session_id}",
                    code=CODE_NOT_FOUND,
                    request_id=rid,
                    data={"session_id": normalized_session_id},
                )

            stop_result = self._preview_stop(session)
            if getattr(stop_result, "status", "") in {"stopped", "already_stopped"}:
                self._sessions.pop(normalized_session_id, None)

        payload = {
            "session_id": stop_result.session_id,
            "status": stop_result.status,
            "was_running": stop_result.was_running,
            "forced_kill": stop_result.forced_kill,
            "return_code": stop_result.return_code,
            "stopped_at": stop_result.stopped_at,
            "error": stop_result.error,
        }
        if stop_result.status == "failed":
            return failure(
                message="failed to stop preview session",
                code=CODE_DOWNSTREAM_ERROR,
                request_id=rid,
                data=payload,
            )
        if stop_result.status == "already_stopped":
            return failure(
                message="preview session already stopped",
                code=CODE_BUSINESS_NOOP,
                request_id=rid,
                data=payload,
            )
        return success(
            message="preview stopped",
            request_id=rid,
            data=payload,
        )

    def preview_status(self, *, session_id: str, request_id: Optional[str] = None) -> dict:
        rid = request_id or ensure_request_id()
        normalized_session_id = self._clean_text(session_id) or ""
        with self._lock:
            session = self._sessions.get(normalized_session_id)
            if session is None:
                return failure(
                    message=f"session not found: {normalized_session_id}",
                    code=CODE_NOT_FOUND,
                    request_id=rid,
                    data={"session_id": normalized_session_id},
                )
            payload = self._session_payload(session)

        return success(
            message="preview status fetched",
            request_id=rid,
            data={"session": payload},
        )

    def shutdown(self) -> list[dict]:
        results: list[dict] = []
        with self._lock:
            sessions = list(self._sessions.values())
            self._sessions.clear()
        for session in sessions:
            try:
                stop_result = self._preview_stop(session)
                results.append(
                    {
                        "session_id": stop_result.session_id,
                        "status": stop_result.status,
                        "error": stop_result.error,
                    }
                )
            except Exception as exc:
                results.append(
                    {
                        "session_id": session.session_id,
                        "status": "failed",
                        "error": str(exc),
                    }
                )
        return results
