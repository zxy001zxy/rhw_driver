from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import subprocess
import threading
import time
from typing import Any, Callable

from rhw_model_scheduler.api_contract import (
    CODE_DOWNSTREAM_ERROR,
    CODE_INVALID_PARAMS,
    CODE_NOT_FOUND,
    CODE_OK,
    ensure_request_id,
    failure,
    json_dumps,
    success,
)
from rhw_model_scheduler.contracts import TASK_TYPE_GAUGE, parse_task_type
from rhw_model_scheduler.gauge_adapter import GaugeTaskAdapter
from rhw_model_scheduler.rust_seg_utils import build_rust_result, parse_roi_polygon
from rhw_model_scheduler.workspace import find_workspace_root
from rhw_model_scheduler.yolo_adapters import YoloTaskAdapter
from rhw_model_scheduler.yolo_registry import ModelLoader, YoloModelRegistry


DEFAULT_MODEL_MANIFEST = Path("models/current/manifest.json")
DEFAULT_OUTPUT_ROOT = Path("runtime/model_scheduler")
RUST_TASK_NAME = "rust_segmentation"


@dataclass(frozen=True, slots=True)
class ModelTaskSpec:
    task_name: str
    task_type: str
    model_path: str
    raw: dict[str, Any]


@dataclass(frozen=True, slots=True)
class ResourceSnapshot:
    system_available_mb: float | None = None
    gpu_free_mb: float | None = None


AdapterFactory = Callable[[ModelTaskSpec], Any]
ResourceProbe = Callable[[], ResourceSnapshot]


def _clean_text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _resolve_workspace_path(workspace_root: Path, path: str | Path) -> Path:
    raw_path = Path(path)
    if raw_path.is_absolute():
        return raw_path.resolve()
    return (workspace_root / raw_path).resolve()


def _workspace_relative(workspace_root: Path, path: str | Path) -> str:
    resolved = Path(path).resolve()
    try:
        return resolved.relative_to(workspace_root).as_posix()
    except ValueError:
        return resolved.as_posix()


def load_model_manifest(
    manifest_path: str | Path = DEFAULT_MODEL_MANIFEST,
    *,
    workspace_root: str | Path | None = None,
) -> dict[str, ModelTaskSpec]:
    root = Path(workspace_root).resolve() if workspace_root is not None else find_workspace_root()
    resolved_manifest = _resolve_workspace_path(root, manifest_path)
    payload = json.loads(resolved_manifest.read_text(encoding="utf-8"))
    models = payload.get("models")
    if not isinstance(models, list):
        raise ValueError("model manifest must contain a models list")

    specs: dict[str, ModelTaskSpec] = {}
    for index, raw_model in enumerate(models):
        if not isinstance(raw_model, dict):
            raise ValueError(f"model manifest entry {index} must be an object")
        task_name = _clean_text(raw_model.get("task"))
        task_type = _clean_text(raw_model.get("task_type"))
        model_path = _clean_text(raw_model.get("current_path"))
        if task_name is None:
            raise ValueError(f"model manifest entry {index} is missing task")
        if task_type is None:
            raise ValueError(f"model manifest entry {task_name} is missing task_type")
        if model_path is None:
            raise ValueError(f"model manifest entry {task_name} is missing current_path")
        if task_name in specs:
            raise ValueError(f"duplicate model task: {task_name}")

        normalized_task_type = parse_task_type(task_type)
        resolved_model_path = _resolve_workspace_path(root, model_path)
        if not resolved_model_path.is_file():
            raise FileNotFoundError(f"model file not found for {task_name}: {model_path}")
        specs[task_name] = ModelTaskSpec(
            task_name=task_name,
            task_type=normalized_task_type,
            model_path=str(resolved_model_path),
            raw=dict(raw_model),
        )
    return specs


def default_resource_probe() -> ResourceSnapshot:
    return ResourceSnapshot(
        system_available_mb=_system_available_mb(),
        gpu_free_mb=_gpu_free_mb(),
    )


def _system_available_mb() -> float | None:
    try:
        import psutil

        return float(psutil.virtual_memory().available) / (1024.0 * 1024.0)
    except Exception:
        pass

    try:
        meminfo = Path("/proc/meminfo").read_text(encoding="utf-8").splitlines()
    except Exception:
        return None
    for line in meminfo:
        if not line.startswith("MemAvailable:"):
            continue
        parts = line.split()
        if len(parts) >= 2:
            return float(parts[1]) / 1024.0
    return None


def _gpu_free_mb() -> float | None:
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.free", "--format=csv,noheader,nounits"],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except Exception:
        return None
    if result.returncode != 0:
        return None
    values: list[float] = []
    for raw_line in result.stdout.splitlines():
        text = raw_line.strip()
        if not text or text.upper() == "N/A":
            continue
        try:
            values.append(float(text.split()[0]))
        except ValueError:
            continue
    if not values:
        return None
    return max(values)


def parse_params_json(params_json: str | None) -> dict[str, Any]:
    text = _clean_text(params_json)
    if text is None:
        return {}
    payload = json.loads(text)
    if not isinstance(payload, dict):
        raise ValueError("params_json must be a JSON object")
    return payload


class ModelTaskSchedulerCore:
    def __init__(
        self,
        *,
        workspace_root: str | Path | None = None,
        manifest_path: str | Path = DEFAULT_MODEL_MANIFEST,
        output_root: str | Path = DEFAULT_OUTPUT_ROOT,
        adapter_factory: AdapterFactory | None = None,
        model_loader: ModelLoader | None = None,
        resource_probe: ResourceProbe = default_resource_probe,
    ) -> None:
        self.workspace_root = Path(workspace_root).resolve() if workspace_root is not None else find_workspace_root()
        self.manifest_path = manifest_path
        self.output_root = Path(output_root)
        self.specs = load_model_manifest(manifest_path, workspace_root=self.workspace_root)
        self._adapter_factory = adapter_factory or self._build_default_adapter_factory(model_loader)
        self._resource_probe = resource_probe
        self._adapters: dict[str, Any] = {}
        self._adapter_lock = threading.Lock()
        self._inference_lock = threading.Lock()

    def _build_default_adapter_factory(self, model_loader: ModelLoader | None) -> AdapterFactory:
        def _factory(spec: ModelTaskSpec) -> Any:
            task_model_loader = model_loader or _yolo_loader_for_task(spec.task_type)
            registry = YoloModelRegistry(
                model_loader=task_model_loader,
                model_paths={spec.task_type: spec.model_path},
                prefer_tensorrt=True,
            )
            if spec.task_type == TASK_TYPE_GAUGE:
                return GaugeTaskAdapter(registry)
            return YoloTaskAdapter(registry)

        return _factory

    def list_tasks(self) -> list[dict[str, Any]]:
        return [
            {
                "task_name": spec.task_name,
                "task_type": spec.task_type,
                "model_path": spec.model_path,
                "loaded": spec.task_name in self._adapters,
            }
            for spec in self.specs.values()
        ]

    def warmup_all(
        self,
        *,
        min_system_mem_available_mb: float = 2048.0,
        min_gpu_mem_free_mb: float = 2048.0,
    ) -> dict[str, Any]:
        statuses: dict[str, Any] = {}
        for spec in self.specs.values():
            snapshot = self._resource_probe()
            skip_reason = self._resource_skip_reason(
                snapshot,
                min_system_mem_available_mb=min_system_mem_available_mb,
                min_gpu_mem_free_mb=min_gpu_mem_free_mb,
            )
            if skip_reason is not None:
                statuses[spec.task_name] = {
                    "task_name": spec.task_name,
                    "task_type": spec.task_type,
                    "loaded": False,
                    "warmup_error": skip_reason,
                    "system_available_mb": snapshot.system_available_mb,
                    "gpu_free_mb": snapshot.gpu_free_mb,
                }
                continue
            try:
                adapter = self._adapter_for(spec)
                status = dict(adapter.warmup(spec.task_type))
                status.update({"task_name": spec.task_name, "loaded": status.get("warmup_error") is None})
                statuses[spec.task_name] = status
            except Exception as exc:
                statuses[spec.task_name] = {
                    "task_name": spec.task_name,
                    "task_type": spec.task_type,
                    "loaded": False,
                    "warmup_error": str(exc),
                }
        return statuses

    @staticmethod
    def _resource_skip_reason(
        snapshot: ResourceSnapshot,
        *,
        min_system_mem_available_mb: float,
        min_gpu_mem_free_mb: float,
    ) -> str | None:
        if (
            snapshot.system_available_mb is not None
            and min_system_mem_available_mb > 0
            and snapshot.system_available_mb < min_system_mem_available_mb
        ):
            return "system_memory_below_preload_threshold"
        if (
            snapshot.gpu_free_mb is not None
            and min_gpu_mem_free_mb > 0
            and snapshot.gpu_free_mb < min_gpu_mem_free_mb
        ):
            return "gpu_memory_below_preload_threshold"
        return None

    def _adapter_for(self, spec: ModelTaskSpec) -> Any:
        with self._adapter_lock:
            adapter = self._adapters.get(spec.task_name)
            if adapter is None:
                adapter = self._adapter_factory(spec)
                self._adapters[spec.task_name] = adapter
            return adapter

    def run_task(
        self,
        *,
        request_id: str | None,
        task_name: str,
        frame_path: str | Path,
        conf: float = 0.25,
        iou: float = 0.45,
        max_det: int = 100,
        params_json: str | None = None,
    ) -> dict[str, Any]:
        rid = _clean_text(request_id) or ensure_request_id()
        normalized_task_name = _clean_text(task_name)
        if normalized_task_name is None:
            return self._invalid_params(rid, "task_name", "task_name is required")
        spec = self.specs.get(normalized_task_name)
        if spec is None:
            return self._invalid_params(rid, "task_name", f"unknown task_name: {normalized_task_name}")

        resolved_frame_path = _resolve_workspace_path(self.workspace_root, frame_path)
        if not resolved_frame_path.is_file():
            return failure(
                message=f"frame_path not found: {frame_path}",
                code=CODE_NOT_FOUND,
                request_id=rid,
                data={"field": "frame_path", "error_category": "frame_not_found"},
            )

        try:
            params = parse_params_json(params_json)
        except (json.JSONDecodeError, ValueError) as exc:
            return self._invalid_params(rid, "params_json", str(exc))

        run_conf = float(conf) if float(conf or 0.0) > 0.0 else 0.25
        run_iou = float(iou) if float(iou or 0.0) > 0.0 else 0.45
        run_max_det = int(max_det) if int(max_det or 0) > 0 else 100
        device = _clean_text(params.get("device"))

        start = time.perf_counter()
        try:
            adapter = self._adapter_for(spec)
            with self._inference_lock:
                adapter_payload = adapter.run(
                    spec.task_type,
                    str(resolved_frame_path),
                    conf=run_conf,
                    iou=run_iou,
                    max_det=run_max_det,
                    device=device,
                )
        except Exception as exc:
            latency_ms = (time.perf_counter() - start) * 1000.0
            return failure(
                message=str(exc),
                code=CODE_DOWNSTREAM_ERROR,
                request_id=rid,
                data={
                    "task_name": normalized_task_name,
                    "task_type": spec.task_type,
                    "error_category": exc.__class__.__name__.lower(),
                    "latency_ms": latency_ms,
                },
            )

        latency_ms = (time.perf_counter() - start) * 1000.0
        detail = self._build_detail(
            request_id=rid,
            spec=spec,
            frame_path=resolved_frame_path,
            adapter_payload=adapter_payload,
            params=params,
            conf=run_conf,
            iou=run_iou,
            max_det=run_max_det,
            latency_ms=latency_ms,
        )
        result_json_path = self._write_detail_json(spec.task_name, rid, detail)
        data = self._summary_data(
            spec=spec,
            frame_path=resolved_frame_path,
            result_json_path=result_json_path,
            detail=detail,
        )
        errors = list(detail.get("errors") or [])
        if errors and not detail.get("items"):
            return failure(message="model task failed", code=CODE_DOWNSTREAM_ERROR, request_id=rid, data=data)
        return success(message="model task completed", code=CODE_OK, request_id=rid, data=data)

    def _invalid_params(self, request_id: str, field: str, message: str) -> dict[str, Any]:
        return failure(
            message=message,
            code=CODE_INVALID_PARAMS,
            request_id=request_id,
            data={"field": field, "error_category": "invalid_request"},
        )

    def _build_detail(
        self,
        *,
        request_id: str,
        spec: ModelTaskSpec,
        frame_path: Path,
        adapter_payload: dict[str, Any],
        params: dict[str, Any],
        conf: float,
        iou: float,
        max_det: int,
        latency_ms: float,
    ) -> dict[str, Any]:
        items = list(adapter_payload.get("items") or [])
        errors = list(adapter_payload.get("errors") or [])
        meta = dict(adapter_payload.get("meta") or {})
        meta.update(
            {
                "request_id": request_id,
                "task_name": spec.task_name,
                "request_params": params,
                "conf": conf,
                "iou": iou,
                "max_det": max_det,
                "latency_ms": latency_ms,
            }
        )
        detail: dict[str, Any] = {
            "request_id": request_id,
            "task_name": spec.task_name,
            "task_type": spec.task_type,
            "frame_path": _workspace_relative(self.workspace_root, frame_path),
            "model_path": _workspace_relative(self.workspace_root, meta.get("model_path") or spec.model_path),
            "items": items,
            "meta": meta,
            "errors": errors,
        }

        if spec.task_name == RUST_TASK_NAME:
            self._attach_rust_result(detail=detail, frame_path=frame_path, params=params, latency_ms=latency_ms)
        detail["meta"]["item_count"] = len(detail.get("items") or [])
        detail["meta"]["error_count"] = len(detail.get("errors") or [])
        return detail

    def _attach_rust_result(
        self,
        *,
        detail: dict[str, Any],
        frame_path: Path,
        params: dict[str, Any],
        latency_ms: float,
    ) -> None:
        try:
            import cv2

            image = cv2.imread(str(frame_path))
            if image is None:
                raise ValueError(f"cannot read frame image: {frame_path}")
            height, width = image.shape[:2]
            roi_polygon = parse_roi_polygon(_clean_text(params.get("roi_polygon")), width, height)
            min_area = int(params.get("min_area") or 0)
            detail["rust"] = build_rust_result(
                image_name=frame_path.name,
                model_path=str(detail["model_path"]),
                task_type=str(detail["task_type"]),
                items=detail.get("items") or [],
                width=width,
                height=height,
                min_area=min_area,
                roi_polygon=roi_polygon,
                adapter_meta=detail.get("meta") or {},
                errors=detail.get("errors") or [],
                processing_seconds=latency_ms / 1000.0,
            )
        except Exception as exc:
            errors = list(detail.get("errors") or [])
            errors.append({"error_category": "rust_postprocess_error", "stage": "postprocess", "message": str(exc)})
            detail["errors"] = errors

    def _write_detail_json(self, task_name: str, request_id: str, detail: dict[str, Any]) -> Path:
        safe_request_id = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in request_id)
        output_path = (
            _resolve_workspace_path(self.workspace_root, self.output_root) / "results" / task_name / f"{safe_request_id}.json"
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json_dumps(detail), encoding="utf-8")
        return output_path

    def _summary_data(
        self,
        *,
        spec: ModelTaskSpec,
        frame_path: Path,
        result_json_path: Path,
        detail: dict[str, Any],
    ) -> dict[str, Any]:
        meta = dict(detail.get("meta") or {})
        errors = list(detail.get("errors") or [])
        first_error = errors[0] if errors else {}
        return {
            "task_name": spec.task_name,
            "task_type": spec.task_type,
            "model_path": detail.get("model_path") or _workspace_relative(self.workspace_root, spec.model_path),
            "backend": str(meta.get("backend") or ""),
            "frame_path": _workspace_relative(self.workspace_root, frame_path),
            "result_json_path": _workspace_relative(self.workspace_root, result_json_path),
            "item_count": int(meta.get("item_count") or 0),
            "error_count": int(meta.get("error_count") or 0),
            "latency_ms": float(meta.get("latency_ms") or 0.0),
            "error_category": str(first_error.get("error_category") or ""),
            "detail": detail,
        }


def _yolo_loader_for_task(task_type: str) -> ModelLoader:
    yolo_task = {
        "det": "detect",
        "kpt": "pose",
        "seg": "segment",
        "gauge": "pose",
    }[parse_task_type(task_type)]

    def _load(model_path: str) -> Any:
        from ultralytics import YOLO

        return YOLO(model_path, task=yolo_task)

    return _load
