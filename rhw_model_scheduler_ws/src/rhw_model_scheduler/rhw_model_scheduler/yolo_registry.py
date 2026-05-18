from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import Any, Callable

from rhw_model_scheduler.contracts import parse_task_type


ModelLoader = Callable[[str], Any]


def _default_model_loader(model_path: str) -> Any:
    from ultralytics import YOLO

    return YOLO(model_path)


class YoloModelRegistry:
    def __init__(
        self,
        model_loader: ModelLoader | None = None,
        *,
        model_paths: dict[str, str] | None = None,
        engine_paths: dict[str, str] | None = None,
        prefer_tensorrt: bool = True,
    ) -> None:
        self._model_loader = model_loader or _default_model_loader
        self._model_paths = {
            parse_task_type(task_type): str(model_path)
            for task_type, model_path in (model_paths or {}).items()
            if str(model_path or "").strip()
        }
        self._engine_paths = {
            parse_task_type(task_type): str(engine_path)
            for task_type, engine_path in (engine_paths or {}).items()
            if str(engine_path or "").strip()
        }
        self.prefer_tensorrt = bool(prefer_tensorrt)
        self._cache: dict[str, Any] = {}
        self._resolved_paths: dict[str, str] = {}
        self._load_errors: dict[str, str] = {}

    def get_model_path(self, task_type: str) -> str:
        normalized_task = parse_task_type(task_type)
        configured_path = self._model_paths[normalized_task]
        if self.prefer_tensorrt:
            engine_path = self._candidate_engine_path(normalized_task, configured_path)
            if engine_path and Path(engine_path).is_file():
                return engine_path
        return configured_path

    def get_model_status(self, task_type: str) -> dict[str, Any]:
        normalized_task = parse_task_type(task_type)
        configured_path = self._model_paths[normalized_task]
        engine_path = self._candidate_engine_path(normalized_task, configured_path)
        resolved_path = self._resolved_paths.get(normalized_task) or self.get_model_path(normalized_task)
        class_names = self._class_names_for_task(normalized_task)
        return {
            "task_type": normalized_task,
            "configured_model_path": configured_path,
            "engine_path": engine_path,
            "resolved_model_path": resolved_path,
            "backend": self._backend_for_path(resolved_path),
            "prefer_tensorrt": self.prefer_tensorrt,
            "engine_available": bool(engine_path and Path(engine_path).is_file()),
            "loaded": normalized_task in self._cache,
            "load_error": self._load_errors.get(normalized_task),
            "class_names": class_names,
            "class_count": len(class_names),
        }

    def _class_names_for_task(self, task_type: str) -> list[dict[str, Any]]:
        model = self._cache.get(task_type)
        if model is None:
            return []
        names = getattr(model, "names", None)
        if names is None:
            inner_model = getattr(model, "model", None)
            names = getattr(inner_model, "names", None)
        return _normalize_model_names(names)

    @staticmethod
    def _backend_for_path(model_path: str) -> str:
        suffix = Path(model_path).suffix.lower()
        if suffix == ".engine":
            return "tensorrt"
        if suffix == ".onnx":
            return "onnx"
        if suffix == ".pt":
            return "pytorch"
        return suffix.lstrip(".") or "unknown"

    def _candidate_engine_path(self, task_type: str, model_path: str) -> str | None:
        explicit_path = self._engine_paths.get(task_type)
        if explicit_path:
            return explicit_path
        raw_path = Path(model_path)
        if raw_path.suffix.lower() == ".engine":
            return str(raw_path)
        if raw_path.suffix:
            return str(raw_path.with_suffix(".engine"))
        return None

    def get_model(self, task_type: str) -> Any:
        normalized_task = parse_task_type(task_type)
        model = self._cache.get(normalized_task)
        if model is None:
            model_path = self.get_model_path(normalized_task)
            try:
                model = self._model_loader(model_path)
            except Exception as exc:
                self._load_errors[normalized_task] = str(exc)
                raise
            self._cache[normalized_task] = model
            self._resolved_paths[normalized_task] = model_path

        eval_target = getattr(model, "model", model)
        eval_method = getattr(eval_target, "eval", None)
        if callable(eval_method):
            eval_method()
        return model


def _normalize_model_names(names: Any) -> list[dict[str, Any]]:
    if isinstance(names, dict):
        normalized: list[dict[str, Any]] = []
        for raw_class_id, raw_name in names.items():
            try:
                class_id: int | str = int(raw_class_id)
            except (TypeError, ValueError):
                class_id = str(raw_class_id)
            normalized.append({"class_id": class_id, "class_name": str(raw_name)})
        return sorted(normalized, key=lambda item: (isinstance(item["class_id"], str), item["class_id"]))
    if isinstance(names, Sequence) and not isinstance(names, (str, bytes, bytearray)):
        return [{"class_id": index, "class_name": str(name)} for index, name in enumerate(names)]
    return []
