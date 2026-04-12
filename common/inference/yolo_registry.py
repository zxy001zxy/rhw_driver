from __future__ import annotations

import os
from typing import Any, Callable

from common.inference.contracts import TASK_TYPE_DET, TASK_TYPE_KPT, TASK_TYPE_SEG, parse_task_type

ModelLoader = Callable[[str], Any]

_MODEL_CONFIG: dict[str, dict[str, str]] = {
    TASK_TYPE_DET: {
        "env": "YOLO_DET_MODEL_PATH",
        "default": "models/yolov8n.pt",
    },
    TASK_TYPE_KPT: {
        "env": "YOLO_KPT_MODEL_PATH",
        "default": "models/yolov8n-pose.pt",
    },
    TASK_TYPE_SEG: {
        "env": "YOLO_SEG_MODEL_PATH",
        "default": "models/yolov8n-seg.pt",
    },
}


def _default_model_loader(model_path: str) -> Any:
    from ultralytics import YOLO

    return YOLO(model_path)


class YoloModelRegistry:
    def __init__(self, model_loader: ModelLoader | None = None) -> None:
        self._model_loader = model_loader or _default_model_loader
        self._cache: dict[str, Any] = {}

    def get_model_path(self, task_type: str) -> str:
        normalized_task = parse_task_type(task_type)
        config = _MODEL_CONFIG[normalized_task]
        configured_path = os.getenv(config["env"], config["default"]).strip()
        return configured_path or config["default"]

    def get_model(self, task_type: str) -> Any:
        normalized_task = parse_task_type(task_type)
        model = self._cache.get(normalized_task)
        if model is None:
            model = self._model_loader(self.get_model_path(normalized_task))
            self._cache[normalized_task] = model

        eval_target = getattr(model, "model", model)
        eval_method = getattr(eval_target, "eval", None)
        if callable(eval_method):
            eval_method()
        return model
