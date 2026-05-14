from __future__ import annotations

from typing import Any, TypedDict


TASK_TYPE_DET = "det"
TASK_TYPE_KPT = "kpt"
TASK_TYPE_SEG = "seg"
TASK_TYPE_GAUGE = "gauge"
TASK_TYPES = {TASK_TYPE_DET, TASK_TYPE_KPT, TASK_TYPE_SEG, TASK_TYPE_GAUGE}


class InferenceError(TypedDict):
    error_category: str
    stage: str
    message: str


class InferencePayload(TypedDict):
    task_type: str
    items: list[dict[str, Any]]
    meta: dict[str, Any]
    errors: list[InferenceError]


def parse_task_type(value: str) -> str:
    task_type = str(value).strip().lower()
    if task_type not in TASK_TYPES:
        raise ValueError("task_type must be one of: det,kpt,seg,gauge")
    return task_type
