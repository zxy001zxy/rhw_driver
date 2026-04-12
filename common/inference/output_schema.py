from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from common.api_contract import CODE_BUSINESS_NOOP, CODE_DOWNSTREAM_ERROR, CODE_OK


def build_inference_payload(
    task_type: str,
    items: list[dict[str, Any]],
    errors: list[dict[str, Any]],
    *,
    model_path: str,
    coordinate_system: str,
) -> dict[str, Any]:
    normalized_items: list[dict[str, Any]] = []
    for item in items:
        copied = dict(item)
        if task_type == "seg":
            copied.pop("rle", None)
        normalized_items.append(copied)

    item_count = len(normalized_items)
    error_count = len(errors)
    partial_success = item_count > 0 and error_count > 0

    return {
        "task_type": task_type,
        "items": normalized_items,
        "meta": {
            "request_ts": datetime.now(timezone.utc).isoformat(),
            "model_path": model_path,
            "coordinate_system": coordinate_system,
            "item_count": item_count,
            "error_count": error_count,
            "partial_success": partial_success,
        },
        "errors": list(errors),
    }


def map_phase6_http_status(payload: dict[str, Any]) -> tuple[int, str]:
    meta = payload.get("meta", {})
    item_count = int(meta.get("item_count", len(payload.get("items", []))))
    error_count = int(meta.get("error_count", len(payload.get("errors", []))))

    if item_count > 0:
        return 200, CODE_OK
    if error_count > 0:
        return 502, CODE_DOWNSTREAM_ERROR
    return 200, CODE_BUSINESS_NOOP
