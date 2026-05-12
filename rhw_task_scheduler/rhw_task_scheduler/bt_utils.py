"""行为树运行辅助函数。"""
from __future__ import annotations

import json
import re
from typing import Any


def parse_task_params(waypoint: dict[str, Any] | None) -> dict[str, Any]:
    if waypoint is None:
        return {}
    raw = waypoint.get('task_params', '')
    if not raw:
        return {}
    try:
        parsed = json.loads(raw)
        return parsed if isinstance(parsed, dict) else {}
    except (json.JSONDecodeError, TypeError):
        return {}


def safe_slug(value: str, *, fallback: str = 'tree') -> str:
    normalized = re.sub(r'[^0-9A-Za-z_.-]+', '_', str(value or fallback)).strip('._')
    return normalized or fallback
