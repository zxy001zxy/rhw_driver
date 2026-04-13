"""调试辅助函数：mock 行为、参数解析、树文件名处理。"""
from __future__ import annotations

import json
import re
import time
from typing import Any, Callable

import py_trees


def is_debug_mock_enabled(node: Any) -> bool:
    return bool(node.get_parameter('debug_mock_enabled').value)


def get_debug_delay_sec(node: Any) -> float:
    return max(float(node.get_parameter('debug_mock_delay_sec').value), 0.0)


def parse_debug_result(value: str, *, default: str = 'success') -> py_trees.common.Status:
    normalized = str(value or default).strip().lower()
    if normalized in {'success', 'ok', 'pass', 'done', 'completed'}:
        return py_trees.common.Status.SUCCESS
    if normalized in {'running', 'pending', 'wait'}:
        return py_trees.common.Status.RUNNING
    if normalized in {'failure', 'failed', 'error'}:
        return py_trees.common.Status.FAILURE
    return py_trees.common.Status.SUCCESS if default == 'success' else py_trees.common.Status.FAILURE


def run_mock_action(
    *,
    node: Any,
    start_time: float | None,
    result_parameter: str,
    default_result: str = 'success',
    on_success: Callable[[], None] | None = None,
    on_failure: Callable[[], None] | None = None,
) -> py_trees.common.Status:
    if start_time is None:
        return py_trees.common.Status.RUNNING

    delay_sec = get_debug_delay_sec(node)
    if (time.monotonic() - start_time) < delay_sec:
        return py_trees.common.Status.RUNNING

    configured = str(node.get_parameter(result_parameter).value)
    status = parse_debug_result(configured, default=default_result)
    if status == py_trees.common.Status.SUCCESS and on_success is not None:
        on_success()
    if status == py_trees.common.Status.FAILURE and on_failure is not None:
        on_failure()
    return status


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