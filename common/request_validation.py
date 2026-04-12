"""Strict JSON body parsing and payload validation utilities.

Handler integration example:

    request_id = ensure_request_id(self.headers)
    try:
        body = parse_json_body(self)
        data = validate_payload(...)
    except ValidationError as exc:
        send_json(
            self,
            failure(
                message=str(exc),
                code=CODE_INVALID_PARAMS,
                request_id=request_id,
                data={"field": exc.field},
            ),
            status=400,
        )
        return
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Optional


@dataclass
class ValidationError(Exception):
    field: str
    message: str
    value: Any = None

    def __str__(self) -> str:
        if self.field:
            return f"{self.field}: {self.message}"
        return self.message


def parse_json_body(handler: Any) -> dict[str, Any]:
    content_len = int(handler.headers.get("Content-Length", "0"))
    if content_len <= 0:
        return {}
    raw = handler.rfile.read(content_len)
    try:
        data = json.loads(raw.decode("utf-8"))
    except json.JSONDecodeError as exc:
        raise ValidationError(field="body", message=f"invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ValidationError(field="body", message="JSON body must be object", value=type(data).__name__)
    return data


def _is_expected_type(value: Any, expected: Any) -> bool:
    if isinstance(expected, tuple):
        return any(_is_expected_type(value, item) for item in expected)
    if expected is int:
        return isinstance(value, int) and not isinstance(value, bool)
    return isinstance(value, expected)


def validate_payload(
    payload: Mapping[str, Any],
    *,
    allowed: Optional[Iterable[str]] = None,
    required: Optional[Iterable[str]] = None,
    typed: Optional[Mapping[str, Any]] = None,
    clamp: Optional[Mapping[str, tuple[Optional[float], Optional[float]]]] = None,
) -> dict[str, Any]:
    allowed_set = set(allowed or [])
    required_set = set(required or [])
    typed_map = dict(typed or {})
    clamp_map = dict(clamp or {})

    if allowed_set:
        unknown = sorted(set(payload.keys()) - allowed_set)
        if unknown:
            raise ValidationError(field=unknown[0], message=f"unknown field: {unknown[0]}", value=payload.get(unknown[0]))

    normalized: dict[str, Any] = dict(payload)

    for field in sorted(required_set):
        if field not in normalized:
            raise ValidationError(field=field, message=f"{field} is required")

    for field, expected_type in typed_map.items():
        if field not in normalized or normalized[field] is None:
            continue
        value = normalized[field]
        if not _is_expected_type(value, expected_type):
            raise ValidationError(
                field=field,
                message=f"{field} must be {expected_type.__name__}",
                value=value,
            )

    for field, (min_value, max_value) in clamp_map.items():
        if field not in normalized or normalized[field] is None:
            continue
        value = normalized[field]
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ValidationError(field=field, message=f"{field} must be numeric for clamp", value=value)
        if min_value is not None and value < min_value:
            value = min_value
        if max_value is not None and value > max_value:
            value = max_value
        if isinstance(normalized[field], int):
            value = int(value)
        normalized[field] = value

    return normalized
