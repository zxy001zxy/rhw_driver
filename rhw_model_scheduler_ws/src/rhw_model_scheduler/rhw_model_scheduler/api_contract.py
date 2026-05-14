from __future__ import annotations

import json
from typing import Any, Mapping, Optional
from uuid import uuid4


CODE_OK = "OK"
CODE_INVALID_PARAMS = "INVALID_PARAMS"
CODE_NOT_FOUND = "NOT_FOUND"
CODE_DOWNSTREAM_ERROR = "DOWNSTREAM_ERROR"


def ensure_request_id(headers: Optional[Mapping[str, str]] = None) -> str:
    if headers:
        for key in ("X-Request-ID", "x-request-id"):
            value = headers.get(key)
            if value:
                return str(value).strip()
    return uuid4().hex


def success(
    *,
    message: str,
    data: Any = None,
    request_id: Optional[str] = None,
    code: str = CODE_OK,
) -> dict[str, Any]:
    rid = request_id or ensure_request_id()
    return {
        "ok": True,
        "code": code,
        "message": message,
        "data": data,
        "request_id": rid,
    }


def failure(
    *,
    message: str,
    code: str,
    request_id: Optional[str] = None,
    data: Any = None,
) -> dict[str, Any]:
    rid = request_id or ensure_request_id()
    return {
        "ok": False,
        "code": code,
        "message": message,
        "data": data,
        "request_id": rid,
    }


def json_dumps(data: Any, *, indent: int | None = 2) -> str:
    return json.dumps(data, ensure_ascii=False, indent=indent)
