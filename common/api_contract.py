"""Unified API response envelope helpers.

Handler integration example:

    request_id = ensure_request_id(self.headers)
    payload = success(message="服务可用", data={"health": "ok"}, request_id=request_id)
    send_json(self, payload, status=200)

Business-noop policy (e.g. expected but not executed action):
return HTTP 200 with ok=false and code=BUSINESS_NOOP.
"""

from __future__ import annotations

import json
from typing import Any, Mapping, Optional
from uuid import uuid4


CODE_OK = "OK"
CODE_INVALID_PARAMS = "INVALID_PARAMS"
CODE_NOT_FOUND = "NOT_FOUND"
CODE_DOWNSTREAM_ERROR = "DOWNSTREAM_ERROR"
CODE_BUSINESS_NOOP = "BUSINESS_NOOP"


def ensure_request_id(headers: Optional[Mapping[str, str]] = None) -> str:
    """Return request id from headers if provided, otherwise generate one.

    Header lookup is case-insensitive and prioritizes `X-Request-ID`.
    """
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


def send_json(handler: Any, payload: Mapping[str, Any], *, status: int = 200) -> None:
    """Write JSON response using existing http.server handler.

    Business-noop policy: for expected-but-not-executed flows we return
    `status=200` with `ok=false` and code `BUSINESS_NOOP`.
    """
    body = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)
