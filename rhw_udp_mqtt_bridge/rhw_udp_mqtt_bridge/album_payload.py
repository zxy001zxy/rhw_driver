"""Helpers for building inspection album upload payloads."""
from __future__ import annotations

import hashlib
import json
from collections.abc import Callable
from typing import Any


def build_album_payload(
    *,
    trace_id: str,
    partner_id: str,
    version: str,
    device_id: str,
    image_base64: str,
    task_id: str,
    point_name: str,
    point_id: str,
    encryption_enabled: bool,
    encrypt_data: Callable[[str], str],
    signature_enabled: bool,
    fixed_signature: str,
    signature_secret: str,
    include_device_id: bool,
) -> dict[str, Any]:
    data_plain: dict[str, Any] = {
        'taskId': str(task_id),
        'pointName': str(point_name or point_id),
        'pointId': str(point_id),
        'base64': str(image_base64),
    }
    if include_device_id:
        data_plain = {'deviceId': str(device_id), **data_plain}

    data_text = json.dumps(data_plain, ensure_ascii=False, separators=(',', ':'))
    data_field: str | dict[str, Any]
    data_field = encrypt_data(data_text) if encryption_enabled else data_plain

    return {
        'traceId': str(trace_id),
        'partnerId': str(partner_id),
        'version': str(version),
        'data': data_field,
        'signature': build_signature(
            trace_id=str(trace_id),
            data_field=data_field,
            signature_enabled=signature_enabled,
            fixed_signature=str(fixed_signature),
            signature_secret=str(signature_secret),
        ),
    }


def build_signature(
    *,
    trace_id: str,
    data_field: str | dict[str, Any],
    signature_enabled: bool,
    fixed_signature: str,
    signature_secret: str,
) -> str:
    if fixed_signature:
        return fixed_signature
    if not signature_enabled:
        return ''
    if not signature_secret:
        raise ValueError('signature_secret is required when signature_enabled=true')
    data_text = (
        data_field
        if isinstance(data_field, str)
        else json.dumps(data_field, ensure_ascii=False, separators=(',', ':'))
    )
    source = f'{trace_id}{data_text}{signature_secret}'
    return hashlib.md5(source.encode('utf-8')).hexdigest()
