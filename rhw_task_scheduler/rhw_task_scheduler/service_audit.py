from __future__ import annotations

import json
import time
from typing import Any

from std_msgs.msg import String


def ros_value_to_python(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return [ros_value_to_python(item) for item in value]
    if isinstance(value, dict):
        return {str(key): ros_value_to_python(val) for key, val in value.items()}
    if hasattr(value, 'get_fields_and_field_types'):
        return {
            field: ros_value_to_python(getattr(value, field))
            for field in value.get_fields_and_field_types().keys()
        }
    if hasattr(value, 'tolist'):
        return value.tolist()
    return str(value)


class ServiceAuditPublisher:
    def __init__(self, node: Any, topic: str = '/service_events') -> None:
        self._node = node
        self._publisher = node.create_publisher(String, topic, 50)
        self._topic = topic

    @property
    def topic(self) -> str:
        return self._topic

    def publish(
        self,
        *,
        service: str,
        role: str,
        phase: str,
        request: Any | None = None,
        response: Any | None = None,
        success: bool | None = None,
        duration_ms: float | None = None,
        details: dict[str, Any] | None = None,
    ) -> None:
        payload = {
            'timestamp': time.time(),
            'node': self._node.get_name(),
            'service': service,
            'role': role,
            'phase': phase,
        }
        if request is not None:
            payload['request'] = ros_value_to_python(request)
        if response is not None:
            payload['response'] = ros_value_to_python(response)
        if success is not None:
            payload['success'] = success
        if duration_ms is not None:
            payload['duration_ms'] = round(float(duration_ms), 3)
        if details:
            payload['details'] = ros_value_to_python(details)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._publisher.publish(msg)
