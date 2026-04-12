from __future__ import annotations

from dataclasses import dataclass
import json
import threading
import time
from typing import Any, Dict, Optional, Tuple
import xml.etree.ElementTree as ET

import requests
from requests.auth import HTTPDigestAuth
from requests.exceptions import RequestException


class PtzError(Exception):
    """Raised when PTZ control fails."""

    def __init__(self, message: str, *, category: str = "unknown") -> None:
        super().__init__(message)
        self.category = category


SUPPORTED_DIRECTIONS = {
    "left",
    "right",
    "up",
    "down",
    "leftup",
    "rightup",
    "leftdown",
    "rightdown",
    "zoomin",
    "zoomout",
    "stop",
}


def _normalize_direction(direction: str) -> str:
    normalized = direction.strip().lower()
    if not normalized:
        raise PtzError("direction is required", category="invalid_direction")
    if normalized not in SUPPORTED_DIRECTIONS:
        raise PtzError(f"unsupported direction: {direction}", category="invalid_direction")
    return normalized


def _normalize_speed(speed: int) -> int:
    value = int(speed)
    if value < 1 or value > 100:
        raise PtzError("speed must be between 1 and 100", category="invalid_speed")
    return value


def _normalize_channel(channel: int) -> int:
    value = int(channel)
    if value <= 0:
        raise PtzError("channel must be >= 1", category="invalid_channel")
    return value


def _normalize_duration_ms(duration_ms: int) -> int:
    value = int(duration_ms)
    if value < 0:
        raise PtzError("duration_ms must be >= 0", category="invalid_duration")
    return value


def _local_name(tag: str) -> str:
    if "}" in tag:
        return tag.split("}", 1)[1]
    return tag


def _find_text(root: ET.Element, name: str) -> str | None:
    for elem in root.iter():
        if _local_name(elem.tag) == name:
            if elem.text is None:
                return None
            return elem.text.strip()
    return None


def _to_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _json_find_number(data: Any, key: str) -> float | None:
    if isinstance(data, dict):
        if key in data:
            parsed = _to_float(data.get(key))
            if parsed is not None:
                return parsed
        for value in data.values():
            found = _json_find_number(value, key)
            if found is not None:
                return found
    if isinstance(data, list):
        for item in data:
            found = _json_find_number(item, key)
            if found is not None:
                return found
    return None


def _number_literal(value: float) -> int | float:
    if float(value).is_integer():
        return int(value)
    return float(value)


def _number_text(value: float) -> str:
    literal = _number_literal(value)
    return str(literal)


def _direction_to_axes(direction: str, speed: int) -> Tuple[int, int, int]:
    ptz_speed = _normalize_speed(speed)
    normalized = _normalize_direction(direction)

    pan = 0
    tilt = 0
    zoom = 0

    if normalized == "left":
        pan = -ptz_speed
    elif normalized == "right":
        pan = ptz_speed
    elif normalized == "up":
        tilt = ptz_speed
    elif normalized == "down":
        tilt = -ptz_speed
    elif normalized == "leftup":
        pan, tilt = -ptz_speed, ptz_speed
    elif normalized == "rightup":
        pan, tilt = ptz_speed, ptz_speed
    elif normalized == "leftdown":
        pan, tilt = -ptz_speed, -ptz_speed
    elif normalized == "rightdown":
        pan, tilt = ptz_speed, -ptz_speed
    elif normalized == "zoomin":
        zoom = ptz_speed
    elif normalized == "zoomout":
        zoom = -ptz_speed
    elif normalized == "stop":
        pass
    return pan, tilt, zoom


@dataclass
class PtzController:
    ip: str
    username: str
    password: str
    port: int = 80
    use_https: bool = False
    verify_ssl: bool = False
    timeout: float = 5.0

    @property
    def base_url(self) -> str:
        scheme = "https" if self.use_https else "http"
        return f"{scheme}://{self.ip}:{self.port}"

    def _url(self, channel: int) -> str:
        return f"{self.base_url}/ISAPI/PTZCtrl/channels/{channel}/continuous"

    def _request(
        self,
        *,
        method: str,
        url: str,
        data: bytes | None = None,
        headers: Optional[Dict[str, str]] = None,
    ) -> requests.Response:
        normalized_method = method.upper()
        try:
            kwargs = {
                "data": data,
                "headers": headers or {},
                "auth": HTTPDigestAuth(self.username, self.password),
                "timeout": self.timeout,
                "verify": self.verify_ssl if self.use_https else True,
            }
            if normalized_method == "PUT":
                return requests.put(url, **kwargs)
            if normalized_method == "GET":
                return requests.get(url, **kwargs)
            return requests.request(method=normalized_method, url=url, **kwargs)
        except RequestException as exc:
            raise PtzError(f"ptz request failed: {exc}", category="request_failed") from exc

    def _parse_status_body(self, response: requests.Response) -> Dict[str, object]:
        output: Dict[str, object] = {}
        try:
            root = ET.fromstring(response.text) if response.text else None
            if root is not None:
                status_string = _find_text(root, "statusString")
                sub_status_code = _find_text(root, "subStatusCode")
                if status_string is not None:
                    output["status_string"] = status_string
                if sub_status_code is not None:
                    output["sub_status_code"] = sub_status_code
        except ET.ParseError:
            if response.text:
                output["raw_body"] = response.text
        return output

    def _request_body(self, direction: str, speed: int) -> str:
        pan, tilt, zoom = _direction_to_axes(direction=direction, speed=speed)
        return (
            '<?xml version="1.0" encoding="UTF-8"?>'
            '<PTZData version="2.0" xmlns="http://www.isapi.org/ver20/XMLSchema">'
            f"<pan>{pan}</pan>"
            f"<tilt>{tilt}</tilt>"
            f"<zoom>{zoom}</zoom>"
            "</PTZData>"
        )

    def control_once(self, *, direction: str, speed: int = 40, channel: int = 1) -> Dict[str, object]:
        normalized_direction = _normalize_direction(direction)
        normalized_speed = _normalize_speed(speed)
        normalized_channel = _normalize_channel(channel)

        payload = self._request_body(direction=normalized_direction, speed=normalized_speed)
        url = self._url(channel=normalized_channel)
        response = self._request(
            method="PUT",
            url=url,
            data=payload.encode("utf-8"),
            headers={"Content-Type": "application/xml; charset=UTF-8"},
        )

        output: Dict[str, object] = {
            "ok": response.status_code in (200, 201, 202, 204),
            "status_code": response.status_code,
            "direction": normalized_direction,
            "speed": normalized_speed,
            "channel": normalized_channel,
            "url": url,
        }

        output.update(self._parse_status_body(response))

        if not output["ok"]:
            output["error"] = f"unexpected status code: {response.status_code}"
            output["error_category"] = "downstream_status"

        return output

    def control(
        self,
        *,
        direction: str,
        speed: int = 40,
        channel: int = 1,
        duration_ms: int = 350,
    ) -> Dict[str, object]:
        normalized = _normalize_direction(direction)
        normalized_speed = _normalize_speed(speed)
        normalized_channel = _normalize_channel(channel)
        normalized_duration_ms = _normalize_duration_ms(duration_ms)

        first = self.control_once(direction=normalized, speed=normalized_speed, channel=normalized_channel)

        auto_stop = None
        execution_mode = "single"
        if normalized != "stop" and normalized_duration_ms > 0 and bool(first.get("ok")):
            execution_mode = "timed_auto_stop"

            def _auto_stop() -> None:
                time.sleep(normalized_duration_ms / 1000.0)
                try:
                    self.control_once(direction="stop", speed=normalized_speed, channel=normalized_channel)
                except Exception:
                    pass

            thread = threading.Thread(target=_auto_stop, daemon=True, name="ptz-auto-stop")
            thread.start()
            auto_stop = {
                "scheduled": True,
                "duration_ms": normalized_duration_ms,
            }
        elif normalized != "stop" and normalized_duration_ms <= 0:
            execution_mode = "continuous_until_manual_stop"

        return {
            "ok": bool(first.get("ok")),
            "execution_mode": execution_mode,
            "command": first,
            "auto_stop": auto_stop,
        }

    def _channel_base(self, channel: int) -> str:
        normalized_channel = _normalize_channel(channel)
        return f"{self.base_url}/ISAPI/PTZCtrl/channels/{normalized_channel}"

    def _common_result(
        self,
        *,
        action: str,
        response: requests.Response,
        extra: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, object]:
        output: Dict[str, object] = {
            "ok": response.status_code in (200, 201, 202, 204),
            "action": action,
            "status_code": response.status_code,
        }
        if extra:
            output.update(extra)
        output.update(self._parse_status_body(response))
        if not output["ok"]:
            output["error"] = f"unexpected status code: {response.status_code}"
            output["error_category"] = "downstream_status"
        return output

    def get_system_capabilities(self) -> Dict[str, object]:
        url = f"{self.base_url}/ISAPI/System/capabilities"
        response = self._request(method="GET", url=url)
        output = self._common_result(action="get_system_capabilities", response=response, extra={"url": url})
        text = response.text or ""
        if output.get("ok"):
            lowered = text.lower()
            output["is_support_patrols"] = "<issupportpatrols>true</issupportpatrols>" in lowered
            output["is_support_time_cap"] = "<issupporttimecap>true</issupporttimecap>" in lowered
            output["is_support_time"] = "<issupporttime>true</issupporttime>" in lowered
        return output

    def get_patrol_capabilities(self, *, channel: int = 1) -> Dict[str, object]:
        normalized_channel = _normalize_channel(channel)
        url = f"{self._channel_base(normalized_channel)}/patrols/capabilities"
        response = self._request(method="GET", url=url)
        output = self._common_result(
            action="get_patrol_capabilities",
            response=response,
            extra={"url": url, "channel": normalized_channel},
        )
        if output.get("ok"):
            body = (response.text or "").lower()
            output["is_support_patrols"] = "issupportpatrols" in body and ">true<" in body
        return output

    def goto_preset(self, *, channel: int = 1, preset_id: int = 1) -> Dict[str, object]:
        normalized_channel = _normalize_channel(channel)
        normalized_preset = int(preset_id)
        if normalized_preset <= 0:
            raise PtzError("preset_id must be >= 1", category="invalid_preset")
        url = f"{self._channel_base(normalized_channel)}/presets/{normalized_preset}/goto"
        response = self._request(method="PUT", url=url)
        return self._common_result(
            action="goto_preset",
            response=response,
            extra={"url": url, "channel": normalized_channel, "preset_id": normalized_preset},
        )

    def start_patrol(self, *, channel: int = 1, patrol_id: int = 1) -> Dict[str, object]:
        normalized_channel = _normalize_channel(channel)
        normalized_patrol = int(patrol_id)
        if normalized_patrol <= 0:
            raise PtzError("patrol_id must be >= 1", category="invalid_patrol")
        url = f"{self._channel_base(normalized_channel)}/patrols/{normalized_patrol}/start"
        response = self._request(method="PUT", url=url)
        return self._common_result(
            action="start_patrol",
            response=response,
            extra={"url": url, "channel": normalized_channel, "patrol_id": normalized_patrol},
        )

    def stop_patrol(self, *, channel: int = 1, patrol_id: int = 1) -> Dict[str, object]:
        normalized_channel = _normalize_channel(channel)
        normalized_patrol = int(patrol_id)
        if normalized_patrol <= 0:
            raise PtzError("patrol_id must be >= 1", category="invalid_patrol")
        url = f"{self._channel_base(normalized_channel)}/patrols/{normalized_patrol}/stop"
        response = self._request(method="PUT", url=url)
        return self._common_result(
            action="stop_patrol",
            response=response,
            extra={"url": url, "channel": normalized_channel, "patrol_id": normalized_patrol},
        )

    def get_channel_capabilities(self, *, channel: int = 1) -> Dict[str, object]:
        normalized_channel = _normalize_channel(channel)
        url = f"{self._channel_base(normalized_channel)}/capabilities"
        response = self._request(method="GET", url=url)
        output = self._common_result(
            action="get_channel_capabilities",
            response=response,
            extra={"url": url, "channel": normalized_channel},
        )
        if output.get("ok"):
            body = (response.text or "").lower()
            output["is_support_absolute_ex"] = "<issupportabsoluteex>true</issupportabsoluteex>" in body
        return output

    def get_absolute_ex_capabilities(self, *, channel: int = 1, json_format: bool = True) -> Dict[str, object]:
        normalized_channel = _normalize_channel(channel)
        url_json = f"{self._channel_base(normalized_channel)}/absoluteEx/capabilities?format=json"
        url_xml = f"{self._channel_base(normalized_channel)}/absoluteEx/capabilities"
        if json_format:
            response_json = self._request(method="GET", url=url_json)
            output_json = self._common_result(
                action="get_absolute_ex_capabilities",
                response=response_json,
                extra={"url": url_json, "channel": normalized_channel, "transport": "json"},
            )
            if output_json.get("ok"):
                try:
                    output_json["capabilities_json"] = response_json.json()
                except ValueError:
                    pass
                return output_json

            # Some devices reject ?format=json but still support XML capabilities.
            response_xml = self._request(method="GET", url=url_xml)
            output_xml = self._common_result(
                action="get_absolute_ex_capabilities",
                response=response_xml,
                extra={
                    "url": url_xml,
                    "channel": normalized_channel,
                    "transport": "xml",
                    "fallback_from_json": {
                        "status_code": output_json.get("status_code"),
                        "error": output_json.get("error"),
                        "error_category": output_json.get("error_category"),
                    },
                },
            )
            return output_xml

        response_xml = self._request(method="GET", url=url_xml)
        output_xml = self._common_result(
            action="get_absolute_ex_capabilities",
            response=response_xml,
            extra={"url": url_xml, "channel": normalized_channel, "transport": "xml"},
        )
        return output_xml

    def get_absolute_ex(self, *, channel: int = 1, json_format: bool = True) -> Dict[str, object]:
        normalized_channel = _normalize_channel(channel)
        url_json = f"{self._channel_base(normalized_channel)}/absoluteEx?format=json"
        url_xml = f"{self._channel_base(normalized_channel)}/absoluteEx"
        response = None
        output: Dict[str, object]
        if json_format:
            response_json = self._request(method="GET", url=url_json)
            output_json = self._common_result(
                action="get_absolute_ex",
                response=response_json,
                extra={"url": url_json, "channel": normalized_channel, "transport": "json"},
            )
            if output_json.get("ok"):
                response = response_json
                output = output_json
            else:
                response_xml = self._request(method="GET", url=url_xml)
                output_xml = self._common_result(
                    action="get_absolute_ex",
                    response=response_xml,
                    extra={
                        "url": url_xml,
                        "channel": normalized_channel,
                        "transport": "xml",
                        "fallback_from_json": {
                            "status_code": output_json.get("status_code"),
                            "error": output_json.get("error"),
                            "error_category": output_json.get("error_category"),
                        },
                    },
                )
                response = response_xml
                output = output_xml
        else:
            response_xml = self._request(method="GET", url=url_xml)
            output = self._common_result(
                action="get_absolute_ex",
                response=response_xml,
                extra={"url": url_xml, "channel": normalized_channel, "transport": "xml"},
            )
            response = response_xml

        azimuth = None
        elevation = None
        if output.get("ok") and output.get("transport") == "json":
            try:
                payload = response.json() if response is not None else {}
                output["position_json"] = payload
                azimuth = _json_find_number(payload, "azimuth")
                elevation = _json_find_number(payload, "elevation")
            except ValueError:
                pass
        if azimuth is None or elevation is None:
            try:
                root = ET.fromstring(response.text) if response is not None and response.text else None
                if root is not None:
                    if azimuth is None:
                        azimuth = _to_float(_find_text(root, "azimuth"))
                    if elevation is None:
                        elevation = _to_float(_find_text(root, "elevation"))
            except ET.ParseError:
                pass

        if azimuth is not None:
            output["azimuth"] = azimuth
        if elevation is not None:
            output["elevation"] = elevation
        return output

    def _absolute_ex_xml_payload(
        self,
        *,
        root_name: str,
        azimuth: float,
        elevation: float,
        pan_speed_tag: str,
        tilt_speed_tag: str,
        azimuth_speed: int | None,
        elevation_speed: int | None,
    ) -> str:
        parts = [
            '<?xml version="1.0" encoding="UTF-8"?>',
            f'<{root_name} version="2.0" xmlns="http://www.isapi.org/ver20/XMLSchema">',
            f"<azimuth>{_number_text(azimuth)}</azimuth>",
            f"<elevation>{_number_text(elevation)}</elevation>",
        ]
        if azimuth_speed is not None:
            parts.append(f"<{pan_speed_tag}>{azimuth_speed}</{pan_speed_tag}>")
        if elevation_speed is not None:
            parts.append(f"<{tilt_speed_tag}>{elevation_speed}</{tilt_speed_tag}>")
        parts.append(f"</{root_name}>")
        return "".join(parts)

    def move_absolute_ex(
        self,
        *,
        channel: int = 1,
        azimuth: float,
        elevation: float,
        azimuth_speed: int | None = None,
        elevation_speed: int | None = None,
    ) -> Dict[str, object]:
        normalized_channel = _normalize_channel(channel)
        normalized_azimuth = float(azimuth)
        normalized_elevation = float(elevation)
        normalized_azimuth_speed = _normalize_speed(azimuth_speed) if azimuth_speed is not None else None
        normalized_elevation_speed = _normalize_speed(elevation_speed) if elevation_speed is not None else None

        json_url = f"{self._channel_base(normalized_channel)}/absoluteEx?format=json"
        json_body = {
            "AbsoluteEx": {
                "azimuth": _number_literal(normalized_azimuth),
                "elevation": _number_literal(normalized_elevation),
            }
        }
        if normalized_azimuth_speed is not None:
            json_body["AbsoluteEx"]["azimuthSpeed"] = normalized_azimuth_speed
        if normalized_elevation_speed is not None:
            json_body["AbsoluteEx"]["elevationSpeed"] = normalized_elevation_speed

        json_response = self._request(
            method="PUT",
            url=json_url,
            data=json.dumps(json_body, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json; charset=UTF-8"},
        )
        json_result = self._common_result(
            action="move_absolute_ex_json",
            response=json_response,
            extra={
                "url": json_url,
                "channel": normalized_channel,
                "azimuth": normalized_azimuth,
                "elevation": normalized_elevation,
                "azimuth_speed": normalized_azimuth_speed,
                "elevation_speed": normalized_elevation_speed,
            },
        )
        if json_result.get("ok"):
            json_result["transport"] = "json"
            return json_result

        xml_url = f"{self._channel_base(normalized_channel)}/absoluteEx"
        xml_variants = [
            ("AbsoluteEx", "azimuthSpeed", "elevationSpeed"),
            ("PTZAbsoluteEx", "azimuthSpeed", "elevationSpeed"),
            ("PTZAbsoluteExData", "azimuthSpeed", "elevationSpeed"),
            ("PTZAbsoluteEx", "horizontalSpeed", "verticalSpeed"),
            ("PTZAbsoluteExData", "horizontalSpeed", "verticalSpeed"),
        ]

        last_xml_result: Dict[str, object] = {}
        for root_name, pan_speed_tag, tilt_speed_tag in xml_variants:
            xml_payload = self._absolute_ex_xml_payload(
                root_name=root_name,
                azimuth=normalized_azimuth,
                elevation=normalized_elevation,
                pan_speed_tag=pan_speed_tag,
                tilt_speed_tag=tilt_speed_tag,
                azimuth_speed=normalized_azimuth_speed,
                elevation_speed=normalized_elevation_speed,
            )
            xml_response = self._request(
                method="PUT",
                url=xml_url,
                data=xml_payload.encode("utf-8"),
                headers={"Content-Type": "application/xml; charset=UTF-8"},
            )
            xml_result = self._common_result(
                action="move_absolute_ex_xml",
                response=xml_response,
                extra={
                    "url": xml_url,
                    "channel": normalized_channel,
                    "azimuth": normalized_azimuth,
                    "elevation": normalized_elevation,
                    "azimuth_speed": normalized_azimuth_speed,
                    "elevation_speed": normalized_elevation_speed,
                    "xml_root": root_name,
                    "pan_speed_tag": pan_speed_tag,
                    "tilt_speed_tag": tilt_speed_tag,
                    "fallback_from_json": {
                        "status_code": json_result.get("status_code"),
                        "error": json_result.get("error"),
                        "error_category": json_result.get("error_category"),
                    },
                },
            )
            xml_result["transport"] = "xml"
            if xml_result.get("ok"):
                return xml_result
            last_xml_result = xml_result

        return last_xml_result if last_xml_result else json_result

    def get_time_config(self) -> Dict[str, object]:
        url = f"{self.base_url}/ISAPI/System/time"
        response = self._request(method="GET", url=url)
        output = self._common_result(action="get_time_config", response=response, extra={"url": url})
        if not output.get("ok"):
            return output
        try:
            root = ET.fromstring(response.text) if response.text else None
            if root is None:
                return output
            time_mode = _find_text(root, "timeMode")
            local_time = _find_text(root, "localTime")
            time_zone = _find_text(root, "timeZone")
            if time_mode is not None:
                output["time_mode"] = time_mode
            if local_time is not None:
                output["local_time"] = local_time
            if time_zone is not None:
                output["time_zone"] = time_zone
        except ET.ParseError:
            pass
        return output

    def set_time_manual(self, *, local_time: str, time_zone: str | None = None) -> Dict[str, object]:
        normalized_local_time = str(local_time).strip()
        if not normalized_local_time:
            raise PtzError("local_time is required", category="invalid_time")
        normalized_time_zone = str(time_zone).strip() if time_zone is not None else ""
        body = [
            '<?xml version="1.0" encoding="UTF-8"?>',
            '<Time version="2.0" xmlns="http://www.isapi.org/ver20/XMLSchema">',
            "<timeMode>manual</timeMode>",
            f"<localTime>{normalized_local_time}</localTime>",
        ]
        if normalized_time_zone:
            body.append(f"<timeZone>{normalized_time_zone}</timeZone>")
        body.append("</Time>")

        url = f"{self.base_url}/ISAPI/System/time"
        response = self._request(
            method="PUT",
            url=url,
            data="".join(body).encode("utf-8"),
            headers={"Content-Type": "application/xml; charset=UTF-8"},
        )
        return self._common_result(
            action="set_time_manual",
            response=response,
            extra={"url": url, "local_time": normalized_local_time, "time_zone": normalized_time_zone or None},
        )
