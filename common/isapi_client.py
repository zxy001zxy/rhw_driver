from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional
import xml.etree.ElementTree as ET

import requests
from requests.auth import HTTPDigestAuth
from requests.exceptions import RequestException


class IsapiError(Exception):
    """Raised when ISAPI connection/auth fails."""


def _local_name(tag: str) -> str:
    if "}" in tag:
        return tag.split("}", 1)[1]
    return tag


def _find_text(root: ET.Element, name: str) -> Optional[str]:
    for elem in root.iter():
        if _local_name(elem.tag) == name:
            if elem.text is None:
                return None
            return elem.text.strip()
    return None


@dataclass
class IsapiClient:
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

    def _get(self, path: str) -> requests.Response:
        url = f"{self.base_url}{path}"
        try:
            response = requests.get(
                url,
                auth=HTTPDigestAuth(self.username, self.password),
                timeout=self.timeout,
                verify=self.verify_ssl if self.use_https else True,
            )
            return response
        except RequestException as exc:
            raise IsapiError(f"network request failed: {exc}") from exc

    def user_check(self) -> Dict[str, Any]:
        response = self._get("/ISAPI/Security/userCheck")
        if response.status_code in (401, 403):
            return {
                "ok": False,
                "status_code": response.status_code,
                "error": "authentication failed",
            }
        if response.status_code != 200:
            return {
                "ok": False,
                "status_code": response.status_code,
                "error": f"unexpected status code: {response.status_code}",
            }

        status_value = None
        try:
            root = ET.fromstring(response.text)
            status_value = _find_text(root, "statusValue")
        except ET.ParseError:
            # Some old firmware may not return strict XML; status 200 is still useful.
            pass

        if status_value is not None and status_value != "200":
            return {
                "ok": False,
                "status_code": response.status_code,
                "error": f"userCheck returned statusValue={status_value}",
            }

        return {
            "ok": True,
            "status_code": response.status_code,
            "status_value": status_value or "unknown",
        }

    def device_info(self) -> Dict[str, Any]:
        response = self._get("/ISAPI/System/deviceInfo")
        if response.status_code != 200:
            return {
                "ok": False,
                "status_code": response.status_code,
                "error": f"deviceInfo failed: {response.status_code}",
            }

        try:
            root = ET.fromstring(response.text)
            return {
                "ok": True,
                "status_code": response.status_code,
                "device_name": _find_text(root, "deviceName"),
                "model": _find_text(root, "model"),
                "serial_number": _find_text(root, "serialNumber"),
                "firmware_version": _find_text(root, "firmwareVersion"),
            }
        except ET.ParseError:
            return {
                "ok": True,
                "status_code": response.status_code,
                "raw_body": response.text,
            }

    def connect(self) -> Dict[str, Any]:
        check = self.user_check()
        if not check["ok"]:
            return {
                "ok": False,
                "message": check["error"],
                "user_check": check,
                "device_info": None,
            }

        info = self.device_info()
        return {
            "ok": True,
            "message": "connected and authenticated",
            "user_check": check,
            "device_info": info,
        }

