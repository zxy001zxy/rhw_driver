"""inspection_reporter_node — 巡检结果 HTTPS 上报节点。"""
from __future__ import annotations

import base64
import binascii
import json
import random
import time
from pathlib import Path
from typing import Any

import rclpy
import requests
from cryptography.hazmat.primitives import padding
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rhw_msgs.msg import InspectionAlbumReport
from rhw_msgs.srv import InspectionAlbumUpload

from .album_payload import build_album_payload


class _AlbumReporterConfigError(ValueError):
    """Raised when album reporter cryptographic/signature config is invalid."""


class InspectionReporterNode(Node):
    """订阅抓拍结果事件，并按平台 HTTPS 接口上报相册结果。"""

    def __init__(self) -> None:
        super().__init__('inspection_reporter_node')

        self._declare_parameters()
        self._read_parameters()
        self._callback_group = ReentrantCallbackGroup()

        self.create_subscription(
            InspectionAlbumReport,
            self._album_report_topic,
            self._on_album_report,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(
            InspectionAlbumUpload,
            self._album_upload_service,
            self._handle_album_upload,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            'inspection_reporter_node started '
            f'enabled={self._enabled} topic={self._album_report_topic} '
            f'upload_service={self._album_upload_service} '
            f'album_url={self._album_report_url or "<empty>"}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('enabled', True)
        self.declare_parameter('album_report_topic', '/inspection/album_reports')
        self.declare_parameter('album_upload_service', '/inspection/album_report/upload')
        self.declare_parameter('device_id', 'DOG001')
        self.declare_parameter('album_report_url', '')
        self.declare_parameter('partner_id', '')
        self.declare_parameter('version', '1.0')
        self.declare_parameter('aes_key', '')
        self.declare_parameter('aes_iv', '')
        self.declare_parameter('signature', '')
        self.declare_parameter('signature_secret', '')
        self.declare_parameter('encryption_enabled', True)
        self.declare_parameter('signature_enabled', True)
        self.declare_parameter('include_device_id', False)
        self.declare_parameter('timeout_sec', 5.0)
        self.declare_parameter('retry_count', 2)
        self.declare_parameter('verify_tls', True)
        self.declare_parameter('debug_log_payload', False)

    def _read_parameters(self) -> None:
        self._enabled = bool(self.get_parameter('enabled').value)
        self._album_report_topic = str(self.get_parameter('album_report_topic').value)
        self._album_upload_service = str(self.get_parameter('album_upload_service').value)
        self._device_id = str(self.get_parameter('device_id').value)
        self._album_report_url = str(self.get_parameter('album_report_url').value)
        self._partner_id = str(self.get_parameter('partner_id').value)
        self._version = str(self.get_parameter('version').value)
        self._aes_key = str(self.get_parameter('aes_key').value)
        self._aes_iv = str(self.get_parameter('aes_iv').value)
        self._fixed_signature = str(self.get_parameter('signature').value)
        self._signature_secret = str(self.get_parameter('signature_secret').value)
        self._encryption_enabled = bool(self.get_parameter('encryption_enabled').value)
        self._signature_enabled = bool(self.get_parameter('signature_enabled').value)
        self._include_device_id = bool(self.get_parameter('include_device_id').value)
        self._timeout_sec = max(float(self.get_parameter('timeout_sec').value), 0.1)
        self._retry_count = max(int(self.get_parameter('retry_count').value), 0)
        self._verify_tls = bool(self.get_parameter('verify_tls').value)
        self._debug_log_payload = bool(self.get_parameter('debug_log_payload').value)

    def _on_album_report(self, msg: InspectionAlbumReport) -> None:
        result = self._report_album(msg)
        if result['ok']:
            return
        self.get_logger().error(
            'Album report topic upload failed: '
            f'task_id={msg.task_id} point_id={msg.point_id} '
            f'code={result["code"]} message={result["message"]}'
        )

    def _album_upload_request_to_msg(
        self,
        request: InspectionAlbumUpload.Request,
    ) -> InspectionAlbumReport:
        msg = InspectionAlbumReport()
        msg.task_id = str(request.task_id)
        msg.point_id = str(request.point_id)
        msg.point_name = str(request.point_name or request.point_id)
        msg.image_path = str(request.image_path)
        msg.capture_url = str(request.capture_url)
        msg.file_size = max(0, min(int(request.file_size), 4294967295))
        return msg

    def _report_album(self, msg: InspectionAlbumReport) -> dict[str, Any]:
        if not self._enabled:
            return {
                'ok': False,
                'code': 'DISABLED',
                'message': 'inspection reporter is disabled',
                'trace_id': '',
                'http_status': 0,
                'response_body': '',
            }
        if not self._album_report_url:
            return {
                'ok': False,
                'code': 'CONFIG_ERROR',
                'message': 'album_report_url is empty',
                'trace_id': '',
                'http_status': 0,
                'response_body': '',
            }

        try:
            payload = self._build_album_payload(msg)
        except _AlbumReporterConfigError as exc:
            return {
                'ok': False,
                'code': 'CONFIG_ERROR',
                'message': str(exc),
                'trace_id': '',
                'http_status': 0,
                'response_body': '',
            }
        except Exception as exc:
            return {
                'ok': False,
                'code': 'PAYLOAD_ERROR',
                'message': f'build album report payload failed: {exc}',
                'trace_id': '',
                'http_status': 0,
                'response_body': '',
            }

        result = self._post_with_retries(payload, msg)
        result['trace_id'] = str(payload.get('traceId', ''))
        return result

    def _handle_album_upload(
        self,
        request: InspectionAlbumUpload.Request,
        response: InspectionAlbumUpload.Response,
    ) -> InspectionAlbumUpload.Response:
        msg = self._album_upload_request_to_msg(request)
        result = self._report_album(msg)
        response.ok = bool(result['ok'])
        response.code = str(result['code'])
        response.message = str(result['message'])
        response.trace_id = str(result.get('trace_id', ''))
        response.http_status = int(result.get('http_status', 0))
        response.response_body = str(result.get('response_body', ''))[:2000]
        return response

    def _build_album_payload(self, msg: InspectionAlbumReport) -> dict[str, Any]:
        image_base64 = self._read_image_base64(msg.image_path)
        trace_id = self._new_trace_id()

        return build_album_payload(
            trace_id=trace_id,
            partner_id=self._partner_id,
            version=self._version,
            device_id=self._device_id,
            image_base64=image_base64,
            task_id=str(msg.task_id),
            point_name=str(msg.point_name or msg.point_id),
            point_id=str(msg.point_id),
            encryption_enabled=self._encryption_enabled,
            encrypt_data=self._encrypt_data,
            signature_enabled=self._signature_enabled,
            fixed_signature=self._fixed_signature,
            signature_secret=self._signature_secret,
            include_device_id=self._include_device_id,
        )

    def _read_image_base64(self, image_path: str) -> str:
        path = Path(image_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'image file not found: {path}')
        return base64.b64encode(path.read_bytes()).decode('ascii')

    def _encrypt_data(self, data_text: str) -> str:
        key = self._decode_secret(self._aes_key, label='aes_key')
        iv = self._decode_secret(self._aes_iv, label='aes_iv')
        if len(key) not in (16, 24, 32):
            raise _AlbumReporterConfigError('aes_key must be 16, 24 or 32 bytes')
        if len(iv) != 16:
            raise _AlbumReporterConfigError('aes_iv must be 16 bytes')

        padder = padding.PKCS7(algorithms.AES.block_size).padder()
        padded = padder.update(data_text.encode('utf-8')) + padder.finalize()
        encryptor = Cipher(algorithms.AES(key), modes.CBC(iv)).encryptor()
        encrypted = encryptor.update(padded) + encryptor.finalize()
        return base64.b64encode(encrypted).decode('ascii')

    def _post_with_retries(
        self,
        payload: dict[str, Any],
        msg: InspectionAlbumReport,
    ) -> dict[str, Any]:
        max_attempts = self._retry_count + 1
        last_result = {
            'ok': False,
            'code': 'POST_NOT_ATTEMPTED',
            'message': 'post was not attempted',
            'trace_id': str(payload.get('traceId', '')),
            'http_status': 0,
            'response_body': '',
        }
        for attempt in range(1, max_attempts + 1):
            try:
                if self._debug_log_payload:
                    self.get_logger().info(
                        'Album report payload: '
                        + json.dumps(payload, ensure_ascii=False)
                    )
                response = requests.post(
                    self._album_report_url,
                    json=payload,
                    headers={'Content-Type': 'application/json'},
                    timeout=self._timeout_sec,
                    verify=self._verify_tls,
                )
                success, detail = self._response_success(response)
                if success:
                    self.get_logger().info(
                        'Album report uploaded: '
                        f'task_id={msg.task_id} point_id={msg.point_id} '
                        f'trace_id={payload["traceId"]}'
                    )
                    return {
                        'ok': True,
                        'code': 'OK',
                        'message': detail,
                        'trace_id': str(payload.get('traceId', '')),
                        'http_status': response.status_code,
                        'response_body': response.text[:2000],
                    }
                last_result = {
                    'ok': False,
                    'code': 'POST_FAILED',
                    'message': detail,
                    'trace_id': str(payload.get('traceId', '')),
                    'http_status': response.status_code,
                    'response_body': response.text[:2000],
                }
                self.get_logger().warning(
                    'Album report failed: '
                    f'attempt={attempt}/{max_attempts} task_id={msg.task_id} '
                    f'point_id={msg.point_id} detail={detail}'
                )
            except Exception as exc:
                last_result = {
                    'ok': False,
                    'code': 'POST_EXCEPTION',
                    'message': str(exc),
                    'trace_id': str(payload.get('traceId', '')),
                    'http_status': 0,
                    'response_body': '',
                }
                self.get_logger().warning(
                    'Album report exception: '
                    f'attempt={attempt}/{max_attempts} task_id={msg.task_id} '
                    f'point_id={msg.point_id} error={exc}'
                )

            if attempt < max_attempts:
                time.sleep(0.5)

        self.get_logger().error(
            'Album report exhausted retries: '
            f'task_id={msg.task_id} point_id={msg.point_id}'
        )
        return last_result

    @staticmethod
    def _response_success(response: requests.Response) -> tuple[bool, str]:
        text = response.text[:500]
        if not 200 <= response.status_code < 300:
            return False, f'http_status={response.status_code} body={text}'

        try:
            body = response.json()
        except ValueError:
            return False, f'http_status={response.status_code} non_json_body={text}'

        if not isinstance(body, dict):
            return False, f'http_status={response.status_code} body={body}'
        if 'code' not in body:
            return False, f'http_status={response.status_code} missing_code body={body}'

        try:
            code = int(body.get('code', -1))
        except (TypeError, ValueError):
            return False, f'http_status={response.status_code} invalid_code body={body}'
        return code == 0, f'http_status={response.status_code} body={body}'

    @staticmethod
    def _decode_secret(value: str, *, label: str) -> bytes:
        if not value:
            raise _AlbumReporterConfigError(f'{label} is required')

        if value.startswith('base64:'):
            try:
                return base64.b64decode(value[len('base64:'):], validate=True)
            except (binascii.Error, ValueError) as exc:
                raise _AlbumReporterConfigError(
                    f'{label} must be valid base64'
                ) from exc
        if value.startswith('hex:'):
            try:
                return bytes.fromhex(value[len('hex:'):])
            except ValueError as exc:
                raise _AlbumReporterConfigError(f'{label} must be valid hex') from exc
        return value.encode('utf-8')

    @staticmethod
    def _new_trace_id() -> str:
        return f'{time.time_ns()}{random.randint(10000, 99999)}'


def main() -> None:
    rclpy.init()
    node = InspectionReporterNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
