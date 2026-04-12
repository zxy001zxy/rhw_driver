"""ROS 2 node that exposes PtzController over standard services and topics."""
from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

from rhw_ptz_controller.ptz_controller import PtzController, PtzError

from rhw_msgs.msg import PtzStatus
from rhw_msgs.srv import (
    CaptureImage,
    PtzAbsoluteMove,
    PtzControl,
    PtzGetPosition,
    PtzGotoPreset,
    PtzPatrol,
)


class PtzControllerNode(Node):
    """Thin ROS 2 wrapper around :class:`PtzController`."""

    def __init__(self) -> None:
        super().__init__('ptz_controller_node')

        # ---- 参数 ----
        self.declare_parameter('camera_ip', '192.168.10.64')
        self.declare_parameter('camera_port', 80)
        self.declare_parameter('camera_username', 'admin')
        self.declare_parameter('camera_password', 'rhw1314000')
        self.declare_parameter('use_https', False)
        self.declare_parameter('verify_ssl', False)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('default_channel', 1)
        self.declare_parameter('default_speed', 40)
        self.declare_parameter('default_duration_ms', 350)
        self.declare_parameter('status_publish_period', 2.0)
        self.declare_parameter('capture_save_dir', '/tmp/ptz_captures')

        self._default_channel: int = self.get_parameter('default_channel').value
        self._capture_save_dir: str = str(self.get_parameter('capture_save_dir').value)
        self._default_speed: int = self.get_parameter('default_speed').value
        self._default_duration_ms: int = self.get_parameter('default_duration_ms').value

        # ---- PtzController 实例 ----
        self._ptz = PtzController(
            ip=str(self.get_parameter('camera_ip').value),
            username=str(self.get_parameter('camera_username').value),
            password=str(self.get_parameter('camera_password').value),
            port=int(self.get_parameter('camera_port').value),
            use_https=bool(self.get_parameter('use_https').value),
            verify_ssl=bool(self.get_parameter('verify_ssl').value),
            timeout=float(self.get_parameter('timeout').value),
        )

        # ---- 服务 ----
        self.create_service(PtzControl, '/ptz/control', self._handle_control)
        self.create_service(PtzGotoPreset, '/ptz/goto_preset', self._handle_goto_preset)
        self.create_service(PtzPatrol, '/ptz/patrol', self._handle_patrol)
        self.create_service(PtzAbsoluteMove, '/ptz/absolute_move', self._handle_absolute_move)
        self.create_service(PtzGetPosition, '/ptz/get_position', self._handle_get_position)
        self.create_service(CaptureImage, '/ptz/capture_image', self._handle_capture_image)

        # ---- 状态话题 ----
        self._status_pub = self.create_publisher(PtzStatus, '/ptz/status', 10)
        period = float(self.get_parameter('status_publish_period').value)
        self._status_timer = self.create_timer(period, self._publish_status)

        # ---- 运行时状态 ----
        self._online = False
        self._last_azimuth: float = 0.0
        self._last_elevation: float = 0.0
        self._active_action: str = 'idle'

        self.get_logger().info(
            f'ptz_controller_node started  camera={self._ptz.ip}:{self._ptz.port}'
        )

    # ===================================================================
    #  /ptz/control — 方向控制、变倍、停止
    # ===================================================================
    def _handle_control(
        self, request: PtzControl.Request, response: PtzControl.Response
    ) -> PtzControl.Response:
        direction = request.direction or 'stop'
        speed = int(request.speed) if request.speed else self._default_speed
        channel = int(request.channel) if request.channel else self._default_channel
        duration_ms = int(request.duration_ms) if request.duration_ms else self._default_duration_ms

        try:
            result = self._ptz.control(
                direction=direction,
                speed=speed,
                channel=channel,
                duration_ms=duration_ms,
            )
            ok = bool(result.get('ok'))
            response.result = 1 if ok else 0
            response.execution_mode = str(result.get('execution_mode', ''))
            response.message = self._extract_message(result)

            if ok:
                self._online = True
                self._active_action = direction if direction != 'stop' else 'idle'
            return response

        except PtzError as exc:
            response.result = 0
            response.execution_mode = ''
            response.message = f'[{exc.category}] {exc}'
            return response
        except Exception as exc:
            response.result = 0
            response.execution_mode = ''
            response.message = str(exc)
            return response

    # ===================================================================
    #  /ptz/goto_preset — 跳转预置位
    # ===================================================================
    def _handle_goto_preset(
        self, request: PtzGotoPreset.Request, response: PtzGotoPreset.Response
    ) -> PtzGotoPreset.Response:
        channel = int(request.channel) if request.channel else self._default_channel
        preset_id = int(request.preset_id) if request.preset_id else 1

        try:
            result = self._ptz.goto_preset(channel=channel, preset_id=preset_id)
            ok = bool(result.get('ok'))
            response.result = 1 if ok else 0
            response.message = self._extract_message(result)
            if ok:
                self._online = True
                self._active_action = f'preset:{preset_id}'
            return response
        except PtzError as exc:
            response.result = 0
            response.message = f'[{exc.category}] {exc}'
            return response
        except Exception as exc:
            response.result = 0
            response.message = str(exc)
            return response

    # ===================================================================
    #  /ptz/patrol — 启停巡航
    # ===================================================================
    def _handle_patrol(
        self, request: PtzPatrol.Request, response: PtzPatrol.Response
    ) -> PtzPatrol.Response:
        channel = int(request.channel) if request.channel else self._default_channel
        patrol_id = int(request.patrol_id) if request.patrol_id else 1
        action = int(request.action)

        try:
            if action == 1:
                result = self._ptz.start_patrol(channel=channel, patrol_id=patrol_id)
                action_name = 'start_patrol'
            else:
                result = self._ptz.stop_patrol(channel=channel, patrol_id=patrol_id)
                action_name = 'stop_patrol'

            ok = bool(result.get('ok'))
            response.result = 1 if ok else 0
            response.message = self._extract_message(result)
            if ok:
                self._online = True
                self._active_action = f'patrol:{patrol_id}' if action == 1 else 'idle'
            return response
        except PtzError as exc:
            response.result = 0
            response.message = f'[{exc.category}] {exc}'
            return response
        except Exception as exc:
            response.result = 0
            response.message = str(exc)
            return response

    # ===================================================================
    #  /ptz/absolute_move — 绝对位置移动
    # ===================================================================
    def _handle_absolute_move(
        self, request: PtzAbsoluteMove.Request, response: PtzAbsoluteMove.Response
    ) -> PtzAbsoluteMove.Response:
        channel = int(request.channel) if request.channel else self._default_channel
        azimuth = float(request.azimuth)
        elevation = float(request.elevation)
        azimuth_speed: Optional[int] = int(request.azimuth_speed) if request.azimuth_speed else None
        elevation_speed: Optional[int] = int(request.elevation_speed) if request.elevation_speed else None

        try:
            result = self._ptz.move_absolute_ex(
                channel=channel,
                azimuth=azimuth,
                elevation=elevation,
                azimuth_speed=azimuth_speed,
                elevation_speed=elevation_speed,
            )
            ok = bool(result.get('ok'))
            response.result = 1 if ok else 0
            response.message = self._extract_message(result)
            if ok:
                self._online = True
                self._active_action = 'moving'
            return response
        except PtzError as exc:
            response.result = 0
            response.message = f'[{exc.category}] {exc}'
            return response
        except Exception as exc:
            response.result = 0
            response.message = str(exc)
            return response

    # ===================================================================
    #  /ptz/get_position — 获取当前角度
    # ===================================================================
    def _handle_get_position(
        self, request: PtzGetPosition.Request, response: PtzGetPosition.Response
    ) -> PtzGetPosition.Response:
        channel = int(request.channel) if request.channel else self._default_channel

        try:
            result = self._ptz.get_absolute_ex(channel=channel)
            ok = bool(result.get('ok'))
            response.result = 1 if ok else 0

            if ok:
                azimuth = result.get('azimuth')
                elevation = result.get('elevation')
                response.azimuth = float(azimuth) if azimuth is not None else 0.0
                response.elevation = float(elevation) if elevation is not None else 0.0
                self._last_azimuth = response.azimuth
                self._last_elevation = response.elevation
                self._online = True
            else:
                response.azimuth = 0.0
                response.elevation = 0.0

            response.message = self._extract_message(result)
            return response
        except PtzError as exc:
            response.result = 0
            response.azimuth = 0.0
            response.elevation = 0.0
            response.message = f'[{exc.category}] {exc}'
            return response
        except Exception as exc:
            response.result = 0
            response.azimuth = 0.0
            response.elevation = 0.0
            response.message = str(exc)
            return response

    # ===================================================================
    #  /ptz/capture_image — 手动抓拍并保存
    # ===================================================================
    def _handle_capture_image(
        self, request: CaptureImage.Request, response: CaptureImage.Response
    ) -> CaptureImage.Response:
        channel = int(request.channel) if request.channel else self._default_channel
        image_type = 'JPEG'
        url_type = request.url_type.strip() if request.url_type else 'localURL'
        channel_format = request.channel_format.strip() if request.channel_format else ''

        # ---------- 确定保存路径 ----------
        save_path = request.save_path.strip() if request.save_path else ''
        if not save_path:
            # 自动生成：<capture_save_dir>/capture_<时间戳>.jpg
            ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            save_path = os.path.join(self._capture_save_dir, f'capture_{ts}.jpg')

        try:
            # 确保目录存在
            save_dir = os.path.dirname(save_path)
            if save_dir:
                Path(save_dir).mkdir(parents=True, exist_ok=True)

            if url_type == 'localURL':
                result = self._ptz.capture_picture_binary(channel=channel)
                ok = bool(result.get('ok'))
                response.capture_url = str(result.get('url') or '')
                image_data: bytes = result.get('image_data', b'')

                if not ok or not image_data:
                    response.result = 0
                    response.file_path = ''
                    response.file_size = 0
                    response.saved = False
                    response.message = self._extract_message(result)
                    return response
            else:
                result = self._ptz.capture_picture(
                    channel=channel,
                    image_type=image_type,
                    url_type=url_type,
                    channel_format=channel_format,
                )
                ok = bool(result.get('ok'))
                response.capture_url = str(result.get('capture_url') or '')

                if not ok:
                    response.result = 0
                    response.capture_url = ''
                    response.file_path = ''
                    response.file_size = 0
                    response.saved = False
                    message = self._extract_message(result)
                    payload_keys = result.get('payload_keys')
                    if payload_keys:
                        message = f"{message}; payload_keys={payload_keys}"
                    response.message = message
                    return response

                download_result = self._ptz.download_capture_url(
                    capture_url=response.capture_url,
                )
                download_ok = bool(download_result.get('ok'))
                image_data = download_result.get('image_data', b'')
                if not download_ok or not image_data:
                    response.result = 0
                    response.file_path = ''
                    response.file_size = 0
                    response.saved = False
                    response.message = self._extract_message(download_result)
                    return response

            with open(save_path, 'wb') as f:
                f.write(image_data)

            file_size = len(image_data)
            response.result = 1
            response.file_path = save_path
            response.file_size = file_size
            response.saved = True
            response.message = f'ok, saved {file_size} bytes'
            self._online = True
            self.get_logger().info(
                f'Captured image saved: {save_path} ({file_size} bytes), url={response.capture_url}'
            )

            return response

        except PtzError as exc:
            response.result = 0
            response.capture_url = ''
            response.file_path = ''
            response.file_size = 0
            response.saved = False
            response.message = f'[{exc.category}] {exc}'
            return response
        except Exception as exc:
            response.result = 0
            response.capture_url = ''
            response.file_path = ''
            response.file_size = 0
            response.saved = False
            response.message = str(exc)
            return response

    # ===================================================================
    #  /ptz/status — 定时轮询并发布
    # ===================================================================
    def _publish_status(self) -> None:
        msg = PtzStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.channel = self._default_channel

        try:
            result = self._ptz.get_absolute_ex(channel=self._default_channel)
            ok = bool(result.get('ok'))
            if ok:
                azimuth = result.get('azimuth')
                elevation = result.get('elevation')
                self._last_azimuth = float(azimuth) if azimuth is not None else self._last_azimuth
                self._last_elevation = float(elevation) if elevation is not None else self._last_elevation
                self._online = True
            else:
                self._online = False
        except Exception:
            self._online = False

        msg.online = self._online
        msg.azimuth = self._last_azimuth
        msg.elevation = self._last_elevation
        msg.active_action = self._active_action
        msg.message = 'ok' if self._online else 'camera unreachable'
        self._status_pub.publish(msg)

    # ===================================================================
    #  工具
    # ===================================================================
    @staticmethod
    def _extract_message(result: dict) -> str:
        """从 PtzController 返回的 dict 中提取人类可读信息。"""
        if result.get('error'):
            return str(result['error'])
        if result.get('status_string'):
            return str(result['status_string'])
        command = result.get('command')
        if isinstance(command, dict):
            if command.get('error'):
                return str(command['error'])
            if command.get('status_string'):
                return str(command['status_string'])
        return 'ok'


def main() -> None:
    rclpy.init()
    node = PtzControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
