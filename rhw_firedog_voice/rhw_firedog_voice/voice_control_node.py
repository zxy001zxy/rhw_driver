from __future__ import annotations

import json
import socket
from datetime import datetime
from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class VoiceControlNode(Node):
    """Subscribe voice commands and send UDP control packets to the robot dog."""

    def __init__(self) -> None:
        super().__init__('voice_control_node')

        self.declare_parameter('voice_command_topic', '/voice_command')
        self.declare_parameter('robot_host', '10.21.31.103')
        self.declare_parameter('robot_port', 30000)
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('bind_port', 0)
        self.declare_parameter('socket_timeout_sec', 1.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('command_hold_sec', 0.5)
        self.declare_parameter('heartbeat_period_sec', 1.0)
        self.declare_parameter('heartbeat_type', 100)
        self.declare_parameter('heartbeat_command', 100)
        self.declare_parameter('packet_type', 2)
        self.declare_parameter('packet_command', 21)
        self.declare_parameter('packet_header', [235, 145, 235, 144])
        self.declare_parameter('time_format', '%Y-%m-%d %H:%M:%S')
        self.declare_parameter('forward_x', 0.1)
        self.declare_parameter('backward_x', -0.1)
        self.declare_parameter('left_yaw', 15.0)
        self.declare_parameter('right_yaw', -15.0)
        self.declare_parameter('default_y', 0.0)
        self.declare_parameter('default_z', 0.0)
        self.declare_parameter('default_roll', 0.0)
        self.declare_parameter('default_pitch', 0.0)

        self._voice_command_topic = str(self.get_parameter('voice_command_topic').value)
        self._robot_host = str(self.get_parameter('robot_host').value)
        self._robot_port = int(self.get_parameter('robot_port').value)
        self._bind_host = str(self.get_parameter('bind_host').value)
        self._bind_port = int(self.get_parameter('bind_port').value)
        self._socket_timeout_sec = max(float(self.get_parameter('socket_timeout_sec').value), 0.1)
        self._control_rate_hz = max(float(self.get_parameter('control_rate_hz').value), 1.0)
        self._command_hold_sec = max(float(self.get_parameter('command_hold_sec').value), 0.05)
        self._heartbeat_period_sec = max(float(self.get_parameter('heartbeat_period_sec').value), 0.1)
        self._heartbeat_type = int(self.get_parameter('heartbeat_type').value)
        self._heartbeat_command = int(self.get_parameter('heartbeat_command').value)
        self._packet_type = int(self.get_parameter('packet_type').value)
        self._packet_command = int(self.get_parameter('packet_command').value)
        self._packet_header = bytes(int(x) & 0xFF for x in self.get_parameter('packet_header').value)
        self._time_format = str(self.get_parameter('time_format').value)
        self._forward_x = float(self.get_parameter('forward_x').value)
        self._backward_x = float(self.get_parameter('backward_x').value)
        self._left_yaw = float(self.get_parameter('left_yaw').value)
        self._right_yaw = float(self.get_parameter('right_yaw').value)
        self._default_y = float(self.get_parameter('default_y').value)
        self._default_z = float(self.get_parameter('default_z').value)
        self._default_roll = float(self.get_parameter('default_roll').value)
        self._default_pitch = float(self.get_parameter('default_pitch').value)

        self._qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._command_map = {
            'MOVE_FORWARD': {'X': self._forward_x, 'Yaw': 0.0},
            'MOVE_BACKWARD': {'X': self._backward_x, 'Yaw': 0.0},
            'TURN_LEFT': {'X': 0.0, 'Yaw': self._left_yaw},
            'TURN_RIGHT': {'X': 0.0, 'Yaw': self._right_yaw},
            '前进': {'X': self._forward_x, 'Yaw': 0.0},
            '后退': {'X': self._backward_x, 'Yaw': 0.0},
            '左转': {'X': 0.0, 'Yaw': self._left_yaw},
            '右转': {'X': 0.0, 'Yaw': self._right_yaw},
        }

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self._bind_host, self._bind_port))
        self._sock.settimeout(self._socket_timeout_sec)
        self._remote_addr = (self._robot_host, self._robot_port)
        self._msg_id = 0
        self._idle_items = self._build_control_items()
        self._active_items = self._idle_items.copy()
        self._command_deadline = 0.0
        self._sent_stop_after_expire = True

        self._sub = self.create_subscription(
            String,
            self._voice_command_topic,
            self._on_voice_command,
            self._qos,
        )
        self._send_timer = self.create_timer(1.0 / self._control_rate_hz, self._on_send_timer)
        self._heartbeat_timer = self.create_timer(self._heartbeat_period_sec, self._send_heartbeat)

        self.get_logger().info(
            f'voice_control_node started topic={self._voice_command_topic} '
            f'remote={self._robot_host}:{self._robot_port} '
            f'bind={self._bind_host}:{self._sock.getsockname()[1]} '
            f'rate={self._control_rate_hz:.1f}Hz hold={self._command_hold_sec:.2f}s'
        )
        self.get_logger().info('Waiting for /voice_command messages... start this node before running ASR')

    def destroy_node(self) -> bool:
        try:
            self._sock.close()
        except OSError:
            pass
        return super().destroy_node()

    def _on_voice_command(self, msg: String) -> None:
        command_text = str(msg.data).strip()
        if not command_text:
            self.get_logger().warning('Received empty voice command, ignored')
            return

        items = self._build_items_for_command(command_text)
        if items is None:
            self.get_logger().warning(f'Unsupported voice command: {command_text}')
            return

        self._active_items = items
        self._command_deadline = monotonic() + self._command_hold_sec
        self._sent_stop_after_expire = False
        self.get_logger().info(
            f'Accepted voice command: {command_text} -> {json.dumps(items, ensure_ascii=False)}'
        )
        payload_text = json.dumps(self._build_payload(self._active_items), ensure_ascii=False)
        if not self._send_udp_json(payload_text):
            self.get_logger().warning('Failed to send immediate voice control payload')

    def _on_send_timer(self) -> None:
        now = monotonic()
        if now <= self._command_deadline:
            payload = self._build_payload(self._active_items)
            payload_text = json.dumps(payload, ensure_ascii=False)
            if not self._send_udp_json(payload_text):
                self.get_logger().warning('Failed to send active voice control payload')
            return

        if not self._sent_stop_after_expire:
            self._active_items = self._idle_items.copy()
            payload = self._build_payload(self._active_items)
            payload_text = json.dumps(payload, ensure_ascii=False)
            if self._send_udp_json(payload_text):
                self.get_logger().info('Voice control command expired, sent stop payload')
            else:
                self.get_logger().warning('Voice control command expired, failed to send stop payload')
            self._sent_stop_after_expire = True

    def _build_control_items(
        self,
        *,
        x: float = 0.0,
        y: float | None = None,
        yaw: float = 0.0,
    ) -> dict[str, float]:
        return {
            'X': float(x),
            'Y': self._default_y if y is None else float(y),
            'Z': 0.0,
            'Roll': 0.0,
            'Pitch': 0.0,
            'Yaw': float(yaw),
        }

    def _build_items_for_command(self, command_text: str) -> dict[str, float] | None:
        mapping = self._command_map.get(command_text)
        if mapping is None:
            return None

        return self._build_control_items(
            x=float(mapping.get('X', 0.0)),
            y=float(mapping.get('Y', self._default_y)),
            yaw=float(mapping.get('Yaw', 0.0)),
        )

    def _build_payload(self, items: dict[str, float]) -> dict:
        return {
            'PatrolDevice': {
                'Type': self._packet_type,
                'Command': self._packet_command,
                'Time': datetime.now().strftime(self._time_format),
                'Items': items,
            }
        }

    def _send_heartbeat(self) -> None:
        payload = {
            'PatrolDevice': {
                'Type': self._heartbeat_type,
                'Command': self._heartbeat_command,
                'Time': datetime.now().strftime(self._time_format),
                'Items': {},
            }
        }
        payload_text = json.dumps(payload, ensure_ascii=False)
        if not self._send_udp_json(payload_text):
            self.get_logger().warning('Failed to send heartbeat payload')

    def _send_udp_json(self, payload_text: str) -> bool:
        payload = payload_text.encode('utf-8')
        packet = bytearray(16 + len(payload))
        packet[0:4] = self._packet_header
        data_len = len(payload)
        packet[4] = data_len & 0xFF
        packet[5] = (data_len >> 8) & 0xFF
        packet[6] = self._msg_id & 0xFF
        packet[7] = (self._msg_id >> 8) & 0xFF
        packet[8] = 0x01
        packet[16:] = payload
        self._msg_id = (self._msg_id + 1) % 65536
        try:
            sent = self._sock.sendto(packet, self._remote_addr)
        except OSError as exc:
            self.get_logger().warning(f'Failed to send UDP payload: {exc}')
            return False
        return sent == len(packet)


def main() -> None:
    rclpy.init()
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
