from __future__ import annotations

import json
import socket
import threading
from datetime import datetime
from typing import Any

import rclpy
from rhw_msgs.msg import UdpBasicStatus, UdpBatteryStatus, UdpMotionStatus
from rclpy.node import Node


class UdpBridgeNode(Node):
    """Receive UDP robot status packets and publish structured ROS 2 topics."""

    def __init__(self) -> None:
        super().__init__('udp_bridge_node')

        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('bind_port', 30000)
        self.declare_parameter('robot_host', '10.21.31.103')
        self.declare_parameter('robot_port', 30000)
        self.declare_parameter('receive_filter_host', '10.21.41.1')
        self.declare_parameter('receive_filter_port', 30000)
        self.declare_parameter('heartbeat_period_sec', 1.0)
        self.declare_parameter('heartbeat_type', 100)
        self.declare_parameter('heartbeat_command', 100)
        self.declare_parameter('heartbeat_time_format', '%Y-%m-%d %H:%M:%S')
        self.declare_parameter('basic_status_topic', '/robot/basic_status')
        self.declare_parameter('motion_status_topic', '/robot/motion_status')
        self.declare_parameter('battery_status_topic', '/robot/battery_status')
        self.declare_parameter('status_frame_id', 'base_link')
        self.declare_parameter('motion_command', 4)
        self.declare_parameter('battery_command', 5)
        self.declare_parameter('basic_command', 6)
        self.declare_parameter('packet_header', [235, 145, 235, 144])
        self.declare_parameter('socket_timeout_sec', 0.5)

        self._bind_host = str(self.get_parameter('bind_host').value)
        self._bind_port = int(self.get_parameter('bind_port').value)
        self._robot_host = str(self.get_parameter('robot_host').value)
        self._robot_port = int(self.get_parameter('robot_port').value)
        self._receive_filter_host = str(self.get_parameter('receive_filter_host').value)
        self._receive_filter_port = int(self.get_parameter('receive_filter_port').value)
        self._heartbeat_period_sec = float(self.get_parameter('heartbeat_period_sec').value)
        self._heartbeat_type = int(self.get_parameter('heartbeat_type').value)
        self._heartbeat_command = int(self.get_parameter('heartbeat_command').value)
        self._heartbeat_time_format = str(self.get_parameter('heartbeat_time_format').value)
        self._basic_status_topic = str(self.get_parameter('basic_status_topic').value)
        self._motion_status_topic = str(self.get_parameter('motion_status_topic').value)
        self._battery_status_topic = str(self.get_parameter('battery_status_topic').value)
        self._status_frame_id = str(self.get_parameter('status_frame_id').value)
        self._motion_command = int(self.get_parameter('motion_command').value)
        self._battery_command = int(self.get_parameter('battery_command').value)
        self._basic_command = int(self.get_parameter('basic_command').value)
        self._packet_header = bytes(int(x) & 0xFF for x in self.get_parameter('packet_header').value)
        self._socket_timeout_sec = max(float(self.get_parameter('socket_timeout_sec').value), 0.1)

        self._basic_pub = self.create_publisher(UdpBasicStatus, self._basic_status_topic, 10)
        self._motion_pub = self.create_publisher(UdpMotionStatus, self._motion_status_topic, 10)
        self._battery_pub = self.create_publisher(UdpBatteryStatus, self._battery_status_topic, 10)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self._bind_host, self._bind_port))
        self._sock.settimeout(self._socket_timeout_sec)
        self._remote_addr = (self._robot_host, self._robot_port)
        self._msg_id = 0
        self._running = True

        self._recv_thread = threading.Thread(target=self._udp_receive_loop, daemon=True)
        self._recv_thread.start()

        self._heartbeat_timer = self.create_timer(
            self._heartbeat_period_sec,
            self._send_heartbeat,
        )

        self.get_logger().info(
            f'udp_bridge_node started bind={self._bind_host}:{self._bind_port} '
            f'remote={self._robot_host}:{self._robot_port}'
        )

    def destroy_node(self) -> bool:
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass
        if self._recv_thread.is_alive():
            self._recv_thread.join(timeout=2.0)
        return super().destroy_node()

    def _time_string(self) -> str:
        return datetime.now().strftime(self._heartbeat_time_format)

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
            self.get_logger().warning(f'Failed to send UDP packet: {exc}')
            return False
        return sent == len(packet)

    def _send_heartbeat(self) -> None:
        heartbeat = {
            'PatrolDevice': {
                'Type': self._heartbeat_type,
                'Command': self._heartbeat_command,
                'Time': self._time_string(),
                'Items': {},
            }
        }
        self._send_udp_json(json.dumps(heartbeat, ensure_ascii=False))

    def _udp_receive_loop(self) -> None:
        while self._running:
            try:
                packet, sender = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            if sender[0] != self._receive_filter_host or sender[1] != self._receive_filter_port:
                continue
            if len(packet) < 16:
                continue
            if packet[:4] != self._packet_header:
                continue

            asdu_len = packet[4] | (packet[5] << 8)
            if asdu_len <= 0 or (16 + asdu_len) > len(packet):
                continue

            payload_text = packet[16:16 + asdu_len].decode('utf-8', errors='ignore')
            try:
                payload = json.loads(payload_text)
            except json.JSONDecodeError:
                continue

            patrol_device = payload.get('PatrolDevice', {})
            if int(patrol_device.get('Type', -1)) != 1002:
                continue

            command = int(patrol_device.get('Command', -1))
            if command not in (self._motion_command, self._battery_command, self._basic_command):
                continue

            if command == self._motion_command:
                self._publish_motion_status(payload)
            elif command == self._battery_command:
                self._publish_battery_status(payload)
            elif command == self._basic_command:
                self._publish_basic_status(payload)


    def _publish_motion_status(self, payload: dict[str, Any]) -> None:
        items = self._as_dict(payload.get('PatrolDevice', {}).get('Items', {}))
        motion = self._as_dict(items.get('MotionStatus', {}))
        motor = self._as_dict(items.get('MotorStatus', {}))

        msg = UdpMotionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._status_frame_id
        msg.roll = self._pick_float(motion, ['Roll', 'roll'])
        msg.pitch = self._pick_float(motion, ['Pitch', 'pitch'])
        msg.yaw = self._pick_float(motion, ['Yaw', 'yaw'])
        msg.omega_z = self._pick_float(motion, ['OmegaZ', 'omega_z', 'Omega', 'omega'])
        msg.line_x = self._pick_float(motion, ['LineX', 'line_x', 'LinearX', 'linear_x'])
        msg.line_y = self._pick_float(motion, ['LineY', 'line_y', 'LinearY', 'linear_y'])
        msg.height = self._pick_float(motion, ['Height', 'height'])
        msg.remain_mile = self._pick_float(motor, ['RemainMile', 'remain_mile'])
        self._motion_pub.publish(msg)

    def _publish_battery_status(self, payload: dict[str, Any]) -> None:
        items = self._as_dict(payload.get('PatrolDevice', {}).get('Items', {}))
        battery = self._as_dict(items.get('BatteryStatus', {}))

        msg = UdpBatteryStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._status_frame_id

        msg.voltage_left = self._pick_float(battery, ['VoltageLeft', 'voltage_left'])
        msg.voltage_right = self._pick_float(battery, ['VoltageRight', 'voltage_right'])
        msg.battery_level_left = self._pick_float(battery, ['BatteryLevelLeft', 'battery_level_left'])
        msg.battery_level_right = self._pick_float(battery, ['BatteryLevelRight', 'battery_level_right'])
        msg.battery_temperature_left = self._pick_float(
            battery,
            ['battery_temperatureLeft', 'BatteryTemperatureLeft', 'battery_temperature_left'],
        )
        msg.battery_temperature_right = self._pick_float(
            battery,
            ['battery_temperatureRight', 'BatteryTemperatureRight', 'battery_temperature_right'],
        )
        msg.charge_left = self._pick_bool(battery, ['chargeLeft', 'ChargeLeft', 'charge_left'])
        msg.charge_right = self._pick_bool(battery, ['chargeRight', 'ChargeRight', 'charge_right'])
        self._battery_pub.publish(msg)

    def _publish_basic_status(self, payload: dict[str, Any]) -> None:
        items = self._as_dict(payload.get('PatrolDevice', {}).get('Items', {}))
        basic = self._as_dict(items.get('BasicStatus', {}))

        msg = UdpBasicStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._status_frame_id
        msg.motion_state = self._pick_int(basic, ['MotionState', 'motion_state'])
        msg.gait = self._pick_int(basic, ['Gait', 'gait'])
        msg.charge = self._pick_int(basic, ['Charge', 'charge'])
        msg.hes = self._pick_int(basic, ['HES', 'hes'])
        msg.control_usage_mode = self._pick_int(basic, ['ControlUsageMode', 'control_usage_mode'])
        msg.direction = self._pick_int(basic, ['Direction', 'direction'])
        msg.ooa = self._pick_int(basic, ['OOA', 'ooa'])
        msg.power_management = self._pick_int(basic, ['PowerManagement', 'power_management'])
        msg.sleep = self._pick_int(basic, ['Sleep', 'sleep'])
        msg.version = self._pick_str(basic, ['Version', 'version'])
        self._basic_pub.publish(msg)

    def _as_dict(self, value: Any) -> dict[str, Any]:
        if isinstance(value, dict):
            return value
        return {}

    def _pick_float(self, source: dict[str, Any], keys: list[str], default: float = 0.0) -> float:
        value = self._pick_value(source, keys, default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def _pick_int(self, source: dict[str, Any], keys: list[str], default: int = 0) -> int:
        value = self._pick_value(source, keys, default)
        try:
            return int(value)
        except (TypeError, ValueError):
            return int(default)

    def _pick_bool(self, source: dict[str, Any], keys: list[str], default: bool = False) -> bool:
        value = self._pick_value(source, keys, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in ('1', 'true', 'yes', 'on'):
                return True
            if lowered in ('0', 'false', 'no', 'off'):
                return False
        return bool(default)

    def _pick_str(self, source: dict[str, Any], keys: list[str], default: str = '') -> str:
        value = self._pick_value(source, keys, default)
        if value is None:
            return default
        return str(value)

    def _pick_value(self, source: dict[str, Any], keys: list[str], default: Any) -> Any:
        for key in keys:
            if key in source:
                return source.get(key)
        return default

def main() -> None:
    rclpy.init()
    node = UdpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
