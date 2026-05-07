from __future__ import annotations

import importlib
import json
import math
import time
from typing import Any

import rclpy
from rclpy.node import Node

from rhw_msgs.msg import MissionStatus, RobotPosition, UdpBasicStatus, UdpBatteryStatus

mqtt = None


def ensure_paho_mqtt_imported():
    global mqtt
    if mqtt is not None:
        return mqtt

    try:
        mqtt = importlib.import_module('paho.mqtt.client')
    except ImportError as exc:
        print('[ERROR] 未安装 paho-mqtt，请先执行: python3 -m pip install paho-mqtt')
        raise SystemExit(1) from exc
    return mqtt


class MqttForwarderNode(Node):
    """Forward ROS 2 robot status topics to MQTT heartbeat messages."""

    def __init__(self) -> None:
        super().__init__('mqtt_forwarder_node')

        self.declare_parameter('enabled', True)
        self.declare_parameter('broker_host', '127.0.0.1')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('client_id', 'rhw_udp_mqtt_bridge')
        self.declare_parameter('username', '')
        self.declare_parameter('password', '')
        self.declare_parameter('mqtt_topic', '/robot-dog/DOG001/Upload/Data')
        self.declare_parameter('heartbeat_publish_period_sec', 1.0)
        self.declare_parameter('status_timeout_sec', 3.0)
        self.declare_parameter('default_signal_strength', -65)
        self.declare_parameter('map_id', 1)
        self.declare_parameter('qos', 0)
        self.declare_parameter('retain', False)
        self.declare_parameter('keep_alive_sec', 60)
        self.declare_parameter('basic_status_topic', '/robot/basic_status')
        self.declare_parameter('battery_status_topic', '/robot/battery_status')
        self.declare_parameter('robot_position_topic', '/robot_position')
        self.declare_parameter('mission_status_topic', '/mission/status')
        self.declare_parameter('debug_log_payload', False)
        self.declare_parameter('missing_state_log_period_sec', 5.0)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._broker_host = str(self.get_parameter('broker_host').value)
        self._broker_port = int(self.get_parameter('broker_port').value)
        self._client_id = str(self.get_parameter('client_id').value)
        self._username = str(self.get_parameter('username').value)
        self._password = str(self.get_parameter('password').value)
        self._mqtt_topic = str(self.get_parameter('mqtt_topic').value)
        self._heartbeat_publish_period_sec = max(
            float(self.get_parameter('heartbeat_publish_period_sec').value),
            0.1,
        )
        self._status_timeout_sec = max(float(self.get_parameter('status_timeout_sec').value), 0.1)
        self._default_signal_strength = int(self.get_parameter('default_signal_strength').value)
        self._map_id = int(self.get_parameter('map_id').value)
        self._qos = int(self.get_parameter('qos').value)
        self._retain = bool(self.get_parameter('retain').value)
        self._keep_alive_sec = max(int(self.get_parameter('keep_alive_sec').value), 1)
        self._basic_status_topic = str(self.get_parameter('basic_status_topic').value)
        self._battery_status_topic = str(self.get_parameter('battery_status_topic').value)
        self._robot_position_topic = str(self.get_parameter('robot_position_topic').value)
        self._mission_status_topic = str(self.get_parameter('mission_status_topic').value)
        self._debug_log_payload = bool(self.get_parameter('debug_log_payload').value)
        self._missing_state_log_period_sec = max(
            float(self.get_parameter('missing_state_log_period_sec').value),
            0.1,
        )

        self._basic_status: UdpBasicStatus | None = None
        self._battery_status: UdpBatteryStatus | None = None
        self._robot_position: RobotPosition | None = None
        self._mission_status: MissionStatus | None = None

        self._basic_status_received_at = 0.0
        self._battery_status_received_at = 0.0
        self._robot_position_received_at = 0.0
        self._mission_status_received_at = 0.0
        self._last_missing_state_log_at = 0.0
        self._msg_id = 1
        self._mqtt_connected = False

        self.create_subscription(
            UdpBasicStatus,
            self._basic_status_topic,
            self._on_basic_status,
            10,
        )
        self.create_subscription(
            UdpBatteryStatus,
            self._battery_status_topic,
            self._on_battery_status,
            10,
        )
        self.create_subscription(
            RobotPosition,
            self._robot_position_topic,
            self._on_robot_position,
            10,
        )
        self.create_subscription(
            MissionStatus,
            self._mission_status_topic,
            self._on_mission_status,
            10,
        )

        self._mqtt_client = None
        if self._enabled:
            mqtt_module = ensure_paho_mqtt_imported()
            self._mqtt_client = mqtt_module.Client(client_id=self._client_id, protocol=mqtt_module.MQTTv311)
            if self._username:
                self._mqtt_client.username_pw_set(self._username, self._password)
            self._mqtt_client.on_connect = self._on_mqtt_connect
            self._mqtt_client.on_disconnect = self._on_mqtt_disconnect
            self._mqtt_client.connect_async(self._broker_host, self._broker_port, keepalive=self._keep_alive_sec)
            self._mqtt_client.loop_start()
        else:
            self.get_logger().warning('mqtt_forwarder_node is disabled by parameter enabled=false')

        self._publish_timer = self.create_timer(
            self._heartbeat_publish_period_sec,
            self._publish_heartbeat,
        )

        self.get_logger().info(
            f'mqtt_forwarder_node started enabled={self._enabled} broker={self._broker_host}:{self._broker_port} '
            f'topic={self._mqtt_topic} timeout={self._status_timeout_sec:.1f}s'
        )
        self.get_logger().info(
            'waiting topics: '
            f'{self._basic_status_topic}, {self._battery_status_topic}, '
            f'{self._robot_position_topic}, {self._mission_status_topic}'
        )

    def destroy_node(self) -> bool:
        if self._mqtt_client is not None:
            try:
                self._mqtt_client.loop_stop()
                self._mqtt_client.disconnect()
            except Exception:
                pass
        return super().destroy_node()

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        code = self._reason_code_to_int(reason_code)
        self._mqtt_connected = (code == 0)
        if self._mqtt_connected:
            self.get_logger().info(
                f'Connected to MQTT broker {self._broker_host}:{self._broker_port}, topic={self._mqtt_topic}'
            )
        else:
            self.get_logger().warning(
                f'Failed to connect to MQTT broker {self._broker_host}:{self._broker_port}, code={code}'
            )

    def _on_mqtt_disconnect(self, client, userdata, reason_code, properties=None) -> None:
        code = self._reason_code_to_int(reason_code)
        self._mqtt_connected = False
        self.get_logger().warning(f'Disconnected from MQTT broker, code={code}')

    def _reason_code_to_int(self, reason_code: Any) -> int:
        try:
            return int(reason_code)
        except (TypeError, ValueError):
            pass
        try:
            return int(getattr(reason_code, 'value'))
        except (AttributeError, TypeError, ValueError):
            return -1

    def _on_basic_status(self, msg: UdpBasicStatus) -> None:
        self._basic_status = msg
        self._basic_status_received_at = time.monotonic()

    def _on_battery_status(self, msg: UdpBatteryStatus) -> None:
        self._battery_status = msg
        self._battery_status_received_at = time.monotonic()

    def _on_robot_position(self, msg: RobotPosition) -> None:
        self._robot_position = msg
        self._robot_position_received_at = time.monotonic()

    def _on_mission_status(self, msg: MissionStatus) -> None:
        self._mission_status = msg
        self._mission_status_received_at = time.monotonic()

    def _publish_heartbeat(self) -> None:
        if not self._enabled:
            return

        missing = self._missing_required_states()
        if missing:
            self._log_missing_states(missing)
            return

        if not self._mqtt_connected or self._mqtt_client is None:
            return

        payload = self._build_payload()
        payload_text = json.dumps(payload, ensure_ascii=False)
        result = self._mqtt_client.publish(
            self._mqtt_topic,
            payload_text,
            qos=self._qos,
            retain=self._retain,
        )
        if result.rc != 0:
            self.get_logger().warning(f'Failed to publish MQTT heartbeat, rc={result.rc}')
            return

        if self._debug_log_payload:
            self.get_logger().info(f'MQTT heartbeat payload: {payload_text}')

        self._msg_id += 1

    def _missing_required_states(self) -> list[str]:
        missing = []
        if self._basic_status is None:
            missing.append(self._basic_status_topic)
        if self._battery_status is None:
            missing.append(self._battery_status_topic)
        if self._robot_position is None:
            missing.append(self._robot_position_topic)
        if self._mission_status is None:
            missing.append(self._mission_status_topic)
        return missing

    def _log_missing_states(self, missing: list[str]) -> None:
        now = time.monotonic()
        if (now - self._last_missing_state_log_at) < self._missing_state_log_period_sec:
            return
        self._last_missing_state_log_at = now
        self.get_logger().info(
            'mqtt_forwarder_node waiting for required topics before publishing heartbeat: '
            + ', '.join(missing)
        )

    def _build_payload(self) -> dict[str, Any]:
        basic = self._basic_status
        battery = self._battery_status
        robot_position = self._robot_position
        mission_status = self._mission_status
        assert basic is not None and battery is not None and robot_position is not None and mission_status is not None

        world_position = robot_position.world_position
        quat_z = math.sin(float(world_position.theta) / 2.0)
        quat_w = math.cos(float(world_position.theta) / 2.0)

        return {
            'type': 'upload',
            'method': 'heart',
            'msgid': self._msg_id,
            'message': {
                'runMode': int(basic.control_usage_mode),
                'workStatus': self._work_status_from_mission(mission_status),
                'battery': self._battery_min_percent(battery),
                'healthStatus': 0,
                'motionStatus': int(basic.motion_state),
                'chargeStatus': int(basic.charge),
                'signalStrength': int(self._default_signal_strength),
                'onlineStatus': self._compute_online_status(),
                'location': {
                    'mapId': int(self._map_id),
                    'worldPose': {
                        'orientation': {
                            'w': quat_w,
                            'x': 0.0,
                            'y': 0.0,
                            'z': quat_z,
                        },
                        'position': {
                            'x': float(world_position.x),
                            'y': float(world_position.y),
                            'z': 0.0,
                        },
                    },
                },
            },
        }

    def _battery_min_percent(self, battery: UdpBatteryStatus) -> int:
        min_value = min(float(battery.battery_level_left), float(battery.battery_level_right))
        clipped = max(0.0, min(100.0, min_value))
        return int(round(clipped))

    def _compute_online_status(self) -> int:
        now = time.monotonic()
        timestamps = (
            self._basic_status_received_at,
            self._battery_status_received_at,
            self._robot_position_received_at,
            self._mission_status_received_at,
        )
        if any(stamp <= 0.0 for stamp in timestamps):
            return 1
        if any((now - stamp) > self._status_timeout_sec for stamp in timestamps):
            return 1
        return 0

    def _work_status_from_mission(self, mission_status: MissionStatus) -> int:
        return 1 if int(mission_status.status) == int(MissionStatus.RUNNING) else 0


def main() -> None:
    rclpy.init()
    node = MqttForwarderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
