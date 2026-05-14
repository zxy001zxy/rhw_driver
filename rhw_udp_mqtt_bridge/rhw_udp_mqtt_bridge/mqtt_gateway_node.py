"""mqtt_gateway_node — 统一 MQTT 网关节点。"""
from __future__ import annotations

import importlib
import json
import math
import threading
import time
import uuid
from typing import Any

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String

from rhw_msgs.msg import MissionStatus, RobotPosition, UdpBasicStatus, UdpBatteryStatus, WaypointTask
from rhw_msgs.srv import GetWaypoints, StartMission

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


class MqttGatewayNode(Node):
    """统一 MQTT 网关：心跳、点位同步、任务下发、任务状态上报。"""

    def __init__(self) -> None:
        super().__init__('mqtt_gateway_node')

        self._callback_group = ReentrantCallbackGroup()
        self._msg_id_lock = threading.Lock()

        self._declare_parameters()
        self._read_parameters()

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
        self._received_topics: set[str] = set()
        self._all_required_topics_reported = False
        self._waypoint_sync_pending = False

        self._active_task_id = ''
        self._active_task_started_at = 0.0
        self._last_task_status_signature: tuple[Any, ...] | None = None

        self._create_ros_interfaces()
        self._setup_mqtt()

        self._heartbeat_timer = self.create_timer(
            self._heartbeat_publish_period_sec,
            self._publish_heartbeat,
            callback_group=self._callback_group,
        )
        self._waypoint_sync_retry_timer = self.create_timer(
            self._waypoint_sync_retry_period_sec,
            self._retry_pending_waypoint_sync,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            'mqtt_gateway_node started '
            f'enabled={self._enabled} broker={self._broker_host}:{self._broker_port} '
            f'upload={self._upload_topic} download={self._download_topic}'
        )
        self.get_logger().info(
            'subscribed ROS topics: '
            f'{self._basic_status_topic}, {self._battery_status_topic}, '
            f'{self._robot_position_topic}, {self._mission_status_topic}, '
            f'{self._waypoint_event_topic}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('enabled', True)
        self.declare_parameter('broker_host', '127.0.0.1')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('client_id', 'rhw_mqtt_gateway_DOG001')
        self.declare_parameter('username', '')
        self.declare_parameter('password', '')
        self.declare_parameter('upload_topic', '/robot-dog/DOG001/Upload/Data')
        self.declare_parameter('download_topic', '/robot-dog/DOG001/Download/Data')
        self.declare_parameter('mqtt_topic', '')
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
        self.declare_parameter('waypoint_event_topic', '/waypoint_manager/events')
        self.declare_parameter('mission_start_service', '/mission/start')
        self.declare_parameter('get_waypoints_service', '/waypoint_manager/get_waypoints')
        self.declare_parameter('default_task_map_name', 'factory_map')
        self.declare_parameter('waypoint_sync_on_connect', True)
        self.declare_parameter('waypoint_sync_retry_period_sec', 5.0)
        self.declare_parameter('debug_log_payload', False)
        self.declare_parameter('missing_state_log_period_sec', 5.0)

    def _read_parameters(self) -> None:
        self._enabled = bool(self.get_parameter('enabled').value)
        self._broker_host = str(self.get_parameter('broker_host').value)
        self._broker_port = int(self.get_parameter('broker_port').value)
        self._client_id = str(self.get_parameter('client_id').value)
        self._username = str(self.get_parameter('username').value)
        self._password = str(self.get_parameter('password').value)
        legacy_mqtt_topic = str(self.get_parameter('mqtt_topic').value)
        upload_topic = str(self.get_parameter('upload_topic').value)
        self._upload_topic = legacy_mqtt_topic or upload_topic
        self._download_topic = str(self.get_parameter('download_topic').value)
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
        self._waypoint_event_topic = str(self.get_parameter('waypoint_event_topic').value)
        self._mission_start_service = str(self.get_parameter('mission_start_service').value)
        self._get_waypoints_service = str(self.get_parameter('get_waypoints_service').value)
        self._default_task_map_name = str(self.get_parameter('default_task_map_name').value)
        self._waypoint_sync_on_connect = bool(self.get_parameter('waypoint_sync_on_connect').value)
        self._waypoint_sync_retry_period_sec = max(
            float(self.get_parameter('waypoint_sync_retry_period_sec').value),
            0.5,
        )
        self._debug_log_payload = bool(self.get_parameter('debug_log_payload').value)
        self._missing_state_log_period_sec = max(
            float(self.get_parameter('missing_state_log_period_sec').value),
            0.1,
        )

    def _create_ros_interfaces(self) -> None:
        self.create_subscription(
            UdpBasicStatus,
            self._basic_status_topic,
            self._on_basic_status,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            UdpBatteryStatus,
            self._battery_status_topic,
            self._on_battery_status,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            RobotPosition,
            self._robot_position_topic,
            self._on_robot_position,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            MissionStatus,
            self._mission_status_topic,
            self._on_mission_status,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            String,
            self._waypoint_event_topic,
            self._on_waypoint_event,
            10,
            callback_group=self._callback_group,
        )
        self._mission_start_client = self.create_client(
            StartMission,
            self._mission_start_service,
            callback_group=self._callback_group,
        )
        self._get_waypoints_client = self.create_client(
            GetWaypoints,
            self._get_waypoints_service,
            callback_group=self._callback_group,
        )

    def _setup_mqtt(self) -> None:
        self._mqtt_client = None
        if not self._enabled:
            self.get_logger().warning('mqtt_gateway_node is disabled by parameter enabled=false')
            return

        mqtt_module = ensure_paho_mqtt_imported()
        self._mqtt_client = mqtt_module.Client(
            client_id=self._client_id,
            protocol=mqtt_module.MQTTv311,
        )
        if self._username:
            self._mqtt_client.username_pw_set(self._username, self._password)
        self._mqtt_client.on_connect = self._on_mqtt_connect
        self._mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self._mqtt_client.on_message = self._on_mqtt_message
        self._mqtt_client.connect_async(
            self._broker_host,
            self._broker_port,
            keepalive=self._keep_alive_sec,
        )
        self._mqtt_client.loop_start()

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
            client.subscribe(self._download_topic, qos=self._qos)
            self.get_logger().info(
                f'Connected to MQTT broker {self._broker_host}:{self._broker_port}, '
                f'subscribed={self._download_topic}'
            )
            if self._waypoint_sync_on_connect:
                self._request_waypoint_sync('', reason='mqtt_connect')
        else:
            self.get_logger().warning(
                f'Failed to connect to MQTT broker {self._broker_host}:{self._broker_port}, code={code}'
            )

    def _on_mqtt_disconnect(self, client, userdata, reason_code, properties=None) -> None:
        code = self._reason_code_to_int(reason_code)
        self._mqtt_connected = False
        self.get_logger().warning(f'Disconnected from MQTT broker, code={code}')

    def _on_mqtt_message(self, client, userdata, msg) -> None:
        if msg.topic != self._download_topic:
            return

        try:
            payload = json.loads(msg.payload.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Invalid MQTT payload on {msg.topic}: {exc}')
            return

        method = str(payload.get('method', ''))
        if method == 'task':
            self._handle_task_download(payload)
        else:
            self.get_logger().warning(f'Unsupported MQTT download method: {method}')

    @staticmethod
    def _reason_code_to_int(reason_code: Any) -> int:
        try:
            return int(reason_code)
        except (TypeError, ValueError):
            pass
        try:
            return int(getattr(reason_code, 'value'))
        except (AttributeError, TypeError, ValueError):
            return -1

    def _next_msg_id(self) -> int:
        with self._msg_id_lock:
            msg_id = self._msg_id
            self._msg_id += 1
            return msg_id

    # ================================================================
    #  ROS 状态输入
    # ================================================================

    def _on_basic_status(self, msg: UdpBasicStatus) -> None:
        self._basic_status = msg
        self._basic_status_received_at = time.monotonic()
        self._mark_topic_received(self._basic_status_topic)

    def _on_battery_status(self, msg: UdpBatteryStatus) -> None:
        self._battery_status = msg
        self._battery_status_received_at = time.monotonic()
        self._mark_topic_received(self._battery_status_topic)

    def _on_robot_position(self, msg: RobotPosition) -> None:
        self._robot_position = msg
        self._robot_position_received_at = time.monotonic()
        self._mark_topic_received(self._robot_position_topic)

    def _on_mission_status(self, msg: MissionStatus) -> None:
        self._mission_status = msg
        self._mission_status_received_at = time.monotonic()
        self._mark_topic_received(self._mission_status_topic)
        self._publish_task_status_from_mission(msg)

    def _on_waypoint_event(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Waypoint event parse failed: {exc}')
            return

        map_name = str(payload.get('map_name', ''))
        reason = str(payload.get('reason', 'waypoint_event'))
        self._request_waypoint_sync(map_name, reason=reason)

    def _mark_topic_received(self, topic_name: str) -> None:
        if topic_name not in self._received_topics:
            self._received_topics.add(topic_name)
            self.get_logger().info(f'Received first message on {topic_name}')

        if not self._all_required_topics_reported and not self._missing_required_states():
            self._all_required_topics_reported = True
            self.get_logger().info('All required topics received, MQTT heartbeat publishing is unblocked')

    # ================================================================
    #  MQTT 发布
    # ================================================================

    def _publish_mqtt_payload(self, payload: dict[str, Any], *, label: str) -> bool:
        if not self._enabled:
            return False
        if not self._mqtt_connected or self._mqtt_client is None:
            self.get_logger().debug(f'Skip MQTT publish for {label}: MQTT is not connected')
            return False

        payload_text = json.dumps(payload, ensure_ascii=False)
        result = self._mqtt_client.publish(
            self._upload_topic,
            payload_text,
            qos=self._qos,
            retain=self._retain,
        )
        if result.rc != 0:
            self.get_logger().warning(f'Failed to publish MQTT {label}, rc={result.rc}')
            return False

        if self._debug_log_payload:
            self.get_logger().info(f'MQTT {label} payload: {payload_text}')
        return True

    def _publish_heartbeat(self) -> None:
        if not self._enabled:
            return

        missing = self._missing_required_states()
        if missing:
            self._log_missing_states(missing)
            return

        payload = self._build_heartbeat_payload()
        self._publish_mqtt_payload(payload, label='heartbeat')

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
            'mqtt_gateway_node waiting for required topics before publishing heartbeat: '
            + ', '.join(missing)
        )

    def _build_heartbeat_payload(self) -> dict[str, Any]:
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
            'msgid': self._next_msg_id(),
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

    # ================================================================
    #  任务下发/状态上报
    # ================================================================

    def _handle_task_download(self, payload: dict[str, Any]) -> None:
        message = payload.get('message', {})
        if not isinstance(message, dict):
            self._publish_task_ack(
                task_id='',
                accepted=False,
                error_msg='message must be an object',
                msgid=payload.get('msgid'),
            )
            return

        cmd_type = str(message.get('cmdType', 'create'))
        task_id = str(message.get('taskId', ''))
        map_name = str(
            message.get('mapName')
            or message.get('map_name')
            or self._default_task_map_name
        )
        waypoint_ids = message.get('pointIdList', message.get('waypoint_ids', []))
        if not isinstance(waypoint_ids, list):
            waypoint_ids = []
        waypoint_ids = [str(item) for item in waypoint_ids if str(item)]

        if cmd_type != 'create':
            self._publish_task_ack(
                task_id=task_id,
                accepted=False,
                error_msg=f'unsupported cmdType: {cmd_type}',
                msgid=payload.get('msgid'),
            )
            return

        if not task_id or not map_name or not waypoint_ids:
            self._publish_task_ack(
                task_id=task_id,
                accepted=False,
                error_msg='taskId, mapName/default_task_map_name and pointIdList are required',
                msgid=payload.get('msgid'),
            )
            return

        if not self._mission_start_client.service_is_ready():
            self._publish_task_ack(
                task_id=task_id,
                accepted=False,
                error_msg=f'{self._mission_start_service} service not ready',
                msgid=payload.get('msgid'),
            )
            return

        req = StartMission.Request()
        req.task_id = task_id
        req.map_name = map_name
        req.waypoint_ids = waypoint_ids
        future = self._mission_start_client.call_async(req)
        future.add_done_callback(
            lambda done: self._on_start_mission_response(
                done,
                task_id=task_id,
                waypoint_count=len(waypoint_ids),
                msgid=payload.get('msgid'),
            )
        )
        self.get_logger().info(
            f'MQTT task received: task_id={task_id} map={map_name} waypoints={len(waypoint_ids)}'
        )

    def _on_start_mission_response(
        self,
        future,
        *,
        task_id: str,
        waypoint_count: int,
        msgid: Any,
    ) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._publish_task_ack(
                task_id=task_id,
                accepted=False,
                error_msg=f'/mission/start call failed: {exc}',
                msgid=msgid,
            )
            return

        accepted = int(response.result) == 1
        self._publish_task_ack(
            task_id=task_id,
            accepted=accepted,
            error_msg='' if accepted else str(response.message),
            msgid=msgid,
        )
        if accepted:
            self._active_task_id = task_id
            self._active_task_started_at = time.monotonic()
            self._last_task_status_signature = None
            self.get_logger().info(
                f'MQTT task accepted by mission scheduler: task_id={task_id} waypoints={waypoint_count}'
            )
        else:
            self.get_logger().warning(
                f'MQTT task rejected by mission scheduler: task_id={task_id} reason={response.message}'
            )

    def _publish_task_ack(
        self,
        *,
        task_id: str,
        accepted: bool,
        error_msg: str,
        msgid: Any,
    ) -> None:
        self._publish_task_status(
            task_id=task_id,
            progress=0,
            task_status=2 if accepted else 6,
            code=0 if accepted else 1,
            error_msg=error_msg,
            msgid=msgid,
            duration_ms=0,
            force=True,
        )

    def _publish_task_status_from_mission(self, msg: MissionStatus) -> None:
        if not self._active_task_id:
            return

        task_status = self._task_status_from_mission(msg)
        progress = self._task_progress(msg)
        duration_ms = self._active_task_duration_ms()
        error_msg = msg.message if task_status == 6 else ''
        signature = (
            self._active_task_id,
            task_status,
            progress,
            int(msg.completed_waypoints),
            int(msg.total_waypoints),
            error_msg,
        )
        if signature == self._last_task_status_signature:
            return

        self._last_task_status_signature = signature
        self._publish_task_status(
            task_id=self._active_task_id,
            progress=progress,
            task_status=task_status,
            code=1 if task_status == 6 else 0,
            error_msg=error_msg,
            duration_ms=duration_ms,
        )

        if task_status in (4, 5, 6):
            self._active_task_id = ''
            self._active_task_started_at = 0.0

    def _publish_task_status(
        self,
        *,
        task_id: str,
        progress: int,
        task_status: int,
        code: int,
        error_msg: str,
        duration_ms: int,
        msgid: Any | None = None,
        force: bool = False,
    ) -> None:
        payload = {
            'type': 'response',
            'method': 'task',
            'code': int(code),
            'msgid': int(msgid) if self._is_int_like(msgid) else self._next_msg_id(),
            'message': {
                'taskId': task_id,
                'taskProgress': int(max(0, min(100, progress))),
                'taskDuration': int(max(0, duration_ms)),
                'taskStatus': int(task_status),
                'errorMsg': str(error_msg or ''),
            },
        }
        if self._publish_mqtt_payload(payload, label='task_status') or force:
            self.get_logger().info(
                f'MQTT task status: task_id={task_id} status={task_status} progress={progress}%'
            )

    @staticmethod
    def _task_status_from_mission(msg: MissionStatus) -> int:
        status = int(msg.status)
        if status == int(MissionStatus.RUNNING):
            return 3
        if status == int(MissionStatus.PAUSED):
            return 3
        if status == int(MissionStatus.COMPLETED):
            return 4
        if status == int(MissionStatus.FAILED):
            return 6
        return 5

    @staticmethod
    def _task_progress(msg: MissionStatus) -> int:
        total = int(msg.total_waypoints)
        if total <= 0:
            return 0
        return int(round((int(msg.completed_waypoints) / float(total)) * 100.0))

    def _active_task_duration_ms(self) -> int:
        if self._active_task_started_at <= 0.0:
            return 0
        return int((time.monotonic() - self._active_task_started_at) * 1000.0)

    # ================================================================
    #  点位同步
    # ================================================================

    def _retry_pending_waypoint_sync(self) -> None:
        if self._waypoint_sync_pending and self._mqtt_connected:
            self._request_waypoint_sync('', reason='retry_pending')

    def _request_waypoint_sync(self, map_name: str, *, reason: str) -> None:
        if not self._enabled or not self._mqtt_connected:
            return

        if not self._get_waypoints_client.service_is_ready():
            self._waypoint_sync_pending = True
            self.get_logger().warning(
                f'Skip waypoint MQTT sync: {self._get_waypoints_service} service not ready'
            )
            return

        req = GetWaypoints.Request()
        req.map_name = map_name
        future = self._get_waypoints_client.call_async(req)
        future.add_done_callback(
            lambda done: self._on_get_waypoints_response(
                done,
                requested_map_name=map_name,
                reason=reason,
            )
        )
        if not map_name:
            self._waypoint_sync_pending = False

    def _on_get_waypoints_response(self, future, *, requested_map_name: str, reason: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'GetWaypoints for MQTT sync failed: {exc}')
            return

        if int(response.result) != 1:
            self.get_logger().warning(f'GetWaypoints for MQTT sync rejected: {response.message}')
            return

        map_snapshots = self._group_waypoints_by_map(
            list(response.waypoints),
            requested_map_name=requested_map_name,
        )
        payload = self._build_waypoint_sync_payload(map_snapshots)
        if self._publish_mqtt_payload(payload, label='waypoint_sync'):
            total_points = sum(len(waypoints) for waypoints in map_snapshots.values())
            self.get_logger().info(
                'MQTT waypoint sync published: '
                f'maps={len(map_snapshots)} total_points={total_points} reason={reason}'
            )

    @staticmethod
    def _group_waypoints_by_map(
        waypoints: list[WaypointTask],
        *,
        requested_map_name: str,
    ) -> dict[str, list[WaypointTask]]:
        grouped: dict[str, list[WaypointTask]] = {}
        for waypoint in waypoints:
            map_name = waypoint.map_name or requested_map_name
            grouped.setdefault(map_name, []).append(waypoint)

        if requested_map_name and requested_map_name not in grouped:
            grouped[requested_map_name] = []
        return grouped

    def _build_waypoint_sync_payload(
        self,
        map_snapshots: dict[str, list[WaypointTask]],
    ) -> dict[str, Any]:
        message = []
        for map_name, waypoints in map_snapshots.items():
            point_ids = []
            point_names = []
            for waypoint in waypoints:
                point_ids.append(str(waypoint.waypoint_id))
                point_names.append(str(waypoint.label or waypoint.waypoint_id))

            message.append(
                {
                    'mapId': self._map_id_for_name(map_name),
                    'mapName': map_name,
                    'pointCount': len(point_ids),
                    'pointId': point_ids,
                    'pointName': point_names,
                }
            )

        return {
            'type': 'response',
            'method': 'map',
            'code': 0,
            'msgid': self._next_msg_id(),
            'message': message,
        }

    @staticmethod
    def _map_id_for_name(map_name: str) -> str:
        return uuid.uuid5(uuid.NAMESPACE_URL, f'rhw-map:{map_name}').hex

    # ================================================================
    #  工具
    # ================================================================

    @staticmethod
    def _battery_min_percent(battery: UdpBatteryStatus) -> int:
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

    @staticmethod
    def _work_status_from_mission(mission_status: MissionStatus) -> int:
        return 1 if int(mission_status.status) == int(MissionStatus.RUNNING) else 0

    @staticmethod
    def _is_int_like(value: Any) -> bool:
        try:
            int(value)
            return True
        except (TypeError, ValueError):
            return False


def main() -> None:
    rclpy.init()
    node = MqttGatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
