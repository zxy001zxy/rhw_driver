"""waypoint_manager — 航点持久化管理节点.

提供 AddWaypoint / DeleteWaypoint / GetWaypoints 三个 Service，
以地图名为 key 将航点信息持久化为 JSON 文件。
"""
from __future__ import annotations

import json
import os
import time
import uuid
from pathlib import Path
from threading import Lock
from typing import Any

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import String

from rhw_msgs.msg import WaypointTask
from rhw_msgs.srv import AddWaypoint, DeleteWaypoint, GetWaypoints
from rhw_task_scheduler.service_audit import ServiceAuditPublisher


class WaypointManagerNode(Node):
    """航点持久化管理节点."""

    def __init__(self) -> None:
        super().__init__('waypoint_manager')

        # ---- 参数 ----
        self.declare_parameter('storage_dir', '~/.rhw/waypoints')
        self.declare_parameter('add_waypoint_service', '/waypoint_manager/add_waypoint')
        self.declare_parameter('delete_waypoint_service', '/waypoint_manager/delete_waypoint')
        self.declare_parameter('get_waypoints_service', '/waypoint_manager/get_waypoints')
        self.declare_parameter('waypoint_event_topic', '/waypoint_manager/events')

        raw_dir = str(self.get_parameter('storage_dir').value)
        self._storage_dir = Path(os.path.expanduser(os.path.expandvars(raw_dir)))
        self._storage_dir.mkdir(parents=True, exist_ok=True)

        add_srv = str(self.get_parameter('add_waypoint_service').value)
        del_srv = str(self.get_parameter('delete_waypoint_service').value)
        get_srv = str(self.get_parameter('get_waypoints_service').value)
        self._waypoint_event_topic = str(self.get_parameter('waypoint_event_topic').value)

        # ---- 内存缓存: map_name -> list[dict] ----
        self._lock = Lock()
        self._waypoints: dict[str, list[dict[str, Any]]] = {}
        self._map_ids: dict[str, str] = {}
        self._load_all()

        # ---- Services ----
        self._service_audit = ServiceAuditPublisher(self)
        self.create_service(AddWaypoint, add_srv, self._handle_add)
        self.create_service(DeleteWaypoint, del_srv, self._handle_delete)
        self.create_service(GetWaypoints, get_srv, self._handle_get)
        self._waypoint_event_pub = self.create_publisher(
            String,
            self._waypoint_event_topic,
            10,
        )

        self.get_logger().info(
            f'waypoint_manager started  storage={self._storage_dir}  '
            f'maps_loaded={len(self._waypoints)}'
        )
        self.get_logger().info(
            f'service audit publisher enabled on {self._service_audit.topic}'
        )
        self.get_logger().info(f'waypoint change events published on {self._waypoint_event_topic}')

    # ================================================================
    #  持久化
    # ================================================================

    def _map_file(self, map_name: str) -> Path:
        """返回指定地图的 JSON 文件路径."""
        safe_name = map_name.replace('/', '_').replace('\\', '_')
        return self._storage_dir / f'{safe_name}.json'

    def _load_all(self) -> None:
        """启动时从磁盘加载所有地图的航点."""
        count = 0
        for fp in self._storage_dir.glob('*.json'):
            try:
                with fp.open('r', encoding='utf-8') as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    map_name = str(data.get('map_name') or fp.stem)
                    self._map_ids[map_name] = str(
                        data.get('map_id') or self._default_map_id(map_name)
                    )
                    raw_waypoints = data.get('waypoints', [])
                    wps = []
                    if isinstance(raw_waypoints, list):
                        for waypoint in raw_waypoints:
                            if not isinstance(waypoint, dict):
                                continue
                            normalized_waypoint = dict(waypoint)
                            normalized_waypoint.setdefault('map_name', map_name)
                            wps.append(normalized_waypoint)
                    self._waypoints[map_name] = wps
                    count += len(wps)
            except (json.JSONDecodeError, OSError) as exc:
                self.get_logger().warning(f'Failed to load {fp}: {exc}')
        self.get_logger().info(f'Loaded {count} waypoints from {len(self._waypoints)} maps')

    @staticmethod
    def _default_map_id(map_name: str) -> str:
        """为地图名生成稳定的唯一标识。"""
        return uuid.uuid5(uuid.NAMESPACE_URL, f'rhw-map:{map_name}').hex

    def _ensure_map_id(self, map_name: str) -> str:
        map_id = self._map_ids.get(map_name)
        if map_id:
            return map_id

        map_id = self._default_map_id(map_name)
        self._map_ids[map_name] = map_id
        return map_id

    def _waypoints_for_storage(self, map_name: str) -> list[dict[str, Any]]:
        """返回写盘格式，避免在每条 waypoint 中重复保存 map_name."""
        stored_waypoints = []
        for waypoint in self._waypoints.get(map_name, []):
            if not isinstance(waypoint, dict):
                continue
            stored_waypoint = dict(waypoint)
            stored_waypoint.pop('map_name', None)
            stored_waypoints.append(stored_waypoint)
        return stored_waypoints

    def _save_map(self, map_name: str) -> None:
        """将指定地图的航点回写到 JSON 文件."""
        fp = self._map_file(map_name)
        payload = {
            'map_name': map_name,
            'map_id': self._ensure_map_id(map_name),
            'waypoints': self._waypoints_for_storage(map_name),
        }
        try:
            with fp.open('w', encoding='utf-8') as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
        except OSError as exc:
            self.get_logger().error(f'Failed to save {fp}: {exc}')

    # ================================================================
    #  消息转换
    # ================================================================

    @staticmethod
    def _wp_to_dict(wp: WaypointTask) -> dict[str, Any]:
        return {
            'waypoint_id': wp.waypoint_id,
            'map_name': wp.map_name,
            'pose': {'x': wp.pose.x, 'y': wp.pose.y, 'theta': wp.pose.theta},
            'waypoint_type': int(wp.waypoint_type),
            'label': wp.label,
            'task_params': wp.task_params,
        }

    @staticmethod
    def _dict_to_wp(d: dict[str, Any]) -> WaypointTask:
        wp = WaypointTask()
        wp.waypoint_id = str(d.get('waypoint_id', ''))
        wp.map_name = str(d.get('map_name', ''))
        pose = d.get('pose', {})
        wp.pose = Pose2D(
            x=float(pose.get('x', 0.0)),
            y=float(pose.get('y', 0.0)),
            theta=float(pose.get('theta', 0.0)),
        )
        wp.waypoint_type = int(d.get('waypoint_type', 0))
        wp.label = str(d.get('label', ''))
        wp.task_params = str(d.get('task_params', ''))
        return wp

    # ================================================================
    #  点位变更事件
    # ================================================================

    def _publish_waypoint_event(self, map_name: str, reason: str) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'map_name': map_name,
                'reason': reason,
                'timestamp': time.time(),
            },
            ensure_ascii=False,
        )
        self._waypoint_event_pub.publish(msg)

    # ================================================================
    #  Service Handlers
    # ================================================================

    def _handle_add(
        self, request: AddWaypoint.Request, response: AddWaypoint.Response
    ) -> AddWaypoint.Response:
        started_at = time.monotonic()
        self._service_audit.publish(
            service='/waypoint_manager/add_waypoint',
            role='server',
            phase='request',
            request=request,
        )
        wp = request.waypoint
        map_name = wp.map_name
        wid = wp.waypoint_id

        if not map_name:
            response.result = 0
            response.message = 'map_name is required'
            self._service_audit.publish(
                service='/waypoint_manager/add_waypoint',
                role='server',
                phase='response',
                request=request,
                response=response,
                success=False,
                duration_ms=(time.monotonic() - started_at) * 1000.0,
            )
            return response
        if not wid:
            response.result = 0
            response.message = 'waypoint_id is required'
            self._service_audit.publish(
                service='/waypoint_manager/add_waypoint',
                role='server',
                phase='response',
                request=request,
                response=response,
                success=False,
                duration_ms=(time.monotonic() - started_at) * 1000.0,
            )
            return response

        with self._lock:
            wps = self._waypoints.setdefault(map_name, [])
            self._ensure_map_id(map_name)
            # 检查 ID 唯一性
            for existing in wps:
                if existing.get('waypoint_id') == wid:
                    response.result = 0
                    response.message = f'waypoint_id "{wid}" already exists in map "{map_name}"'
                    self._service_audit.publish(
                        service='/waypoint_manager/add_waypoint',
                        role='server',
                        phase='response',
                        request=request,
                        response=response,
                        success=False,
                        duration_ms=(time.monotonic() - started_at) * 1000.0,
                    )
                    return response
            wps.append(self._wp_to_dict(wp))
            self._save_map(map_name)

        self.get_logger().info(f'Added waypoint {wid} to map {map_name}')
        self._publish_waypoint_event(map_name, reason='add_waypoint')
        response.result = 1
        response.message = 'ok'
        self._service_audit.publish(
            service='/waypoint_manager/add_waypoint',
            role='server',
            phase='response',
            request=request,
            response=response,
            success=True,
            duration_ms=(time.monotonic() - started_at) * 1000.0,
        )
        return response

    def _handle_delete(
        self, request: DeleteWaypoint.Request, response: DeleteWaypoint.Response
    ) -> DeleteWaypoint.Response:
        started_at = time.monotonic()
        self._service_audit.publish(
            service='/waypoint_manager/delete_waypoint',
            role='server',
            phase='request',
            request=request,
        )
        map_name = request.map_name
        wid = request.waypoint_id

        if not map_name or not wid:
            response.result = 0
            response.message = 'map_name and waypoint_id are required'
            self._service_audit.publish(
                service='/waypoint_manager/delete_waypoint',
                role='server',
                phase='response',
                request=request,
                response=response,
                success=False,
                duration_ms=(time.monotonic() - started_at) * 1000.0,
            )
            return response

        with self._lock:
            wps = self._waypoints.get(map_name)
            if wps is None:
                response.result = 0
                response.message = f'map "{map_name}" not found'
                self._service_audit.publish(
                    service='/waypoint_manager/delete_waypoint',
                    role='server',
                    phase='response',
                    request=request,
                    response=response,
                    success=False,
                    duration_ms=(time.monotonic() - started_at) * 1000.0,
                )
                return response

            original_len = len(wps)
            self._waypoints[map_name] = [w for w in wps if w.get('waypoint_id') != wid]

            if len(self._waypoints[map_name]) == original_len:
                response.result = 0
                response.message = f'waypoint_id "{wid}" not found in map "{map_name}"'
                self._service_audit.publish(
                    service='/waypoint_manager/delete_waypoint',
                    role='server',
                    phase='response',
                    request=request,
                    response=response,
                    success=False,
                    duration_ms=(time.monotonic() - started_at) * 1000.0,
                )
                return response

            self._save_map(map_name)

            # 如果地图为空则清理文件
            if not self._waypoints[map_name]:
                del self._waypoints[map_name]
                self._map_ids.pop(map_name, None)
                fp = self._map_file(map_name)
                if fp.exists():
                    fp.unlink()

        self.get_logger().info(f'Deleted waypoint {wid} from map {map_name}')
        self._publish_waypoint_event(map_name, reason='delete_waypoint')
        response.result = 1
        response.message = 'ok'
        self._service_audit.publish(
            service='/waypoint_manager/delete_waypoint',
            role='server',
            phase='response',
            request=request,
            response=response,
            success=True,
            duration_ms=(time.monotonic() - started_at) * 1000.0,
        )
        return response

    def _handle_get(
        self, request: GetWaypoints.Request, response: GetWaypoints.Response
    ) -> GetWaypoints.Response:
        started_at = time.monotonic()
        self._service_audit.publish(
            service='/waypoint_manager/get_waypoints',
            role='server',
            phase='request',
            request=request,
        )
        map_name = request.map_name

        with self._lock:
            if map_name:
                wps = self._waypoints.get(map_name, [])
            else:
                # 返回所有地图的航点
                wps = []
                for v in self._waypoints.values():
                    wps.extend(v)

        response.result = 1
        response.waypoints = [self._dict_to_wp(d) for d in wps]
        response.message = f'{len(response.waypoints)} waypoints'
        self._service_audit.publish(
            service='/waypoint_manager/get_waypoints',
            role='server',
            phase='response',
            request=request,
            response=response,
            success=True,
            duration_ms=(time.monotonic() - started_at) * 1000.0,
        )
        return response


def main() -> None:
    rclpy.init()
    node = WaypointManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
