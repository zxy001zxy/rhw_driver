"""waypoint_manager — 航点持久化管理节点.

提供 AddWaypoint / DeleteWaypoint / GetWaypoints 三个 Service，
以地图名为 key 将航点信息持久化为 JSON 文件。
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from threading import Lock
from typing import Any

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node

from rhw_msgs.msg import WaypointTask
from rhw_msgs.srv import AddWaypoint, DeleteWaypoint, GetWaypoints


class WaypointManagerNode(Node):
    """航点持久化管理节点."""

    def __init__(self) -> None:
        super().__init__('waypoint_manager')

        # ---- 参数 ----
        self.declare_parameter('storage_dir', '~/.rhw/waypoints')
        self.declare_parameter('add_waypoint_service', '/waypoint_manager/add_waypoint')
        self.declare_parameter('delete_waypoint_service', '/waypoint_manager/delete_waypoint')
        self.declare_parameter('get_waypoints_service', '/waypoint_manager/get_waypoints')

        raw_dir = str(self.get_parameter('storage_dir').value)
        self._storage_dir = Path(os.path.expanduser(os.path.expandvars(raw_dir)))
        self._storage_dir.mkdir(parents=True, exist_ok=True)

        add_srv = str(self.get_parameter('add_waypoint_service').value)
        del_srv = str(self.get_parameter('delete_waypoint_service').value)
        get_srv = str(self.get_parameter('get_waypoints_service').value)

        # ---- 内存缓存: map_name -> list[dict] ----
        self._lock = Lock()
        self._waypoints: dict[str, list[dict[str, Any]]] = {}
        self._load_all()

        # ---- Services ----
        self.create_service(AddWaypoint, add_srv, self._handle_add)
        self.create_service(DeleteWaypoint, del_srv, self._handle_delete)
        self.create_service(GetWaypoints, get_srv, self._handle_get)

        self.get_logger().info(
            f'waypoint_manager started  storage={self._storage_dir}  '
            f'maps_loaded={len(self._waypoints)}'
        )

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
                    map_name = data.get('map_name', fp.stem)
                    wps = data.get('waypoints', [])
                    self._waypoints[map_name] = list(wps)
                    count += len(wps)
            except (json.JSONDecodeError, OSError) as exc:
                self.get_logger().warning(f'Failed to load {fp}: {exc}')
        self.get_logger().info(f'Loaded {count} waypoints from {len(self._waypoints)} maps')

    def _save_map(self, map_name: str) -> None:
        """将指定地图的航点回写到 JSON 文件."""
        fp = self._map_file(map_name)
        payload = {
            'map_name': map_name,
            'waypoints': self._waypoints.get(map_name, []),
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
    #  Service Handlers
    # ================================================================

    def _handle_add(
        self, request: AddWaypoint.Request, response: AddWaypoint.Response
    ) -> AddWaypoint.Response:
        wp = request.waypoint
        map_name = wp.map_name
        wid = wp.waypoint_id

        if not map_name:
            response.result = 0
            response.message = 'map_name is required'
            return response
        if not wid:
            response.result = 0
            response.message = 'waypoint_id is required'
            return response

        with self._lock:
            wps = self._waypoints.setdefault(map_name, [])
            # 检查 ID 唯一性
            for existing in wps:
                if existing.get('waypoint_id') == wid:
                    response.result = 0
                    response.message = f'waypoint_id "{wid}" already exists in map "{map_name}"'
                    return response
            wps.append(self._wp_to_dict(wp))
            self._save_map(map_name)

        self.get_logger().info(f'Added waypoint {wid} to map {map_name}')
        response.result = 1
        response.message = 'ok'
        return response

    def _handle_delete(
        self, request: DeleteWaypoint.Request, response: DeleteWaypoint.Response
    ) -> DeleteWaypoint.Response:
        map_name = request.map_name
        wid = request.waypoint_id

        if not map_name or not wid:
            response.result = 0
            response.message = 'map_name and waypoint_id are required'
            return response

        with self._lock:
            wps = self._waypoints.get(map_name)
            if wps is None:
                response.result = 0
                response.message = f'map "{map_name}" not found'
                return response

            original_len = len(wps)
            self._waypoints[map_name] = [w for w in wps if w.get('waypoint_id') != wid]

            if len(self._waypoints[map_name]) == original_len:
                response.result = 0
                response.message = f'waypoint_id "{wid}" not found in map "{map_name}"'
                return response

            self._save_map(map_name)

            # 如果地图为空则清理文件
            if not self._waypoints[map_name]:
                del self._waypoints[map_name]
                fp = self._map_file(map_name)
                if fp.exists():
                    fp.unlink()

        self.get_logger().info(f'Deleted waypoint {wid} from map {map_name}')
        response.result = 1
        response.message = 'ok'
        return response

    def _handle_get(
        self, request: GetWaypoints.Request, response: GetWaypoints.Response
    ) -> GetWaypoints.Response:
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
        rclpy.shutdown()
