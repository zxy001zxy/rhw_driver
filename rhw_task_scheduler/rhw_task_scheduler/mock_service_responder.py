"""mock_service_responder — 为 BT mock 测试提供所有下游服务的假响应节点.

功能:
  1. 提供 /move_base_simple/goal        (Goal.srv)       → 模拟导航成功
  2. 提供 /move_base/cancel             (Cancel.srv)     → 模拟取消成功
  3. 提供 /ptz/goto_preset              (PtzGotoPreset)  → 模拟云台跳转成功
  4. 提供 /ptz/capture_image            (CaptureImage)   → 模拟抓拍成功
  5. 提供 /recharge                     (Recharge)       → 模拟回充成功
  6. 订阅 /service_events 话题 → 彩色终端打印每条审计事件

运行方式:
    ros2 run rhw_task_scheduler mock_service_responder

可选参数:
    --ros-args -p response_delay_sec:=0.5 -p nav_result:=3 ...

配合 mock_mission_runner 使用时，可关闭 mission_bt_node 的 mock 模式，
让 BT action 真正发出服务请求，由本节点返回假结果，完整走通审计链路。
"""
from __future__ import annotations

import json
import os
import time
from datetime import datetime

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from rhw_msgs.msg import NavigationStatus
from rhw_msgs.srv import Cancel, Goal
from rhw_msgs.srv import CaptureImage as CaptureImageSrv
from rhw_msgs.srv import PtzGotoPreset as PtzGotoPresetSrv
from rhw_msgs.srv import Recharge as RechargeSrv


# ── 终端颜色 ──────────────────────────────────────────────
_RESET = '\033[0m'
_BOLD = '\033[1m'
_GREEN = '\033[32m'
_YELLOW = '\033[33m'
_CYAN = '\033[36m'
_RED = '\033[31m'
_MAGENTA = '\033[35m'
_DIM = '\033[2m'

_PHASE_COLOR = {
    'request': _CYAN,
    'response': _GREEN,
}

_SRV_SHORT = {
    '/move_base_simple/goal': 'NAV/Goal',
    '/move_base/cancel': 'NAV/Cancel',
    '/ptz/goto_preset': 'PTZ/Preset',
    '/ptz/capture_image': 'PTZ/Capture',
    '/recharge': 'Recharge',
    '/mission/start': 'Mission/Start',
    '/mission/stop': 'Mission/Stop',
    '/mission/pause': 'Mission/Pause',
    '/waypoint_manager/get_waypoints': 'WP/Get',
    '/waypoint_manager/add_waypoint': 'WP/Add',
    '/waypoint_manager/delete_waypoint': 'WP/Del',
}


class MockServiceResponder(Node):
    """提供下游服务的 mock 响应，同时监听 /service_events 审计流."""

    def __init__(self) -> None:
        super().__init__('mock_service_responder')
        self._cb = ReentrantCallbackGroup()

        # ── 可配置参数 ──
        self.declare_parameter('response_delay_sec', 0.3)
        self.declare_parameter('nav_result', 3)          # 3=已到达
        self.declare_parameter('cancel_result', 2)       # 2=已取消
        self.declare_parameter('ptz_preset_result', 1)   # 1=成功
        self.declare_parameter('capture_result', 1)      # 1=成功
        self.declare_parameter('capture_save_dir', '/tmp/rhw_mock_captures')
        self.declare_parameter('recharge_result', 0)     # >=0 成功
        self.declare_parameter('publish_nav_status', True)
        self.declare_parameter('goal_service', '/move_base_simple/goal')
        self.declare_parameter('cancel_service', '/move_base/cancel')
        self.declare_parameter('ptz_goto_preset_service', '/ptz/goto_preset')
        self.declare_parameter('ptz_capture_service', '/ptz/capture_image')
        self.declare_parameter('recharge_service', '/recharge')
        self.declare_parameter('nav_status_topic', '/navigation_status')
        self.declare_parameter('service_events_topic', '/service_events')

        # ── 读参数 ──
        self._delay = float(self.get_parameter('response_delay_sec').value)
        self._nav_result = int(self.get_parameter('nav_result').value)
        self._cancel_result = int(self.get_parameter('cancel_result').value)
        self._ptz_result = int(self.get_parameter('ptz_preset_result').value)
        self._capture_result = int(self.get_parameter('capture_result').value)
        self._capture_dir = str(self.get_parameter('capture_save_dir').value)
        self._recharge_result = int(self.get_parameter('recharge_result').value)
        self._publish_nav_status = bool(self.get_parameter('publish_nav_status').value)

        # ── 创建 mock 服务 ──
        self.create_service(
            Goal,
            str(self.get_parameter('goal_service').value),
            self._handle_goal,
            callback_group=self._cb,
        )
        self.create_service(
            Cancel,
            str(self.get_parameter('cancel_service').value),
            self._handle_cancel,
            callback_group=self._cb,
        )
        self.create_service(
            PtzGotoPresetSrv,
            str(self.get_parameter('ptz_goto_preset_service').value),
            self._handle_ptz_preset,
            callback_group=self._cb,
        )
        self.create_service(
            CaptureImageSrv,
            str(self.get_parameter('ptz_capture_service').value),
            self._handle_capture,
            callback_group=self._cb,
        )
        self.create_service(
            RechargeSrv,
            str(self.get_parameter('recharge_service').value),
            self._handle_recharge,
            callback_group=self._cb,
        )

        # ── 导航状态发布 ──
        if self._publish_nav_status:
            nav_topic = str(self.get_parameter('nav_status_topic').value)
            self._nav_pub = self.create_publisher(NavigationStatus, nav_topic, 10)
        else:
            self._nav_pub = None

        # ── 订阅 /service_events 审计流 ──
        self._event_sub = self.create_subscription(
            String,
            str(self.get_parameter('service_events_topic').value),
            self._on_service_event,
            50,
        )
        self._event_count = 0

        self.get_logger().info(
            f'{_BOLD}{_GREEN}Mock Service Responder 已启动{_RESET}\n'
            f'  delay={self._delay}s  nav_result={self._nav_result}  '
            f'capture_dir={self._capture_dir}\n'
            f'  监听 /service_events 审计事件...'
        )

    # ================================================================
    #  服务处理
    # ================================================================

    def _handle_goal(self, request: Goal.Request, response: Goal.Response) -> Goal.Response:
        nav_type = request.type & 0x0F
        nav_mode = (request.type >> 4) & 0x0F
        type_names = {0: '自由导航', 1: '手绘路径', 2: '录制路径'}
        mode_names = {0: '前进', 1: '后退', 2: '横移'}
        pos = request.goal.pose.position

        self.get_logger().info(
            f'{_CYAN}[Goal]{_RESET} {type_names.get(nav_type, "?")}·{mode_names.get(nav_mode, "?")} '
            f'→ ({pos.x:.2f}, {pos.y:.2f})'
        )

        # 模拟耗时
        if self._delay > 0:
            time.sleep(self._delay)

        # 发布导航中状态
        if self._nav_pub:
            nav_msg = NavigationStatus()
            nav_msg.status = NavigationStatus.STATUS_NAVIGATING
            self._nav_pub.publish(nav_msg)

        response.result = self._nav_result

        # 稍后发布到达状态
        if self._nav_pub and self._nav_result in (1, 3):
            self.create_timer(
                1.0,
                lambda: self._publish_nav_reached(),
                callback_group=self._cb,
            )

        return response

    def _publish_nav_reached(self) -> None:
        """延迟发布导航到达状态."""
        if self._nav_pub:
            msg = NavigationStatus()
            msg.status = NavigationStatus.STATUS_REACHED
            self._nav_pub.publish(msg)
            self.get_logger().info(f'{_GREEN}[NavStatus]{_RESET} 发布 REACHED')

    def _handle_cancel(
        self, request: Cancel.Request, response: Cancel.Response
    ) -> Cancel.Response:
        self.get_logger().info(f'{_YELLOW}[Cancel]{_RESET} cancel={request.cancel}')
        if self._delay > 0:
            time.sleep(self._delay)
        response.result = self._cancel_result
        return response

    def _handle_ptz_preset(
        self, request: PtzGotoPresetSrv.Request, response: PtzGotoPresetSrv.Response
    ) -> PtzGotoPresetSrv.Response:
        self.get_logger().info(
            f'{_MAGENTA}[PTZ Preset]{_RESET} ch={request.channel} preset={request.preset_id}'
        )
        if self._delay > 0:
            time.sleep(self._delay)
        response.result = self._ptz_result
        response.message = 'mock OK' if self._ptz_result == 1 else 'mock FAIL'
        return response

    def _handle_capture(
        self, request: CaptureImageSrv.Request, response: CaptureImageSrv.Response
    ) -> CaptureImageSrv.Response:
        self.get_logger().info(
            f'{_MAGENTA}[Capture]{_RESET} ch={request.channel} url_type={request.url_type}'
        )
        if self._delay > 0:
            time.sleep(self._delay)

        response.result = self._capture_result
        if self._capture_result == 1:
            os.makedirs(self._capture_dir, exist_ok=True)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            file_path = f'{self._capture_dir}/mock_ch{request.channel}_{ts}.jpg'
            # 写一个空文件占位
            try:
                with open(file_path, 'wb') as f:
                    f.write(b'\xff\xd8\xff\xe0')  # minimal JPEG header
                response.file_path = file_path
                response.file_size = 4
                response.saved = True
                response.capture_url = f'file://{file_path}'
            except OSError:
                response.file_path = file_path
                response.saved = False
            response.message = 'mock capture OK'
        else:
            response.message = 'mock capture FAIL'
        return response

    def _handle_recharge(
        self, request: RechargeSrv.Request, response: RechargeSrv.Response
    ) -> RechargeSrv.Response:
        pos = request.recharge_goal.pose.position
        self.get_logger().info(
            f'{_YELLOW}[Recharge]{_RESET} goal=({pos.x:.2f}, {pos.y:.2f})'
        )
        if self._delay > 0:
            time.sleep(self._delay)
        response.result = self._recharge_result
        return response

    # ================================================================
    #  /service_events 审计监听
    # ================================================================

    def _on_service_event(self, msg: String) -> None:
        self._event_count += 1
        try:
            evt = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f'Invalid JSON in /service_events: {msg.data[:100]}')
            return

        ts = evt.get('timestamp', 0)
        ts_str = datetime.fromtimestamp(ts).strftime('%H:%M:%S.%f')[:-3] if ts else '???'
        node_name = evt.get('node', '?')
        service = evt.get('service', '?')
        role = evt.get('role', '?')
        phase = evt.get('phase', '?')
        success = evt.get('success')
        duration = evt.get('duration_ms')
        mock_flag = evt.get('details', {}).get('mock', False) if isinstance(evt.get('details'), dict) else False

        # 缩短服务名
        srv_short = _SRV_SHORT.get(service, service)
        color = _PHASE_COLOR.get(phase, '')
        mock_tag = f' {_DIM}(mock){_RESET}' if mock_flag else ''

        # 成功/失败标记
        if success is True:
            status_icon = f'{_GREEN}✓{_RESET}'
        elif success is False:
            status_icon = f'{_RED}✗{_RESET}'
        else:
            status_icon = '→'

        # 耗时
        dur_str = f' {_DIM}{duration:.1f}ms{_RESET}' if duration is not None else ''

        # 请求/响应摘要
        summary = ''
        if phase == 'request':
            req = evt.get('request', {})
            if isinstance(req, dict):
                parts = []
                for k, v in list(req.items())[:4]:
                    if isinstance(v, dict):
                        inner = ', '.join(f'{ik}={iv}' for ik, iv in list(v.items())[:3])
                        parts.append(f'{k}={{{inner}}}')
                    else:
                        parts.append(f'{k}={v}')
                summary = f' {_DIM}{", ".join(parts)}{_RESET}'
        elif phase == 'response':
            resp = evt.get('response', {})
            if isinstance(resp, dict):
                parts = [f'{k}={v}' for k, v in list(resp.items())[:3]]
                summary = f' {_DIM}{", ".join(parts)}{_RESET}'

        print(
            f'{_DIM}{ts_str}{_RESET} '
            f'{_BOLD}#{self._event_count:03d}{_RESET} '
            f'{status_icon} '
            f'{color}[{phase.upper():8s}]{_RESET} '
            f'{_BOLD}{srv_short:16s}{_RESET} '
            f'{node_name}/{role}'
            f'{mock_tag}{dur_str}{summary}'
        )


# ================================================================
#  入口
# ================================================================

def main() -> None:
    rclpy.init()
    node = MockServiceResponder()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
