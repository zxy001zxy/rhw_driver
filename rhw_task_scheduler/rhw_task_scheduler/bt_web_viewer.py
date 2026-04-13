#!/usr/bin/env python3
"""bt_web_viewer — 行为树 Web 实时可视化.

功能:
  - 订阅 /mission_bt_node/snapshots (BehaviourTree msg)
  - HTTP 端口 8765 提供页面, WebSocket 端口 8766 推送数据
  - 浏览器打开即可看到实时行为树状态，自动刷新

启动:
  ros2 run rhw_task_scheduler bt_web_viewer
  ros2 run rhw_task_scheduler bt_web_viewer --ros-args -p http_port:=8765 -p ws_port:=8766

然后浏览器打开 http://localhost:8765
"""
from __future__ import annotations

import asyncio
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Any, Set

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from py_trees_ros_interfaces.msg import BehaviourTree as BehaviourTreeMsg
    _HAS_INTERFACES = True
except ImportError:
    _HAS_INTERFACES = False

import websockets
import websockets.asyncio.server

# ── py_trees_ros_interfaces 常量映射 ─────────────────────────────
# Behaviour.msg 中定义:  INVALID=1, RUNNING=2, SUCCESS=3, FAILURE=4
_STATUS_MAP = {1: "INVALID", 2: "RUNNING", 3: "SUCCESS", 4: "FAILURE"}
# Behaviour.msg 中定义:  UNKNOWN_TYPE=0, BEHAVIOUR=1, SEQUENCE=2, SELECTOR=3, PARALLEL=4, CHOOSER=5, DECORATOR=6
_TYPE_MAP = {
    0: "UNKNOWN", 1: "BEHAVIOUR", 2: "SEQUENCE", 3: "SELECTOR",
    4: "PARALLEL", 5: "CHOOSER", 6: "DECORATOR",
}


def _build_html(ws_port: int) -> str:
    """构建 HTML 页面, WebSocket 端口硬编码在 JS 中."""
    return (
        '<!DOCTYPE html>\n<html lang="zh">\n<head>\n'
        '<meta charset="utf-8">\n<title>BT Live Viewer</title>\n'
        '<style>\n'
        '  * { margin:0; padding:0; box-sizing:border-box; }\n'
        '  body { font-family:"Segoe UI","PingFang SC",sans-serif; background:#1e1e2e; color:#cdd6f4; }\n'
        '  #header { background:#313244; padding:10px 20px; display:flex; justify-content:space-between; align-items:center; }\n'
        '  #header h1 { font-size:18px; font-weight:600; }\n'
        '  #status { font-size:13px; padding:4px 12px; border-radius:12px; }\n'
        '  .connected { background:#a6e3a1; color:#1e1e2e; }\n'
        '  .disconnected { background:#f38ba8; color:#1e1e2e; }\n'
        '  #info { padding:8px 20px; font-size:13px; color:#a6adc8; background:#181825; }\n'
        '  #main { display:grid; grid-template-columns:minmax(420px, 2fr) minmax(360px, 1fr); height:calc(100vh - 88px); }\n'
        '  .panel { min-height:0; overflow:auto; }\n'
        '  .panel-title { position:sticky; top:0; background:#181825; color:#bac2de; padding:10px 16px; font-size:13px; border-bottom:1px solid #313244; z-index:1; }\n'
        '  #tree-container { padding:16px; overflow:auto; }\n'
        '  #events-panel { border-left:1px solid #313244; background:#11111b; }\n'
        '  #events-container { padding:12px; display:flex; flex-direction:column; gap:10px; }\n'
        '  .node { margin:2px 0; padding:6px 14px; border-radius:6px; font-size:14px;\n'
        '          font-family:"JetBrains Mono","Fira Code",monospace; white-space:nowrap; }\n'
        '  .indent { display:inline-block; }\n'
        '  .status-RUNNING  { background:#89b4fa22; border-left:3px solid #89b4fa; color:#89b4fa; }\n'
        '  .status-SUCCESS  { background:#a6e3a122; border-left:3px solid #a6e3a1; color:#a6e3a1; }\n'
        '  .status-FAILURE  { background:#f38ba822; border-left:3px solid #f38ba8; color:#f38ba8; }\n'
        '  .status-INVALID  { background:#6c708622; border-left:3px solid #6c7086; color:#6c7086; }\n'
        '  .type-tag { font-size:11px; padding:1px 6px; border-radius:3px; margin-right:6px;\n'
        '              background:#45475a; color:#bac2de; }\n'
        '  .tick-info { font-size:12px; color:#585b70; margin-left:12px; }\n'
        '  #no-data { text-align:center; padding:80px 20px; color:#585b70; font-size:16px; }\n'
        '  .event-card { border:1px solid #313244; border-radius:10px; background:#1e1e2e; padding:10px 12px; }\n'
        '  .event-meta { display:flex; gap:8px; flex-wrap:wrap; align-items:center; margin-bottom:6px; }\n'
        '  .event-chip { font-size:11px; padding:2px 8px; border-radius:999px; background:#313244; color:#cdd6f4; }\n'
        '  .role-server { background:#94e2d522; color:#94e2d5; }\n'
        '  .role-client { background:#f9e2af22; color:#f9e2af; }\n'
        '  .phase-request { background:#89b4fa22; color:#89b4fa; }\n'
        '  .phase-response { background:#a6e3a122; color:#a6e3a1; }\n'
        '  .success-true { background:#a6e3a122; color:#a6e3a1; }\n'
        '  .success-false { background:#f38ba822; color:#f38ba8; }\n'
        '  .event-service { font-weight:600; margin-bottom:6px; }\n'
        '  .event-time { color:#7f849c; font-size:12px; }\n'
        '  .event-body { display:grid; gap:6px; }\n'
        '  .event-label { color:#bac2de; font-size:12px; }\n'
        '  .event-json { margin:0; max-height:180px; overflow:auto; border-radius:8px; background:#181825; padding:8px; color:#cdd6f4; font-size:12px; line-height:1.4; }\n'
        '</style>\n</head>\n<body>\n'
        '<div id="header">\n'
        '  <h1>\U0001f333 Behaviour Tree + Service Timeline</h1>\n'
        '  <span id="status" class="disconnected">\u65ad\u5f00</span>\n'
        '</div>\n'
        '<div id="info">\u6b63\u5728\u8fde\u63a5 WebSocket ...</div>\n'
        '<div id="main">\n'
        '  <div class="panel">\n'
        '    <div class="panel-title">行为树实时状态</div>\n'
        '    <div id="tree-container">\n'
        '      <div id="no-data">\u7b49\u5f85\u884c\u4e3a\u6811\u5feb\u7167 ...</div>\n'
        '    </div>\n'
        '  </div>\n'
        '  <div id="events-panel" class="panel">\n'
        '    <div class="panel-title">服务事件时间线</div>\n'
        '    <div id="events-container">\n'
        '      <div id="no-data">\u7b49\u5f85\u670d\u52a1\u4e8b\u4ef6 ...</div>\n'
        '    </div>\n'
        '  </div>\n'
        '</div>\n'
        '<script>\n'
        f'const WS_PORT = {ws_port};\n'
        'const WS_HOST = location.hostname || "localhost";\n'
        'const WS_URL = "ws://" + WS_HOST + ":" + WS_PORT;\n'
        'let ws;\n'
        'let tickCount = 0;\n'
        'let lastUpdate = null;\n'
        'let latestState = {tree: null, service_events: []};\n'
        '\n'
        'document.getElementById("info").textContent = "\u8fde\u63a5 " + WS_URL + " ...";\n'
        '\n'
        'function connect() {\n'
        '  ws = new WebSocket(WS_URL);\n'
        '  ws.onopen = () => {\n'
        '    document.getElementById("status").className = "connected";\n'
        '    document.getElementById("status").textContent = "\u5df2\u8fde\u63a5";\n'
        '    document.getElementById("info").textContent = "\u5df2\u8fde\u63a5 " + WS_URL + "\uff0c\u7b49\u5f85\u6570\u636e ...";\n'
        '  };\n'
        '  ws.onclose = () => {\n'
        '    document.getElementById("status").className = "disconnected";\n'
        '    document.getElementById("status").textContent = "\u65ad\u5f00";\n'
        '    setTimeout(connect, 2000);\n'
        '  };\n'
        '  ws.onerror = () => { ws.close(); };\n'
        '  ws.onmessage = (evt) => {\n'
        '    const data = JSON.parse(evt.data);\n'
        '    latestState = data;\n'
        '    if (data.tree) {\n'
        '      tickCount++;\n'
        '      lastUpdate = new Date();\n'
        '      renderTree(data.tree);\n'
        '    }\n'
        '    renderEvents(data.service_events || []);\n'
        '  };\n'
        '}\n'
        '\n'
        'function renderTree(data) {\n'
        '  const behaviours = data.behaviours || [];\n'
        '  if (behaviours.length === 0) return;\n'
        '  const nodeMap = {};\n'
        '  behaviours.forEach(b => { nodeMap[b.own_id] = b; });\n'
        '  let root = behaviours.find(b =>\n'
        '    !b.parent_id || b.parent_id === "00000000000000000000000000000000"\n'
        '  ) || behaviours[0];\n'
        '  const childrenMap = {};\n'
        '  behaviours.forEach(b => {\n'
        '    b.child_ids.forEach(cid => {\n'
        '      if (!childrenMap[b.own_id]) childrenMap[b.own_id] = [];\n'
        '      childrenMap[b.own_id].push(cid);\n'
        '    });\n'
        '  });\n'
        '  const rootStatus = root ? (root.status || "INVALID") : "?";\n'
        '  document.getElementById("info").innerHTML =\n'
        '    "Tick #" + tickCount + " | \u8282\u70b9\u6570: " + behaviours.length +\n'
        '    " | \u6839\u72b6\u6001: <b>" + rootStatus + "</b>" +\n'
        '    " | \u66f4\u65b0: " + lastUpdate.toLocaleTimeString();\n'
        '  const container = document.getElementById("tree-container");\n'
        '  let html = "";\n'
        '  function renderNode(id, depth) {\n'
        '    const n = nodeMap[id];\n'
        '    if (!n) return;\n'
        '    const status = n.status || "INVALID";\n'
        '    const type = n.type || "BEHAVIOUR";\n'
        '    const icon = type.includes("SEQUENCE") ? "\u2192" :\n'
        '                 type.includes("SELECTOR") ? "?" :\n'
        '                 type.includes("PARALLEL") ? "\u21c9" :\n'
        '                 type.includes("DECORATOR") ? "\u25c7" : "\u25cf";\n'
        '    html += \'<div class="node status-\' + status + \'">\';\n'
        '    html += \'<span class="indent" style="width:\' + (depth*24) + \'px"></span>\';\n'
        '    html += \'<span class="type-tag">\' + icon + " " + type + "</span>";\n'
        '    html += "<b>" + (n.name || "?") + "</b>";\n'
        '    html += \'<span class="tick-info">\' + status + "</span>";\n'
        '    html += "</div>";\n'
        '    const children = childrenMap[id] || [];\n'
        '    children.forEach(cid => renderNode(cid, depth + 1));\n'
        '  }\n'
        '  renderNode(root.own_id, 0);\n'
        '  container.innerHTML = html || \'<div id="no-data">\u7a7a\u6811</div>\';\n'
        '}\n'
        '\n'
        'function prettyJson(value) {\n'
        '  return JSON.stringify(value ?? {}, null, 2);\n'
        '}\n'
        '\n'
        'function renderEvents(events) {\n'
        '  const container = document.getElementById("events-container");\n'
        '  if (!events.length) {\n'
        '    container.innerHTML = \'<div id="no-data">\u7b49\u5f85\u670d\u52a1\u4e8b\u4ef6 ...</div>\';\n'
        '    return;\n'
        '  }\n'
        '  container.innerHTML = events.slice().reverse().map((event) => {\n'
        '    const when = event.timestamp ? new Date(event.timestamp * 1000).toLocaleTimeString() : "?";\n'
        '    const successClass = event.success === true ? "success-true" : (event.success === false ? "success-false" : "");\n'
        '    const successText = event.success === true ? "success" : (event.success === false ? "failed" : "pending");\n'
        '    let body = "";\n'
        '    if (event.request !== undefined) {\n'
        '      body += \'<div><div class="event-label">request</div><pre class="event-json">\' + escapeHtml(prettyJson(event.request)) + "</pre></div>";\n'
        '    }\n'
        '    if (event.response !== undefined) {\n'
        '      body += \'<div><div class="event-label">response</div><pre class="event-json">\' + escapeHtml(prettyJson(event.response)) + "</pre></div>";\n'
        '    }\n'
        '    if (event.details !== undefined) {\n'
        '      body += \'<div><div class="event-label">details</div><pre class="event-json">\' + escapeHtml(prettyJson(event.details)) + "</pre></div>";\n'
        '    }\n'
        '    return \'<div class="event-card">\' +\n'
        '      \'<div class="event-service">\' + escapeHtml(event.service || "?") + "</div>" +\n'
        '      \'<div class="event-meta">\' +\n'
        '        \'<span class="event-chip role-\' + escapeHtml(event.role || "unknown") + \'">\' + escapeHtml(event.role || "unknown") + "</span>" +\n'
        '        \'<span class="event-chip phase-\' + escapeHtml(event.phase || "unknown") + \'">\' + escapeHtml(event.phase || "unknown") + "</span>" +\n'
        '        \'<span class="event-chip \'+ successClass + \'">\' + escapeHtml(successText) + "</span>" +\n'
        '        (event.duration_ms !== undefined ? \'<span class="event-chip">\' + event.duration_ms + " ms</span>" : "") +\n'
        '        \'<span class="event-time">\' + when + "</span>" +\n'
        '      "</div>" +\n'
        '      \'<div class="event-body">\' + body + "</div>" +\n'
        '    "</div>";\n'
        '  }).join("");\n'
        '}\n'
        '\n'
        'function escapeHtml(text) {\n'
        '  return String(text)\n'
        '    .replaceAll("&", "&amp;")\n'
        '    .replaceAll("<", "&lt;")\n'
        '    .replaceAll(">", "&gt;");\n'
        '}\n'
        '\n'
        'connect();\n'
        '</script>\n</body>\n</html>'
    )


class BtWebViewer(Node):
    """订阅行为树快照并通过 HTTP + WebSocket 提供实时可视化."""

    def __init__(self) -> None:
        super().__init__('bt_web_viewer')
        self.declare_parameter('http_port', 8765)
        self.declare_parameter('ws_port', 8766)
        self.declare_parameter('snapshot_topic', '/mission_bt_node/snapshots')
        self.declare_parameter('service_events_topic', '/service_events')
        self.declare_parameter('service_events_buffer_size', 100)

        self._http_port = int(self.get_parameter('http_port').value)
        self._ws_port = int(self.get_parameter('ws_port').value)
        self._snapshot_topic = str(self.get_parameter('snapshot_topic').value)
        self._service_events_topic = str(self.get_parameter('service_events_topic').value)
        self._service_events_buffer_size = max(
            int(self.get_parameter('service_events_buffer_size').value), 10
        )

        self._ws_clients: Set[websockets.asyncio.server.ServerConnection] = set()
        self._latest_tree: dict[str, Any] | None = None
        self._service_events: list[dict[str, Any]] = []
        self._loop: asyncio.AbstractEventLoop | None = None

        if not _HAS_INTERFACES:
            self.get_logger().error(
                'py_trees_ros_interfaces not found. '
                'Install: sudo apt install ros-humble-py-trees-ros-interfaces'
            )
            return

        # ROS 订阅
        self.create_subscription(
            BehaviourTreeMsg,
            self._snapshot_topic,
            self._on_snapshot,
            10,
        )
        self.create_subscription(
            String,
            self._service_events_topic,
            self._on_service_event,
            50,
        )

        # HTTP 服务器线程
        self._http_thread = threading.Thread(target=self._run_http, daemon=True)
        self._http_thread.start()

        # WebSocket 服务器线程
        self._ws_thread = threading.Thread(target=self._run_ws, daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            f'BT Web Viewer started:\n'
            f'  HTTP  -> http://localhost:{self._http_port}\n'
            f'  WS    -> ws://localhost:{self._ws_port}\n'
            f'  Tree  -> {self._snapshot_topic}\n'
            f'  Audit -> {self._service_events_topic}'
        )

    # ── UUID 转换 ────────────────────────────────────────────────
    @staticmethod
    def _uuid_to_hex(u) -> str:
        try:
            return bytes(u.uuid).hex()
        except Exception:
            return str(u)

    # ── ROS 回调 ─────────────────────────────────────────────────
    def _on_snapshot(self, msg: BehaviourTreeMsg) -> None:
        behaviours = []
        for b in msg.behaviours:
            behaviours.append({
                'name': b.name,
                'class_name': b.class_name,
                'own_id': self._uuid_to_hex(b.own_id),
                'parent_id': self._uuid_to_hex(b.parent_id),
                'child_ids': [self._uuid_to_hex(c) for c in b.child_ids],
                'status': _STATUS_MAP.get(b.status, str(b.status)),
                'type': _TYPE_MAP.get(b.type, str(b.type)),
                'message': b.message,
                'is_active': b.is_active,
            })

        self._latest_tree = {'behaviours': behaviours}
        self._push_state()

    def _on_service_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            event = {
                'timestamp': None,
                'service': 'unknown',
                'role': 'unknown',
                'phase': 'unknown',
                'details': {'raw': msg.data},
            }
        self._service_events.append(event)
        if len(self._service_events) > self._service_events_buffer_size:
            self._service_events = self._service_events[-self._service_events_buffer_size:]
        self._push_state()

    def _push_state(self) -> None:
        payload = json.dumps(
            {
                'tree': self._latest_tree,
                'service_events': self._service_events,
            },
            ensure_ascii=False,
        )

        # 跨线程推送给 WebSocket 客户端
        loop = self._loop
        if loop is not None and loop.is_running():
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), loop)

    async def _broadcast(self, payload: str) -> None:
        dead = set()
        for ws in set(self._ws_clients):
            try:
                await ws.send(payload)
            except Exception:
                dead.add(ws)
        self._ws_clients -= dead

    # ── HTTP 服务器 ──────────────────────────────────────────────
    def _run_http(self) -> None:
        html_bytes = _build_html(self._ws_port).encode('utf-8')
        node_logger = self.get_logger()

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self_handler):
                self_handler.send_response(200)
                self_handler.send_header('Content-Type', 'text/html; charset=utf-8')
                self_handler.send_header('Content-Length', str(len(html_bytes)))
                self_handler.end_headers()
                self_handler.wfile.write(html_bytes)

            def log_message(self_handler, fmt, *args):
                pass

        server = HTTPServer(('0.0.0.0', self._http_port), Handler)
        node_logger.info(f'HTTP listening on 0.0.0.0:{self._http_port}')
        server.serve_forever()

    # ── WebSocket 服务器 ─────────────────────────────────────────
    def _run_ws(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._ws_serve())

    async def _ws_serve(self) -> None:
        async with websockets.asyncio.server.serve(
            self._ws_handler,
            '0.0.0.0',
            self._ws_port,
        ):
            self.get_logger().info(f'WebSocket listening on 0.0.0.0:{self._ws_port}')
            await asyncio.get_event_loop().create_future()  # run forever

    async def _ws_handler(self, ws) -> None:
        addr = ws.remote_address
        self.get_logger().info(f'WS client connected: {addr}')
        self._ws_clients.add(ws)
        try:
            # 立即发送最新快照
            await ws.send(json.dumps({
                'tree': self._latest_tree,
                'service_events': self._service_events,
            }, ensure_ascii=False))
            # 保持连接 (只做推送, 忽略客户端消息)
            async for _ in ws:
                pass
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._ws_clients.discard(ws)
            self.get_logger().info(f'WS client disconnected: {addr}')


def main() -> None:
    rclpy.init()
    node = BtWebViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
