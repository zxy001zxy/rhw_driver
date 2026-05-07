# rhw_udp_mqtt_bridge

用于解析 UDP 信息并发布为结构化 ROS 2 话题，以及后续将 ROS 2 话题转发为 MQTT 协议。

## 当前状态

当前已完成：

- `UDP -> ROS 2`：解析并发布 `Command 4/5/6` 结构化状态话题
- `ROS 2 -> MQTT`：新增 `mqtt_forwarder_node`，按约定格式上报设备心跳

## 当前模块

- `udp_bridge_node`：监听 UDP 数据、筛选 `Command 4/5/6`，并发布结构化状态话题
- `mqtt_forwarder_node`：订阅 `/robot/basic_status`、`/robot/battery_status`、`/robot_position`、`/mission/status`，组装 MQTT 心跳消息并上报

## 已保留的 UDP 指令

基于当前提供的 UDP JSON 协议，预留以下三类状态解析：

- `Command=4`：`MotionStatus` / `MotorStatus`
- `Command=5`：`BatteryStatus`
- `Command=6`：`BasicStatus`

当前 `udp_bridge_node` 已实现：

- 定时发送心跳 UDP 报文
- 接收并解析 `Command 4/5/6`
- 发布结构化状态话题

## 已实现话题映射

| 来源 | JSON 位置 | 话题 | 消息类型 |
|---|---|---|---|
| UDP Command 4 | `Items.MotionStatus`/`Items.MotorStatus` | `/robot/motion_status` | `rhw_msgs/UdpMotionStatus` |
| UDP Command 5 | `Items.BatteryStatus` | `/robot/battery_status` | `rhw_msgs/UdpBatteryStatus` |
| UDP Command 6 | `Items.BasicStatus` | `/robot/basic_status` | `rhw_msgs/UdpBasicStatus` |

## 已实现 MQTT 心跳映射

`mqtt_forwarder_node` 当前按以下规则组装 MQTT 心跳：

| MQTT 字段 | 当前来源/规则 |
|---|---|
| `type` | 固定为 `upload` |
| `method` | 固定为 `heart` |
| `msgid` | 节点内自增 |
| `message.runMode` | `UdpBasicStatus.control_usage_mode` |
| `message.workStatus` | `MissionStatus.status == RUNNING` 时为 `1`，否则为 `0` |
| `message.battery` | `min(UdpBatteryStatus.battery_level_left, battery_level_right)` |
| `message.healthStatus` | 固定为 `0` |
| `message.motionStatus` | `UdpBasicStatus.motion_state` |
| `message.chargeStatus` | `UdpBasicStatus.charge` |
| `message.signalStrength` | 配置参数 `default_signal_strength` |
| `message.onlineStatus` | 最近 `status_timeout_sec` 秒未收到必需状态则为 `1`，否则为 `0` |
| `message.location.mapId` | 配置参数 `map_id` |
| `message.location.worldPose` | `RobotPosition.world_position(x, y, theta)` 转 position + quaternion |

说明：

- 当前节点只发布结构化消息，不再输出原始 JSON 话题。
- 字段名兼容大小写与下划线/驼峰两种常见写法（如 `Roll`/`roll`、`LineX`/`line_x`）。
- 若后续还需 `/odom`、硬件聚合状态或版本话题，可在现有结构上继续扩展。
- 当前 `mqtt_forwarder_node` 将 `/robot_position` 和 `/mission/status` 作为必需输入；未收到这些状态前不会发送 MQTT 心跳。

## 启动

```bash
ros2 launch rhw_udp_mqtt_bridge udp_bridge.launch.py
```

如果同时启动 MQTT 心跳转发：

```bash
ros2 launch rhw_udp_mqtt_bridge udp_bridge.launch.py enable_mqtt_forwarder:=true
```

也可以单独启动：

```bash
ros2 run rhw_udp_mqtt_bridge mqtt_forwarder_node
```

## 关键配置

配置文件：`config/udp_mqtt_bridge.yaml`

- `bind_host` / `bind_port`：本地 UDP 监听地址
- `robot_host` / `robot_port`：机器人 UDP 目标地址
- `receive_filter_host` / `receive_filter_port`：接收报文来源过滤
- `heartbeat_period_sec`：心跳发送周期
- `motion_command` / `battery_command` / `basic_command`：保留的 `Command 4/5/6`
- `basic_status_topic` / `motion_status_topic` / `battery_status_topic`：结构化状态话题
- `status_frame_id`：结构化消息头 `header.frame_id`
- `debug_log_sender`：调试模式下打印收到的 UDP sender 地址与过滤结果
- `mqtt_topic`：MQTT 心跳上报 topic
- `heartbeat_publish_period_sec`：MQTT 心跳发布周期
- `status_timeout_sec`：超过该时间未收到必需状态则心跳标记离线
- `default_signal_strength`：当前默认 WiFi 信号强度
- `map_id`：MQTT 心跳中的位置地图 ID
- `robot_position_topic`：位置输入话题，默认 `/robot_position`
- `mission_status_topic`：任务状态输入话题，默认 `/mission/status`

## 目录结构

```text
rhw_udp_mqtt_bridge/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── config/
│   └── udp_mqtt_bridge.yaml
├── launch/
│   └── udp_bridge.launch.py
├── resource/
│   └── rhw_udp_mqtt_bridge
└── rhw_udp_mqtt_bridge/
    ├── __init__.py
    ├── udp_bridge_node.py
    └── mqtt_forwarder_node.py
```

## 后续建议

- 继续细化 `Command 5/6` 中哪些字段需要结构化发布
- 明确后续是否需要恢复 `/odom` 或新增专用状态消息
- 补充真实 `signalStrength` 来源，而不是使用默认值
- 明确 `runMode`、`healthStatus` 等字段与业务枚举的最终映射
- 若后续需要更丰富心跳，可继续接入 `RobotHardwreStatus`、真实 `signalStrength` 等来源
