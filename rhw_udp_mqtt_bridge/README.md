# rhw_udp_mqtt_bridge

用于解析 UDP 信息并发布为结构化 ROS 2 话题，以及后续将 ROS 2 话题转发为 MQTT 协议。

## 当前状态

当前已完成 `UDP -> ROS 2` 的基础实现，已支持保留并解析 `Command 4/5/6`，仅发布结构化自定义消息。
`ROS 2 -> MQTT` 仍保留为后续功能。

## 当前模块

- `udp_bridge_node`：监听 UDP 数据、筛选 `Command 4/5/6`，并发布结构化状态话题
- `mqtt_forwarder_node`：仅保留配置骨架，暂未实现源码

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

说明：

- 当前节点只发布结构化消息，不再输出原始 JSON 话题。
- 字段名兼容大小写与下划线/驼峰两种常见写法（如 `Roll`/`roll`、`LineX`/`line_x`）。
- 若后续还需 `/odom`、硬件聚合状态或版本话题，可在现有结构上继续扩展。

## 启动

```bash
ros2 launch rhw_udp_mqtt_bridge udp_bridge.launch.py
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
    └── udp_bridge_node.py
```

## 后续建议

- 继续细化 `Command 5/6` 中哪些字段需要结构化发布
- 明确后续是否需要恢复 `/odom` 或新增专用状态消息
- 明确 MQTT 需要转发的结构化 ROS 2 话题、序列化格式和 QoS 要求
- 再补充 `mqtt_forwarder_node` 实现与 launch 编排
