# rhw_udp_mqtt_bridge

用于解析 UDP 信息并发布为结构化 ROS 2 话题，同时作为机器人对接平台的统一 MQTT 网关。

## 当前状态

当前已完成：

- `UDP -> ROS 2`：解析并发布 `Command 4/5/6` 结构化状态话题
- `ROS 2 <-> MQTT`：`mqtt_gateway_node` 统一处理心跳、点位同步、任务下发、任务状态上报
- `ROS 2 -> HTTPS`：`inspection_reporter_node` 订阅抓拍结果，或通过同步 service 上报平台相册接口

## 当前模块

- `udp_bridge_node`：监听 UDP 数据、筛选 `Command 4/5/6`，并发布结构化状态话题
- `mqtt_gateway_node`：使用单个 MQTT client 连接平台，订阅/发布平台协议 topic，并调用/订阅 ROS 2 任务与点位接口
- `inspection_reporter_node`：订阅 `/inspection/album_reports`，并提供 `/inspection/album_report/upload` 同步 service，按第六节 HTTPS 相册接口上报巡检抓拍结果

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

## 已实现 MQTT 网关

`mqtt_gateway_node` 当前处理：

- 心跳上报：订阅 `/robot/basic_status`、`/robot/battery_status`、`/robot_position`、`/mission/status`，向 `upload_topic` 发布 `method=heart`
- 点位同步：订阅 `/waypoint_manager/events`，调用 `/waypoint_manager/get_waypoints`，向 `upload_topic` 发布 `method=map`
- 任务下发：订阅 MQTT `download_topic`，收到 `method=task` 后调用 `/mission/start`
- 任务状态：订阅 `/mission/status`，向 `upload_topic` 发布 `method=task`

心跳字段映射：

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
- 当前 `mqtt_gateway_node` 将 `/robot_position` 和 `/mission/status` 作为心跳必需输入；未收到这些状态前不会发送 MQTT 心跳。

## 启动

```bash
ros2 launch rhw_udp_mqtt_bridge udp_bridge.launch.py
```

如果同时启动 MQTT 网关：

```bash
ros2 launch rhw_udp_mqtt_bridge udp_bridge.launch.py enable_mqtt_gateway:=true
```

也可以单独启动：

```bash
ros2 run rhw_udp_mqtt_bridge mqtt_gateway_node
```

巡检抓拍 HTTPS 上报节点单独启动：

```bash
ros2 run rhw_udp_mqtt_bridge inspection_reporter_node
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
- `upload_topic`：MQTT 上行 topic，如 `/robot-dog/DOG001/Upload/Data`
- `download_topic`：MQTT 下行 topic，如 `/robot-dog/DOG001/Download/Data`
- `client_id`：MQTT client ID，同一个 broker 下必须唯一
- `heartbeat_publish_period_sec`：MQTT 心跳发布周期
- `status_timeout_sec`：超过该时间未收到必需状态则心跳标记离线
- `default_signal_strength`：当前默认 WiFi 信号强度
- `map_id`：MQTT 心跳中的位置地图 ID
- `robot_position_topic`：位置输入话题，默认 `/robot_position`
- `mission_status_topic`：任务状态输入话题，默认 `/mission/status`
- `waypoint_event_topic`：点位变更事件，默认 `/waypoint_manager/events`
- `mission_start_service`：任务启动服务，默认 `/mission/start`
- `get_waypoints_service`：点位查询服务，默认 `/waypoint_manager/get_waypoints`
- `default_task_map_name`：平台任务未带 `mapName` 时使用的默认地图名
- `inspection_reporter_node.album_report_topic`：抓拍结果输入话题，默认 `/inspection/album_reports`
- `inspection_reporter_node.album_upload_service`：同步相册上传服务，默认 `/inspection/album_report/upload`
- `inspection_reporter_node.album_report_url`：平台相册上报接口，如 `https://ip:port/robot-inspect/inspect/album/report`
- `inspection_reporter_node.device_id` / `partner_id` / `version`：HTTPS 外层与业务字段
- `inspection_reporter_node.encryption_enabled`：是否使用 `AES-CBC + PKCS7 + Base64` 加密 `data`
- `inspection_reporter_node.signature_enabled`：是否按 `MD5(traceId + data + signatureSecret)` 生成签名
- `inspection_reporter_node.aes_key` / `aes_iv`：AES 密钥与 IV；支持普通字符串、`base64:...`、`hex:...`
- `inspection_reporter_node.signature_secret`：平台分配的签名密钥
- `inspection_reporter_node.timeout_sec` / `retry_count` / `verify_tls`：HTTPS 超时、重试次数与 TLS 校验

## 巡检抓拍 HTTPS 上报测试

先用明文模式连本地 HTTP mock 服务时，可临时覆盖参数：

```bash
ros2 run rhw_udp_mqtt_bridge inspection_reporter_node --ros-args \
  -p album_report_url:=http://127.0.0.1:8088/robot-inspect/inspect/album/report \
  -p encryption_enabled:=false \
  -p signature_enabled:=false
```

发布一条测试抓拍事件：

```bash
ros2 topic pub --once /inspection/album_reports rhw_msgs/msg/InspectionAlbumReport "{
  task_id: 'XJ-TEST-001',
  point_id: 'P_A01',
  point_name: '大门口',
  image_path: '/tmp/test.jpg',
  capture_url: '',
  file_size: 0
}"
```

### 同步相册上传 Service

行为树通过 `/inspection/album_report/upload` 同步调用 HTTPS 相册上报。该 service 会读取请求里的本机 `image_path`，按 `inspection_reporter_node` 的 `album_report_url`、加密、签名、TLS、重试等配置执行 HTTPS 上报；成功时返回 `ok=true`，失败时返回 `ok=false`，任务调度会把当前视觉点判定为失败。

测试模式与配置和上面的 `inspection_reporter_node` 完全一致。例如连本地 HTTP mock 时，仍然通过 `album_report_url`、`encryption_enabled`、`signature_enabled` 等参数切换。

```bash
ros2 service call /inspection/album_report/upload rhw_msgs/srv/InspectionAlbumUpload "{
  task_id: 'task-test',
  point_id: '001',
  point_name: '测试点位',
  image_path: '/tmp/test.jpg',
  capture_url: '',
  file_size: 12345
}"
```

`image_path` 必须是 `inspection_reporter_node` 所在机器可读的本地图片文件。话题 `/inspection/album_reports` 仍可用于手动发布抓拍事件；真实视觉流程由任务调度行为树调用同步 service，并等待返回结果。

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
    ├── mqtt_gateway_node.py
    └── inspection_reporter_node.py
```

## 后续建议

- 继续细化 `Command 5/6` 中哪些字段需要结构化发布
- 明确后续是否需要恢复 `/odom` 或新增专用状态消息
- 补充真实 `signalStrength` 来源，而不是使用默认值
- 明确 `runMode`、`healthStatus` 等字段与业务枚举的最终映射
- 若后续需要更丰富心跳，可继续接入 `RobotHardwreStatus`、真实 `signalStrength` 等来源
