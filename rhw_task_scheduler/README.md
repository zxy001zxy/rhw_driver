# rhw_task_scheduler

基于行为树的 ROS 2 航点任务调度包，面向“部署点位 + 按顺序执行航点任务”的业务场景。

当前实现将系统拆分为两个节点：

| 节点 | 功能 |
|---|---|
| `waypoint_manager` | 管理点位部署信息，按地图名保存/删除/查询航点 |
| `mission_bt_node` | 接收任务并按顺序执行“导航 → 到达后任务” |

---

## 适用场景

- 外部系统在部署阶段下发地图名、点位坐标、点位类型
- 系统保存点位，供后续实施阶段按 ID 索引
- 外部系统通过 MQTT 或 ROS 2 Service 下发任务列表
- 调度器按顺序执行多个航点任务
- 到达航点后可根据点位类型触发不同动作

当前已支持的点位类型：

- `TYPE_NORMAL=0`：普通导航点，到达即完成
- `TYPE_CHARGE=1`：充电点位，到达后调用充电服务
- `TYPE_VISION=2`：视觉识别点，到达后执行 PTZ + 抓拍 + 视觉检测占位流程

---

## 当前实现内容

### 1. 部署模块：`waypoint_manager`

提供三个服务：

- `/waypoint_manager/add_waypoint`
- `/waypoint_manager/delete_waypoint`
- `/waypoint_manager/get_waypoints`

点位信息按地图名持久化到：

```text
~/.rhw/waypoints/<map_name>.json
```

JSON 顶层保存当前地图名 `map_name` 和稳定唯一标识 `map_id`。

每条点位记录包括：

- `waypoint_id`
- `pose(x, y, theta)`
- `waypoint_type`
- `label`
- `task_params`（JSON 字符串）

当启用 `waypoint_manager` 的 MQTT 同步参数后，保存或删除点位后会主动将当前地图点位快照发布到 MQTT。

### 2. 实施模块：`mission_bt_node`

支持两种任务下发方式：

- ROS 2 Service：`/mission/start`、`/mission/stop`、`/mission/pause`
- MQTT：订阅 `rhw/mission/start`（可选）

执行时：

1. 根据 `map_name` + `waypoint_ids[]` 从 `waypoint_manager` 查询点位详情
2. 按顺序逐个执行航点
3. 每个航点由行为树负责调度导航和到达后任务
4. 实时发布 `/mission/status`

### 3. 内部调试模式

当前已增加调试版能力，可在**不接导航/云台/视觉真实服务**时单独调通调度逻辑。

开启方式：

- 将 `config/task_scheduler.yaml` 中 `debug_mock_enabled` 设为 `true`
- 根据需要设置各动作的 mock 返回值
- 可选择打印行为树文本或导出 DOT 图

适合先验证：

- 航点顺序是否正确
- 普通点 / 视觉点 / 充电点是否进入对应分支
- 暂停 / 恢复 / 停止是否正常
- 失败后是否跳过并继续下一个航点

---

## 行为树设计

每个航点对应一棵行为树：

```text
Sequence (WaypointHandler)
├── CheckBattery
├── NavigateToGoal
└── Selector (TaskSelector)
    ├── Sequence (VisionTask)
    │   ├── IsVisionPoint
  │   ├── PtzAbsoluteMove
    │   ├── WaitPtzStable
    │   ├── CaptureImage
    │   └── TriggerInference
    ├── Sequence (ChargeTask)
    │   ├── IsChargePoint
    │   └── Recharge
    └── IsNormalPoint
```

说明：

- `CheckBattery`：检查电量，低于阈值时返回失败
- `NavigateToGoal`：调用 `/move_base_simple/goal`，监听 `/navigation_status`
- `VisionTask`：用于视觉识别点
- `ChargeTask`：用于充电点
- `IsNormalPoint`：普通导航点到达即结束

---

## 依赖

- ROS 2 Humble
- `rhw_msgs`
- Python 依赖：`py_trees`、`paho-mqtt`、`websockets`

### 1. 安装核心依赖

如果是新环境，先安装 ROS 2 Python 打包工具与本包运行依赖：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pip

python3 -m pip install -U pip
python3 -m pip install py_trees paho-mqtt websockets
```

说明：

- `py_trees`：行为树运行时依赖
- `paho-mqtt`：MQTT 任务下发可选依赖
- `websockets`：`bt_web_viewer` WebSocket 推送依赖

### 2. 安装可选可视化依赖

如果需要行为树快照、Foxglove 或 rosbridge 联调，可额外安装：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  ros-humble-py-trees-ros \
  ros-humble-py-trees-ros-interfaces \
  ros-humble-foxglove-bridge \
  ros-humble-rosbridge-suite
```

说明：

- `py_trees_ros` / `py_trees_ros_interfaces`：启用 `mission_bt_node` 的行为树快照发布
- `foxglove_bridge`：在 Foxglove Studio 中查看 ROS 2 数据
- `rosbridge_suite`：如需通过 rosbridge 向 Web 或外部工具暴露 ROS 接口

### 3. 构建前准备

首次进入工作区建议执行：

```bash
cd ~/Desktop/project/rhw_ws
source /opt/ros/humble/setup.bash
```

---

## 编译

```bash
cd ~/Desktop/project/rhw_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rhw_msgs rhw_task_scheduler
source install/setup.bash
```

---

## 启动

### 启动整个调度系统

```bash
ros2 launch rhw_task_scheduler task_scheduler.launch.py
```

### 单独启动节点

```bash
ros2 run rhw_task_scheduler waypoint_manager
ros2 run rhw_task_scheduler mission_bt_node
```

### 一键灌入测试点位并启动 mock 任务

在 `mission_bt_node` 已启动的情况下，可直接运行：

```bash
ros2 run rhw_task_scheduler mock_mission_runner
```

脚本会自动执行：

1. 打开 `mission_bt_node` 的 mock 调试参数
2. 删除旧的同名测试点位
3. 在主地图写入测试点位：`normal_001`、`vision_001`、`charge_001`
4. 默认额外写入一张虚拟地图 `room_map` 的测试点位，用于多地图点位同步联调
4. 调用 `/mission/start` 启动任务

说明：

- 默认**不会覆盖**你已经手动设置的树可视化参数（如 `debug_print_tree_on_tick`）
- 如需由脚本统一设置树打印参数，可额外传入 `configure_tree_debug_params:=true`

默认地图名为 `factory_map`。

默认会额外写入次级地图 `room_map`。如不需要，可传入 `-p seed_secondary_map:=false`。

如需改参数，可使用 ROS 参数覆盖，例如：

```bash
ros2 run rhw_task_scheduler mock_mission_runner --ros-args \
  -p map_name:=demo_map \
  -p secondary_map_name:=demo_room_map \
  -p mock_delay_sec:=1.0 \
  -p configure_tree_debug_params:=true \
  -p print_tree_on_tick:=true \
  -p export_tree_dot:=true
```

## 启动说明

以下命令默认在工作区根目录执行：

```bash
cd ~/Desktop/project/rhw_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 1. 常规启动

适用于通过 ROS 2 Service 管理点位和任务，不依赖 MQTT。

启动命令：

```bash
ros2 launch rhw_task_scheduler task_scheduler.launch.py
```

启动后可直接通过以下接口联调：

- 点位管理：`/waypoint_manager/add_waypoint`、`/waypoint_manager/delete_waypoint`、`/waypoint_manager/get_waypoints`
- 任务控制：`/mission/start`、`/mission/pause`、`/mission/stop`
- 状态话题：`/mission/status`

### 2. 启动 Web 可视化

如果需要行为树 Web 页面，启动时加上 `bt_viewer:=true`：

```bash
ros2 launch rhw_task_scheduler task_scheduler.launch.py bt_viewer:=true
```

浏览器访问：

```text
http://localhost:8765/
```

说明：

- HTTP 端口默认 `8765`
- WebSocket 端口默认 `8766`
- 需要先安装 `py_trees_ros` 与 `py_trees_ros_interfaces`

### 3. 启用点位 MQTT 主动同步

如果需要在保存或删除点位后，通过 MQTT 主动同步当前地图点位快照，先修改 `config/task_scheduler.yaml` 中 `waypoint_manager` 的以下参数：

- `mqtt_sync_enabled: true`
- `mqtt_broker_host`
- `mqtt_broker_port`
- `mqtt_client_id`
- `mqtt_username` / `mqtt_password`
- `mqtt_waypoint_sync_topic`

修改后重新启动：

```bash
ros2 launch rhw_task_scheduler task_scheduler.launch.py
```

当前点位同步是主动推送模式，触发时机如下：

- `waypoint_manager` 连接到 MQTT broker 后，自动补发当前已保存点位；如果存在多张地图，会在同一条消息的 `message[]` 中返回多张地图快照
- `add_waypoint` 成功后，发布当前地图点位快照
- `delete_waypoint` 成功后，发布当前地图点位快照

当前点位同步消息结构如下：

- 顶层字段固定为 `type`、`method`、`code`、`msgid`、`message`
- `message[]` 中每条记录表示一张地图，包含 `mapId`、`mapName`、`pointCount`、`pointId[]`、`pointName[]`

### 4. 启用 MQTT 任务下发

如果需要通过 MQTT 下发任务给 `mission_bt_node`，先修改 `config/task_scheduler.yaml` 中 `mission_bt_node` 的以下参数：

- `mqtt_enabled: true`
- `mqtt_broker_host`
- `mqtt_broker_port`
- `mqtt_client_id`
- `mqtt_mission_start_topic`
- `mqtt_mission_status_topic`

修改后重新启动：

```bash   
ros2 launch rhw_task_scheduler task_scheduler.launch.py
```

默认 MQTT 任务下发 topic：

```text
rhw/mission/start
```

默认 MQTT 状态回传 topic：

```text
rhw/mission/status
```

### 5. Mock 联调启动

适用于当前没有真实导航、云台、抓拍、充电服务，只验证调度链路。

终端 1：启动 mock 服务响应器

```bash
ros2 run rhw_task_scheduler mock_service_responder
```

终端 2：启动调度系统

```bash
ros2 launch rhw_task_scheduler task_scheduler.launch.py bt_viewer:=true
```

终端 3：灌入测试点位并启动 mock 任务

```bash
ros2 run rhw_task_scheduler mock_mission_runner --ros-args -p enable_mock_params:=false
```

如果希望直接使用 `mission_bt_node` 内部 mock，而不是 mock 服务响应器，请把 `config/task_scheduler.yaml` 里的 `debug_mock_enabled` 改成 `true` 后再启动。

### 6. 启动后检查项

建议按下面顺序检查：

1. 节点是否存在：

```bash
ros2 node list
```

2. 任务状态是否正常发布：

```bash
ros2 topic echo /mission/status
```

3. 服务审计是否正常发布：

```bash
ros2 topic echo /service_events
```

4. 如启用 Web 页面，确认浏览器可打开：

```text
http://localhost:8765/
```

5. 如启用点位 MQTT 同步或任务 MQTT，确认 broker 连通并检查对应 topic 是否有消息。

---

## 配置

配置文件：`config/task_scheduler.yaml`

### waypoint_manager 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `storage_dir` | `~/.rhw/waypoints` | 点位持久化目录 |
| `mqtt_sync_enabled` | `false` | 是否启用点位 MQTT 主动同步 |
| `mqtt_broker_host` | `127.0.0.1` | MQTT broker 地址 |
| `mqtt_broker_port` | `1883` | MQTT broker 端口 |
| `mqtt_client_id` | `rhw_waypoint_manager` | 点位同步 MQTT 客户端 ID |
| `mqtt_username` | `""` | MQTT 用户名 |
| `mqtt_password` | `""` | MQTT 密码 |
| `mqtt_waypoint_sync_topic` | `/robot-dog/DOG001/Upload/Data` | 点位同步上报 topic |
| `mqtt_qos` | `0` | 点位同步 MQTT QoS |
| `mqtt_keep_alive_sec` | `60` | 点位同步 MQTT keep alive |
| `add_waypoint_service` | `/waypoint_manager/add_waypoint` | 添加点位服务名 |
| `delete_waypoint_service` | `/waypoint_manager/delete_waypoint` | 删除点位服务名 |
| `get_waypoints_service` | `/waypoint_manager/get_waypoints` | 查询点位服务名 |

### mission_bt_node 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `bt_tick_rate_hz` | `10.0` | 行为树 tick 频率 |
| `goal_service` | `/move_base_simple/goal` | 导航服务 |
| `cancel_service` | `/move_base/cancel` | 导航取消服务 |
| `nav_status_topic` | `/navigation_status` | 导航状态话题 |
| `nav_retry_max` | `3` | 导航失败最大重试次数 |
| `ptz_absolute_move_service` | `/ptz/absolute_move` | 云台绝对位置移动服务 |
| `ptz_capture_service` | `/ptz/capture_image` | 抓拍服务 |
| `ptz_status_topic` | `/ptz/status` | 云台状态话题 |
| `recharge_service` | `/recharge` | 回充服务 |
| `battery_topic` | `/robot/battery_status` | 电池状态话题 |
| `low_battery_threshold` | `20.0` | 低电量阈值 |
| `mqtt_enabled` | `false` | 是否启用 MQTT |
| `mqtt_mission_start_topic` | `rhw/mission/start` | MQTT 任务下发 topic |
| `mqtt_mission_status_topic` | `rhw/mission/status` | MQTT 状态回传 topic |
| `mission_status_topic` | `/mission/status` | ROS 任务状态话题 |
| `service_events` | `/service_events` | 服务审计事件话题（JSON 字符串） |

### 服务审计话题

节点 `mission_bt_node` 与 `waypoint_manager` 会向 `/service_events` 发布服务调用审计事件，
用于在命令行、Foxglove 或自定义 Web 面板中查看服务请求/响应数据。

消息类型：`std_msgs/msg/String`

JSON 字段示例：

- `timestamp`: Unix 时间戳
- `node`: 发布该审计事件的节点名
- `service`: 服务名，如 `/mission/start`
- `role`: `server` 或 `client`
- `phase`: `request` 或 `response`
- `request`: 请求体
- `response`: 响应体（如果有）
- `success`: 是否成功
- `duration_ms`: 调用耗时（毫秒）
- `details`: 额外补充信息

查看示例：

```bash
source install/setup.bash
ros2 topic echo /service_events
```

仅筛选某个服务：

```bash
source install/setup.bash
ros2 topic echo /service_events | grep '"service": "/mission/start"'
```

### 调试相关参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `debug_mock_enabled` | `false` | 是否启用内部 mock 调试模式 |
| `debug_mock_delay_sec` | `0.5` | mock 动作统一延时 |
| `debug_mock_nav_result` | `success` | 导航 mock 结果：`success` / `failure` / `running` |
| `debug_mock_ptz_result` | `success` | 云台绝对位置移动/等待稳定 mock 结果 |
| `debug_mock_capture_result` | `success` | 抓拍 mock 结果 |
| `debug_mock_charge_result` | `success` | 充电 mock 结果 |
| `debug_mock_inference_result` | `success` | 推理 mock 结果 |
| `debug_mock_battery_level` | `100.0` | mock 电量值 |
| `debug_mock_capture_dir` | `/tmp/rhw_task_scheduler_mock_captures` | mock 抓拍路径前缀 |
| `debug_print_tree_on_build` | `true` | 每次建树时打印文本树 |
| `debug_print_tree_on_tick` | `false` | 每次 tick 周期性打印树状态 |
| `debug_tree_log_every_n_ticks` | `1` | tick 树打印周期 |
| `debug_export_tree_dot` | `false` | 是否导出 DOT / PNG / SVG |
| `debug_tree_output_dir` | `/tmp/rhw_task_scheduler_bt` | 树图输出目录 |

---

## 接口说明

### 点位部署接口

#### 添加点位

```bash
ros2 service call /waypoint_manager/add_waypoint rhw_msgs/srv/AddWaypoint "{
  waypoint: {
    waypoint_id: 'vision_001',
    map_name: 'factory_map',
    pose: {x: 1.2, y: 3.4, theta: 0.0},
    waypoint_type: 2,
    label: '视觉检测点1',
    task_params: '{\"azimuth\":180.0,\"elevation\":0.0,\"channel\":1,\"azimuth_speed\":50,\"elevation_speed\":50,\"inference_type\":\"det\"}'
  }
}"
```

视觉点位说明：

- 视觉点位的 `task_params` 录入的是云台绝对位置，不再使用 `preset_id`
- 必填字段至少包括 `azimuth`、`elevation`
- 可选字段包括 `channel`、`azimuth_speed`、`elevation_speed`、`inference_type`
- 如果视觉点位缺少 `azimuth` 或 `elevation`，行为树会直接判定该视觉动作失败

#### 查询地图点位

```bash
ros2 service call /waypoint_manager/get_waypoints rhw_msgs/srv/GetWaypoints "{
  map_name: 'factory_map'
}"
```

#### 删除点位

```bash
ros2 service call /waypoint_manager/delete_waypoint rhw_msgs/srv/DeleteWaypoint "{
  map_name: 'factory_map',
  waypoint_id: 'vision_001'
}"
```

### 任务执行接口

#### 启动任务

```bash
ros2 service call /mission/start rhw_msgs/srv/StartMission "{
  map_name: 'factory_map',
  waypoint_ids: ['normal_001', 'vision_001', 'charge_001']
}"
```

### 调试模式推荐流程

1. 启动 `waypoint_manager` 和 `mission_bt_node`
2. 直接运行 `mock_mission_runner`
3. 脚本会自动灌入三个测试点并启动任务
5. 订阅 `/mission/status` 观察执行进度
6. 如需看行为树结构，打开 `debug_print_tree_on_build`
7. 如需导出图，打开 `debug_export_tree_dot`

调试模式下：

- 不依赖真实 `/move_base_simple/goal`
- 不依赖真实 `/ptz/*`
- 不依赖真实视觉推理服务
- 仍然会完整走一遍行为树调度流程

#### 暂停任务

```bash
ros2 service call /mission/pause rhw_msgs/srv/PauseMission "{pause: true}"
```

#### 恢复任务

```bash
ros2 service call /mission/pause rhw_msgs/srv/PauseMission "{pause: false}"
```

#### 停止任务

```bash
ros2 service call /mission/stop rhw_msgs/srv/StopMission "{}"
```

---

## MQTT 消息格式

当 `mqtt_enabled=true` 时，`mission_bt_node` 会订阅任务下发 topic。

### 点位主动同步说明

当 `waypoint_manager.mqtt_sync_enabled=true` 时，`waypoint_manager` 会在以下时机主动发布点位同步消息：

- 节点连接到 MQTT broker 后，自动补发当前已保存的所有地图点位快照
- 调用 `/waypoint_manager/add_waypoint` 成功后，发布该地图当前点位快照
- 调用 `/waypoint_manager/delete_waypoint` 成功后，发布该地图当前点位快照

当前阶段仅支持主动同步，不通过平台触发响应式查询同步。

当前点位同步消息示例结构：

```json
{
  "type": "response",
  "method": "map",
  "code": 0,
  "msgid": 1,
  "message": [
    {
      "mapId": "9acb90d53c0d52b89d7f8a6ee4a19b85",
      "mapName": "factory_map",
      "pointCount": 2,
      "pointId": ["P_A01", "P_A02"],
      "pointName": ["大门口", "视觉点"]
    },
    {
      "mapId": "6425dbf2b5665a62b5d8b3c7d6d8f0eb",
      "mapName": "room_map",
      "pointCount": 3,
      "pointId": ["P_B01", "P_B02", "P_B03"],
      "pointName": ["大门口", "视觉点", "视觉点2"]
    }
  ]
}
```

### 下发任务示例

Topic：`rhw/mission/start`

```json
{
  "map_name": "factory_map",
  "waypoint_ids": [
    "normal_001",
    "vision_001",
    "charge_001"
  ]
}
```

### 状态回传示例

Topic：`rhw/mission/status`

```json
{
  "status": 1,
  "current_waypoint_id": "vision_001",
  "total_waypoints": 3,
  "completed_waypoints": 1
}
```

---

## 目录结构

```text
rhw_task_scheduler/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── config/
│   └── task_scheduler.yaml
├── launch/
│   └── task_scheduler.launch.py
├── resource/
│   └── rhw_task_scheduler
└── rhw_task_scheduler/
    ├── __init__.py
    ├── waypoint_manager.py
    ├── mission_bt_node.py
    └── bt_actions/
        ├── __init__.py
        ├── navigate_action.py
        ├── ptz_actions.py
        ├── charge_action.py
        ├── condition_nodes.py
        └── inference_action.py
```

---

## 当前限制

- 当前视觉检测部分 `TriggerInference` 仍为占位实现，尚未接入真实推理服务
- 调试模式仅 mock 外部动作，不替代真实业务联调
- 当前任务执行采用“单航点单行为树”方式串行推进，后续可扩展为整任务树
- 当前未实现任务结果持久化，如需任务审计/报表，可补充数据库或 JSON 日志
- 当前未实现部署阶段的 MQTT 点位管理接口，如外部系统不能直接调用 ROS Service，可后续增加 `waypoint_manager` 的 MQTT 代理

---

## 后续建议

- 将 `py_trees`、`paho-mqtt` 写入 `setup.py` 的 `install_requires`
- 增加视觉任务结果消息与推理服务接口
- 增加任务执行记录与失败原因持久化
- 增加“低电量自动插入充电航点”策略
- 如后续任务复杂度继续提升，可扩展为多层行为树或插件式任务执行器



# 终端1: 启动 mock 服务响应器
ros2 run rhw_task_scheduler mock_service_responder

# 终端2: 启动任务调度（关闭mock）
ros2 launch rhw_task_scheduler task_scheduler.launch.py

# 终端3: 启动任务时不开 mock
ros2 run rhw_task_scheduler mock_mission_runner --ros-args -p enable_mock_params:=false

http://localhost:8765/