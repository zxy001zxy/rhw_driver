# rhw_task_scheduler

基于行为树的 ROS 2 航点任务调度包，面向真实环境中的“部署点位 + 按顺序执行航点任务”。

## 概述

| 节点 | 功能 |
|---|---|
| `waypoint_manager` | 管理点位部署信息，按地图名保存、删除、查询航点 |
| `mission_bt_node` | 接收任务并按顺序执行“导航 -> 到达后任务” |
| `bt_web_viewer` | 可选行为树 Web 可视化 |

支持的点位类型：

- `TYPE_NORMAL=0`：普通导航点，到达即完成
- `TYPE_CHARGE=1`：充电点，到达后调用 `/recharge`
- `TYPE_VISION=2`：视觉点，导航到位后执行 PTZ 绝对位置移动、等待稳定、抓拍、同步相册上传和模型任务

包含独立测试启动文件和 mock 节点，可按需混用真实点位、真实导航、真实云台或模拟接口。

## 行为树流程

每个航点单独构建并执行一棵行为树：

```text
Sequence (WaypointHandler)
├── CheckBattery
├── NavigateToGoal
└── Selector (TaskSelector)
    ├── Sequence (VisionTask)
    │   ├── IsVisionPoint
    │   ├── WaitAfterNavArrived
    │   ├── PtzAbsoluteMove
    │   ├── WaitPtzStable
    │   ├── WaitAfterPtzArrived
    │   ├── CaptureImage
    │   ├── UploadInspectionAlbum
    │   └── RunModelTask
    ├── Sequence (ChargeTask)
    │   ├── IsChargePoint
    │   └── Recharge
    └── IsNormalPoint
```

说明：

- `CheckBattery`：订阅 `/robot/battery_status`，低于阈值时当前航点失败
- `NavigateToGoal`：调用 `/move_base_simple/goal`，监听 `/navigation_status`
- `VisionTask`：调用 `/ptz/absolute_move`，监听 `/ptz/status`，再依次执行 `/ptz/capture_image`、`/inspection/album_report/upload` 同步相册上报和 `/rhw/model/task/run` 模型任务
- `ChargeTask`：调用 `/recharge`
- `IsNormalPoint`：普通导航点到达后直接成功

`TYPE_VISION=2` 视觉点完整流程：

```text
NavigateToGoal
-> WaitAfterNavArrived
-> PtzAbsoluteMove
-> WaitPtzStable
-> WaitAfterPtzArrived
-> CaptureImage
-> UploadInspectionAlbum
-> RunModelTask
```

`task_params.inference_type` 需要填写模型调度服务的完整 `task_name`，例如：

```json
{
  "azimuth": 180.0,
  "elevation": 0.0,
  "zoom": 6.5,
  "channel": 1,
  "inference_type": "fire_equipment_detection"
}
```

## 依赖安装

建议在 ROS 2 Humble 环境下先安装基础工具，再补齐 Python 依赖：

```bash
sudo apt update
sudo apt install ros-humble-py-trees-ros-interfaces
sudo apt install -y python3-pip python3-colcon-common-extensions
pip3 install -U py_trees paho-mqtt
```

如果你的工作区里还没有 `rhw_msgs`，请先把它和本包一起放在同一个 `src` 下，再执行后续编译命令。

## 真实服务接口

`mission_bt_node` 提供：

- `/mission/start`
- `/mission/pause`
- `/mission/stop`

`mission_bt_node` 调用或订阅：

| 接口 | 类型 | 说明 |
|---|---|---|
| `/waypoint_manager/get_waypoints` | Service | 根据地图查询点位 |
| `/move_base_simple/goal` | Service | 发送导航目标 |
| `/navigation_status` | Topic | 导航状态 |
| `/ptz/absolute_move` | Service | 云台绝对位置移动，支持可选 `zoom` |
| `/ptz/status` | Topic | 云台状态 |
| `/ptz/capture_image` | Service | 抓拍 |
| `/inspection/album_report/upload` | Service | 抓拍后同步 HTTPS 相册上报 |
| `/rhw/model/task/run` | Service | 相册上报成功后执行模型任务 |
| `/recharge` | Service | 回充 |
| `/robot/battery_status` | Topic | 电池状态 |
| `/mission/status` | Topic | 任务状态 |
| `/service_events` | Topic | 服务调用审计事件 |

## 编译

```bash
cd ~/Desktop/project/rhw_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rhw_msgs rhw_task_scheduler
source install/setup.bash
```

## 启动

启动调度系统：

```bash
ros2 launch rhw_task_scheduler task_scheduler.launch.py
```

单独启动：

```bash
ros2 run rhw_task_scheduler waypoint_manager
ros2 run rhw_task_scheduler mission_bt_node
```

启动行为树 Web 可视化：

```bash
ros2 launch rhw_task_scheduler task_scheduler.launch.py bt_viewer:=true

ros2 launch rhw_task_scheduler mission_test.launch.py \
  use_real_waypoints:=true \
  use_real_navigation:=false \
  use_real_ptz:=true \
  bt_viewer:=true

```

浏览器访问：

```text
http://localhost:8765/
```

## 全流程测试

如果你要在没有完整导航栈的情况下测试 `/mission/start` 的整条流程，可以直接启动测试环境：

```bash
# 如需使用真实云台，先启动 PTZ 控制器
ros2 launch rhw_ptz_controller ptz_controller.launch.py

# 默认使用真实 PTZ，其余接口走 mock
ros2 launch rhw_task_scheduler mission_test.launch.py
```

全 mock 测试视觉点完整流程：

```bash
ros2 launch rhw_task_scheduler mission_test.launch.py \
  use_real_waypoints:=false \
  use_real_navigation:=false \
  use_real_ptz:=false \
  use_real_album_upload:=false \
  use_real_model_task:=false \
  bt_viewer:=true
```

默认会提供这些测试航点：

- `normal_001`：普通导航点
- `vision_001`：视觉点，走 PTZ 绝对位置移动 + 抓拍 + 相册上传 + 模型任务
- `charge_001`：充电点

然后在 `rqt` 或命令行里调用：

```bash
ros2 service call /mission/start rhw_msgs/srv/StartMission "{
  map_name: 'factory_map',
  waypoint_ids: ['normal_001', 'vision_001', 'charge_001']
}"
```

如果你还没有真实 PTZ，可以把 `use_real_ptz:=false`，测试节点会同时模拟 `/ptz/absolute_move`、`/ptz/status` 和 `/ptz/capture_image`：

```bash
ros2 launch rhw_task_scheduler mission_test.launch.py use_real_ptz:=false
```

如果你已经有真实点位，但暂时没有真实导航，可以保留真实点位管理，只使用真实云台，上传和模型仍使用 mock：

```bash
ros2 launch rhw_task_scheduler mission_test.launch.py \
  use_real_waypoints:=true \
  use_real_navigation:=false \
  use_real_ptz:=true \
  use_real_album_upload:=false \
  use_real_model_task:=false \
  bt_viewer:=true
```

这时 `/mission/start` 里的 `map_name` 和 `waypoint_ids` 必须使用真实 `waypoint_manager` 中已经存在的数据。如果你已经单独启动了 `waypoint_manager`，可以额外加 `launch_waypoint_manager:=false`，避免重复启动。

需要自定义航点时，可以通过 `waypoints_json` 传入 JSON 覆盖默认数据。

`use_real_album_upload:=true` 会让行为树调用真实 `/inspection/album_report/upload`，`use_real_model_task:=true` 会调用真实 `/rhw/model/task/run`；保持为 `false` 时由 `mission_test_mocks` 提供 `/test/...` 服务。`album_upload_result:=false` 或 `model_task_result:=false` 可让对应 mock 返回失败，用于验证视觉点失败分支。

## 配置

配置文件：`config/task_scheduler.yaml`

### waypoint_manager

| 参数 | 默认值 | 说明 |
|---|---|---|
| `storage_dir` | `~/.rhw/waypoints` | 点位持久化目录 |
| `mqtt_sync_enabled` | `true` | 是否启用点位 MQTT 主动同步 |
| `mqtt_broker_host` | `8.130.34.168` | MQTT broker 地址 |
| `mqtt_broker_port` | `1883` | MQTT broker 端口 |
| `mqtt_client_id` | `robot-inspect-forwarder` | MQTT 客户端 ID |
| `mqtt_username` | `admin` | MQTT 用户名 |
| `mqtt_password` | `public` | MQTT 密码 |
| `mqtt_waypoint_sync_topic` | `/robot-dog/DOG001/Upload/Data` | 点位同步 topic |
| `mqtt_qos` | `1` | MQTT QoS |
| `mqtt_keep_alive_sec` | `60` | MQTT keep alive |
| `add_waypoint_service` | `/waypoint_manager/add_waypoint` | 添加点位服务 |
| `delete_waypoint_service` | `/waypoint_manager/delete_waypoint` | 删除点位服务 |
| `get_waypoints_service` | `/waypoint_manager/get_waypoints` | 查询点位服务 |

### mission_bt_node

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
| `inspection_album_upload_service` | `/inspection/album_report/upload` | 抓拍后同步相册上传服务 |
| `album_upload_timeout_sec` | `30.0` | 相册上传服务调用超时 |
| `model_task_run_service` | `/rhw/model/task/run` | 模型调度任务服务 |
| `model_task_timeout_sec` | `60.0` | 模型任务服务调用超时 |
| `ptz_stable_timeout_sec` | `5.0` | 等待云台稳定超时 |
| `default_ptz_channel` | `1` | 默认云台通道 |
| `recharge_service` | `/recharge` | 回充服务 |
| `battery_topic` | `/robot/battery_status` | 电池状态话题 |
| `low_battery_threshold` | `20.0` | 低电量阈值 |
| `waypoint_task_timeout_sec` | `120.0` | 航点任务超时预留参数 |
| `mqtt_enabled` | `false` | 是否启用 MQTT 任务下发 |
| `mqtt_broker_host` | `127.0.0.1` | MQTT broker 地址 |
| `mqtt_broker_port` | `1883` | MQTT broker 端口 |
| `mqtt_client_id` | `rhw_mission_bt` | MQTT 客户端 ID |
| `mqtt_mission_start_topic` | `rhw/mission/start` | MQTT 任务下发 topic |
| `mqtt_mission_status_topic` | `rhw/mission/status` | MQTT 状态回传 topic |
| `mission_status_topic` | `/mission/status` | ROS 任务状态话题 |
| `debug_print_tree_on_build` | `true` | 构建航点树时打印文本树 |
| `debug_print_tree_on_tick` | `false` | tick 时周期打印文本树 |
| `debug_tree_show_status` | `true` | 文本树显示节点状态 |
| `debug_tree_log_every_n_ticks` | `1` | 文本树打印周期 |
| `debug_export_tree_dot` | `false` | 是否导出 DOT/PNG/SVG |
| `debug_tree_output_dir` | `/tmp/rhw_task_scheduler_bt` | 树图导出目录 |

## 点位接口

### 添加点位

```bash
ros2 service call /waypoint_manager/add_waypoint rhw_msgs/srv/AddWaypoint "{
  waypoint: {
    waypoint_id: 'vision_001',
    map_name: 'factory_map',
    pose: {x: 1.2, y: 3.4, theta: 0.0},
    waypoint_type: 2,
    label: '视觉检测点1',
    task_params: '{\"azimuth\":180.0,\"elevation\":0.0,\"zoom\":6.5,\"channel\":1,\"azimuth_speed\":50,\"elevation_speed\":50,\"inference_type\":\"fire_equipment_detection\"}'
  }
}"
```

视觉点位 `task_params`：

- 必填：`azimuth`、`elevation`、`inference_type`
- 可选：`zoom`、`channel`、`azimuth_speed`、`elevation_speed`
- `zoom` 对应设备 ISAPI 的 `absoluteZoom` 原始值；不填或填 `0` 表示不调整倍率
- `inference_type` 必须是模型调度服务的 `task_name`，例如 `fire_equipment_detection`

普通导航点和充电点可将 `task_params` 留空。

### 查询点位

```bash
ros2 service call /waypoint_manager/get_waypoints rhw_msgs/srv/GetWaypoints "{
  map_name: 'factory_map'
}"
```

### 删除点位

```bash
ros2 service call /waypoint_manager/delete_waypoint rhw_msgs/srv/DeleteWaypoint "{
  map_name: 'factory_map',
  waypoint_id: 'vision_001'
}"
```

## 任务接口

### 启动任务

```bash
ros2 service call /mission/start rhw_msgs/srv/StartMission "{
  map_name: 'factory_map',
  waypoint_ids: ['normal_001', 'vision_001', 'charge_001']
}"
```

### 暂停 / 恢复 / 停止

```bash
ros2 service call /mission/pause rhw_msgs/srv/PauseMission "{pause: true}"
ros2 service call /mission/pause rhw_msgs/srv/PauseMission "{pause: false}"
ros2 service call /mission/stop rhw_msgs/srv/StopMission "{}"
```

### 查看状态

```bash
ros2 topic echo /mission/status
ros2 topic echo /service_events
```

## MQTT

当 `mission_bt_node.mqtt_enabled=true` 时，节点订阅任务下发 topic。

任务下发示例：

```json
{
  "map_name": "factory_map",
  "waypoint_ids": ["normal_001", "vision_001", "charge_001"]
}
```

状态回传示例：

```json
{
  "status": 1,
  "current_waypoint_id": "vision_001",
  "total_waypoints": 3,
  "completed_waypoints": 1
}
```

当 `waypoint_manager.mqtt_sync_enabled=true` 时，保存或删除点位后会主动发布当前地图点位快照。

## 目录结构

```text
rhw_task_scheduler/
├── README.md
├── config/task_scheduler.yaml
├── launch/task_scheduler.launch.py
├── launch/mission_test.launch.py
├── setup.py
└── rhw_task_scheduler/
    ├── waypoint_manager.py
    ├── mission_bt_node.py
    ├── mission_test_mocks.py
    ├── bt_utils.py
    ├── service_audit.py
    ├── bt_web_viewer.py
    └── bt_actions/
        ├── condition_nodes.py
        ├── navigate_action.py
        ├── ptz_actions.py
        ├── vision_actions.py
        └── charge_action.py
```

## 当前限制

- 当前任务执行采用“单航点单行为树”方式串行推进。
- 当前未实现任务结果持久化，如需任务审计或报表，可补充数据库或 JSON 日志。
- 当前未实现部署阶段的 MQTT 点位管理请求接口；点位同步为主动上报模式。
