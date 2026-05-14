# 视觉点抓拍、相册上报与模型推理行为树接入设计

## 背景

当前巡检任务调度已经支持视觉点的导航、云台移动、等待稳定和抓拍。云台抓拍接口会把图片保存到本地，并返回 `file_path`、`capture_url` 和 `file_size`。HTTPS 相册上报已经由 `rhw_udp_mqtt_bridge` 中的 `inspection_reporter_node` 负责平台协议、加密、签名和重试。模型调度服务已经由 `rhw_model_scheduler` 提供，服务接口为 `/rhw/model/task/run`，类型为 `rhw_msgs/srv/ModelTaskRun`。

这次接入的目标是让视觉点任务在行为树中显式执行三个解耦步骤：

1. 云台抓拍，得到本地图片路径。
2. 调用 HTTPS 相册上报任务，上传抓拍图片。
3. 调用模型推理任务，根据点位部署中的 `inference_type` 执行对应模型任务。

## 目标

- 在行为树中新增独立的相册上传节点和模型推理节点。
- `CaptureImage` 只负责抓拍和记录本地图片信息，不直接负责 HTTPS 或模型调用。
- HTTPS 上传失败、模型推理失败都让当前视觉点失败，并沿用现有任务调度逻辑跳到下一个点。
- 点位 `task_params.inference_type` 直接对应模型调度服务的 `task_name`，例如 `fire_equipment_detection`。
- `ModelTaskRun` 接口纳入顶层 `rhw_msgs` 生成列表，保证任务调度工作区可稳定导入该服务类型。

## 非目标

- 不把 HTTPS 平台协议、AES、签名逻辑移动到 `rhw_task_scheduler`。
- 不改变模型调度服务内部实现。
- 不实现告警上报 `alarm/report`。
- 不把旧的 `inference_type: "det"` 自动映射成新的模型任务名；点位部署侧需要写入完整 `task_name`。

## 总体方案

推荐使用 service 方式让行为树同步获得相册上传结果。保留已有 `/inspection/album_reports` topic 用于异步事件或调试，但视觉点成败判定使用新增上传 service。

视觉点行为树顺序调整为：

```text
VisionTask
├── IsVisionPoint
├── WaitAfterNavArrived
├── PtzAbsoluteMove
├── WaitPtzStable
├── WaitAfterPtzArrived
├── CaptureImage
├── UploadInspectionAlbum
└── RunModelTask
```

每个节点只做一件事：

- `CaptureImage` 调用 `/ptz/capture_image`，成功后写入 blackboard。
- `UploadInspectionAlbum` 从 blackboard 读取抓拍结果，调用 `inspection_reporter_node` 提供的 ROS service。
- `RunModelTask` 从当前航点 `task_params` 读取 `inference_type`，调用 `/rhw/model/task/run`。

## 接口设计

### 抓拍结果 blackboard

`CaptureImage` 成功后写入以下 blackboard 字段：

```text
/last_capture_path: str
/last_capture_url: str
/last_capture_file_size: int
```

现有 `/last_capture_path` 保持不变，新增 URL 和文件大小是为了让上传节点不依赖 `CaptureImage` 的内部状态。

### 相册上传 service

在 `rhw_msgs/srv` 新增 `InspectionAlbumUpload.srv`：

```text
string task_id
string point_id
string point_name
string image_path
string capture_url
uint32 file_size
---
bool ok
string code
string message
string trace_id
int32 http_status
string response_body
```

`inspection_reporter_node` 新增 service server，默认服务名：

```text
/inspection/album_report/upload
```

服务端复用当前 HTTPS 上报实现，包括图片读取、Base64、加密、签名、超时和重试。返回 `ok=true` 表示平台请求已成功完成；HTTP 超时、非成功响应、平台业务失败或本地图片读取失败都返回 `ok=false`。

### 模型推理 service

使用现有服务：

```text
/rhw/model/task/run
rhw_msgs/srv/ModelTaskRun
```

`RunModelTask` 请求字段：

```text
request_id: 自动生成，建议格式为 <mission_task_id>-<waypoint_id>-<timestamp>
task_name: task_params.inference_type
conf: 0.25
iou: 0.45
max_det: 100
wait_for_frame_timeout_sec: 3.0
max_frame_age_sec: 2.0
params_json: ''
```

其他参数保持默认值。后续如需要单点覆盖，可以在 `task_params` 中扩展 `model_conf`、`model_iou` 等字段，本轮不实现。

## 数据流

1. `/mission/start` 启动任务，`mission_bt_node` 生成或记录 `task_id`。
2. 到达视觉点后，`CaptureImage` 抓拍并保存本地图片。
3. `UploadInspectionAlbum` 用当前任务 ID、点位 ID、点位名称和抓拍结果调用上传 service。
4. 上传 service 内部完成 HTTPS POST，并返回明确成功或失败。
5. `RunModelTask` 调用模型调度服务，`inference_type` 作为 `task_name`。
6. 三个后置步骤全部成功时，视觉点完成；任一步失败时，视觉点失败并进入下一个点。

## 失败处理

- `CaptureImage` 返回失败：视觉点失败。
- 图片路径为空或文件不存在：上传 service 返回失败，视觉点失败。
- HTTPS 上传超时、重试后失败、平台返回业务失败：视觉点失败。
- `task_params.inference_type` 为空：模型推理节点返回失败，视觉点失败。
- 模型服务不可用、超时、抛异常或响应 `ok=false`：视觉点失败。

行为树根节点失败后的任务级处理沿用现有逻辑：记录当前点失败，然后跳到下一个点；所有点处理完后任务结束。

## 配置

`mission_bt_node` 新增参数：

```yaml
inspection_album_upload_service: "/inspection/album_report/upload"
model_task_run_service: "/rhw/model/task/run"
model_task_timeout_sec: 60.0
album_upload_timeout_sec: 30.0
```

`inspection_reporter_node` 新增或复用参数：

```yaml
album_upload_service: "/inspection/album_report/upload"
```

默认保留 `enable_inspection_reporter:=false`，避免未配置平台地址时误发。需要全流程验证时显式开启。

## 文档更新

- `rhw_task_scheduler/README.md` 增加视觉点完整流程说明。
- `WaypointTask.msg` 示例把 `inference_type` 改为完整模型任务名，例如 `fire_equipment_detection`。
- `rhw_udp_mqtt_bridge/README.md` 补充同步上传 service 的测试命令。
- `rhw_model_scheduler_ws/README.md` 可补充“点位部署中的 `inference_type` 应填写 `task_name`”。

## 测试计划

编译检查：

```bash
colcon build --packages-select rhw_msgs rhw_task_scheduler rhw_udp_mqtt_bridge
python3 -m compileall rhw_task_scheduler rhw_udp_mqtt_bridge
```

接口检查：

```bash
ros2 interface show rhw_msgs/srv/InspectionAlbumUpload
ros2 interface show rhw_msgs/srv/ModelTaskRun
```

单点上传测试：

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

模型服务测试：

```bash
ros2 service call /rhw/model/task/run rhw_msgs/srv/ModelTaskRun "{
  request_id: 'det-001',
  task_name: 'fire_equipment_detection',
  conf: 0.25,
  iou: 0.45,
  max_det: 100,
  wait_for_frame_timeout_sec: 3.0,
  max_frame_age_sec: 2.0,
  params_json: ''
}"
```

全流程测试：

1. 启动云台、模型调度、`inspection_reporter_node` 和任务调度。
2. 点位 `task_params` 填写 PTZ 参数和 `inference_type: "fire_equipment_detection"`。
3. 通过 `/mission/start` 启动包含视觉点的任务。
4. 确认行为树中依次出现抓拍、上传、模型推理成功。
5. 分别关闭上传服务或模型服务，确认当前视觉点失败并跳到下一个点。

## 后续扩展

- 如平台需要模型结果 HTTPS 上报，可在 `RunModelTask` 后新增独立结果上报节点。
- 如同一视觉点需要多个模型任务，可把 `inference_type` 扩展为数组字段，例如 `inference_types`。
- 如需要兼容旧值 `det/kpt/seg/gauge`，可在配置文件增加映射表，但本轮不默认实现。
