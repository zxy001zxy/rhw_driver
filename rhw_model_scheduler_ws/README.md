# RHW ROS2 模型调度工作空间

这是一个独立 ROS2 workspace，可以直接复制到工控机运行，不依赖 `/home/wu/data/python_refactor`，也不使用软链接回原仓库。

## 内容

```text
rhw_model_scheduler_ws/
├── src/rhw_msgs/                 # ModelTaskRun.srv
├── src/rhw_model_scheduler/      # ROS2 node、调度核心、smoke、benchmark
├── models/current/               # 当前调度模型 .pt 与工控机生成的 .engine
├── sample_data/                  # 现场录制视频样例
└── runtime/                      # 运行输出
```

当前任务：

| task_name | task_type | 模型 |
| --- | --- | --- |
| `fire_equipment_detection` | `det` | `models/current/fire_equipment_detection.pt` |
| `front_panel_pose` | `kpt` | `models/current/front_panel_pose.pt` |
| `rust_segmentation` | `seg` | `models/current/rust_segmentation.pt` |
| `colormeter_gauge` | `gauge` | `models/current/colormeter_gauge.pt` |

## 构建

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/pytorch_env/bin/activate
cd /home/test/data/rhw_model_scheduler_ws
python -m colcon build --symlink-install --packages-select rhw_msgs rhw_model_scheduler
source install/setup.bash
```

必须使用激活后的 `python -m colcon build`，避免 `ros2 run` 入口脚本落到系统 Python，导致 `torch`、`ultralytics` 或 `cv2` 不可用。

## 启动服务

```bash
ros2 run rhw_model_scheduler rhw_model_scheduler_node \
  --ros-args \
  -p workspace_root:=/home/test/data/rhw_model_scheduler_ws \
  -p camera_stream_url:='rtsp://USER:PASS@CAMERA_IP:554/Streaming/Channels/101' \
  -p preload_models:=true
```

服务：

```text
/rhw/model/task/run
```

类型：

```text
rhw_msgs/srv/ModelTaskRun
```

示例调用：

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

### 任务调度接入约定

任务调度行为树会把视觉点 `task_params.inference_type` 直接作为本服务的 `task_name`。点位部署时请填写完整任务名，例如 `fire_equipment_detection`、`front_panel_pose`、`rust_segmentation` 或 `colormeter_gauge`。

除 `request_id` 和 `task_name` 外，任务调度保持模型调用参数为默认值：`conf=0.25`、`iou=0.45`、`max_det=100`、`wait_for_frame_timeout_sec=3.0`、`max_frame_age_sec=2.0`、`params_json=''`。

## Smoke 验证

启动 node 后，另开一个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/pytorch_env/bin/activate
source /home/test/data/rhw_model_scheduler_ws/install/setup.bash
cd /home/test/data/rhw_model_scheduler_ws

ros2 run rhw_model_scheduler rhw_model_scheduler_smoke \
  --task-name colormeter_gauge \
  --timeout-sec 60
```

## TensorRT 加速

在工控机本机导出 TensorRT engine。engine 与 `.pt` 同目录同名保存，例如 `front_panel_pose.engine`。

```bash
ros2 run rhw_model_scheduler rhw_model_export_tensorrt \
  --workspace-root /home/test/data/rhw_model_scheduler_ws \
  --imgsz 640 \
  --device 0
```

默认导出 FP16 engine。导出完成后，调度器会对 manifest 中的任务自动优先使用同名 `.engine`；如果 engine 不存在，会回退到 `.pt`。

## 视频延时 Benchmark

默认用 `sample_data/20260428_142826.mp4`，均匀抽样 100 帧。未指定 `--tasks` 时会跑 manifest 中的全部任务：

```bash
ros2 run rhw_model_scheduler rhw_model_latency_benchmark \
  --video sample_data/20260428_142826.mp4 \
  --sample-count 100
```

显式指定四个当前任务：

```bash
ros2 run rhw_model_scheduler rhw_model_latency_benchmark \
  --video sample_data/20260428_142826.mp4 \
  --sample-count 100 \
  --tasks fire_equipment_detection front_panel_pose rust_segmentation colormeter_gauge
```

输出：

```text
runtime/latency_benchmark/<run_id>/summary.json
runtime/latency_benchmark/<run_id>/summary.csv
```

统计字段包括：

- `count`
- `mean_ms`
- `p50_ms`
- `p90_ms`
- `p95_ms`
- `min_ms`
- `max_ms`
- `error_count`

默认会先 warmup 每个模型一次，warmup 不计入延时统计。

## 真实相机验证

启动节点时传入完整 RTSP 地址。不要把真实账号密码写进仓库文档或提交历史。

```bash
ros2 run rhw_model_scheduler rhw_model_scheduler_node \
  --ros-args \
  -p workspace_root:=/home/test/data/rhw_model_scheduler_ws \
  -p camera_stream_url:='rtsp://USER:PASS@CAMERA_IP:554/Streaming/Channels/101'
```

另开一个终端运行 smoke。smoke 只调用 service，不再发布合成图像：

```bash
ros2 run rhw_model_scheduler rhw_model_scheduler_smoke \
  --task-name fire_equipment_detection \
  --timeout-sec 60
```

## 同步到工控机

本机同步：

```bash
rsync -az --delete --info=progress2 \
  --exclude='models/current/*.engine' \
  --exclude='runtime/' \
  --exclude='build/' \
  --exclude='install/' \
  --exclude='log/' \
  /home/wu/data/rhw_model_scheduler_ws/ \
  robot_179-pc:/home/test/data/rhw_model_scheduler_ws/
```

同步后在工控机重新构建并运行上面的 smoke 与 benchmark。
