# RHW 模型调度接入说明

本目录用于记录任务调度行为树与模型调度服务的接入约定。模型调度实现可以放在独立 ROS 2 workspace 中，但需要提供下面的 service。

## Service

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

## 任务调度接入约定

任务调度行为树会把视觉点 `task_params.inference_type` 直接作为本服务的 `task_name`。点位部署时请填写完整任务名，例如 `fire_equipment_detection`、`front_panel_pose`、`rust_segmentation` 或 `colormeter_gauge`。

除 `request_id` 和 `task_name` 外，任务调度保持模型调用参数为默认值：`conf=0.25`、`iou=0.45`、`max_det=100`、`wait_for_frame_timeout_sec=3.0`、`max_frame_age_sec=2.0`、`params_json=''`。

模型服务返回 `ok=false` 时，行为树会把当前视觉点判定为失败；返回 `ok=true` 时会把 `result_json_path` 写入黑板 `/last_model_result_json_path`，供后续扩展节点使用。


当前任务：

| task_name | task_type | 模型 |
| --- | --- | --- |
| `fire_equipment_detection` | `det` | `models/current/fire_equipment_detection.pt` |
| `front_panel_pose` | `kpt` | `models/current/front_panel_pose.pt` |
| `rust_segmentation` | `seg` | `models/current/rust_segmentation.pt` |
| `colormeter_gauge` | `gauge` | `models/current/colormeter_gauge.pt` |