# AGENTS

这个目录是独立 ROS2 模型调度工作空间，交付给同事开发和工控机验证使用。

## 核心规则

- 工作空间根目录固定为 `/home/wu/data/rhw_model_scheduler_ws`。
- 工控机验证路径固定为 `/home/test/data/rhw_model_scheduler_ws`。
- 不要使用软链接回 `/home/wu/data/python_refactor` 或 `/home/test/data/python_refactor`。
- 模型源权重维护 manifest 中的 `.pt`：`det`、`kpt`、`seg`、`gauge`；不要重新加入 ONNX 任务。
- 工控机端允许生成同名 `.engine` TensorRT 文件；这是设备相关运行产物，调度器会优先使用 `.engine`，不存在时回退到 `.pt`。
- 服务接口是 `/rhw/model/task/run`，类型是 `rhw_msgs/srv/ModelTaskRun`。
- 相机输入由节点参数 `camera_stream_url` 指定，传完整 RTSP 地址；不要把真实账号密码写入文档或提交历史。

## 构建与验证

工控机或本地 ROS2 环境中：

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/pytorch_env/bin/activate
cd /home/test/data/rhw_model_scheduler_ws
python -m colcon build --symlink-install --packages-select rhw_msgs rhw_model_scheduler
source install/setup.bash
```

验证必须至少包含：

```bash
python -m unittest discover -s src/rhw_model_scheduler/tests
ros2 run rhw_model_scheduler rhw_model_scheduler_smoke --task-name fire_equipment_detection --timeout-sec 60
ros2 run rhw_model_scheduler rhw_model_scheduler_smoke --task-name colormeter_gauge --timeout-sec 60
ros2 run rhw_model_scheduler rhw_model_export_tensorrt --workspace-root /home/test/data/rhw_model_scheduler_ws --imgsz 640 --device 0
ros2 run rhw_model_scheduler rhw_model_latency_benchmark --video sample_data/20260428_142826.mp4 --sample-count 100 --tasks fire_equipment_detection front_panel_pose rust_segmentation colormeter_gauge
```

验证完成后要汇报 det/kpt/seg/gauge 的 `count`、`mean_ms`、`p50_ms`、`p90_ms`、`p95_ms`、`min_ms`、`max_ms`、`error_count`，以及结果 JSON/CSV 路径。
