# RHW Workspace Dependency Requirements

本文根据各功能包 README、`package.xml`、`setup.py` 与源码 import 整理，用于 ROS 2 Humble 环境安装依赖；不包含视觉检测/模型调度部分。

## 1. 基础环境

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  python3-pip \
  python3-colcon-common-extensions \
  python3-opencv \
  alsa-utils
```

## 2. ROS 2 / apt 依赖

```bash
sudo apt install -y \
  ros-humble-ament-cmake \
  ros-humble-ament-index-python \
  ros-humble-action-msgs \
  ros-humble-builtin-interfaces \
  ros-humble-cv-bridge \
  ros-humble-geometry-msgs \
  ros-humble-launch \
  ros-humble-launch-ros \
  ros-humble-nav-msgs \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-py-trees-ros \
  ros-humble-py-trees-ros-interfaces \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-rviz2 \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-tf2-ros
```

说明：

- `rhw_msgs` 需要 `rosidl_default_generators/runtime`、`geometry_msgs`、`std_msgs`、`nav_msgs`。
- `rhw_task_scheduler` 需要 `nav2_msgs`、`py_trees`、`py_trees_ros_interfaces`。
- `rhw_ptz_controller` 的图像发布需要 `sensor_msgs`、`cv_bridge`、OpenCV。
- `rhw_map_manager` 的导航模式会调用 `nav2_bringup`、`rviz2`、`tf2_ros`，并依赖现场已有 `lightning` 建图/定位环境。
- `rhw_firedog_voice` 实时语音采集使用系统 `arecord`，由 `alsa-utils` 提供。

## 3. Python / pip 依赖

推荐使用系统 Python 或已配置 ROS 环境的虚拟环境：

```bash
python3 -m pip install -U pip
python3 -m pip install -r requirements.txt
```

`requirements.txt` 覆盖以下功能：

- PTZ HTTP/HTTPS 控制：`requests`
- MQTT 网关：`paho-mqtt`
- HTTPS 相册加密/签名：`cryptography`
- 行为树：`py_trees`
- 行为树 Web 可视化：`websockets`
- 语音识别：`vosk`
- 摄像头图像发布：`opencv-python`
- 测试：`pytest`

注意：

- 若希望完全使用 apt 版 OpenCV，可安装 `python3-opencv` 后跳过或移除 `requirements.txt` 中的 `opencv-python`。

## 4. 编译顺序

```bash
cd ~/rhw_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果只编译核心任务链路，可先编译：

```bash
colcon build --packages-select \
  rhw_msgs \
  rhw_ptz_controller \
  rhw_udp_mqtt_bridge \
  rhw_task_scheduler \
  rhw_map_manager
```

## 5. 外部运行环境

以下不是 pip/apt 能完整安装的普通 Python 依赖，需要现场系统提供：

- ROS 2 Humble
- Nav2 运行栈与地图服务
- `lightning` 建图/定位程序及其配置
- 海康 PTZ/RTSP 摄像头网络连通性
- MQTT broker 与 HTTPS 平台接口地址
- Vosk 中文模型目录：`rhw_firedog_voice/model/vosk-model-small-cn-0.22`
