# rhw_ptz_controller

海康威视 PTZ 云台控制 + 摄像头图像发布 ROS 2 功能包。

## 概述

本包提供两个 ROS 2 节点：

| 节点 | 功能 |
|---|---|
| `ptz_controller_node` | 通过 ISAPI 协议控制海康 PTZ 云台（方向/变倍/预置位/巡航/绝对位置），并周期发布云台状态 |
| `camera_publisher_node` | 通过 RTSP 取流，以 `sensor_msgs/Image` 话题发布摄像头画面 |

---

## 依赖

- ROS 2 Humble
- `rhw_msgs`（自定义接口包）
- `sensor_msgs`、`cv_bridge`、`std_msgs`
- Python 依赖：`requests`、`opencv-python`

---

## 编译

```bash
cd ~/Desktop/project/rhw_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rhw_msgs rhw_ptz_controller
source install/setup.bash
```

---

## 启动

### 一键启动（PTZ + 摄像头）

```bash
ros2 launch rhw_ptz_controller ptz_controller.launch.py
```

### 单独启动

```bash
# 仅 PTZ 控制
ros2 run rhw_ptz_controller ptz_controller_node

# 仅摄像头发布
ros2 run rhw_ptz_controller camera_publisher_node
```

---

## 配置

配置文件：`config/ptz_controller.yaml`

### ptz_controller_node 参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `camera_ip` | string | `192.168.10.64` | 摄像头 IP |
| `camera_port` | int | `80` | ISAPI HTTP 端口 |
| `camera_username` | string | `admin` | 登录用户名 |
| `camera_password` | string | `rhw1314000` | 登录密码 |
| `use_https` | bool | `false` | 是否使用 HTTPS |
| `verify_ssl` | bool | `false` | 是否验证 SSL 证书 |
| `timeout` | float | `5.0` | HTTP 请求超时(秒) |
| `default_channel` | int | `1` | 默认通道号 |
| `default_speed` | int | `40` | 默认速度 1-100 |
| `default_duration_ms` | int | `350` | 默认持续时间(ms)，0=持续到手动stop |
| `status_publish_period` | float | `2.0` | 状态发布周期(秒) |
| `capture_save_dir` | string | `/tmp/ptz_captures` | 抓拍图片默认保存目录 |

### camera_publisher_node 参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `camera_ip` | string | `192.168.10.64` | 摄像头 IP |
| `rtsp_port` | int | `554` | RTSP 端口 |
| `rtsp_username` | string | `admin` | RTSP 用户名 |
| `rtsp_password` | string | `rhw1314000` | RTSP 密码 |
| `rtsp_path` | string | `/Streaming/Channels/101` | RTSP 路径（101=主码流，102=子码流） |
| `rtsp_url_override` | string | `""` | 非空则直接使用此完整 RTSP URL |
| `frame_rate` | float | `30.0` | 发布帧率 (Hz) |
| `image_topic` | string | `/camera/rgb/image_raw` | 图像话题名 |
| `frame_id` | string | `camera_link` | 图像帧的 frame_id |
| `reconnect_interval` | float | `3.0` | 断线重连间隔(秒) |
| `rtsp_transport` | string | `tcp` | RTSP 传输方式，`tcp` 更稳定 |
| `publish_compressed` | bool | `true` | 是否同时发布压缩图像 |
| `jpeg_quality` | int | `70` | 压缩图像 JPEG 质量 |
| `output_width` | int | `1920` | 输出宽度，0 表示保持原始尺寸 |
| `output_height` | int | `1080` | 输出高度，0 表示保持原始尺寸 |
| `qos_reliability` | string | `reliable` | `reliable` / `best_effort` |
| `qos_depth` | int | `1` | 发布队列深度 |

---

## 服务接口

### /ptz/control — 方向控制

```bash
ros2 service call /ptz/control rhw_msgs/srv/PtzControl \
  "{direction: 'left', speed: 40, channel: 1, duration_ms: 500}"
```

支持方向：`left` `right` `up` `down` `leftup` `rightup` `leftdown` `rightdown` `zoomin` `zoomout` `stop`

| 请求字段 | 类型 | 说明 |
|---|---|---|
| `direction` | string | 方向 |
| `speed` | uint8 | 速度 1-100 |
| `channel` | uint8 | 通道号，默认 1 |
| `duration_ms` | uint32 | 持续时间(ms)，0=不自动停止 |

| 响应字段 | 类型 | 说明 |
|---|---|---|
| `result` | int8 | 0=失败 1=成功 |
| `execution_mode` | string | single / timed_auto_stop / continuous_until_manual_stop |
| `message` | string | 结果说明 |

### /ptz/goto_preset — 跳转预置位

```bash
ros2 service call /ptz/goto_preset rhw_msgs/srv/PtzGotoPreset \
  "{channel: 1, preset_id: 1}"
```

### /ptz/patrol — 启停巡航

```bash
# 启动巡航
ros2 service call /ptz/patrol rhw_msgs/srv/PtzPatrol \
  "{channel: 1, patrol_id: 1, action: 1}"

# 停止巡航
ros2 service call /ptz/patrol rhw_msgs/srv/PtzPatrol \
  "{channel: 1, patrol_id: 1, action: 0}"
```

### /ptz/absolute_move — 绝对位置移动

```bash
ros2 service call /ptz/absolute_move rhw_msgs/srv/PtzAbsoluteMove \
  "{channel: 1, azimuth: 180.0, elevation: 0.0, azimuth_speed: 50, elevation_speed: 50}"
```

### /ptz/get_position — 获取当前角度

```bash
ros2 service call /ptz/get_position rhw_msgs/srv/PtzGetPosition \
  "{channel: 1}"
```

| 响应字段 | 类型 | 说明 |
|---|---|---|
| `result` | int8 | 0=失败 1=成功 |
| `azimuth` | float32 | 方位角（水平，度） |
| `elevation` | float32 | 俯仰角（垂直，度） |
| `message` | string | 结果说明 |

### /ptz/capture_image — 手动抓拍并保存

```bash
# 本地保存模式（推荐）
ros2 service call /ptz/capture_image rhw_msgs/srv/CaptureImage \
  "{channel: 101, url_type: 'localURL', channel_format: 'streamTrack', save_path: '/tmp/ptz_captures/cap_101.jpg', image_type: 'JPEG'}"

# 云端 URL 模式（设备需已配置图片服务器）
ros2 service call /ptz/capture_image rhw_msgs/srv/CaptureImage \
  "{channel: 101, url_type: 'cloudURL', channel_format: 'streamTrack', save_path: '/tmp/ptz_captures/cap_cloud_101.jpg', image_type: 'JPEG'}"
```

| 请求字段 | 类型 | 说明 |
|---|---|---|
| `channel` | uint8 | 通道号；普通通道可传 `1`，码流通道可传 `101/102` |
| `url_type` | string | `localURL` 或 `cloudURL` |
| `channel_format` | string | 留空表示普通通道；`streamTrack` 表示按码流 ID 抓图 |
| `save_path` | string | 本地保存完整路径；留空则自动保存在 `capture_save_dir` |
| `image_type` | string | 当前设备仅支持 `JPEG` |

| 响应字段 | 类型 | 说明 |
|---|---|---|
| `result` | int8 | `0=失败`，`1=成功` |
| `capture_url` | string | 抓拍使用的 URL；`cloudURL` 时为设备返回的存储 URL，`localURL` 时为同步抓图接口地址 |
| `file_path` | string | 实际保存到本地的文件路径 |
| `file_size` | uint32 | 图片大小（字节） |
| `saved` | bool | 是否已成功保存到本地 |
| `message` | string | 结果说明 |

抓拍逻辑说明：

- `localURL`：优先使用同步快照接口 `GET /ISAPI/Streaming/channels/<channel>/picture`，直接返回 JPEG 并保存到本地；这是当前设备实测可用的推荐方式。
- `cloudURL`：调用异步接口 `GET /ISAPI/Streaming/channels/<channel>/picture/async?...`，从返回的 `PictureData.url` 下载并保存；前提是设备已配置图片服务器。
- `channel_format=streamTrack` 时，`channel` 可传 `101`、`102` 这类码流 ID；留空则按普通通道号处理。
- 当前设备的抓拍能力接口仅声明支持 `imageType=JPEG` 和 `URLType=cloudURL`，因此本地保存模式采用同步快照接口兜底。

---

## 话题

### /ptz/status — 云台状态（PtzStatus）

周期发布，默认 2 秒。

| 字段 | 类型 | 说明 |
|---|---|---|
| `header` | Header | 时间戳 |
| `online` | bool | 设备是否在线 |
| `channel` | uint8 | 通道号 |
| `azimuth` | float32 | 当前方位角 |
| `elevation` | float32 | 当前俯仰角 |
| `active_action` | string | 当前动作（idle / patrol:N / moving / preset:N） |
| `message` | string | 附加说明 |

### /camera/rgb/image_raw — 摄像头图像（sensor_msgs/Image）

| 字段 | 说明 |
|---|---|
| `encoding` | `bgr8` |
| `header.frame_id` | 可配置，默认 `camera_link` |
| 帧率 | 可配置，默认 30 Hz |

查看图像：

```bash
# 使用 rqt
ros2 run rqt_image_view rqt_image_view

# 或查看话题信息
ros2 topic hz /camera/rgb/image_raw
ros2 topic info /camera/rgb/image_raw
```

---

## 文件结构

```
rhw_ptz_controller/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   └── ptz_controller.yaml          # 参数配置
├── launch/
│   └── ptz_controller.launch.py     # 启动文件
├── resource/
│   └── rhw_ptz_controller
└── rhw_ptz_controller/
    ├── __init__.py
    ├── ptz_controller.py             # ISAPI PTZ 控制库
    ├── ptz_controller_node.py        # PTZ ROS 2 节点
    └── camera_publisher_node.py      # 摄像头图像发布节点
```


## websocket
```bash
ros2 launch rhw_ptz_controller ptz_controller.launch.py
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9080
ros2 run web_video_server web_video_server
```

`web_video_server` 默认端口为 `8080`。