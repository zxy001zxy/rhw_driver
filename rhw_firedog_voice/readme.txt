# FireDog Voice Demo

当前目录已整理为标准 ROS 2 Python 功能包：`rhw_firedog_voice`

---

## 0. 依赖安装

先安装 ROS 2 Python 构建工具：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-pip
```

再安装语音识别依赖：

```bash
python3 -m pip install -U pip
python3 -m pip install vosk
```

说明：
- `vosk` 负责离线语音识别
- `rclpy`、`std_msgs` 由 ROS 2 Humble 提供

---

## 1. 项目简介

当前已完成的功能：
- 使用 ReSpeaker XVF3800 进行语音采集
- 使用 Vosk 进行离线语音识别
- 对固定命令集进行规则匹配
- 输出标准命令码
- 记录识别日志

当前支持 4 条基础命令：
- 前进
- 后退
- 左转
- 右转

对应标准命令码：
- 前进 -> `MOVE_FORWARD`
- 后退 -> `MOVE_BACKWARD`
- 左转 -> `TURN_LEFT`
- 右转 -> `TURN_RIGHT`

目前语音侧已经跑通，后续需要和机器人控制接口对接。

---

## 2. 项目结构

```bash
rhw_firedog_voice/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── rhw_firedog_voice
├── rhw_firedog_voice/
│   ├── __init__.py
│   └── asr_intent_demonew.py
├── model/
│   └── vosk-model-small-cn-0.22/
├── scripts/
│   └── asr_intent_demonew.py
├── test_audio/
│   ├── forward.wav
│   ├── backward.wav
│   ├── left.wav
│   └── right.wav
└── logs/
    └── asr_demo_log.jsonl

说明：
model/：Vosk 中文模型
`rhw_firedog_voice/asr_intent_demonew.py`：当前主要使用的包内入口
`scripts/asr_intent_demonew.py`：兼容旧用法的包装脚本
test_audio/：测试音频
logs/：历史识别日志（当前默认日志写入 `~/.ros/rhw_firedog_voice/`）

---

## 3. 构建与运行

在工作区根目录执行：

```bash
cd ~/Desktop/project/rhw_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rhw_firedog_voice
source install/setup.bash
```

使用默认测试音频运行：

```bash
ros2 run rhw_firedog_voice asr_intent_demonew
```

指定音频文件运行：

```bash
ros2 run rhw_firedog_voice asr_intent_demonew -- ~/Desktop/project/rhw_ws/src/rhw_firedog_voice/test_audio/forward.wav
```

兼容旧方式直接执行脚本：

```bash
python3 scripts/asr_intent_demonew.py ~/Desktop/project/rhw_ws/src/rhw_firedog_voice/test_audio/forward.wav
```

默认行为：
- 模型目录：包内 `model/vosk-model-small-cn-0.22`
- 默认测试音频：包内 `test_audio/forward.wav`
- 发布话题：`/voice_command`
- 默认日志：`~/.ros/rhw_firedog_voice/asr_demo_log.jsonl`

输出示例：
音频文件 : /home/test/firedog_voice/test_audio/forward.wav
声道数   : 2
采样率   : 16000
============================================================
[left] RMS      : 3457
[left] 识别文本 : 前进
[left] 命令码   : MOVE_FORWARD
------------------------------------------------------------
[right] RMS      : 2385
[right] 识别文本 : 前进
[right] 命令码   : MOVE_FORWARD
------------------------------------------------------------
[mix] RMS      : 2748
[mix] 识别文本 : 前进
[mix] 命令码   : MOVE_FORWARD
------------------------------------------------------------
[最终采用] 通道   : left
[最终采用] RMS    : 3457
[最终采用] 文本   : 前进
[最终采用] 命令   : MOVE_FORWARD
[EXEC] 执行: 前进
============================================================
日志已写入: /home/test/firedog_voice/logs/asr_demo_log.jsonl
============================================================