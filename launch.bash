#!/bin/bash

set -e

# 定义公共环境初始化命令
ROS_ENV="source /opt/ros/humble/setup.bash; source ~/rhw_ws/install/setup.bash"

# CPU 亲和性配置：本机逻辑 CPU 为 0-15。
# 建议把视频/云台相关进程放到后半部分核心，给雷达/导航预留前半部分核心。
PTZ_CPUS="${PTZ_CPUS:-8-15}"
ROSBRIDGE_CPUS="${ROSBRIDGE_CPUS:-8-15}"
WEB_VIDEO_CPUS="${WEB_VIDEO_CPUS:-8-15}"

run_in_terminal() {
  local title="$1"
  local cpus="$2"
  local command="$3"

  gnome-terminal --title="$title" -- bash -c \
    "$ROS_ENV; echo '[CPU affinity] $title -> $cpus'; taskset -c $cpus $command; exec bash"
}

# 分别在新终端中执行，并用 taskset 绑定 CPU 核心。
# ros2 launch 拉起的子进程会继承该 CPU 亲和性。
run_in_terminal "rhw_ptz_controller" "$PTZ_CPUS" \
  "ros2 launch rhw_ptz_controller ptz_controller.launch.py"
sleep 2
run_in_terminal "rosbridge_server" "$ROSBRIDGE_CPUS" \
  "ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9080"
sleep 2
run_in_terminal "web_video_server" "$WEB_VIDEO_CPUS" \
  "ros2 run web_video_server web_video_server"
