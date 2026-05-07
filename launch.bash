#!/bin/bash

# 定义公共环境初始化命令
ROS_ENV="source /opt/ros/humble/setup.bash; source ~/rhw_ws/install/setup.bash"

# 分别在新终端中执行 roslaunch
gnome-terminal -- bash -c "$ROS_ENV; ros2 launch rhw_ptz_controller ptz_controller.launch.py; exec bash"
sleep 2
gnome-terminal -- bash -c "$ROS_ENV; ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9080; exec bash"
sleep 2
gnome-terminal -- bash -c "$ROS_ENV; ros2 run web_video_server web_video_server; exec bash"
