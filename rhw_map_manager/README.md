# rhw_map_manager

地图/模式控制节点。

## 功能

- 提供 `/mode_manager/switch_mode`
- 提供 `/mode_manager/save_map`
- 提供 `/map_manager/get_map_list`
- 提供 `/map_manager/load_map`
- 发布 `/mode_state`

## 当前控制逻辑

- 建图模式：执行 `ros2 run lightning run_slam_online --config ./config/default.yaml`
- 导航模式：同时执行以下命令
    - `ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False map:="<选中的地图yaml>"`
    - `ros2 run rviz2 rviz2 -d nav_ws/nav2_default_view.rviz`
    - `ros2 run tf2_ros static_transform_publisher 0.38 0 0.31 0 0 0 base_link rslidar`
    - `ros2 run lightning run_loc_online --config ./config/default.yaml`
- 保存地图：执行 `ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: <map_name>}"`
- 地图目录：默认使用 `lightning-lm/data/<map_name>/map.yaml`

## 启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rhw_map_manager map_manager.launch.py
```

## 参数

参数在 `config/map_manager.yaml` 中：

- `mapping_workspace`
- `mapping_command`
- `navigation_workspace`
- `navigation_command_template`
- `navigation_extra_commands`
- `save_map_workspace`
- `save_map_command_template`
- `map_dir`
- `map_yaml_name`


## APP触发事件对应ROS服务

1.  APP 点击开始建图
    调 `SwitchMode(target_mode=0)` 启动建图
    ros2 service call /mode_manager/switch_mode rhw_msgs/srv/SwitchMode "{target_mode: 0, launch_profile: '', force_restart: false, launch_args: []}"

2.  APP 通过 `/cmd_vel_app` 控制小车移动

3.  APP 点击保存地图设置地图名
    调 `SaveMap(map_name=xxx)`，节点内部转调 `lightning` 的 `/lightning/save_map`
    ros2 service call /mode_manager/save_map rhw_msgs/srv/SaveMap "{map_name: new_map}"

4.  算法将地图保存到 `lightning-lm/data/xxx/map.yaml`

5.  APP 获取所有地图列表
    调 `GetMapList` 刷新列表
    ros2 service call /map_manager/get_map_list rhw_msgs/srv/GetMapList "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}}"

6.  APP 选择当前机器人所在环境地图
    调 `LoadMap(map_name=xxx)` 选中地图
    ros2 service call /map_manager/load_map rhw_msgs/srv/LoadMap "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, map_name: new_map}"

7.  APP 调 `SwitchMode(target_mode=1)` 进入导航
    节点会依次拉起 Nav2、RViz、静态 TF 和 `lightning` 定位
    ros2 service call /mode_manager/switch_mode rhw_msgs/srv/SwitchMode "{target_mode: 1, launch_profile: '', force_restart: false, launch_args: []}"




