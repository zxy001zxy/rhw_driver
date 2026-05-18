cmake_minimum_required(VERSION 3.8)
project(rhw_msgs)

# 默认编译选项（可选）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 定义消息文件列表
set(msg_files
  "msg/RobotPosition.msg"
  "msg/RobotHardwreStatus.msg"
  "msg/Version.msg"
  "msg/UdpMotionStatus.msg"
  "msg/UdpBatteryStatus.msg"
  "msg/UdpBasicStatus.msg"
  "msg/SetCostmaps.msg"
  "msg/ModeState.msg"
  "msg/Ultrasonic.msg"
  "msg/SetRegion.msg"
  "msg/NavigationStatus.msg"
  "msg/PtzStatus.msg"
  "msg/WaypointTask.msg"
  "msg/MissionStatus.msg"
  "msg/InspectionAlbumReport.msg"
  # "msg/DynamicParams.msg"
  # "msg/Gps.msg"
)

# 定义服务文件列表
set(srv_files
  "srv/Pause.srv"
  "srv/Goal.srv"
  "srv/Cancel.srv"
  "srv/PathIsValid.srv"
  "srv/Recharge.srv"
  "srv/DisRecharge.srv"
  "srv/SaveMap.srv"
  "srv/SwitchMode.srv"
  "srv/GetMapList.srv"
  "srv/LoadMap.srv"
  "srv/DeleteMap.srv"
  "srv/InitialPose.srv"
  "srv/PtzControl.srv"
  "srv/PtzGotoPreset.srv"
  "srv/PtzPatrol.srv"
  "srv/PtzAbsoluteMove.srv"
  "srv/PtzGetPosition.srv"
  "srv/CaptureImage.srv"
  "srv/InspectionAlbumUpload.srv"
  "srv/ModelTaskRun.srv"
  "srv/AddWaypoint.srv"
  "srv/DeleteWaypoint.srv"
  "srv/GetWaypoints.srv"
  "srv/StartMission.srv"
  "srv/StopMission.srv"
  "srv/PauseMission.srv"
)

# 生成接口（消息和服务）
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    geometry_msgs
    std_msgs
    nav_msgs
)

# 导出依赖（供其他包使用）
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(nav_msgs)

# 打包
ament_package()
