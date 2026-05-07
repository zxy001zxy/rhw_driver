# 话题 /robot_position
# 机器人在世界坐标系和栅格地图中的位置
std_msgs/Header header                  # 标准消息头，包含时间戳和坐标系

geometry_msgs/Pose2D world_position     # 世界坐标系下的二维位姿

geometry_msgs/Pose2D grid_position      # 栅格地图坐标系下的二维位姿