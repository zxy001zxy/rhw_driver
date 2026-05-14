# 航点任务定义
# 包含点位坐标、类型和任务执行参数

# 航点类型枚举
uint8 TYPE_NORMAL=0             # 普通导航点（到达即完成）
uint8 TYPE_CHARGE=1             # 充电点位（到达后执行充电）
uint8 TYPE_VISION=2             # 视觉识别点（到达后执行 PTZ+抓拍+推理）

# 航点唯一标识（UUID 或自定义编号）
string waypoint_id

# 所属地图名称
string map_name

# 航点在地图坐标系中的 2D 位姿（x, y, theta）
geometry_msgs/Pose2D pose

# 航点类型，取值见上方枚举
uint8 waypoint_type

# 人可读的航点名称/标签
string label

# 任务参数，JSON 字符串
# 示例（TYPE_VISION）:
#   {"azimuth": 180.0, "elevation": 0.0, "channel": 1,
#    "zoom": 6.5, "azimuth_speed": 50, "elevation_speed": 50,
#    "inference_type": "fire_equipment_detection"}
# 示例（TYPE_CHARGE）:
#   {"timeout_sec": 300}
# 普通导航点可留空
string task_params
