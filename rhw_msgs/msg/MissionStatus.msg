# 任务调度状态
# 实时反馈当前巡检任务的执行进度

# 消息时间戳
std_msgs/Header header

# 任务状态枚举
uint8 IDLE=0                    # 空闲，无任务
uint8 RUNNING=1                 # 任务执行中
uint8 PAUSED=2                  # 任务已暂停
uint8 COMPLETED=3               # 所有航点执行完毕
uint8 FAILED=4                  # 任务执行失败

# 当前任务状态
uint8 status

# 当前正在执行的航点 ID（空闲时为空）
string current_waypoint_id

# 航点总数
uint32 total_waypoints

# 已完成的航点数
uint32 completed_waypoints

# 状态附加说明
string message
