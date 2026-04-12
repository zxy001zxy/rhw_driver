# 话题: /navigation_status
# 导航任务实时状态，APP 用于展示导航进度和结果

# 消息时间戳
std_msgs/Header header

# 导航状态枚举
uint8 STATUS_IDLE=0             # 空闲，无导航任务
uint8 STATUS_NAVIGATING=1       # 导航中
uint8 STATUS_PAUSED=2           # 已暂停
uint8 STATUS_REACHED=3          # 已到达目标点
uint8 STATUS_CANCELLED=4        # 已取消
uint8 STATUS_FAILED=5           # 导航失败（不可达等）
uint8 STATUS_WAITING=6          # 等待障碍物清除

# 当前导航状态
uint8 status

# 当前目标点位姿
geometry_msgs/PoseStamped current_goal

# 剩余路径长度（米），-1 表示未知
float32 remaining_distance

# 预计剩余时间（秒），-1 表示未知
float32 estimated_time

# 状态说明
string message
