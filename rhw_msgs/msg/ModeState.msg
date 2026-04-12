# 话题: /mode_state
# 当前系统运行模式与 launch 切换状态

# 消息时间戳
std_msgs/Header header

# 模式枚举
uint8 MODE_MAPPING=0
uint8 MODE_NAVIGATION=1

# 当前模式
uint8 mode

# 是否正在执行模式切换
bool switching

# 当前 launch 是否健康，通常表示进程仍在运行
bool healthy

# 当前激活的 launch 标识，例如 mapping 或 navigation
string active_launch

# 额外状态说明
string message