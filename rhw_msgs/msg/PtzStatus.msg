# 话题: /ptz/status
# 云台状态，定时发布

std_msgs/Header header

# 设备是否在线
bool online

# 通道号
uint8 channel

# 当前方位角
float32 azimuth

# 当前俯仰角
float32 elevation

# 当前动作描述（idle / patrol / moving 等）
string active_action

# 附加说明
string message
