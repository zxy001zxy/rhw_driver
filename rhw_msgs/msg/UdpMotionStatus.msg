# UDP Command=4 结构化运动状态
std_msgs/Header header                  # 标准消息头，包含时间戳和坐标系

float32 roll                            # 横滚角（rad）
float32 pitch                           # 俯仰角（rad）
float32 yaw                             # 航向角（rad）
float32 omega_z                         # 角速度 Z（rad/s）
float32 line_x                          # 线速度 X（m/s）
float32 line_y                          # 线速度 Y（m/s）
float32 height                          # 底盘高度（m）
float32 remain_mile                     # 剩余里程（km）
