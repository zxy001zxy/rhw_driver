# UDP Command=5 结构化电池状态
std_msgs/Header header                  # 标准消息头，包含时间戳和坐标系

float32 voltage_left                    # 左侧电压
float32 voltage_right                   # 右侧电压
float32 battery_level_left              # 左侧电量百分比
float32 battery_level_right             # 右侧电量百分比
float32 battery_temperature_left        # 左侧电池温度
float32 battery_temperature_right       # 右侧电池温度
bool charge_left                        # 左侧是否充电
bool charge_right                       # 右侧是否充电
