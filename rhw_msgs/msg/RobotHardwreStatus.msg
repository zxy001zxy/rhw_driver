# 机器人硬件状态汇总信息
std_msgs/Header header                  # 标准消息头，包含时间戳和坐标系

uint8 type                              # 底盘类型
uint8 lidar_num                         # 激光雷达数量
uint8 camera_num                        # 相机数量
uint8 ultrasonic_num                    # 超声波数量
uint8 gps_num                           # GPS 数量
bool emergency_stop_status              # 急停按钮状态，false: 未触发，true: 触发
bool odom_status                        # 里程计状态，false: 异常，true: 正常
bool gps_status                         # GPS 状态，false: 异常，true: 正常
bool lidar_status                       # 激光雷达状态，false: 异常，true: 正常
bool imu_status                         # IMU 状态，false: 异常，true: 正常
uint8 ultrasonic_status                 # 超声波整体状态，0: 异常，1: 正常
uint8 cameras_status                    # 摄像头状态，0: 前后都异常，1: 前正常后异常，2: 前异常后正常，3: 前后都正常

uint8 ultrasonic_get_obs                # 超声波避障触发位图，位值 1 表示触发，0 表示未触发
                                        # bit0 对应第 1 个超声波，bit7 对应第 8 个超声波

uint8 cameras_get_obs                   # 相机检测障碍物结果，0: 都未检测到，1: 前相机检测到，2: 后相机检测到，3: 前后都检测到

uint8 anticollision_status              # 防撞条状态，0: 未触发，1: 前触发，2: 后触发，3: 前后都触发

uint16 error_code                       # 故障代码，0 表示正常

uint8 battery_percentage                # 电量百分比，范围 0 到 100

uint16 io_status                        # IO 状态位，0 表示无动作

uint8 charge_status                     # 充电状态，0: 未充电，1: 充电中
