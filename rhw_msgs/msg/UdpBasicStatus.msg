# UDP Command=6 结构化基础状态
std_msgs/Header header                  # 标准消息头，包含时间戳和坐标系

int32 motion_state                      # 运动状态
int32 gait                              # 步态
int32 charge                            # 充电状态
int32 hes                               # HES 状态
int32 control_usage_mode                # 控制使用模式
int32 direction                         # 方向状态
int32 ooa                               # OOA 状态
int32 power_management                  # 电源管理状态
int32 sleep                             # 休眠状态
string version                          # 版本标识
