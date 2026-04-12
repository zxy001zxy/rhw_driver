# 地图特殊区域配置
std_msgs/Header header                  # 标准消息头，包含时间戳和坐标系

geometry_msgs/PoseArray[] slope_area    # 斜坡区域多边形顶点集合

geometry_msgs/PoseArray[] slow_area     # 减速区域多边形顶点集合

geometry_msgs/PoseArray[] narrow_area   # 窄道区域多边形顶点集合

geometry_msgs/PoseArray[] forbidden_area # 禁行区域多边形顶点集合
