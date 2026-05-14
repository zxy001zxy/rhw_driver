# 巡检相册/抓拍结果上报事件

# 平台任务 ID；手动启动任务时由任务调度节点生成
string task_id

# 抓拍点位 ID
string point_id

# 抓拍点位名称
string point_name

# 本地图片文件路径
string image_path

# 云台/摄像机抓拍接口返回的图片 URL
string capture_url

# 图片文件大小，单位字节
uint32 file_size
