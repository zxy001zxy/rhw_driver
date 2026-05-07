# MQTT 消息格式确认文档

## 概述
本文档定义了机器人平台与机器狗设备之间的 MQTT 通信协议，请厂商确认消息格式是否正确。

---

## 一、点位信息同步（机器狗 → 机器人平台）

### 1.1 功能说明
机器狗上报地图标记点信息，机器人平台接收后存储到数据库。

### 1.2 Topic
```
/{ProductID}/{DogID}/Upload/Data
```

**示例**：`/robot-dog/DOG001/Upload/Data`

### 1.3 同步频率/时机

**需要厂商补充**

### 1.4消息体

```json
{
  "type": "response",
  "method": "map",
  "code": 0,
  "msgid": 1,
  "message": [
    {
      "mapId": 1,
      "pointCount": 10,
      "pointId": "P_A01",
      "pointName": "大门口"
    },
    {
      "mapId": 2,
      "pointCount": 20,
      "pointId": "P_A02",
      "pointName": "保安亭"
    }
  ]
}
```

### 1.5 字段说明

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| type | String | 是 | 消息类型，固定值 "response" |
| method | String | 是 | 方法名，固定值 "map" |
| code | Integer | 是 | 响应码，0 表示成功 |
| msgid | Integer | 是 | 消息 ID |
| message | Array | 是 | 点位信息数组 |
| message[].mapId | Integer | 是 | 点云图 ID |
| message[].pointCount | Integer | 是 | 关联的标注点数量 |
| message[].pointId | String | 是 | 标注点 ID，唯一标识 |
| message[].pointName | String | 是 | 标注点名称 |

### 1.6 处理逻辑
- 机器人平台根据 `pointId` 判断点位是否已存在
- 如果已存在则跳过，不存在则插入数据库
- 自动关联设备所属社区

---

## 二、巡检任务下发（机器人平台 → 机器狗）

### 2.1 功能说明
机器人平台下发巡检任务到机器狗，机器狗接收后执行巡检。

### 2.2 Topic
```
/{ProductID}/{DogID}/Download/Data
```

**示例**：`/robot-dog/DOG001/Download/Data`

### 2.3 消息体
```json
{
  "type": "download",
  "method": "task",
  "msgid": 1,
  "message": {
    "cmdType": "create",
    "taskId": "123456",
    "taskType": 1,
    "taskLevel": 1,
    "taskPeriod": 1,
    "inspectCount": 1,
    "taskStartTime": 1713081600000,
    "pointIdList": ["P_A01", "P_A02", "P_B03", "P_C01"]
  }
}
```

### 2.4 字段说明

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| type | String | 是 | 消息类型，固定值 "download" |
| method | String | 是 | 方法名，固定值 "task" |
| msgid | Integer | 是 | 消息 ID |
| message | Object | 是 | 任务信息对象 |
| message.cmdType | String | 是 | 命令类型，"create" 表示创建新任务 |
| message.taskId | String | 是 | 任务 ID，唯一标识 |
| message.taskType | Integer | 是 | 任务类型，枚举值（参见附表 A.5） |
| message.taskLevel | Integer | 是 | 任务级别，枚举值（参见附表 A.8） |
| message.taskPeriod | Integer | 是 | 任务周期：1=单次执行，2=每天重复执行 |
| message.inspectCount | Integer | 是 | 巡检次数 |
| message.taskStartTime | Long | 是 | 任务开始时间，Unix 时间戳（毫秒）（这个是时间戳还是时间要确认） |
| message.pointIdList | Array | 是 | 巡检点位 ID 列表，按顺序执行 |

### 2.5 处理逻辑
- 机器狗接收任务后，通过消息 3 返回响应
- 如果接受任务，返回 code=0
- 如果拒绝任务，返回 code!=0 并说明原因

---

## 三、任务状态更新（机器狗 → 机器人平台）

### 3.1 功能说明
机器狗上报任务执行状态，包括任务响应、进度更新、完成通知等。

### 3.2 Topic
```
/{ProductID}/{DogID}/Upload/Data
```

**示例**：`/robot-dog/DOG001/Upload/Data`

### 3.3  同步频率/时机

**需要厂商补充**

### 3.4 消息体

#### 3.4.1 接受任务
```json
{
  "type": "response",
  "method": "task",
  "code": 0,
  "msgid": 1,
  "message": {
    "taskId": "123456",
    "taskProgress": 0,
    "taskDuration": 0,
    "taskStatus": 2,
    "errorMsg": ""
  }
}
```

#### 3.4.2 拒绝任务
```json
{
  "type": "response",
  "method": "task",
  "code": 1,
  "msgid": 1,
  "message": {
    "taskId": "123456",
    "taskProgress": 0,
    "taskDuration": 0,
    "taskStatus": 6,
    "errorMsg": "当前正在执行高优先级任务"
  }
}
```

#### 3.4.3 进度更新
```json
{
  "type": "response",
  "method": "task",
  "code": 0,
  "msgid": 2,
  "message": {
    "taskId": "123456",
    "taskProgress": 78,
    "taskDuration": 13666,
    "taskStatus": 3,
    "errorMsg": ""
  }
}
```

### 3.4 字段说明

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| type | String | 是 | 消息类型，固定值 "response" |
| method | String | 是 | 方法名，固定值 "task" |
| code | Integer | 是 | 响应码：0=接受任务，非 0=拒绝任务 |
| msgid | Integer | 是 | 消息 ID |
| message | Object | 是 | 任务状态信息 |
| message.taskId | String | 是 | 任务 ID |
| message.taskProgress | Integer | 是 | 任务进度，0-100 |
| message.taskDuration | Long | 是 | 任务执行时长，单位：毫秒 |
| message.taskStatus | Integer | 是 | 任务状态（见下表） |
| message.errorMsg | String | 否 | 错误信息，拒绝任务或异常时填写 |

### 3.5 任务状态枚举

| 状态值 | 说明 |
|--------|------|
| 1 | 待下发 |
| 2 | 已下发（已接受） |
| 3 | 进行中 |
| 4 | 已完成 |
| 5 | 已取消 |
| 6 | 异常 |

### 3.6 处理逻辑
- code=0：机器人平台更新任务状态为taskStatus
- code!=0：机器人平台更新任务状态为"异常"，记录错误原因
- 任务执行过程中可多次上报进度

---

## 四、设备心跳（机器狗 → 机器人平台）

### 4.1 功能说明
机器狗定期上报设备状态，包括电量、信号强度、在线状态等。

### 4.2 Topic
```
/{ProductID}/{DogID}/Upload/Data
```

**示例**：`/robot-dog/DOG001/Upload/Data`

### 4.6 心跳频率

**请厂商确认**：心跳消息的发送频率是多少？

### 4.3 消息体

```json
{
  "type": "upload",
  "method": "heart",
  "msgid": 1,
  "message": {
    "runMode": 2,
    "workStatus": 0,
    "battery": 85,
    "healthStatus": 0,
    "motionStatus": 0,
    "chargeStatus": 0,
    "signalStrength": -65,
    "onlineStatus": 0,
    "location":{
    "mapId":1,
    "worldPose":{
      "orientation":{
        "w":0.9991511204590953,
        "x":0,
        "y":0,
        "z":-0.041195126961019124
      },
      "position":{
        "x":-1.873163616425341,
        "y":-12.877138903739242,
        "z":0
      }
    }
  }
}
}
```

### 4.4 字段说明

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| type | String | 是 | 消息类型，固定值 "upload" |
| method | String | 是 | 方法名，固定值 "heart" |
| msgid | Integer | 是 | 消息 ID |
| message | Object | 是 | 设备状态信息 |
| message.runMode | Integer | 是 | 运行模式（枚举值待确认） |
| message.workStatus | Integer | 是 | 工作状态：0=空闲，1=工作中 |
| message.battery | Integer | 是 | 电量百分比，0-100 |
| message.batteryStatus | Integer | 是 | 电池状态（枚举值待确认） |
| message.healthStatus | Integer | 是 | 健康状态：0=正常，非 0=异常（参见附表 1） |
| message.motionStatus | Integer | 是 | 运动状态（枚举值待确认） |
| message.chargeStatus | Integer | 是 | 充电状态：0=未充电，1=充电中 |
| message.signalStrength | Integer | 是 | WiFi 信号强度，单位：dBm（如 -65） |
| message.onlineStatus | Integer | 是 | 在线状态：0=在线，1=离线 |

### 4.5 处理逻辑
- 机器人平台更新设备表中的电量、信号强度
- 根据 `onlineStatus`、`healthStatus`、`workStatus` 综合判断设备状态：
  - onlineStatus=1 → 设备状态=离线
  - healthStatus!=0 → 设备状态=告警
  - workStatus=0 → 设备状态=空闲
  - 其他 → 设备状态=在线

---

## 五、MQTT 连接参数

### 5.1 连接信息（待厂商提供）

| 参数 | 说明 | 示例值 |
|------|------|--------|
| Broker URL | MQTT 服务器地址 | tcp://mqtt.example.com:1883 |
| Client ID | 客户端标识 | robot-inspect-server |
| Username | 用户名 | admin |
| Password | 密码 | ******** |
| Product ID | 产品 ID | robot-dog |
| QoS | 服务质量等级 | 1 |
| Keep Alive | 心跳间隔（秒） | 60 |

### 5.2 订阅 Topic
机器人平台启动时自动订阅：
```
/{ProductID}/+/Upload/Data
```
使用通配符 `+` 匹配所有设备 ID，接收所有机器狗的上报消息。

---

## 六、需要厂商确认的问题

### 6.1 消息格式
- [ ] 以上 4 种消息的 JSON 格式是否正确？
- [ ] 字段名称、类型、必填项是否准确？
- [ ] 是否有遗漏的字段？

### 6.2 枚举值定义
- [ ] `healthStatus`（健康状态）的枚举值定义？

### 6.3 连接参数
- [ ] MQTT Broker 地址和端口？
- [ ] 连接用户名和密码？
- [ ] Product ID 的具体值？
- [ ] 是否需要 SSL/TLS 加密连接？

### 6.4 业务逻辑
- [ ] 心跳消息的发送频率？
- [ ] 任务进度更新的频率？
- [ ] 点位同步在什么时机触发？
- [ ] 任务下发后，机器狗多久响应？机器狗是否在接收任务后会上报一次任务状态，待进行之类的？

---

## 七、测试场景

### 7.1 正常流程
1. 机器狗上线，发送心跳消息
2. 机器狗上报点位信息
3. 机器人平台下发巡检任务
4. 机器狗接受任务，返回响应（code=0）
5. 机器狗执行任务，定期上报进度
6. 任务完成，上报最终状态（taskStatus=4）

### 7.2 异常流程
1. 机器狗拒绝任务（电量不足、正在充电等）
2. 任务执行中出现异常（障碍物、网络中断等）
3. 机器人平台离线时的消息缓存

---

## 八、附录

### 附录 A：完整测试消息示例

#### A.1 点位同步完整示例
```json
{
  "type": "response",
  "method": "map",
  "code": 0,
  "msgid": 1001,
  "message": [
    {"mapId": 1, "pointCount": 10, "pointId": "P_A01", "pointName": "大门口"},
    {"mapId": 1, "pointCount": 10, "pointId": "P_A02", "pointName": "保安亭"},
    {"mapId": 1, "pointCount": 10, "pointId": "P_B01", "pointName": "1号楼入口"},
    {"mapId": 1, "pointCount": 10, "pointId": "P_B02", "pointName": "2号楼入口"},
    {"mapId": 1, "pointCount": 10, "pointId": "P_C01", "pointName": "停车场"}
  ]
}
```

#### A.2 任务下发完整示例
```json
{
  "type": "download",
  "method": "task",
  "msgid": 2001,
  "message": {
    "cmdType": "create",
    "taskId": "XJ-20260415-00001",
    "taskType": 1,
    "taskLevel": 1,
    "taskPeriod": 2,
    "inspectCount": 3,
    "taskStartTime": 1713081600000,
    "pointIdList": ["P_A01", "P_A02", "P_B01", "P_B02", "P_C01"]
  }
}
```

#### A.3 任务完整生命周期
```json
// 1. 接受任务
{"type": "response", "method": "task", "code": 0, "msgid": 3001,
 "message": {"taskId": "XJ-20260415-00001", "taskProgress": 0, "taskDuration": 0, "taskStatus": 2, "errorMsg": ""}}

// 2. 进度 20%
{"type": "response", "method": "task", "code": 0, "msgid": 3002,
 "message": {"taskId": "XJ-20260415-00001", "taskProgress": 20, "taskDuration": 3000, "taskStatus": 3, "errorMsg": ""}}

// 3. 进度 50%
{"type": "response", "method": "task", "code": 0, "msgid": 3003,
 "message": {"taskId": "XJ-20260415-00001", "taskProgress": 50, "taskDuration": 7500, "taskStatus": 3, "errorMsg": ""}}

// 4. 进度 80%
{"type": "response", "method": "task", "code": 0, "msgid": 3004,
 "message": {"taskId": "XJ-20260415-00001", "taskProgress": 80, "taskDuration": 12000, "taskStatus": 3, "errorMsg": ""}}

// 5. 任务完成
{"type": "response", "method": "task", "code": 0, "msgid": 3005,
 "message": {"taskId": "XJ-20260415-00001", "taskProgress": 100, "taskDuration": 15000, "taskStatus": 4, "errorMsg": ""}}
```

#### A.4 心跳消息序列
```json
// 正常状态
{"type": "upload", "method": "heart", "msgid": 4001,
 "message": {"runMode": 2, "workStatus": 0, "battery": 100, "batteryStatus": 0, "healthStatus": 0, "motionStatus": 0, "chargeStatus": 0, "signalStrength": -55, "onlineStatus": 0}}

// 电量降低
{"type": "upload", "method": "heart", "msgid": 4002,
 "message": {"runMode": 2, "workStatus": 1, "battery": 75, "batteryStatus": 0, "healthStatus": 0, "motionStatus": 1, "chargeStatus": 0, "signalStrength": -60, "onlineStatus": 0}}

// 低电量告警
{"type": "upload", "method": "heart", "msgid": 4003,
 "message": {"runMode": 2, "workStatus": 0, "battery": 15, "batteryStatus": 1, "healthStatus": 0, "motionStatus": 0, "chargeStatus": 0, "signalStrength": -65, "onlineStatus": 0}}

// 充电中
{"type": "upload", "method": "heart", "msgid": 4004,
 "message": {"runMode": 1, "workStatus": 0, "battery": 20, "batteryStatus": 0, "healthStatus": 0, "motionStatus": 0, "chargeStatus": 1, "signalStrength": -50, "onlineStatus": 0}}
```

