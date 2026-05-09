# 机器人平台对接接口确认文档

## 概述
本文档定义机器人平台与机器狗设备之间的 MQTT 与 HTTPS 对接接口，用于和厂商确认消息格式、字段含义、触发时机及连接参数。

当前文档包含两类接口：

- MQTT 接口：点位信息同步、巡检任务下发、任务状态更新、设备心跳
- HTTPS 接口：定点拍照上报、告警信息上报

---

## 一、点位信息同步（机器狗 → 机器人平台）

### 1.1 功能说明
平台接收机器狗上报的地图标记点信息并存储到数据库。

当前阶段采用主动同步模式：

- 机器狗在点位保存或删除后，主动上报当前地图点位快照
- 机器狗连接 MQTT broker 后，可补发当前已保存点位快照

当前暂不采用平台触发响应式同步。

### 1.2 Topic

```text
/{ProductID}/{DogID}/Upload/Data
```

示例：`/robot-dog/DOG001/Upload/Data`

### 1.3 同步频率/时机

需要厂商确认：

- 主动上报的触发时机
- 是否每次返回全量点位，还是仅返回增量点位

### 1.4 消息体

```json
{
  "type": "response",
  "method": "map",
  "code": 0,
  "msgid": 1,
  "message": [
    {
      "mapId": "9acb90d53c0d52b89d7f8a6ee4a19b85",
      "mapName": "factory_map",
      "pointCount": 2,
      "pointId": ["P_A01", "P_A02"],
      "pointName": ["大门口", "视觉点"]
    },
    {
      "mapId": "6425dbf2b5665a62b5d8b3c7d6d8f0eb",
      "mapName": "room_map",
      "pointCount": 3,
      "pointId": ["P_B01", "P_B02", "P_B03"],
      "pointName": ["大门口", "视觉点", "视觉点2"]
    }
  ]
}
```

### 1.5 字段说明

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| type | String | 是 | 消息类型，固定值 `response` |
| method | String | 是 | 方法名，固定值 `map` |
| code | Integer | 是 | 响应码，`0` 表示成功 |
| msgid | Integer | 是 | 消息 ID |
| message | Array | 是 | 地图点位快照数组，每个元素对应一张地图 |
| message[].mapId | String | 是 | 当前地图的稳定唯一标识，持久化保存在地图 JSON 顶层 `map_id` 字段 |
| message[].mapName | String | 是 | 当前地图名称 |
| message[].pointCount | Integer | 是 | 当前地图点位总数 |
| message[].pointId | Array[String] | 是 | 当前地图下的点位 ID 列表，顺序与 `pointName` 一一对应 |
| message[].pointName | Array[String] | 是 | 当前地图下的点位名称列表，顺序与 `pointId` 一一对应 |

### 1.6 处理逻辑

- 机器人平台根据 `pointId` 判断点位是否已存在
- 如果已存在则跳过，不存在则插入数据库
- 自动关联设备所属社区

---

## 二、巡检任务下发（机器人平台 → 机器狗）

### 2.1 功能说明
机器人平台下发巡检任务到机器狗，机器狗接收后执行巡检。

### 2.2 Topic

```text
/{ProductID}/{DogID}/Download/Data
```

示例：`/robot-dog/DOG001/Download/Data`

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
| type | String | 是 | 消息类型，固定值 `download` |
| method | String | 是 | 方法名，固定值 `task` |
| msgid | Integer | 是 | 消息 ID |
| message | Object | 是 | 任务信息对象 |
| message.cmdType | String | 是 | 命令类型，`create` 表示创建新任务 |
| message.taskId | String | 是 | 任务 ID，唯一标识 |
| message.taskType | Integer | 是 | 任务类型，枚举值待厂商确认 |
| message.taskLevel | Integer | 是 | 任务级别，枚举值待厂商确认 |
| message.taskPeriod | Integer | 是 | 任务周期：`1=单次执行`，`2=每天重复执行` |
| message.inspectCount | Integer | 是 | 巡检次数 |
| message.taskStartTime | Long | 是 | 任务开始时间，Unix 时间戳，单位毫秒 |
| message.pointIdList | Array | 是 | 巡检点位 ID 列表，按顺序执行 |

### 2.5 处理逻辑

- 机器狗接收任务后，通过第三类消息返回响应
- 如果接受任务，返回 `code=0`
- 如果拒绝任务，返回 `code!=0` 并说明原因

---

## 三、任务状态更新（机器狗 → 机器人平台）

### 3.1 功能说明
机器狗上报任务执行状态，包括任务响应、进度更新、完成通知等。

### 3.2 Topic

```text
/{ProductID}/{DogID}/Upload/Data
```

示例：`/robot-dog/DOG001/Upload/Data`

### 3.3 同步频率/时机

需要厂商补充。

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

### 3.5 字段说明

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| type | String | 是 | 消息类型，固定值 `response` |
| method | String | 是 | 方法名，固定值 `task` |
| code | Integer | 是 | 响应码：`0=接受任务`，非 `0=拒绝任务` |
| msgid | Integer | 是 | 消息 ID |
| message | Object | 是 | 任务状态信息 |
| message.taskId | String | 是 | 任务 ID |
| message.taskProgress | Integer | 是 | 任务进度，`0-100` |
| message.taskDuration | Long | 是 | 任务执行时长，单位毫秒 |
| message.taskStatus | Integer | 是 | 任务状态 |
| message.errorMsg | String | 否 | 错误信息，拒绝任务或异常时填写 |

### 3.6 任务状态枚举

| 状态值 | 说明 |
|--------|------|
| 1 | 待下发 |
| 2 | 已下发（已接受） |
| 3 | 进行中 |
| 4 | 已完成 |
| 5 | 已取消 |
| 6 | 异常 |

### 3.7 处理逻辑

- `code=0`：机器人平台更新任务状态为 `taskStatus`
- `code!=0`：机器人平台更新任务状态为异常，并记录错误原因
- 任务执行过程中可多次上报进度

---

## 四、设备心跳（机器狗 → 机器人平台）

### 4.1 功能说明
机器狗定期上报设备状态，包括电量、信号强度、在线状态等。

### 4.2 Topic

```text
/{ProductID}/{DogID}/Upload/Data
```

示例：`/robot-dog/DOG001/Upload/Data`

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
    "location": {
      "mapId": 1,
      "worldPose": {
        "orientation": {
          "w": 0.9991511204590953,
          "x": 0,
          "y": 0,
          "z": -0.041195126961019124
        },
        "position": {
          "x": -1.873163616425341,
          "y": -12.877138903739242,
          "z": 0
        }
      }
    }
  }
}
```

### 4.4 字段说明

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| type | String | 是 | 消息类型，固定值 `upload` |
| method | String | 是 | 方法名，固定值 `heart` |
| msgid | Integer | 是 | 消息 ID |
| message | Object | 是 | 设备状态信息 |
| message.runMode | Integer | 是 | 运行模式，枚举值待确认 |
| message.workStatus | Integer | 是 | 工作状态：`0=空闲`，`1=工作中` |
| message.battery | Integer | 是 | 电量百分比，`0-100` |
| message.batteryStatus | Integer | 否 | 电池状态，枚举值待确认 |
| message.healthStatus | Integer | 是 | 健康状态：`0=正常`，非 `0=异常` |
| message.motionStatus | Integer | 是 | 运动状态，枚举值待确认 |
| message.chargeStatus | Integer | 是 | 充电状态：`0=未充电`，`1=充电中` |
| message.signalStrength | Integer | 是 | WiFi 信号强度，单位 dBm |
| message.onlineStatus | Integer | 是 | 在线状态：`0=在线`，`1=离线` |
| message.location.mapId | Integer | 否 | 地图 ID |
| message.location.worldPose | Object | 否 | 世界坐标系位姿 |

### 4.5 处理逻辑

- 机器人平台更新设备表中的电量、信号强度
- 根据 `onlineStatus`、`healthStatus`、`workStatus` 综合判断设备状态
- 判定规则如下：
  - `onlineStatus=1`：设备状态为离线
  - `healthStatus!=0`：设备状态为告警
  - `workStatus=0`：设备状态为空闲
  - 其他：设备状态为在线

### 4.6 心跳频率

请厂商确认：心跳消息的发送频率是多少。

---

## 五、MQTT 连接参数

### 5.1 连接信息（待厂商提供）

| 参数 | 说明 | 示例值 |
|------|------|--------|
| Broker URL | MQTT 服务器地址 | `tcp://mqtt.example.com:1883` |
| Client ID | 客户端标识 | `robot-inspect-server` |
| Username | 用户名 | `admin` |
| Password | 密码 | `********` |
| Product ID | 产品 ID | `robot-dog` |
| QoS | 服务质量等级 | `1` |
| Keep Alive | 心跳间隔（秒） | `60` |

### 5.2 订阅 Topic

机器人平台启动时自动订阅：

```text
/{ProductID}/+/Upload/Data
```

使用通配符 `+` 匹配所有设备 ID，接收所有机器狗的上报消息。

---

## 六、巡检结果 HTTPS 上报接口

### 6.1 定点拍照上报接口

#### 6.1.1 功能说明
在机器狗巡检过程中，定点触发摄像头拍照，并将拍照结果上报到平台。

#### 6.1.2 协议说明

- 提供方：平台
- 接入方：机器狗
- 协议：HTTPS

#### 6.1.3 接口地址

```text
https://ip:port/robot-inspect/inspect/album/report
```

#### 6.1.4 请求方式

`POST`

#### 6.1.5 请求头

```text
Content-Type: application/json
```

#### 6.1.6 外层请求参数

| 名称 | 必填 | 类型 | 说明 |
|------|------|------|------|
| traceId | 是 | String | 请求流水号，格式建议：`System.nanoTime() + 五位随机数` |
| partnerId | 是 | String | 合作方 ID，由机器人平台分配 |
| version | 是 | String | 版本号，由机器人平台分配 |
| data | 是 | String | AES 加密后的业务报文字符串 |
| signature | 是 | String | 签名，取值为 `MD5(traceId + data + signatureSecret)` |

#### 6.1.7 data 解密后字段

| 名称 | 必填 | 类型 | 说明 |
|------|------|------|------|
| deviceId | 是 | String | 设备唯一识别号 |
| base64 | 是 | String | 照片 Base64 字符串 |
| taskId | 是 | String | 任务 ID |
| pointName | 是 | String | 拍照点位名称 |
| pointId | 是 | String | 拍照点位 ID |

#### 6.1.8 请求示例

外层请求示例：

```json
{
  "traceId": "176242122021989062",
  "partnerId": "1000007",
  "version": "1.0",
  "data": "<AES_ENCRYPTED_PAYLOAD>",
  "signature": "57af9cf3feb29d08540e75d568bbfe29"
}
```

`data` 明文字段示例：

```json
{
  "deviceId": "DOG001",
  "base64": "<IMAGE_BASE64>",
  "taskId": "XJ-20260415-00001",
  "pointName": "大门口",
  "pointId": "P_A01"
}
```

#### 6.1.9 响应参数

| 名称 | 类型 | 说明 |
|------|------|------|
| code | Integer | 状态码，`0` 为成功，其他为失败 |
| traceId | String | 请求参数中的流水号 |
| msg | String | 错误信息或成功信息 |
| timestamp | Long | 响应时的毫秒级时间戳 |

#### 6.1.10 响应示例

```json
{
  "msg": "操作成功",
  "traceId": "174737636857728828",
  "code": 0,
  "timestamp": 1747376371135
}
```

### 6.2 上报告警信息接口

#### 6.2.1 功能说明
在机器狗巡检过程中识别到隐患信息后，上报告警信息到平台。

#### 6.2.2 协议说明

- 提供方：平台
- 接入方：机器狗
- 协议：HTTPS

#### 6.2.3 接口地址

```text
https://ip:port/robot-inspect/inspect/alarm/report
```

#### 6.2.4 请求方式

`POST`

#### 6.2.5 请求头

```text
Content-Type: application/json
```

#### 6.2.6 外层请求参数

| 名称 | 必填 | 类型 | 说明 |
|------|------|------|------|
| traceId | 是 | String | 请求流水号，格式建议：`System.nanoTime() + 五位随机数` |
| partnerId | 是 | String | 合作方 ID，由机器人平台分配 |
| version | 是 | String | 版本号，由机器人平台分配 |
| data | 是 | String | AES 加密后的业务报文字符串 |
| signature | 是 | String | 签名，取值为 `MD5(traceId + data + signatureSecret)` |

#### 6.2.7 data 解密后字段

| 名称 | 必填 | 类型 | 说明 |
|------|------|------|------|
| deviceId | 是 | String | 设备唯一识别号 |
| base64 | 否 | String | 隐患照片 Base64 字符串 |
| taskId | 是 | String | 任务 ID |
| pointId | 是 | String | 告警点位 ID |
| pointName | 是 | String | 告警点位名称 |
| alarmType | 是 | String | 告警类型，需机器狗提供枚举定义 |
| alarmContent | 是 | String | 告警内容 |
| alarmTime | 是 | String | 告警时间，建议使用统一时间字符串格式，最终格式待双方确认 |

#### 6.2.8 请求示例

外层请求示例：

```json
{
  "traceId": "176242122021989062",
  "partnerId": "1000007",
  "version": "1.0",
  "data": "<AES_ENCRYPTED_PAYLOAD>",
  "signature": "57af9cf3feb29d08540e75d568bbfe29"
}
```

`data` 明文字段示例：

```json
{
  "deviceId": "DOG001",
  "base64": "<IMAGE_BASE64>",
  "taskId": "XJ-20260415-00001",
  "pointId": "P_A01",
  "pointName": "大门口",
  "alarmType": "SMOKE",
  "alarmContent": "疑似烟雾告警",
  "alarmTime": "2026-04-21T10:30:00+08:00"
}
```

#### 6.2.9 响应参数

| 名称 | 类型 | 说明 |
|------|------|------|
| code | Integer | 状态码，`0` 为成功，其他为失败 |
| traceId | String | 请求参数中的流水号 |
| msg | String | 错误信息或成功信息 |
| timestamp | Long | 响应时的毫秒级时间戳 |

#### 6.2.10 响应示例

```json
{
  "msg": "操作成功",
  "traceId": "174737636857728828",
  "code": 0,
  "timestamp": 1747376371135
}
```

---

## 七、需要厂商确认的问题

### 7.1 MQTT 消息格式

- [ ] 前述 4 类 MQTT 消息的 JSON 格式是否正确
- [ ] 字段名称、类型、必填项是否准确
- [ ] 是否有遗漏字段

### 7.2 枚举值定义

- [ ] `healthStatus` 的枚举值定义
- [ ] `runMode` 的枚举值定义
- [ ] `motionStatus` 的枚举值定义
- [ ] `alarmType` 的枚举值定义

### 7.3 连接参数

- [ ] MQTT Broker 地址和端口
- [ ] 连接用户名和密码
- [ ] Product ID 的具体值
- [ ] 是否需要 SSL/TLS 加密连接

### 7.4 业务逻辑

- [ ] 心跳消息的发送频率
- [ ] 任务进度更新的频率
- [ ] 点位同步在什么时机触发
- [ ] 是否支持平台触发响应式点位同步
- [ ] 任务下发后机器狗多久响应
- [ ] 机器狗在接收任务后是否会上报一次“已接受/待执行”状态

### 7.5 HTTPS 接口

- [ ] `data` 字段的 AES 加密模式、填充方式、编码方式
- [ ] `signatureSecret` 的分配方式与签名规则是否最终确定
- [ ] `partnerId`、`version` 的取值规则
- [ ] 拍照与告警上报接口的超时时间、重试策略
- [ ] `alarmTime` 的最终时间格式
- [ ] `base64` 图片大小限制与压缩要求

---

## 八、测试场景

### 8.1 正常流程

1. 机器狗上线，发送心跳消息
2. 机器狗上报点位信息
3. 机器人平台下发巡检任务
4. 机器狗接受任务，返回响应（`code=0`）
5. 机器狗执行任务，定期上报进度
6. 任务完成，上报最终状态（`taskStatus=4`）
7. 机器狗在巡检点位触发拍照并通过 HTTPS 上报结果
8. 机器狗识别到隐患后通过 HTTPS 上报告警信息

### 8.2 异常流程

1. 机器狗拒绝任务（电量不足、正在充电等）
2. 任务执行中出现异常（障碍物、网络中断等）
3. 机器人平台离线时的消息缓存
4. HTTPS 上报失败后的重试或补偿机制

---

## 九、附录

### 附录 A：完整测试消息示例

#### A.1 点位同步完整示例

```json
{
  "type": "response",
  "method": "map",
  "code": 0,
  "msgid": 1001,
  "message": [
    {"mapId": "9acb90d53c0d52b89d7f8a6ee4a19b85", "mapName": "factory_map", "pointCount": 5, "pointId": ["P_A01", "P_A02", "P_B01", "P_B02", "P_C01"], "pointName": ["大门口", "保安亭", "1号楼入口", "2号楼入口", "停车场"]},
    {"mapId": "6425dbf2b5665a62b5d8b3c7d6d8f0eb", "mapName": "room_map", "pointCount": 3, "pointId": ["P_R01", "P_R02", "P_R03"], "pointName": ["会议室门口", "配电柜前", "角落巡检点"]}
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
{"type": "response", "method": "task", "code": 0, "msgid": 3001,
 "message": {"taskId": "XJ-20260415-00001", "taskProgress": 0, "taskDuration": 0, "taskStatus": 2, "errorMsg": ""}}

{"type": "response", "method": "task", "code": 0, "msgid": 3002,
 "message": {"taskId": "XJ-20260415-00001", "taskProgress": 20, "taskDuration": 3000, "taskStatus": 3, "errorMsg": ""}}

{"type": "response", "method": "task", "code": 0, "msgid": 3003,
 "message": {"taskId": "XJ-20260415-00001", "taskProgress": 50, "taskDuration": 7500, "taskStatus": 3, "errorMsg": ""}}

{"type": "response", "method": "task", "code": 0, "msgid": 3004,
 "message": {"taskId": "XJ-20260415-00001", "taskProgress": 80, "taskDuration": 12000, "taskStatus": 3, "errorMsg": ""}}

{"type": "response", "method": "task", "code": 0, "msgid": 3005,
 "message": {"taskId": "XJ-20260415-00001", "taskProgress": 100, "taskDuration": 15000, "taskStatus": 4, "errorMsg": ""}}
```

#### A.4 心跳消息序列

```json
{"type": "upload", "method": "heart", "msgid": 4001,
 "message": {"runMode": 2, "workStatus": 0, "battery": 100, "batteryStatus": 0, "healthStatus": 0, "motionStatus": 0, "chargeStatus": 0, "signalStrength": -55, "onlineStatus": 0}}

{"type": "upload", "method": "heart", "msgid": 4002,
 "message": {"runMode": 2, "workStatus": 1, "battery": 75, "batteryStatus": 0, "healthStatus": 0, "motionStatus": 1, "chargeStatus": 0, "signalStrength": -60, "onlineStatus": 0}}

{"type": "upload", "method": "heart", "msgid": 4003,
 "message": {"runMode": 2, "workStatus": 0, "battery": 15, "batteryStatus": 1, "healthStatus": 0, "motionStatus": 0, "chargeStatus": 0, "signalStrength": -65, "onlineStatus": 0}}

{"type": "upload", "method": "heart", "msgid": 4004,
 "message": {"runMode": 1, "workStatus": 0, "battery": 20, "batteryStatus": 0, "healthStatus": 0, "motionStatus": 0, "chargeStatus": 1, "signalStrength": -50, "onlineStatus": 0}}
```
