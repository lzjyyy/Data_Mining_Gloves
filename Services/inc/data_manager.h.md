# Services/inc/data_manager.h 解析

## 文件职责

`data_manager.h` 声明数据管理服务接口。它把内存池和 RTOS 指针队列封装起来，为任务层提供统一的 `Alloc -> Publish -> Get -> Release` 数据流。

## 主要类型

### `DataConsumer_t`

用于选择消费队列。

| 枚举 | 含义 |
| --- | --- |
| `DATA_CONSUMER_ALGORITHM` | 算法任务消费 RawFrame。 |
| `DATA_CONSUMER_STORAGE` | 存储任务消费 FullFrame。 |
| `DATA_CONSUMER_RS485` | RS485 通信任务消费 FullFrame。 |

### 数据块结构

| 类型 | 内容 | 当前流向 |
| --- | --- | --- |
| `GloveImuSensorBlock_t` | `GloveImuSensorData_t` + 引用计数 | IMU 采集任务到合帧任务。 |
| `GloveTouchSensorBlock_t` | `GloveTouchSensorData_t` + 引用计数 | Touch 采集任务到合帧任务。 |
| `GloveRawFrameBlock_t` | `GloveRawFrame_t` + 引用计数 | 合帧任务到算法任务。 |
| `GloveFullFrameBlock_t` | `GloveFullFrame_t` + 引用计数 | 算法任务到存储和 RS485。 |

### `DataManagerStats_t`

汇总 DataManager 运行统计和四类内存池统计。

## 函数声明分组

### 初始化

| 函数 | 作用 |
| --- | --- |
| `DataManager_Init()` | 初始化所有内存池和消息队列。 |

### 分配

| 函数 | 作用 |
| --- | --- |
| `DataManager_AllocImuSensor()` | 分配 IMU Sensor 数据块。 |
| `DataManager_AllocTouchSensor()` | 分配 Touch Sensor 数据块。 |
| `DataManager_AllocRawFrame()` | 分配 RawFrame 数据块。 |
| `DataManager_AllocFullFrame()` | 分配 FullFrame 数据块。 |

### 发布

| 函数 | 作用 |
| --- | --- |
| `DataManager_PublishImuSensor()` | 发布 IMU 数据给合帧任务。 |
| `DataManager_PublishTouchSensor()` | 发布 Touch 数据给合帧任务。 |
| `DataManager_PublishRawFrame()` | 发布 RawFrame 给算法任务。 |
| `DataManager_PublishFullFrame()` | 发布 FullFrame 给 Storage 和 RS485 两个消费者。 |

### 获取和释放

| 函数 | 作用 |
| --- | --- |
| `DataManager_GetImuSensor()` | 合帧任务获取 IMU 数据块。 |
| `DataManager_GetTouchSensor()` | 合帧任务获取 Touch 数据块。 |
| `DataManager_GetRawFrame()` | 指定消费者获取 RawFrame。 |
| `DataManager_GetFullFrame()` | 指定消费者获取 FullFrame。 |
| `DataManager_ReleaseImuSensor()` | 释放 IMU 数据块引用。 |
| `DataManager_ReleaseTouchSensor()` | 释放 Touch 数据块引用。 |
| `DataManager_ReleaseRawFrame()` | 释放 RawFrame 引用。 |
| `DataManager_ReleaseFullFrame()` | 释放 FullFrame 引用。 |
| `DataManager_GetStats()` | 获取运行统计。 |

## 使用约定

- `Alloc` 成功后，生产者拥有一个引用。
- `Publish` 返回后，生产者不再拥有该数据块。
- `Get` 成功后，消费者必须在用完后调用对应 `Release`。
- FullFrame 当前会发布给两个消费者，因此引用计数用于保证两个消费者都释放后再归还内存池。
