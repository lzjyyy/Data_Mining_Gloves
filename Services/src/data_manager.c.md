# Services/src/data_manager.c 解析

## 文件职责

`data_manager.c` 是工程的数据流中心。它管理四类静态内存池和五条指针消息队列，让采集、合帧、算法、存储和通信任务之间传递大数据帧时不发生大结构体拷贝。

## 静态资源

| 资源 | 作用 |
| --- | --- |
| `s_imu_sensor_pool` | IMU Sensor 数据块内存池。 |
| `s_touch_sensor_pool` | Touch Sensor 数据块内存池。 |
| `s_raw_pool` | RawFrame 数据块内存池。 |
| `s_full_pool` | FullFrame 数据块内存池。 |
| `s_queues` | 保存五条 RTOS 指针队列句柄。 |
| `s_stats` | DataManager 运行统计。 |
| `s_initialized` | 初始化完成标志。 |
| `s_*_blocks` | 四类数据块的静态存储区。 |
| `s_*_free_stack` | 四类内存池的空闲索引栈。 |

## 内部函数解析

### `CreatePointerQueue(uint32_t depth, const char *name)`

创建只传递指针的 CMSIS-RTOS2 消息队列。

- 队列元素大小固定为 `sizeof(void *)`。
- 队列名称用于 RTOS 调试识别。
- 大数据帧留在内存池中，队列里只放数据块指针。

### `TimeoutMsToTicks(uint32_t timeout_ms)`

将毫秒超时转换为 RTOS tick。

- 支持 `osWaitForever`。
- 非零毫秒至少转换成 1 tick。
- 对过大结果做上限保护。

### `StatsIncrement(uint32_t *value)`

在线程临界区中递增统计值。

- 用于发布计数、丢帧计数、内存池分配失败和队列发送失败计数。

### `SendPointer(osMessageQueueId_t queue, void *ptr, uint32_t timeout_ms)`

向指针队列发送数据块指针。

- 队列或指针为空时返回 `GLOVE_STATUS_INVALID_PARAM`。
- 发送失败时增加 `queue_send_failures`，并返回 `GLOVE_STATUS_QUEUE_FULL`。

### `ReceivePointer(osMessageQueueId_t queue, void **ptr, uint32_t timeout_ms)`

从指针队列接收数据块指针。

- 成功时写入 `*ptr` 并返回 `GLOVE_STATUS_OK`。
- 超时返回 `GLOVE_STATUS_TIMEOUT`。
- 其他失败统一返回 `GLOVE_STATUS_QUEUE_EMPTY`。

### `AddRef(volatile uint8_t *ref_count)`

在临界区中增加数据块引用计数。

- 发布到队列前调用，表示消费者即将持有引用。

### `ReleaseRef(FramePool_t *pool, void *block, volatile uint8_t *ref_count)`

释放一次引用。

- 检查池、数据块、引用计数指针以及地址归属。
- 引用计数减到 0 时调用 `FramePool_Free()` 归还内存池。
- 引用计数仍大于 0 时只更新计数，不归还。

### `PublishSingleConsumer(...)`

发布给单个消费者的通用流程。

- 适用于 IMU Sensor、Touch Sensor 和 RawFrame。
- 发布前增加一个消费者引用。
- 队列发送失败时释放消费者引用和生产者引用，并统计丢弃。
- 队列发送成功后释放生产者引用，数据块生命周期转交给消费者。

## 对外函数解析

### `DataManager_Init(void)`

初始化 DataManager。

- 清空队列句柄和统计。
- 初始化四个内存池。
- 创建 IMU、Touch、RawFrame、FullFrame Storage、FullFrame RS485 五条指针队列。
- 全部成功后设置 `s_initialized = 1`。

### `DataManager_AllocImuSensor()` / `DataManager_AllocTouchSensor()`

从对应内存池分配采集数据块。

- 未初始化时返回 `NULL`。
- 分配失败时增加 `pool_alloc_failures`。
- 成功后清空数据内容，并设置 `ref_count = 1`。

### `DataManager_AllocRawFrame()` / `DataManager_AllocFullFrame()`

从对应内存池分配帧数据块。

- 行为与 Sensor 分配函数一致。
- RawFrame 使用 `AppData_ClearRawFrame()` 清零。
- FullFrame 使用 `AppData_ClearFullFrame()` 清零。

### `DataManager_PublishImuSensor()` / `DataManager_PublishTouchSensor()` / `DataManager_PublishRawFrame()`

发布给单消费者队列。

- 参数为空返回 `GLOVE_STATUS_INVALID_PARAM`。
- 内部调用 `PublishSingleConsumer()` 完成引用计数和队列发送。

### `DataManager_PublishFullFrame(GloveFullFrameBlock_t *block, uint32_t timeout_ms)`

发布 FullFrame 给两个消费者。

- 分别向 Storage 队列和 RS485 队列发送同一个数据块指针。
- 每发送一路前都增加一次引用。
- 某一路发送失败时释放该路引用，并记录最终状态。
- 发布者临时引用最后释放。
- 两路都失败时统计 `full_frames_dropped` 并返回队列满。
- 至少一路成功时统计 `full_frames_published`。

### `DataManager_GetImuSensor()` / `DataManager_GetTouchSensor()`

合帧任务从对应队列取采集数据块。

- 参数为空返回 `GLOVE_STATUS_INVALID_PARAM`。
- 成功后消费者必须调用对应 Release。

### `DataManager_GetRawFrame(DataConsumer_t consumer, ...)`

根据消费者类型获取 RawFrame。

- 当前只支持 `DATA_CONSUMER_ALGORITHM`。
- 其他消费者返回 `GLOVE_STATUS_INVALID_PARAM`。

### `DataManager_GetFullFrame(DataConsumer_t consumer, ...)`

根据消费者类型获取 FullFrame。

- `DATA_CONSUMER_STORAGE` 对应存储队列。
- `DATA_CONSUMER_RS485` 对应 RS485 队列。
- 其他消费者返回 `GLOVE_STATUS_INVALID_PARAM`。

### `DataManager_ReleaseImuSensor()` / `DataManager_ReleaseTouchSensor()` / `DataManager_ReleaseRawFrame()` / `DataManager_ReleaseFullFrame()`

释放对应数据块的一次引用。

- 未初始化或参数为空时返回 `GLOVE_STATUS_INVALID_PARAM`。
- 内部调用 `ReleaseRef()`。
- 引用计数归零才真正回收到内存池。

### `DataManager_GetStats(DataManagerStats_t *stats)`

读取 DataManager 和四个内存池的统计。

- 参数为空时直接返回。
- DataManager 统计在临界区中复制。
- 内存池统计通过 `FramePool_GetStats()` 获取。

## 注意点

- FullFrame 是当前唯一多消费者数据块，引用计数逻辑最关键。
- `Publish` 失败后通常已经处理发布者引用，调用方不应再重复释放已交给 `Publish` 的块，除非具体函数返回路径明确没有接管。
- 队列传递的是指针，因此消费者释放前不能长期占用数据块，否则会造成内存池耗尽。
