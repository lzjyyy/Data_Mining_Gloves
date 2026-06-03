# Task/src/frameAssemblerTask.c 解析

## 文件职责

`frameAssemblerTask.c` 实现合帧任务。它从 DataManager 获取 IMU Sensor 数据和 Touch Sensor 数据，根据时间戳判断是否可匹配，成功后生成 RawFrame 并发布给算法任务。

## 宏定义

| 宏 | 含义 |
| --- | --- |
| `FRAME_ASSEMBLER_GET_TIMEOUT_MS` | 从队列取数据的等待时间，当前 10 ms。 |
| `FRAME_ASSEMBLER_IDLE_DELAY_MS` | 无法合帧时的空闲延时，当前 1 ms。 |
| `FRAME_ASSEMBLER_MAX_TIME_DIFF_US` | IMU 和 Touch 允许合帧的最大时间差，当前 5000 us。 |
| `FRAME_ASSEMBLER_PENDING_TIMEOUT_MS` | 单边数据等待另一边数据的最长时间，当前 100 ms。 |

## 静态变量

| 变量 | 作用 |
| --- | --- |
| `s_frame_assembler_stats` | 合帧任务运行统计。 |
| `s_next_frame_id` | 下一帧 RawFrame 的帧号。 |

## 内部函数解析

### `FrameAssembler_MsToTicks(uint32_t timeout_ms)`

将毫秒转换成 RTOS tick。

- 使用 `osKernelGetTickFreq()` 获取 tick 频率。
- 非零毫秒至少转换为 1 tick。
- 对过大值做上限保护。

### `FrameAssembler_TimeDiffSigned(uint32_t newer_or_older, uint32_t reference)`

计算两个 `uint32_t` 时间戳的有符号差值。

- 利用强转为 `int32_t` 处理时间戳回绕场景。
- 返回值小于 0 表示第一个时间戳相对更旧。

### `FrameAssembler_TimeDiffAbsUs(uint32_t a, uint32_t b)`

计算两个时间戳的绝对差值。

- 内部调用 `FrameAssembler_TimeDiffSigned()`。
- 用于判断 IMU 和 Touch 是否在允许时间窗口内。

### `FrameAssembler_IsFirstOlder(uint32_t first_timestamp_us, uint32_t second_timestamp_us)`

判断第一个时间戳是否比第二个旧。

- 返回 `1` 表示第一个更旧。
- 时间差过大时用于决定丢弃 IMU 还是 Touch。

### `FrameAssembler_IsPendingTimeout(uint32_t start_tick)`

判断某个 pending 数据是否等待另一类数据过久。

- 用当前 tick 减去开始等待 tick。
- 超过 `FRAME_ASSEMBLER_PENDING_TIMEOUT_MS` 返回 `1`。

### `FrameAssembler_SelectFrameTimestamp(const GloveImuSensorData_t *imu, const GloveTouchSensorData_t *touch)`

选择 RawFrame 时间戳。

- 当前固定使用 IMU 时间戳。
- Touch 参数当前未使用。
- 这样做适合姿态算法主要依赖 IMU 的场景。

### `FrameAssembler_SetStatus(GloveStatus_t status)`

更新最近一次合帧状态。

### `FrameAssembler_ReleaseImu(GloveImuSensorBlock_t **imu)`

释放 pending IMU 数据块。

- 如果指针有效，调用 `DataManager_ReleaseImuSensor()`。
- 释放后把本地指针置空，避免重复释放。

### `FrameAssembler_ReleaseTouch(GloveTouchSensorBlock_t **touch)`

释放 pending Touch 数据块。

- 行为与 `FrameAssembler_ReleaseImu()` 类似。

### `FrameAssembler_PublishRawFrame(const GloveImuSensorBlock_t *imu, const GloveTouchSensorBlock_t *touch)`

根据 IMU 和 Touch 数据生成并发布 RawFrame。

- 分配 RawFrame，失败时增加 `raw_alloc_failures`。
- 选择帧时间戳。
- 调用 `AppData_BuildRawFrameFromSensors()` 填充 RawFrame。
- 发布 RawFrame，失败时增加 `raw_publish_failures`。
- 成功时增加 `assembled_frames`，记录 `last_frame_id`，并递增 `s_next_frame_id`。

### `FrameAssembler_TryAssemble(GloveImuSensorBlock_t **imu, GloveTouchSensorBlock_t **touch)`

在 IMU 和 Touch 都存在时尝试合帧。

- 检查参数和数据块指针。
- 计算两类数据时间戳差值，并记录到 `last_time_diff_us`。
- 如果时间差超过阈值，统计 `timestamp_mismatch_drops`，丢弃更旧的一包，保留较新的一包等待下一次匹配。
- 如果时间差在允许范围内，发布 RawFrame，并释放 IMU 和 Touch 输入块。

## 对外函数解析

### `FrameAssemblerTask_GetStats(FrameAssemblerStats_t *stats)`

读取合帧任务统计。

- 参数为空时不做任何操作。
- 使用临界区复制统计结构，避免读取过程中被任务更新。

### `FrameAssemblerTask(void *argument)`

合帧任务入口。

- 初始化合帧统计。
- 循环维护 `pending_imu` 和 `pending_touch` 两个待匹配数据块。
- 如果某类 pending 为空，就从 DataManager 队列尝试获取。
- 两类数据都到齐时调用 `FrameAssembler_TryAssemble()`。
- 如果只有一类数据到达，检查是否 pending 超时，超时则释放并统计 stale drop。
- 没有合帧动作时延时 1 ms。

## 合帧策略

- 时间差不超过 5000 us 时合帧。
- 时间差过大时丢弃更旧的数据，保留新的数据等下一包。
- 单边数据等另一边超过 100 ms 后被认为过期并丢弃。

## 注意点

- RawFrame 时间戳当前来自 IMU。
- `FrameAssembler_PublishRawFrame()` 发布失败后没有显式释放 RawFrame；按照当前 `DataManager_PublishRawFrame()` 的失败路径会释放引用，但这依赖 DataManager 的发布语义。
- 该任务只生成 RawFrame，不生成算法结果和 FullFrame。
