# App/src/app_data.c 解析

## 文件职责

`app_data.c` 实现业务数据结构的清零和组帧辅助函数。它不分配内存、不操作队列，只负责把已有数据结构整理成统一帧格式。

## 函数解析

### `AppData_ClearImuSensorData(GloveImuSensorData_t *data)`

清空一份 IMU Sensor 数据。

- 参数为 `NULL` 时直接返回。
- 使用 `memset` 将整个结构体置零。
- 通常在 `DataManager_AllocImuSensor()` 分配数据块后调用，保证生产者拿到的是干净数据。

### `AppData_ClearTouchSensorData(GloveTouchSensorData_t *data)`

清空一份 Touch Sensor 数据。

- 参数为 `NULL` 时直接返回。
- 使用 `memset` 清零整个触觉采样结构。
- 通常在 `DataManager_AllocTouchSensor()` 中使用。

### `AppData_ClearRawFrame(GloveRawFrame_t *frame)`

清空 RawFrame。

- 参数为 `NULL` 时直接返回。
- 用于 RawFrame 内存池块刚分配出来时的初始化。

### `AppData_ClearProcessedFrame(GloveProcessedFrame_t *frame)`

清空算法输出帧。

- 参数为 `NULL` 时直接返回。
- 测试任务中的 `Test_FillProcessedFrame()` 会先调用它，再填入模拟算法结果。

### `AppData_ClearFullFrame(GloveFullFrame_t *frame)`

清空 FullFrame。

- 参数为 `NULL` 时直接返回。
- 用于 FullFrame 内存池块分配后的初始化。

### `AppData_BuildRawFrameFromSensors(...)`

将一份 IMU Sensor 数据和一份 Touch Sensor 数据合成为 `GloveRawFrame_t`。

- 参数任意一个为 `NULL` 时直接返回。
- 设置 `frame_id` 和 `timestamp_us`。
- `valid_flags` 取 IMU 和 Touch 的有效标志按位或。
- 拷贝 IMU 原始数组、四元数数组和触觉数组。
- 时间同步策略不在本函数处理，由 `FrameAssemblerTask` 决定。

### `AppData_BuildFullFrame(...)`

将 RawFrame 和算法输出帧合成为 `GloveFullFrame_t`。

- 参数任意一个为 `NULL` 时直接返回。
- `frame_id`、`timestamp_us` 继承自 RawFrame。
- `valid_flags` 取 RawFrame 与 ProcessedFrame 的有效标志按位或。
- 直接结构体赋值保存完整的原始帧和算法帧。

## 注意点

- 本文件函数都是轻量工具函数，没有内部状态。
- 组帧时使用整块拷贝，结构体尺寸变大时会增加 CPU 和 RAM 带宽开销。
- `BuildRawFrameFromSensors()` 当前选择外部传入时间戳，不自行比较 IMU 和 Touch 时间戳。
