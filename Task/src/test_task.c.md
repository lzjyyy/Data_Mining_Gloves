# Task/src/test_task.c 解析

## 文件职责

`test_task.c` 是一条模拟数据流自测任务。它用虚拟 IMU、Touch 和算法结果跑通以下链路：

`Produce Sensors -> Assemble RawFrame -> Run Algorithm -> Publish FullFrame -> Storage/RS485 Consume`

当前 `StartTestTask()` 中实际调用测试流程的代码被注释，因此任务运行时只每秒延时，不会主动产生测试数据。

## 静态类型和变量

### `TestTaskStats_t`

保存测试任务运行结果。

| 字段 | 含义 |
| --- | --- |
| `pass_count` | 成功跑通完整帧链路次数。 |
| `fail_count` | 失败次数。 |
| `last_status` | 最近一次失败或运行状态。 |
| `last_frame_id` | 最近完成测试的帧号。 |
| `last_storage_frame_id` | Storage 消费者最近收到的帧号。 |
| `last_rs485_frame_id` | RS485 消费者最近收到的帧号。 |

### `s_test_stats`

测试统计全局静态变量。

## 函数解析

### `Test_SetError(GloveStatus_t status)`

记录一次测试错误。

- 更新 `last_status`。
- `fail_count` 加 1。

### `Test_MakeTimestampUs(uint32_t seq)`

根据序号生成模拟时间戳。

- 当前策略为 `seq * 10000U`，即每帧间隔 10 ms。

### `Test_FillImuSensor(GloveImuSensorBlock_t *block, uint32_t seq)`

填充一份模拟 IMU Sensor 数据。

- 设置采集序号、时间戳和有效标志。
- 遍历 16 路 IMU，填充加速度、角速度、温度、状态和四元数。
- 用于模拟 IMU 采集任务输出。

### `Test_FillTouchSensor(GloveTouchSensorBlock_t *block, uint32_t seq)`

填充一份模拟 Touch Sensor 数据。

- 设置采集序号。
- 时间戳比 IMU 晚 100 us，用于模拟两类传感器采集存在轻微偏移。
- 遍历 81 个触觉点，填充当前值和基线值。

### `Test_FillProcessedFrame(GloveProcessedFrame_t *processed, const GloveRawFrame_t *raw)`

根据 RawFrame 生成模拟算法输出。

- 先清空 `processed`。
- 帧号和时间戳继承 RawFrame。
- 设置算法有效标志。
- 将 RawFrame 中的四元数复制为算法姿态输出。
- 填充 21 个关节角和关节角速度的模拟值。

### `Test_ProduceSensors(uint32_t seq)`

生产并发布一组 IMU 和 Touch 数据。

- 从 DataManager 分配 IMU 和 Touch 数据块。
- 分配任意一个失败时释放已分配块并返回 `GLOVE_STATUS_NO_MEMORY`。
- 填充模拟数据。
- 依次发布 IMU 和 Touch。
- 如果 IMU 发布失败，会释放 Touch 数据块。

### `Test_AssembleRawFrame(uint32_t frame_id)`

模拟合帧流程。

- 从 DataManager 获取 IMU 和 Touch 数据块。
- 分配 RawFrame。
- 调用 `AppData_BuildRawFrameFromSensors()` 组装 RawFrame。
- 释放 IMU 和 Touch 输入块。
- 发布 RawFrame 给算法消费者。

### `Test_RunAlgorithm(void)`

模拟算法任务。

- 获取 RawFrame。
- 分配 FullFrame。
- 生成模拟 `GloveProcessedFrame_t`。
- 调用 `AppData_BuildFullFrame()` 组装完整帧。
- 释放 RawFrame。
- 发布 FullFrame 给 Storage 和 RS485 消费者。

### `Test_ConsumeFullFrames(void)`

模拟两个 FullFrame 消费者。

- Storage 队列取一帧，记录帧号并释放。
- RS485 队列取一帧，记录帧号并释放。
- 两个消费者都成功才返回 `GLOVE_STATUS_OK`。

### `Test_RunOneFrame(uint32_t frame_id)`

跑通一帧完整测试链路。

- 依次调用生产传感器、合帧、算法、消费 FullFrame。
- 任一步失败立即返回错误状态。
- 成功时更新 `last_frame_id`。

### `StartTestTask(void *argument)`

测试任务入口。

- 当前完整测试流程代码被注释。
- 实际运行时只执行 `osDelay(1000U)`。
- 如果取消注释，可以每秒打印测试结果并推进 `frame_id`。

## 注意点

- 这是模拟任务，不是真实传感器采集任务。
- 当前被注释后不会向 DataManager 注入任何数据。
- 如果开启测试流程，需要确保 `DataManager_Init()` 已在任务启动前完成。
