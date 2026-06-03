# glovesV1.0_VGT6 工程解析总览

## 范围说明

本次解析覆盖自定义业务代码：

- `App`
- `Services`
- `Task`

按要求未解析 `Core`、`Drivers`、`Middlewares`。这些目录主要包含 CubeMX 生成代码、HAL/CMSIS/FreeRTOS 等底层或第三方代码。

## 工程分层

| 层级 | 目录 | 作用 |
| --- | --- | --- |
| 应用数据层 | `App` | 定义手套业务数据结构、系统配置和帧组装工具。 |
| 服务层 | `Services` | 提供固定内存池、数据管理、指针队列和 UART 标准输入输出重定向。 |
| 任务层 | `Task` | 实现 RTOS 任务入口，包括合帧任务、串口调试任务和测试任务。 |

## 核心数据流

当前设计的数据流如下：

```text
IMU_CAN_Task
    -> GloveImuSensorData_t
    -> DataManager IMU Queue
    -> FrameAssemblerTask

Touch_ADC_Task
    -> GloveTouchSensorData_t
    -> DataManager Touch Queue
    -> FrameAssemblerTask

FrameAssemblerTask
    -> GloveRawFrame_t
    -> DataManager Raw Queue
    -> AlgorithmTask

AlgorithmTask
    -> GloveFullFrame_t
    -> DataManager Full Queues
    -> StorageTask / RS485Task
```

目前工程中已经实现了 DataManager 和 FrameAssemblerTask；真实 IMU、Touch、Algorithm、Storage、RS485 任务从当前业务目录看仍是待接入或未展开状态。`test_task.c` 中有一套模拟链路，但入口里实际调用被注释。

## 关键设计

### 固定内存池

`Services/src/frame_pool.c` 提供固定块内存池。

- 启动时由静态数组准备固定数量数据块。
- 运行时不使用 `malloc/free`。
- 分配和释放通过空闲索引栈完成。
- 临界区保护 `free_count` 和 `free_stack`，适配多任务环境。

### 指针队列

`Services/src/data_manager.c` 创建 CMSIS-RTOS2 消息队列，但队列里只传 `void *` 指针。

- 避免在队列中复制大结构体。
- 数据本体保存在内存池块中。
- 消费者用完后必须调用对应 `Release`。

### 引用计数

每个 DataManager 数据块都带 `ref_count`。

- 单消费者数据：发布成功后生产者引用释放，消费者持有引用。
- FullFrame 数据：同一个块同时发布给 Storage 和 RS485 两个消费者，两边都释放后才回收到内存池。

### 合帧策略

`Task/src/frameAssemblerTask.c` 负责把 IMU 和 Touch 数据合为 RawFrame。

- IMU 和 Touch 时间差不超过 `5000 us` 才合帧。
- 时间差过大时丢弃更旧的一包。
- 单边数据等待另一边超过 `100 ms` 后丢弃。
- RawFrame 时间戳当前选择 IMU 时间戳。

## 主要文件索引

| 文件 | 文档 | 重点 |
| --- | --- | --- |
| `App/inc/app_config.h` | `App/inc/app_config.h.md` | 系统规模、内存池容量、队列深度、有效标志。 |
| `App/inc/app_data.h` | `App/inc/app_data.h.md` | 业务数据结构和帧类型。 |
| `App/src/app_data.c` | `App/src/app_data.c.md` | 数据清零、RawFrame/FullFrame 组装。 |
| `Services/inc/frame_pool.h` | `Services/inc/frame_pool.h.md` | 内存池接口和统计结构。 |
| `Services/src/frame_pool.c` | `Services/src/frame_pool.c.md` | 固定块内存池实现。 |
| `Services/inc/data_manager.h` | `Services/inc/data_manager.h.md` | DataManager 对外接口和数据块类型。 |
| `Services/src/data_manager.c` | `Services/src/data_manager.c.md` | 内存池、队列、引用计数、发布/获取/释放逻辑。 |
| `Services/inc/uart_redirect.h` | `Services/inc/uart_redirect.h.md` | UART 重定向声明。 |
| `Services/src/uart_redirect.c` | `Services/src/uart_redirect.c.md` | `printf/fgetc` 到 USART1 的重定向。 |
| `Task/inc/frameAssemblerTask.h` | `Task/inc/frameAssemblerTask.h.md` | 合帧任务统计和入口声明。 |
| `Task/src/frameAssemblerTask.c` | `Task/src/frameAssemblerTask.c.md` | IMU/Touch 时间同步合帧逻辑。 |
| `Task/inc/uartDebugTask.h` | `Task/inc/uartDebugTask.h.md` | 串口调试任务入口声明。 |
| `Task/src/uartDebugTask.c` | `Task/src/uartDebugTask.c.md` | 周期打印 DataManager 统计。 |
| `Task/inc/test_task.h` | `Task/inc/test_task.h.md` | 测试任务入口声明。 |
| `Task/src/test_task.c` | `Task/src/test_task.c.md` | 模拟数据流自测逻辑。 |

## 当前工程状态观察

- `App` 层的数据模型已经比较完整，能表达 IMU、触觉、原始帧、算法帧和完整帧。
- `Services` 层已经具备无动态内存的数据流管理能力，适合嵌入式实时系统。
- `FrameAssemblerTask` 已实现基本时间同步和丢弃策略。
- `UartDebugTask` 当前只打印 DataManager 统计，未打印合帧任务统计。
- `test_task.c` 中完整模拟链路存在，但入口调用被注释，默认不会产生测试数据。

## 后续接入建议

1. 接入真实 IMU 采集任务：调用 `DataManager_AllocImuSensor()`、填充数据、再 `DataManager_PublishImuSensor()`。
2. 接入真实 Touch 采集任务：调用 `DataManager_AllocTouchSensor()`、填充数据、再 `DataManager_PublishTouchSensor()`。
3. 实现算法任务：从 `DataManager_GetRawFrame(DATA_CONSUMER_ALGORITHM, ...)` 获取 RawFrame，生成 FullFrame 并发布。
4. 实现 Storage 和 RS485 任务：分别从 `DataManager_GetFullFrame()` 获取 FullFrame，用完后释放。
5. 将 `FrameAssemblerTask_GetStats()` 加入串口调试输出，可以更快定位时间戳不匹配和丢帧问题。
