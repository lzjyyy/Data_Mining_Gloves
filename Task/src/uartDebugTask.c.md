# Task/src/uartDebugTask.c 解析

## 文件职责

`uartDebugTask.c` 实现串口调试任务，定时读取 `DataManager` 的运行统计，并通过 `printf` 输出到 UART。

## 宏定义

| 宏 | 含义 |
| --- | --- |
| `UART_DEBUG_PRINT_PERIOD_MS` | 调试信息打印周期，当前为 1000 ms。 |

## 函数解析

### `UartDebugTask(void *argument)`

串口调试线程入口。

- 忽略 `argument`。
- 启动时打印 `[UART] debug task started`。
- 每 1 秒调用 `DataManager_GetStats()` 获取统计。
- 打印 tick 计数、IMU 发布数、Touch 发布数、RawFrame 发布数、FullFrame 发布数、内存池分配失败数和队列发送失败数。
- 循环末尾调用 `osDelay(UART_DEBUG_PRINT_PERIOD_MS)` 让出 CPU。

## 输出内容含义

| 字段 | 含义 |
| --- | --- |
| `tick` | 调试任务自己的秒级计数。 |
| `imu_pub` | 已发布 IMU Sensor 数据块数量。 |
| `touch_pub` | 已发布 Touch Sensor 数据块数量。 |
| `raw_pub` | 已发布 RawFrame 数量。 |
| `full_pub` | 已发布 FullFrame 数量。 |
| `alloc_fail` | 内存池分配失败次数。 |
| `queue_fail` | 队列发送失败次数。 |

## 注意点

- 该任务只观察 DataManager 总体统计，不打印合帧任务的详细统计。
- `printf` 经过 UART 阻塞发送，打印过频会影响实时性；当前 1 秒周期比较温和。
