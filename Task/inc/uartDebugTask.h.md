# Task/inc/uartDebugTask.h 解析

## 文件职责

`uartDebugTask.h` 声明串口调试任务入口函数。任务实现通过 `printf` 周期性输出系统运行统计。

## 函数声明

### `UartDebugTask(void *argument)`

串口调试线程入口。

- 参数 `argument` 是 RTOS 任务入口的通用参数。
- 实现在 `uartDebugTask.c`。
- 依赖 `uart_redirect.c` 将 `printf` 输出到 UART。

## 使用位置

该函数通常由 FreeRTOS/CMSIS-RTOS 任务创建代码作为线程入口传入。
