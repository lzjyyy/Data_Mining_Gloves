# Services/src/uart_redirect.c 解析

## 文件职责

`uart_redirect.c` 将 C 标准库的字符输入输出重定向到 STM32 HAL UART。这样任务中可以直接使用 `printf()` 输出调试信息。

## 函数解析

### `fputc(int ch, FILE *f)`

输出一个字符。

- `printf()` 输出时会间接调用该函数。
- 调用 `HAL_UART_Transmit(&huart1, ...)` 将字符通过 USART1 发送。
- 发送超时参数为 `0xffff`，属于较长阻塞等待。
- 返回写入的字符 `ch`。

### `fgetc(FILE *f)`

读取一个字符。

- 调用 `HAL_UART_Receive(&huart1, ...)` 从 USART1 接收 1 字节。
- 接收超时参数为 `0xffff`，会阻塞等待较长时间。
- 返回读取到的字符。

## 注意点

- `f` 参数当前未使用。
- `huart1` 来自 `main.h` 中的外部 UART 句柄声明。
- 在 RTOS 任务中大量 `printf` 可能阻塞当前任务，调试频率需要控制。
