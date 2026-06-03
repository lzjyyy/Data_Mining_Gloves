# Services/inc/uart_redirect.h 解析

## 文件职责

`uart_redirect.h` 声明标准输入输出重定向函数，让 `printf` 和字符读取可以通过 UART 工作。

## 函数声明

### `fputc(int ch, FILE *f)`

标准库输出字符接口。

- 被 `printf` 底层调用。
- 实现在 `uart_redirect.c` 中，当前输出到 `huart1`。

### `fgetc(FILE *f)`

标准库输入字符接口。

- 被字符读取相关标准库函数调用。
- 实现在 `uart_redirect.c` 中，当前从 `huart1` 阻塞读取。

## 注意点

- 当前头文件只包含 `stdio.h` 并声明两个重定向函数。
- 具体使用哪个串口由 `.c` 文件中的 `HAL_UART_Transmit()` 和 `HAL_UART_Receive()` 决定。
