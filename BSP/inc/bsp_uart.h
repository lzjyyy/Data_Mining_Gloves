#ifndef  __BSP_UART_H
#define  __BSP_UART_H

#include "main.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
#define  UART1_BUFFER_SIZE 150
extern uint8_t uart1_buffer[UART1_BUFFER_SIZE];
extern  uint8_t uart1_buffer_data[UART1_BUFFER_SIZE];
extern uint8_t uart1_data_lenth;
extern uint8_t uart3_buffer[UART1_BUFFER_SIZE];
extern  uint8_t uart3_buffer_data[UART1_BUFFER_SIZE];
extern uint8_t uart3_data_lenth;

typedef enum {
    BAUD_CODE_9600   = 0x0001,
    BAUD_CODE_19200  = 0x0002,
    BAUD_CODE_38400  = 0x0003,
    BAUD_CODE_57600  = 0x0004,
    BAUD_CODE_115200 = 0x0005,
    BAUD_CODE_230400 = 0x0006,
    BAUD_CODE_460800 = 0x0007,
    BAUD_CODE_921600 = 0x0008,
} BaudCode_t;

uint8_t get_uartReceiveFlag(void);
void set_uartReceiveFlag(uint8_t flag);
uint8_t get_uart3ReceiveFlag(void);
void set_uart3ReceiveFlag(uint8_t flag);
void u1_printf(const char* fmt, ...);
void u3_printf(const char* fmt, ...);
void uart1_messageCheck(void);
void bsp_uart_init(void);
uint8_t bsp_uart_get_slave_addr(void);
uint8_t bsp_uart_get_uart3_baud_code(void);
uint32_t bsp_uart_get_uart3_baudrate(void);
void print_hex_with_tag(const char *tag, const uint8_t *data, uint16_t length);
void print3_hex_with_tag(const char *tag, const uint8_t *data, uint16_t length);

void uart1_IDLE_IRQHandle(uint8_t temp);
void uart3_IDLE_IRQHandle(uint8_t temp);

void rs485_uart3_tx(uint8_t *data,uint8_t len);
#endif
