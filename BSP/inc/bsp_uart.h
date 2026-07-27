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
