#include "bsp_uart.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "bsp_eeprom.h"
#include "bsp_status.h"

//variable definition 

 uint8_t uart1_buffer[UART1_BUFFER_SIZE];
 uint8_t uart1_buffer_data[UART1_BUFFER_SIZE];
char UART1_TX_BUF[200];
uint8_t uart1_data_lenth=0;
volatile uint8_t uart1_recv=0;
volatile uint8_t uart_Receive_Flag=0;


volatile uint8_t uart3_Receive_Flag=0;
uint8_t uart3_data_lenth=0;
uint8_t uart3_buffer[UART1_BUFFER_SIZE];
uint8_t uart3_buffer_data[UART1_BUFFER_SIZE];

#define UART3_DEFAULT_SLAVE_ADDR  0xC8
#define UART3_DEFAULT_BAUDRATE    115200U

static uint8_t uart3_slave_addr = UART3_DEFAULT_SLAVE_ADDR;
static uint8_t uart3_baud_code = 4;
static uint32_t uart3_baudrate = UART3_DEFAULT_BAUDRATE;
static uint8_t uart3_write_data_rx_debug_count = 0;

static uint8_t uart3_baud_code_from_baudrate(uint32_t baudrate)
{
	switch (baudrate) {
		case 9600U: return 0;
		case 19200U: return 1;
		case 38400U: return 2;
		case 57600U: return 3;
		case 115200U: return 4;
		case 230400U: return 5;
		case 460800U: return 6;
		case 921600U: return 7;
		default: return 4;
	}
}


static uint32_t uart3_baudrate_from_legacy_code(uint8_t code)
{
	switch (code) {
		case 0: return 9600U;
		case 1: return 19200U;
		case 2: return 38400U;
		case 3: return 57600U;
		case 4: return 115200U;
		case 5: return 230400U;
		case 6: return 460800U;
		case 7: return 921600U;
		default: return UART3_DEFAULT_BAUDRATE;
	}
}
static uint8_t uart3_is_valid_baudrate(uint32_t baudrate)
{
	return baudrate == 9600U || baudrate == 19200U ||
		baudrate == 38400U || baudrate == 57600U ||
		baudrate == 115200U || baudrate == 230400U ||
		baudrate == 460800U || baudrate == 921600U;
}

static void uart3_load_config_from_eeprom(void)
{
	uint8_t slave_addr = UART3_DEFAULT_SLAVE_ADDR;
	uint32_t baudrate = UART3_DEFAULT_BAUDRATE;

	if (EEPROM_ReadByte(EEPROM_SLAVE_ADDR, &slave_addr) == HAL_OK) {
		if (slave_addr != 0x00 && slave_addr != 0xFF) {
			uart3_slave_addr = slave_addr;
		}
	}

	if (EEPROM_ReadBytes(EEPROM_SLAVE_BAUD, (uint8_t *)&baudrate, sizeof(baudrate)) != HAL_OK) {
		baudrate = UART3_DEFAULT_BAUDRATE;
	} else if (!uart3_is_valid_baudrate(baudrate)) {
		uint8_t legacy_code = 0xFF;
		if (EEPROM_ReadByte(EEPROM_SLAVE_BAUD, &legacy_code) == HAL_OK && legacy_code <= 7) {
			baudrate = uart3_baudrate_from_legacy_code(legacy_code);
		} else {
			baudrate = UART3_DEFAULT_BAUDRATE;
		}
	}
	uart3_baudrate = baudrate;
	uart3_baud_code = uart3_baud_code_from_baudrate(uart3_baudrate);
}

static void uart3_apply_baudrate(void)
{
	HAL_UART_DMAStop(&huart3);
	HAL_UART_DeInit(&huart3);
	huart3.Init.BaudRate = uart3_baudrate;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
		Error_Handler();
	}
}

//user defineition function
void bsp_uart_init(void){
	uart3_load_config_from_eeprom();
	uart3_apply_baudrate();
	HAL_UART_Receive_DMA(&huart1, uart1_buffer, UART1_BUFFER_SIZE);  // 开启 DMA 接收
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                  // 开启空闲中断
	HAL_UART_Receive_DMA(&huart3, uart3_buffer, UART1_BUFFER_SIZE);  // 开启 DMA 接收
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                  // 开启空闲中断
}

uint8_t bsp_uart_get_slave_addr(void)
{
	return uart3_slave_addr;
}

uint8_t bsp_uart_get_uart3_baud_code(void)
{
	return uart3_baud_code;
}

uint32_t bsp_uart_get_uart3_baudrate(void)
{
	return uart3_baudrate;
}

void uart1_messageCheck(void){
	if(uart1_data_lenth==0){
		return ;
	}
	if(uart1_data_lenth>=9){
		//首先需要校验帧格式是否正确
		
		if(uart1_buffer_data[0]==0xF1){  //确认帧头
			if(uart1_buffer_data[1]==0x01){//确认命令，读取升级标志的命令
				u1_printf("go to the app!");
			}
			if(uart1_buffer_data[1]==0x02){//确认命令，读取升级长度的命令
				
			}
			if(uart1_buffer_data[1]==0x03){//确认命令，读取新固件的校验码的命令
				
			}
		}
	}
////	switch(uart1_buffer_data[0]){
////		case 'T':  
////			
////		break;
////		case 'V':  
////			
////		break;
////		case 'L':  
////			
////		break;
////		case 'H':  
////			
////		break;
////		default:
////			break;
////	}
}

void u1_printf(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(UART1_TX_BUF, sizeof(UART1_TX_BUF), fmt, ap);  // 更安全，防止溢出
    va_end(ap);

    HAL_UART_Transmit(&huart1, (uint8_t*)UART1_TX_BUF, strlen(UART1_TX_BUF), HAL_MAX_DELAY);
}
void u3_printf(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(UART1_TX_BUF, sizeof(UART1_TX_BUF), fmt, ap);  // 更安全，防止溢出
    va_end(ap);
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port,RS485_RE_Pin,GPIO_PIN_SET);
    HAL_UART_Transmit(&huart3, (uint8_t*)UART1_TX_BUF, strlen(UART1_TX_BUF), HAL_MAX_DELAY);
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port,RS485_RE_Pin,GPIO_PIN_RESET);
}

uint8_t get_uartReceiveFlag(void){
	return uart_Receive_Flag;
}

void set_uartReceiveFlag(uint8_t flag){
	uart_Receive_Flag=flag;
}
void print_hex_with_tag(const char *tag, const uint8_t *data, uint16_t length)
{
    u1_printf("%s: ", tag);
    for (uint16_t i = 0; i < length; i++)
    {
        u1_printf("%02X ", data[i]);
    }
    u1_printf("\r\n");
}


uint8_t get_uart3ReceiveFlag(void){
	return uart3_Receive_Flag;
}

void set_uart3ReceiveFlag(uint8_t flag){
	uart3_Receive_Flag=flag;
}
void print3_hex_with_tag(const char *tag, const uint8_t *data, uint16_t length)
{
    u3_printf("%s: ", tag);
    for (uint16_t i = 0; i < length; i++)
    {
        u3_printf("%02X ", data[i]);
    }
    u3_printf("\r\n");
}

void uart3_IDLE_IRQHandle(uint8_t temp){
	//u1_printf("have enter the uart3 IRQHANDLE\r\n");
	uart3_data_lenth = UART1_BUFFER_SIZE - temp;
	if (ctx.state == UPG_WRITE_DATA && uart3_write_data_rx_debug_count < 10) {
		u1_printf("uart3 rx irq len=%u dma_left=%u head=0x%02X\r\n", uart3_data_lenth, temp, uart3_buffer[0]);
		uart3_write_data_rx_debug_count++;
	}
	if (uart3_data_lenth == 0 || uart3_data_lenth >= UART1_BUFFER_SIZE)
	{
		uart3_data_lenth = 0;
		set_uart3ReceiveFlag(0);
		HAL_UART_Receive_DMA(&huart3, uart3_buffer, UART1_BUFFER_SIZE);  // 开启MDA传输
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
		return;
	}

	memcpy(uart3_buffer_data, uart3_buffer, uart3_data_lenth);
	set_uart3ReceiveFlag(1);
	HAL_UART_Receive_DMA(&huart3, uart3_buffer, UART1_BUFFER_SIZE);  // 开启MDA传输
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

void rs485_uart3_tx(uint8_t *data,uint8_t len){
	HAL_GPIO_WritePin(RS485_RE_GPIO_Port,RS485_RE_Pin,GPIO_PIN_SET);
	HAL_UART_Transmit(&huart3,data,len,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(RS485_RE_GPIO_Port,RS485_RE_Pin,GPIO_PIN_RESET);
	
}
