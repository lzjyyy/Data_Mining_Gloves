#ifndef __RS485RECVTASK_H__
#define __RS485RECVTASK_H__


#include "stdint.h"
#include <stdio.h>
#include "Kin_Slover.h"

/* 二、定义串口数据缓冲区配置 */
#define UART3_BUFFER_SIZE        (128)
#define UART3_BUFFER_QUANTITY    (6)
 
/* 三、定义串口数据结构体，DMA传输的数据将保存在这里 */
typedef struct{
	uint8_t buffer[UART3_BUFFER_SIZE];   /* 存放数据的空间 */
	uint16_t size;                      /* 已存放数据的大小 */
} UART3_RX_TypeDef;


extern uint8_t uart3_buff_ctrl;
extern UART3_RX_TypeDef uart3_rx_data_t[UART3_BUFFER_QUANTITY];

void USART3_DMA_Handler(void);
void StartRs485RecvTask(void *argument);

#endif



