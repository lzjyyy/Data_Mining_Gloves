#ifndef _CAN_1_H_
#define _CAN_1_H_

#include "main.h"
#include "data.h"
#include "stm32f4xx_hal.h"

unsigned char CAN1_Init(CO_Data* d, uint32_t bitrate);
unsigned char canSend(CAN_PORT notused, Message* m);
unsigned char canChangeBaudRate_driver(CAN_HANDLE fd, char* baud);

#endif