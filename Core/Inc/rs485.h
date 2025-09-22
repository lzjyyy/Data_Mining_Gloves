#ifndef __RS485_H__
#define __RS485_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1; // RS485A_CH
extern UART_HandleTypeDef huart3; // RS485B_CH
extern UART_HandleTypeDef huart4;	// RS485C_CH

#define RS485A_CH		0
#define RS485B_CH   1
#define RS485C_CH   2

#define RS485C_DIR_PORT	 GPIOD
#define RS485C_DIR_PIN   GPIO_PIN_3

    void RS485_Init(void);
    HAL_StatusTypeDef RS485_Send(uint8_t ch, uint8_t* pData, uint16_t size, uint32_t timeout);
    HAL_StatusTypeDef RS485_Receive(uint8_t ch, uint8_t* pData, uint16_t size, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif