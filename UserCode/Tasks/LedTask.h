#ifndef __LED_TASK_H
#define __LED_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"
#include <stdint.h>


/**
 * 接口函数:9
 * LedTask_SetMode() 		设置LED状态
 * LedTask_GetMode() 		获取LED状态
 * 
 */


typedef enum
{
		LED_MODE_OFF = 0,
		LED_MODE_BOOT,
		LED_MODE_RUN,
		LED_MODE_ERROR,
		LED_MODE_IDLE,
		LED_MODE_RESET
	
} LedMode_t;



void StartLedTask(void *argument);
void LedTask_SetMode(LedMode_t mode);
LedMode_t LedTask_GetMode(void);

void LedTask_SetModeFromISR(LedMode_t mode);

#ifdef __cplusplus
}
#endif

#endif


