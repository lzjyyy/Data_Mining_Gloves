#include "TestTask.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "stdint.h"
#include <stdio.h>

#include "bsp_fdcan.h"
#include "LedTask.h"


void StartTestTask(void *argument)
{



    for (;;)
    {
//				LedTask_SetMode(LED_MODE_BOOT);
//				osDelay(1000);
//				LedTask_SetMode(LED_MODE_RUN);
//				osDelay(1000);
//			  LedTask_SetMode(LED_MODE_IDLE);

        osDelay(1000);
    }
}


