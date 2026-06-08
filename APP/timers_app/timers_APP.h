#ifndef __TIM_APP_H__
#define __TIM_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void Timers_APP_Init(void);
uint8_t Timers_APP_TakeLcdRefreshEvent(void);
void Timers_APP_OnPeriodElapsed(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif
