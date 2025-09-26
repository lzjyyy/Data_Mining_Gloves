#ifndef _TIMER5_H_
#define _TIMER5_H_

#include "timerscfg.h"
#include "applicfg.h"

void TIM5_Init(void);
void setTimer(TIMEVAL value);
TIMEVAL getElapsedTime(void);

#endif