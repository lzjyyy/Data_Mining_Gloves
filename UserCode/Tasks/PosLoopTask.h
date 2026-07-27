#ifndef __POSLOOPTASK_H__
#define __POSLOOPTASK_H__
#include "stdint.h"
#include <stdio.h>
#include "sysConfig.h"

#define POS_FREQ_HZ           100                  // 100Hz
#define POS_PERIOD_MS         (1000 / POS_FREQ_HZ) // 10ms
#define POS_PERIOD_SEC        (1.0f / POS_FREQ_HZ) // 0.01s


extern float posRef[MOTOR_COUNT];
extern float spdTar[MOTOR_COUNT];
extern uint8_t valid[MOTOR_COUNT];

void PositionControl_Init(void);
void StartPosLoopTask(void *argument);
#endif

