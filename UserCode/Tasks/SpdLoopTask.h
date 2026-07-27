/* SpdLoopTask.h */

#ifndef __SPDLOOPTASK_H__
#define __SPDLOOPTASK_H__

#include "cmsis_os2.h"
#include "EncSenseTask.h"  /* 拿到 ENCODER_COUNT、encBuf、pulsePerRev 等 */
#include "sysConfig.h"

extern osThreadId_t SpdLoopTaskHandle;


/**
 * @brief 速度＋位置计算任务；1 kHz 被 EncDataHandle() 用 osThreadFlagsSet 唤醒
 */
void StartSpdLoopTask(void *argument);

void SpeedControl_Init(void);

#endif /* __SPDLOOPTASK_H__ */
