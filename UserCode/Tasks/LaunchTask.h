#ifndef __LAUNCHTASK_H
#define __LAUNCHTASK_H
#include "cmsis_os2.h" 
#include "sysConfig.h"

/*** 事件组定义 ***/
extern osEventFlagsId_t LaunchEventsHandle;

/* LaunchEventsHandle 里用到的所有位 */
#define LAUNCH_DONE_BIT   (1U << 0)  // 发布后，StartPosLoopTask 跑起来
#define CALIB_DONE_BIT    (1U << 1)  // SysCtrlTask 校准完成时发出
#define MODE_CALIB_BIT    (1U << 2)  // 启动时选择校准模式
#define MODE_NORMAL_BIT   (1U << 3)  // 启动时选择正常模式


extern float startMotorPosFloat[MOTOR_COUNT];

uint8_t GenshinStarted(void);
void StartLaunchTask(void *argument);



#endif


