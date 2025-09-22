#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#include "stdint.h"

// 夹爪寄存器定义
#define REG_ENABLE      0x0100
#define REG_POS_HIGH    0x0102
#define REG_POS_LOW     0x0103
#define REG_SPEED       0x0104
#define REG_TORQUE      0x0105
#define REG_TRIGGER     0x0108
#define REG_REALTIME_POS_HIGH   0x0609
#define REG_POS_REACHED         0x0602
#define REG_TORQUE_REACHED      0x0601
#define REG_SPEED_REACHED       0x0603
#define REG_WARNING_INFO        0x0612

#endif
