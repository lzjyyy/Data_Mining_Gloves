/* Action.h */
#ifndef ACTION_H
#define ACTION_H

#include <stdint.h>
#include <stdbool.h>
#include "sysConfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 检测 posRef 数组是否与上一次不同
 * @param posRef 本次的目标位置数组
 * @return true  如果发生了改变（新动作）
 *         false 如果没有改变
 */
bool hasPosRefChanged(const float posRef[MOTOR_COUNT]); 




/**
 * @file    updateSubstepReferences.c
 * @brief   更新各电机子段参考，并在各动作之间添加1秒延时，完成所有动作后锁定输出。
 *
 * 每个电机的动作由多个子段构成，在每个动作（两子段）的结束处插入1秒延时。
 * 具体流程：
 *  1. 判断当前测量位置是否到达或越过目标位置（含容差）。
 *   1.1 若到达且未完成所有动作，检查是否处于动作边界且非最后动作：
 *      - 若满足，则进入延时阶段（100次100Hz循环，共1秒），保持当前位置不变；
 *  1.2 若延时中，则倒计时，延时结束后推进到下一子段或标记完成；
 *  1.3 否则普通推进到下一子段或标记完成。
 * 2. 输出阶段：若延时中或已完成，输出最后目标位置并设速度为0；
 *           否则输出当前子段目标位置和预设速度。
 *
 * @param   PosMea[]   实际测量位置数组，长度 MOTOR_COUNT
 * @param   PosRef[]   输出的目标参考位置数组，长度 MOTOR_COUNT
 * @param   SpdTar[]   输出的目标速度数组，长度 MOTOR_COUNT
 */
void updateSubstepReferences(float PosMea[],float PosRef[],float SpdTar[]);






#ifdef __cplusplus
}
#endif

#endif // ACTION_H


