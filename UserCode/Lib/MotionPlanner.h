#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <stdbool.h>

/**
 * @brief 电机总数量，与 PosLoopTask.c 中的 MOTOR_COUNT 保持一致。
 */
#ifndef MOTOR_COUNT
#define MOTOR_COUNT 6
#endif

/**
 * @brief  动作序列初始化
 *
 * 在程序启动时调用一次，让内部状态机进入初始就绪状态。
 */
void MotionPlanner_Init(void);

/**
 * @brief  动作序列每帧更新接口
 *
 * @param posMea[MOTOR_COUNT]  输入：当前 6 路电机的测量位置（单位同编码器映射，一般是 1/10 圈）
 * @param spdRef[MOTOR_COUNT]  输出：本帧 6 路电机的速度参考（signed float）
 *
 * @return 如果当前“动作库”中的所有步骤都跑完一遍，并且自动轮回到第 0 步，则返回 true；
 *         否则返回 false。用户可根据此返回值判断一轮动作是否刚刚结束。
 */
bool MotionPlanner_Update(const float posMea[MOTOR_COUNT], float spdRef[MOTOR_COUNT]);

#endif // MOTION_PLANNER_H
