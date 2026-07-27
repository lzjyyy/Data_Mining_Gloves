#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/**
 * @brief PID 对象结构体
 */
typedef struct {
    float Kp;          ///< 比例系数
    float Ki;          ///< 积分系数
    float Kd;          ///< 微分系数

    float deadzone;    ///< 死区带宽：|error| <= deadzone 时认为到位

    float dt;          ///< 采样周期 (秒)

    float integral;    ///< 积分累积值（已乘 dt）
    float prevError;   ///< 上一次误差
    float prevPrevError;///< 上上次误差（增量式用）

    float outMin;      ///< 输出限幅下限（位置式用）
    float outMax;      ///< 输出限幅上限（位置式用）

    float integralMin; ///< 积分累积下限
    float integralMax; ///< 积分累积上限
} PID_HandleTypeDef;

/**
 * @brief  初始化 PID 参数
 * @param  hpid           PID 对象指针
 * @param  Kp,Ki,Kd       三项系数
 * @param  deadzone       死区带宽
 * @param  dt             采样周期（秒）
 * @param  outMin,outMax  位置式输出限幅
 * @param  integralMin,integralMax  积分累积限幅
 */
void PID_Init(PID_HandleTypeDef *hpid,
              float Kp, float Ki, float Kd,
              float deadzone, float dt,
              float outMin, float outMax,
              float integralMin, float integralMax);

/**
 * @brief  重置 PID 内部状态（清积分和历史误差）
 */
void PID_Reset(PID_HandleTypeDef *hpid);

/**
 * @brief  位置式 PID 计算
 * @param  hpid        PID 对象
 * @param  setpoint    目标值
 * @param  measurement 测量值
 * @return 限幅后的控制输出
 */
float PID_CalcPosition(PID_HandleTypeDef *hpid,
                       float setpoint,
                       float measurement);

/**
 * @brief  增量式 PID 计算
 * @param  hpid        PID 对象
 * @param  setpoint    目标值
 * @param  measurement 测量值
 * @return 本次输出增量（需要累加到上次输出上）
 */
float PID_CalcIncremental(PID_HandleTypeDef *hpid,
                          float setpoint,
                          float measurement);

#endif // __PID_H
