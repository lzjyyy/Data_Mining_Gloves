#include "pid.h"

/**
 * @brief  初始化 PID 参数
 */
void PID_Init(PID_HandleTypeDef *hpid,
              float Kp, float Ki, float Kd,
              float deadzone, float dt,
              float outMin, float outMax,
              float integralMin, float integralMax)
{
    hpid->Kp = Kp;
    hpid->Ki = Ki;
    hpid->Kd = Kd;
    hpid->deadzone = deadzone;
    hpid->dt = dt;

    hpid->integral = 0.0f;
    hpid->prevError = 0.0f;
    hpid->prevPrevError = 0.0f;

    hpid->outMin = outMin;
    hpid->outMax = outMax;
    hpid->integralMin = integralMin;
    hpid->integralMax = integralMax;
}

/**
 * @brief  重置 PID 内部状态
 */
void PID_Reset(PID_HandleTypeDef *hpid)
{
    hpid->integral = 0.0f;
    hpid->prevError = 0.0f;
    hpid->prevPrevError = 0.0f;
}

/**
 * @brief  位置式 PID 计算
 */
float PID_CalcPosition(PID_HandleTypeDef *hpid,
                       float setpoint,
                       float measurement)
{
    float error = setpoint - measurement;

    // 死区内认为到位：清积分 & 历史误差，返回 0
    if (fabsf(error) <= hpid->deadzone) {
        hpid->integral  = 0.0f;
        hpid->prevError = error;
        return 0.0f;
    }

    // 比例项
    float Pout = hpid->Kp * error;

    // 积分项（累积 dt，限幅）
    hpid->integral += error * hpid->dt;
    if (hpid->integral > hpid->integralMax) hpid->integral = hpid->integralMax;
    else if (hpid->integral < hpid->integralMin) hpid->integral = hpid->integralMin;
    float Iout = hpid->Ki * hpid->integral;

    // 微分项（归一化 dt）
    float derivative = (error - hpid->prevError) / hpid->dt;
    float Dout = hpid->Kd * derivative;

    // 合成输出 & 限幅
    float out = Pout + Iout + Dout;
    if (out > hpid->outMax) out = hpid->outMax;
    else if (out < hpid->outMin) out = hpid->outMin;

    // 保存历史误差
    hpid->prevError = error;

    return out;
}

/**
 * @brief  增量式 PID 计算
 */
float PID_CalcIncremental(PID_HandleTypeDef *hpid,
                          float setpoint,
                          float measurement)
{
    float error = setpoint - measurement;

    // 死区内不累加，也更新历史误差
    if (fabsf(error) <= hpid->deadzone) {
        hpid->prevPrevError = hpid->prevError;
        hpid->prevError     = error;
        return 0.0f;
    }

    // Δ误差
    float dError = error - hpid->prevError;
    // 积分增量（dt归一化）
    float Iout = hpid->Ki * error * hpid->dt;
    // 二阶差分作为微分增量
    float ddError = (error - 2.0f * hpid->prevError + hpid->prevPrevError) / hpid->dt;
    float Dout = hpid->Kd * ddError;
    // 增量输出 = Kp*dError + 积分增量 + 微分增量
    float delta = hpid->Kp * dError + Iout + Dout;
    // 更新历史误差链
    hpid->prevPrevError = hpid->prevError;
    hpid->prevError     = error;

    return delta;
}
