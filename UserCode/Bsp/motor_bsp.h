#ifndef __MOTOR_BSP_H
#define __MOTOR_BSP_H

#include "stdint.h"
#include "sysConfig.h"

/** 电机编号 */
typedef enum {
    MOTOR1 = 0,   ///< 拇指1号 (TIM17_CH1, PE1)
    MOTOR2,       ///< 拇指2号 (TIM12_CH2, PE2)
    MOTOR3,       ///< 食指3号 (TIM14_CH1, PE3)
    MOTOR4,       ///< 中指4号 (TIM15_CH1, PE4)
    MOTOR5,       ///< 无名指5号 (TIM15_CH2, PE5)
    MOTOR6       ///< 小拇指6号 (TIM16_CH1, PE6)
} Motor_Id;

#define DUTY_MAX (1250)    //占空比最大值，由ARR值决定

/** 单路电机配置结构 */
typedef struct {
    float pitch_mm;       	///< 丝杆导程 (mm)
    float travel_min_rev10; ///< 行程最小值 (1/10圈)
    float travel_max_rev10; ///< 行程最大值 (1/10圈)
} MotorConfig_t;


/** 各路电机配置表，按 Motor_Id 下标访问 */
extern const MotorConfig_t motorCfg[MOTOR_COUNT];



/**
 * @brief  初始化所有电机：启动 PWM 并将 PH 置低
 */
void Motor_Init(void);

/**
 * @brief  设置电机转速与方向
 * @param  id    电机编号（0～5）
 * @param  duty  占空比，范围 -DUTY_MAX … +DUTY_MAX
 *              >0：正转（PH=1，EN=(duty/DUTY_MAX)%）
 *              <0：反转（PH=0，EN=|duty/DUTY_MAX|%）
 *               0：制动（PH=0/1 均可，EN=0）
 */
void Motor_SetDuty(Motor_Id id, int16_t duty);



#endif /* __MOTOR_DRIVER_H */



