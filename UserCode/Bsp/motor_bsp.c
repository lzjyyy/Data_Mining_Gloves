#include "motor_bsp.h"
#include "main.h"
#include "tim.h"  // 假设使用定时器PWM输出，需根据实际项目修改

#include "motor_bsp.h"
#include "tim.h"   // CubeMX 生成的定时器句柄 extern
#include "gpio.h"  // CubeMX 生成的 GPIO 宏：MOTx_PH_GPIO_Port/MOTx_PH_Pin
#include "stm32h7xx_hal.h"
#include "sysConfig.h"

/// 定义 PWM 定时器句柄数组
static TIM_HandleTypeDef* const motor_htim[MOTOR_COUNT] = {
    &htim17,  // MOTOR1
    &htim12,  // MOTOR2
    &htim14,  // MOTOR3
    &htim15,  // MOTOR4
    &htim15,  // MOTOR5 (同 TIM15，不同通道)
    &htim16   // MOTOR6
};

/// 定义 PWM 通道号数组
static const uint32_t motor_channel[MOTOR_COUNT] = {
    TIM_CHANNEL_1,  // MOTOR1
    TIM_CHANNEL_2,  // MOTOR2
    TIM_CHANNEL_1,  // MOTOR3
    TIM_CHANNEL_1,  // MOTOR4
    TIM_CHANNEL_2,  // MOTOR5
    TIM_CHANNEL_1   // MOTOR6
};

/// 定义 PH 引脚数组
static GPIO_TypeDef* const motor_ph_port[MOTOR_COUNT] = {
    MOT1_PH_GPIO_Port,
    MOT2_PH_GPIO_Port,
    MOT3_PH_GPIO_Port,
    MOT4_PH_GPIO_Port,
    MOT5_PH_GPIO_Port,
    MOT6_PH_GPIO_Port
};

static const uint16_t motor_ph_pin[MOTOR_COUNT] = {
    MOT1_PH_Pin,
    MOT2_PH_Pin,
    MOT3_PH_Pin,
    MOT4_PH_Pin,
    MOT5_PH_Pin,
    MOT6_PH_Pin
};


/** 各路电机参数（按 Motor_Id 枚举顺序） */
const MotorConfig_t motorCfg[MOTOR_COUNT] = {
    { 3.0f,   0.0f,   1028.48f },   // MOTOR1 拇指1号
    { 3.0f,   0.0f,   2997.65f },   // MOTOR2 拇指2号
    { 5.0f,   0.0f,   2929.92f },   // MOTOR3 食指
    { 5.0f,   0.0f,   2929.92f },   // MOTOR4 中指
    { 5.0f,   0.0f,   2929.92f },   // MOTOR5 无名指
    { 5.0f,   0.0f,   2929.92f }    // MOTOR6 小拇指
};



/*------------------------------------------------------------------------------------------------------------------------------*/
void Motor_Init(void)
{
	for (int i = 0; i < MOTOR_COUNT; i++) 
	{
		HAL_TIM_PWM_Start(motor_htim[i], motor_channel[i]);								   //启动 PWM 输出
		HAL_GPIO_WritePin(motor_ph_port[i], motor_ph_pin[i], GPIO_PIN_RESET);//初始方向置低，制动状态
		__HAL_TIM_SET_COMPARE(motor_htim[i], motor_channel[i], 0);					 //占空比0
	}
}

/*-------------------------------------------------------------------------------------------------------------------------------*/
static const int8_t motor_dir[6] = {
    -1,   
    -1,   
    -1,
     -1,
     -1,
     -1
};

void Motor_SetDuty(Motor_Id id, int16_t duty)
{
	/*① 检查是否是1~6号电机*/
	if(id<0 || id>=MOTOR_COUNT)
	{
		return;
	}	
	
	duty=motor_dir[id]*duty;
	/*② 占空比限幅*/
	if(duty>DUTY_MAX)// 限幅至 [-DUTY_MAX, +DUTY_MAX]
	{
		duty = DUTY_MAX;
	}			
	else if(duty<-DUTY_MAX)
	{
		duty = -DUTY_MAX;
	}	
//	/*③ 检查是否是1~6号电机*/	
//	uint32_t absDuty = (uint32_t)(duty < 0 ? -duty : duty);// 取绝对值占空比 
//	uint32_t period = __HAL_TIM_GET_AUTORELOAD(motor_htim[id]);// 载入定时器自动重装值
//	uint32_t pulse = (absDuty * (period + 1)) / 100U; // 计算 CCR：0…period，对应 0…100%

	if(duty > 0) 
	{
		HAL_GPIO_WritePin(motor_ph_port[id], motor_ph_pin[id], GPIO_PIN_SET);  //正转：PH = 1，EN = pulse
		__HAL_TIM_SET_COMPARE(motor_htim[id], motor_channel[id], duty);
	}
	else if(duty < 0) 
	{
		HAL_GPIO_WritePin(motor_ph_port[id], motor_ph_pin[id], GPIO_PIN_RESET);//反转：PH = 0，EN = pulse
		__HAL_TIM_SET_COMPARE(motor_htim[id], motor_channel[id], -duty);			 //先前这边duty没有加负号，导致目标值为负时候PID调不出来，振荡
	}
	else 
	{     
		__HAL_TIM_SET_COMPARE(motor_htim[id], motor_channel[id], 0);           // duty == 0：制动（PH 任意，只要 EN=0 即可）
		HAL_GPIO_WritePin(motor_ph_port[id], motor_ph_pin[id], GPIO_PIN_RESET);// 可选：保持 PH 不变，或统一置低
	}
}
