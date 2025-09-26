#include "timer5.h"
#include "tim.h"

TIMEVAL last_counter_val = 0;
TIMEVAL elapsed_time = 0;

extern TIM_HandleTypeDef htim5;

// timer5 initial
void TIM5_Init(void)
{
	MX_TIM5_Init();  // CubeMX 生成的初始化函数 (tim.c)

	/* 修改计数器参数 */
	__HAL_TIM_SET_PRESCALER(&htim5, 840 - 1);     // 84MHz / 840 = 100kHz
	__HAL_TIM_SET_AUTORELOAD(&htim5, 65535);      // 最大计数周期

	__HAL_TIM_SET_COUNTER(&htim5, 1);             // 初始计数值

	__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);    // 清中断标志
	__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);   // 开启更新中断

	HAL_TIM_Base_Start_IT(&htim5);                // 启动定时器+中断
}

//Set the timer for the next alarm.
void setTimer(TIMEVAL value)
{
	uint32_t timer = __HAL_TIM_GET_COUNTER(&htim5);   // current val
	elapsed_time += timer - last_counter_val;

	last_counter_val = 65535 - value;
	__HAL_TIM_SET_COUNTER(&htim5, 65535 - value);

	__HAL_TIM_ENABLE(&htim5);                         // 确保定时器启用
}

//Return the elapsed time to tell the Stack how much time is spent since last call.
TIMEVAL getElapsedTime(void)
{
	uint32_t timer = __HAL_TIM_GET_COUNTER(&htim5);

	if (timer < last_counter_val)
		timer += 65535;

	TIMEVAL elapsed = timer - last_counter_val + elapsed_time;
	return elapsed;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM5)
	{
		last_counter_val = 0;
		elapsed_time = 0;
		TimeDispatch();
	}

	if (htim->Instance == TIM6) {
		    HAL_IncTick();
	}
}
		