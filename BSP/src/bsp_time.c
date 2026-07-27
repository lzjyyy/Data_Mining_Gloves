#include "bsp_time.h"

extern  TIM_HandleTypeDef htim6;
volatile uint8_t timer_6=0;
	
void bsp_time_init(void){

	HAL_TIM_Base_Start_IT(&htim6);  // 开启 TIM6 并使能中断
}

void set_Timer6_flag(uint8_t flag){
	timer_6=flag;
}

uint8_t get_Timer6_flag(void){
	
	return timer_6;
}
