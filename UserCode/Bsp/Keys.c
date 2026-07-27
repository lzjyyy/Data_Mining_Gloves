#include "Keys.h"
#include "stdint.h"
#include <stdio.h>
#include "gpio.h"



/**
 * @brief  上电后简单检测一次按键：根据按下时长区分短按/长按
 * @retval 0 = 短按（进入正常模式）
 *         1 = 长按（进入复位模式）
 */
uint8_t Startup_KeyCheck(void)
{
	GPIO_TypeDef* port = KEY1_GPIO_Port;
	uint16_t      pin  = KEY1_Pin;
	uint32_t t0, dt;
	/*—— 等待按下 ——*/ 
	while (HAL_GPIO_ReadPin(port, pin) == KEY_OFF)
	{
printf("上电高电平等待中\n");													//一直阻塞，直到检测到按键按下
	}
	HAL_Delay(DEBOUNCE_MS);
	/*—— 测量按下时长 ——*/   
	t0 = HAL_GetTick();
	while (HAL_GPIO_ReadPin(port, pin) == KEY_ON)
	{
printf("按键被按下\n");													//一直阻塞，直到松开
	}
	dt = HAL_GetTick() - t0;
	HAL_Delay(DEBOUNCE_MS);
printf("dt：%d\n",dt);	
	/*—— 判断长短按 ——*/   
	if (dt >= LONG_PRESS_MS)
	{
			return 1;  					//长按
	}
	else
	{
			return 0;  					//短按
	}
}


