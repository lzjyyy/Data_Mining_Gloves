#ifndef __KEYS_H
#define __KEYS_H

#include "stdint.h"
#include <stdio.h>

#define KEY_ON         GPIO_PIN_RESET   // 低电平 – 按下
#define KEY_OFF        GPIO_PIN_SET     // 高电平 – 松开
#define DEBOUNCE_MS    20U              // 去抖延时（毫秒）
#define LONG_PRESS_MS 2000U             // 长按判定阈值（毫秒）

uint8_t Startup_KeyCheck(void);
#endif 


