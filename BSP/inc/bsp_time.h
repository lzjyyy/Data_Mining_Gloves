#ifndef __BSP_TIME_H
#define __BSP_TIME_H
#include "main.h"

void bsp_time_init(void);
uint8_t get_Timer6_flag(void);
void set_Timer6_flag(uint8_t flag);

#endif 
