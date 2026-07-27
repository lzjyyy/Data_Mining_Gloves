#ifndef __BSP_H_
#define __BSP_H_
#include "main.h"
#include "bsp_boot.h"
#include "bsp_eeprom.h"
#include "bsp_flash.h"
#include "bsp_frame.h"
#include "bsp_status.h"
#include "bsp_time.h"
#include "bsp_uart.h"

//所有函数的初始化
void bsp_init(void);
void frame_message_process(uint8_t flag);
#endif 
