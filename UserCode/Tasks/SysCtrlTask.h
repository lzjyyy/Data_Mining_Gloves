#ifndef __SYSCTRLTASK_H__
#define __SYSCTRLTASK_H__
#include "stdint.h"
#include <stdio.h>
#include "sysConfig.h"
#include <stdbool.h>


extern volatile uint8_t ota_reply_complete_flag;           //OTA升级回复完成标志
extern volatile uint8_t ota_write_eeprom_complete_flag;    //OTA写EEPROM完成标志
extern volatile uint8_t system_reset_request;               //系统复位请求标志
extern volatile bool enc_reset_req[MOTOR_COUNT];
extern volatile float sysctrl[MOTOR_COUNT];

void StartStoreTask(void *argument);
#endif



