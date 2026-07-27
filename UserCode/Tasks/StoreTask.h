#ifndef __STORETASK_H__
#define __STORETASK_H__



#include "stdint.h"
#include <stdio.h>
#include "cmsis_os2.h" 

#define EVT_485ADDR_BIT   (1U << 0)
#define EVT_485BAUD_BIT    (1U << 1)
#define EVT_OTA_FLAG_BIT  (1U << 2)
#define EVT_ALL_BITS  (EVT_485ADDR_BIT | EVT_485BAUD_BIT | EVT_OTA_FLAG_BIT)

#define EEPROM_UPGRADEFLAGE_ADDR 0x7fCC //EEPROM中升级标志存储地址

extern osEventFlagsId_t StoreEventsHandle;
void Rs485Config_InitFlagCheck(void);
void StoreTask_Read485ConfigFromEEPROM(void);

void StartStoreTask(void *argument);
#endif



