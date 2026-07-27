#ifndef __BSP_boot_H
#define __BSP_boot_H
#include "main.h" 
void bootloader_clear(void);
void bootToUserAPP(void);
void bsp_boot_init(void);
uint8_t APP_REAY(void);
uint8_t Boot_CheckUpgradeFlag(void);
#endif
