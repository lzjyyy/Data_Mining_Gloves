#ifndef __BSP_FLASH__H
#define __BSP_FLASH__H
#include "main.h"
#include "bsp_status.h"

uint8_t Flash_EraseSectors1To7(void);
uint8_t Flash_Write128BytesWithCheck(uint32_t addr, uint8_t *data, uint32_t length);
uint32_t BSP_CalculateCRC32(uint8_t *data, uint32_t length,uint32_t pre_crc32);
uint32_t BSP_HW_CalculateCRC32(uint32_t *data, uint32_t word_len, uint32_t init_val);

#endif
