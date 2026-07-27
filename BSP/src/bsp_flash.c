#include "bsp_flash.h"
#include "stm32h7xx_hal.h"
#include "string.h"
#include "bsp_uart.h"
static __attribute__((aligned(32))) uint8_t flash_buf[32];

uint8_t Flash_Write128BytesWithCheck(uint32_t addr, uint8_t *data, uint32_t length) {
    // 1. 数据长度校验
    if (length <32||length>128) {
        return FLASH_ERR_LEN;
    }

    // 2. 目标地址合法性校验
    if (addr < FLASH_USER_START_ADDR || (addr + 127) > FLASH_USER_END_ADDR) {
        return FLASH_ERR_ADDR_RANGE;
    }

    // 3. 地址对齐校验（32字节对齐）
    if ((addr & 0x1F) != 0) { // 0x1F = 31，与运算不为0表示未对齐
        return FLASH_ERR_ADDR_ALIGN;
    }

    // 4. 数据对齐校验（32字节对齐）
    if ((length % 32) != 0) {
        return FLASH_ERR_DATA_ALIGN;
    }

////    // 5. 计算原始数据的CRC32（128字节 = 32个32位字）
////    uint32_t crc_original = CalculateCRC32((uint32_t*)data, 128);
////    if (crc_original == 0xFFFFFFFF) {
////        return FLASH_ERR_CRC; // CRC计算失败
////    }

    // 6. 解锁Flash
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return FLASH_ERR_UNLOCK;
    }



    // 7. 分4块写入128字节数据（每块32字节）
    HAL_StatusTypeDef status;
    for (uint8_t i = 0; i < length/32; i++) {
    uint32_t write_addr = addr + i * 32;
    memcpy(flash_buf,(uint8_t *) &data[i * 32], 32);
		//u1_printf("写入的地址是：%d",write_addr);
    //uint32_t *write_data = (uint32_t *)flash_buf;
		
		__disable_irq();
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, write_addr,(uint32_t)flash_buf );
		  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_4);  
		//status = HAL_OK;
		__enable_irq();
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return FLASH_ERR_PROGRAM;
    }
}

    // 9. 锁定Flash
    HAL_FLASH_Lock();


    return FLASH_OK;
}


   
/**
 * @brief 擦除STM32H723的扇区1至扇区7
 * @param voltage_range：供电电压范围
 * @return 错误码
 */
uint8_t Flash_EraseSectors1To7(void) {
   if (HAL_FLASH_Unlock() != HAL_OK) {
        u1_printf("Flash解锁失败\r\n");
        return FLASH_ERASE_ERR_UNLOCK;
    }
    u1_printf("Flash解锁成功\r\n");

    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t error_sector = 0;

    erase_init.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks        = FLASH_BANK_1;          // 指定 Bank1
    erase_init.Sector       = FLASH_SECTOR_1;        // 起始扇区：1
    erase_init.NbSectors    = 7;                     // 连续 7 个扇区 (1~7)
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7–3.6V

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &error_sector);
    u1_printf("擦除操作执行中...\r\n");
		FLASH_WaitForLastOperation(8000,FLASH_BANK_1);
    HAL_FLASH_Lock();
		

    if (status != HAL_OK) {
        u1_printf("扇区擦除失败，出错扇区: %lu\r\n", error_sector);
        return FLASH_ERASE_ERR_PROGRAM;
    }

    u1_printf("扇区1~7擦除成功\r\n");
    return FLASH_OK;
}

/**
 * @brief 软件实现的CRC32计算（支持累加模式）
 * @param data: 待计算的数据（32位指针，数据按字节处理）
 * @param len: 数据长度（字节）
 * @param pre_crc32: 前一次计算的CRC结果（初始计算传入0xFFFFFFFF）
 * @return 累加后的CRC32值
 */
uint32_t BSP_CalculateCRC32(uint8_t *data, uint32_t length,uint32_t pre_crc32) {
    volatile uint32_t crc = pre_crc32;
		for (uint32_t i = 0; i < length; i++) {
        crc ^= ((uint32_t)data[i] << 24);  // 与 CRC 高 8 位异或（big-endian）
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }
    return crc;

}


/**
 * @brief 硬件CRC32计算（支持自定义初始值）
 * @param data: 待计算数据指针（必须32位对齐）
 * @param word_len: 数据长度（单位：32位字，即4字节）
 * @param init_val: 初始值（首次计算可传入0xFFFFFFFF，或上一次计算结果）
 * @return 计算后的CRC32值，失败返回0xFFFFFFFF
 */
uint32_t BSP_HW_CalculateCRC32(uint32_t *data, uint32_t word_len, uint32_t init_val) {


    // 2. 设置初始值（覆盖默认初始值）
    hcrc.Instance->INIT = init_val;

    // 3. 执行CRC计算
    uint32_t crc_result = HAL_CRC_Accumulate(&hcrc, data, word_len);

    // 4. 返回计算结果
    return crc_result;
}


  




