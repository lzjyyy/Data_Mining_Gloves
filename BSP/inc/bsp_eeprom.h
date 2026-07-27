#ifndef EEPROM_I2C_H
#define EEPROM_I2C_H

#include "main.h"

#define EEPROM_ADDR 0xA0          // 7位器件地址左移1位，实际7bit地址是0x50

#define EEPROM_PAGE_SIZE 64       // AT24C256的页大小，64字节
#define EEPROM_SIZE 32768         // EEPROM 容量32KB

//FILE文件地址相关信息
#define EEPROM_APP_READY  0x7FBC
#define EEPROM_APP_FLAG   0x7FBD
#define EEPROM_FILE_LENGTH_ADDR  0x7FC0
#define EEPROM_CRC32_ADDR        0x7FC4
#define EEPROM_WRITTEN_LEN_ADDR  0x7FC8
#define EEPROM_UPGRADEFLAGE_ADDR 0x7fCC
#define EEPROM_SLAVE_ADDR        64
#define EEPROM_SLAVE_BAUD        65

//IIC接口初始化
void bsp_eeprom_init(void);
// 单字节写入
HAL_StatusTypeDef EEPROM_WriteByte(uint16_t memAddress, uint8_t data);

// 多字节写入，自动分页处理
HAL_StatusTypeDef EEPROM_WriteBytes(uint16_t memAddress, const uint8_t* data, uint16_t len);

// 单字节读取
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t memAddress, uint8_t* data);

// 多字节读取
HAL_StatusTypeDef EEPROM_ReadBytes(uint16_t memAddress, uint8_t* data, uint16_t len);

// 以大端格式写入 uint32_t
HAL_StatusTypeDef EEPROM_WriteUint32_BigEndian(uint16_t memAddress, uint32_t value);

// 以大端格式读取 uint32_t
HAL_StatusTypeDef EEPROM_ReadUint32_BigEndian(uint16_t memAddress, uint32_t* value);

void EEPROM_Test(void);
#endif
