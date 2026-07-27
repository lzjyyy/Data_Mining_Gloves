#ifndef AT24C256_H
#define AT24C256_H

#include "main.h"
#include "i2c.h"

#define I2C_HANDLE           hi2c1			 //这个宏定义便于后面程序切换不同的i2c通道
#define AT24CXX_ADDR_DEVICE  0X00A1      //设备地址
#define AT24CXX_ADDR_WRITE   0X00A0      //写命令
#define AT24CXX_ADDR_READ    0X00A0      //读命令
#define AT24CXX_SIZE_PAGE    0x0040      //描述每个page的大小，这里是64个字节
#define AT24CXX_SIZE_MEM (uint16_t)256   //存储大小

#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64		8189
#define AT24C128	16383
#define AT24C256	32767

#define BufferSize 	AT24C256						//芯片型号
  
																				//根据BufferSize宏的值来设置I2C_MEMADD_SIZE
#if   BufferSize == AT24C01
    #define I2C_MEMADD_SIZE I2C_MEMADD_SIZE_8BIT  
#elif BufferSize == AT24C02
	#define I2C_MEMADD_SIZE I2C_MEMADD_SIZE_8BIT  
#elif BufferSize == AT24C04
    #define I2C_MEMADD_SIZE I2C_MEMADD_SIZE_16BIT 
#elif BufferSize == AT24C08
    #define I2C_MEMADD_SIZE I2C_MEMADD_SIZE_16BIT  
#elif BufferSize == AT24C16
    #define I2C_MEMADD_SIZE I2C_MEMADD_SIZE_16BIT  		
#elif BufferSize == AT24C64
    #define I2C_MEMADD_SIZE I2C_MEMADD_SIZE_16BIT
#elif BufferSize == AT24C128
    #define I2C_MEMADD_SIZE I2C_MEMADD_SIZE_16BIT	
#elif BufferSize == AT24C256
	#define I2C_MEMADD_SIZE   I2C_MEMADD_SIZE_16BIT
#endif




/*用于设备是否准备好*/
HAL_StatusTypeDef AT24C_IsDeviceReady(uint16_t adrr); 




/**
 * @brief        AT24C02任意地址写一个字节数据
 * @param        memAddress —— 写数据的地址（0-255）--本次使用AT24C64，写数据的地址（0-8189），不同芯片大小在EEPROM_IIC_AT24CXX.h中有定义
 * @param        data  —— 存放准备写入的数据的地址
 * @retval       HAL_OK=0x00；HAL_ERROR=0x01；HAL_BUSY=0x02；HAL_TIMEOUT=0x03
*/
HAL_StatusTypeDef AT24Cxx_Write_One_Byte(uint16_t memAddress, uint8_t data);



/**
 * @brief        AT24CXX任意地址读一个字节数据
 * @param        memAddress —— 读数据的地址（0-255）
 * @param        data —— 存放读取到的数据的地址
 * @retval       HAL_OK=0x00；HAL_ERROR=0x01；HAL_BUSY=0x02；HAL_TIMEOUT=0x03
*/
HAL_StatusTypeDef AT24Cxx_Read_One_Byte(uint16_t memAddress, uint8_t *data); 



/**
 * @brief        AT24CXX任意地址连续写多个字节数据
 * @param        addr —— 写数据的地址（0-255）
 * @param        data —— 存放写入数据的地址
 * @retval       HAL_OK=0x00；HAL_ERROR=0x01；HAL_BUSY=0x02；HAL_TIMEOUT=0x03
*/
HAL_StatusTypeDef AT24Cxx_Write_Amount_Byte(uint16_t addr, uint8_t* data, uint16_t size);



/**
 * @brief        AT24CXX任意地址连续读多个字节数据
 * @param        addr —— 读数据的地址（0-255）
 * @param        data —— 存放读出数据的地址
 * @retval       HAL_OK=0x00；HAL_ERROR=0x01；HAL_BUSY=0x02；HAL_TIMEOUT=0x03
*/
HAL_StatusTypeDef AT24Cxx_Read_Amount_Byte(uint16_t addr, uint8_t* recv_buf, uint16_t size);



#endif

