#include "AT24C256.h"
#include "cmsis_os2.h" 

#define TIMEOUT (0XFFFF)  //等待时长
#define TRIALS  (10)      //尝试的次数


/*--------------------------------------------------------------------------------------------------------------------------检查地址为adrr的设备是否做好了通讯准备*/
HAL_StatusTypeDef AT24C_IsDeviceReady(uint16_t adrr) 
{
	return HAL_I2C_IsDeviceReady(&I2C_HANDLE, adrr, TRIALS, TIMEOUT);
}


/*--------------------------------------------------------------------------------------------------------------------------1次写1个字节*/
/**
 * @brief        AT24C02任意地址写一个字节数据
 * @param        memAddress —— 写数据的地址（0-255）--本次使用AT24C64，写数据的地址（0-8189），不同芯片大小在EEPROM_IIC_AT24CXX.h中有定义
 * @param        data  —— 存放准备写入的数据的地址
 * @retval       HAL_OK=0x00；HAL_ERROR=0x01；HAL_BUSY=0x02；HAL_TIMEOUT=0x03
*/
HAL_StatusTypeDef AT24Cxx_Write_One_Byte(uint16_t memAddress, uint8_t data) 
{
	return HAL_I2C_Mem_Write(&I2C_HANDLE, AT24CXX_ADDR_WRITE, memAddress, I2C_MEMADD_SIZE, &data, 1, TIMEOUT);
}




/*--------------------------------------------------------------------------------------------------------------------------1次读1个字节*/
/**
 * @brief        AT24CXX任意地址读一个字节数据
 * @param        memAddress —— 读数据的地址（0-255）
 * @param        data —— 存放读取到的数据的地址
 * @retval       HAL_OK=0x00；HAL_ERROR=0x01；HAL_BUSY=0x02；HAL_TIMEOUT=0x03
*/
HAL_StatusTypeDef AT24Cxx_Read_One_Byte(uint16_t memAddress, uint8_t *data) 
{
	return HAL_I2C_Mem_Read(&I2C_HANDLE, AT24CXX_ADDR_READ, memAddress, I2C_MEMADD_SIZE, data, 1, TIMEOUT);
}



/*--------------------------------------------------------------------------------------------------------------------------1次写size个字节*/
/**
 * @brief        AT24CXX任意地址连续写多个字节数据
 * @param        addr —— 写数据的地址（0-255）
 * @param        data  —— 存放写入数据的地址
 * @retval       HAL_OK=0x00；HAL_ERROR=0x01；HAL_BUSY=0x02；HAL_TIMEOUT=0x03
*/
HAL_StatusTypeDef AT24Cxx_Write_Amount_Byte(uint16_t addr, uint8_t* data, uint16_t size)
{
    uint8_t i = 0;
    uint16_t cnt = 0;          			   // 写入字节计数
    HAL_StatusTypeDef result;  			   // 返回是否写入成功
	
    if(0 == addr % AT24CXX_SIZE_PAGE)  // 起始地址刚好是页开始地址
    {
        if(size <= AT24CXX_SIZE_PAGE)  // 写入的字节数小于页字节数
        {
            result = HAL_I2C_Mem_Write(&I2C_HANDLE, AT24CXX_ADDR_WRITE, addr, I2C_MEMADD_SIZE, data, size, TIMEOUT);// 写入的字节数不大于一页，直接写入
            osDelay(20);    			     // 写完size个字节，延迟久一点
            return result;
        }
        else
        {
            for(i = 0; i < size/AT24CXX_SIZE_PAGE; i++)// 写入的字节数大于一页，先将整页循环写入
            {
                HAL_I2C_Mem_Write(&I2C_HANDLE, AT24CXX_ADDR_WRITE, addr, I2C_MEMADD_SIZE, &data[cnt], AT24CXX_SIZE_PAGE, TIMEOUT);
                osDelay(20);    		   // 写完一页字节，延迟久一点
                addr += AT24CXX_SIZE_PAGE;
                cnt += AT24CXX_SIZE_PAGE;
            }
            result = HAL_I2C_Mem_Write(&I2C_HANDLE, AT24CXX_ADDR_WRITE, addr, I2C_MEMADD_SIZE, &data[cnt], size - cnt, TIMEOUT);// 将剩余的字节写入
            osDelay(20);               // 写完剩下不足一页的字节，延迟久一点
            return result;
        }
    }
    else                               // 起始地址偏离页开始地址
    {
        if(size <= (AT24CXX_SIZE_PAGE - addr % AT24CXX_SIZE_PAGE))			 //在该页可以写完
        {
            result = HAL_I2C_Mem_Write(&I2C_HANDLE, AT24CXX_ADDR_WRITE, addr, I2C_MEMADD_SIZE, data, size, TIMEOUT);
            osDelay(20);               // 写完size个字节，延迟久一点
            return result;
        }
        else													 // 该页写不完
        {
            cnt += AT24CXX_SIZE_PAGE - addr % AT24CXX_SIZE_PAGE;         // 先将该页写完
            HAL_I2C_Mem_Write(&I2C_HANDLE, AT24CXX_ADDR_WRITE, addr, I2C_MEMADD_SIZE, data, cnt, TIMEOUT);
            osDelay(20);    				   // 写完cnt个字节，延迟久一点
            addr += cnt;
            for(i = 0;i < (size - cnt) / AT24CXX_SIZE_PAGE; i++)				 // 循环写整页数据
            {
                HAL_I2C_Mem_Write(&I2C_HANDLE, AT24CXX_ADDR_WRITE, addr, I2C_MEMADD_SIZE, &data[cnt], AT24CXX_SIZE_PAGE, TIMEOUT);
                osDelay(20);           // 写完八个字节，延迟久一点
                addr += AT24CXX_SIZE_PAGE;
                cnt += AT24CXX_SIZE_PAGE;
            }
            result = HAL_I2C_Mem_Write(&I2C_HANDLE, AT24CXX_ADDR_WRITE, addr, I2C_MEMADD_SIZE, &data[cnt], size - cnt, TIMEOUT);// 将剩下的字节写入
            osDelay(20);               // 写完八个字节（最多八个字节），延迟久一点
            return result;
        }            
    }
}



/*--------------------------------------------------------------------------------------------------------------------------1次读size个字节*/
/**
 * @brief        AT24CXX任意地址连续读多个字节数据
 * @param        addr —— 读数据的地址（0-255）
 * @param        data —— 存放读出数据的地址
 * @retval       HAL_OK=0x00；HAL_ERROR=0x01；HAL_BUSY=0x02；HAL_TIMEOUT=0x03
*/
HAL_StatusTypeDef AT24Cxx_Read_Amount_Byte(uint16_t addr, uint8_t* recv_buf, uint16_t size)
{
    return HAL_I2C_Mem_Read(&I2C_HANDLE, AT24CXX_ADDR_READ, addr, I2C_MEMADD_SIZE, recv_buf, size, 0xFFFF);
}
