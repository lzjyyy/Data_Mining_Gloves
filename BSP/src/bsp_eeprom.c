#include "bsp_eeprom.h"
#include "i2c.h"
#include "string.h"
#include "bsp_uart.h"
I2C_HandleTypeDef * EEPROM_I2C;
extern I2C_HandleTypeDef hi2c1;
void bsp_eeprom_init(void){
	EEPROM_I2C=&hi2c1;
}
static HAL_StatusTypeDef EEPROM_WaitStandby(void)
{
    // 等待EEPROM内部写操作完成（ACK polling）
    for (int i = 0; i < 100; i++) {
        if (HAL_I2C_IsDeviceReady(EEPROM_I2C, EEPROM_ADDR, 1, 10) == HAL_OK) {
            return HAL_OK;
        }
        HAL_Delay(5);
    }
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t memAddress, uint8_t data)
{
    uint8_t buf[3];
    buf[0] = (uint8_t)(memAddress >> 8);
    buf[1] = (uint8_t)(memAddress & 0xFF);
    buf[2] = data;

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(EEPROM_I2C, EEPROM_ADDR, buf, 3, 100);
    if (status != HAL_OK) return status;

    return EEPROM_WaitStandby();
}

HAL_StatusTypeDef EEPROM_WriteBytes(uint16_t memAddress, const uint8_t* data, uint16_t len)
{
    HAL_StatusTypeDef status;
    uint16_t bytesWritten = 0;

    while (bytesWritten < len) {
        uint16_t pageOffset = memAddress % EEPROM_PAGE_SIZE;
        uint16_t writeLen = EEPROM_PAGE_SIZE - pageOffset; // 当前页剩余空间
        if (writeLen > (len - bytesWritten)) {
            writeLen = len - bytesWritten;
        }

        uint8_t buf[EEPROM_PAGE_SIZE + 2];
        buf[0] = (uint8_t)(memAddress >> 8);
        buf[1] = (uint8_t)(memAddress & 0xFF);
        memcpy(&buf[2], &data[bytesWritten], writeLen);

        status = HAL_I2C_Master_Transmit(EEPROM_I2C, EEPROM_ADDR, buf, writeLen + 2, 100);
        if (status != HAL_OK) return status;

        status = EEPROM_WaitStandby();
        if (status != HAL_OK) return status;

        memAddress += writeLen;
        bytesWritten += writeLen;
    }

    return HAL_OK;
}

HAL_StatusTypeDef EEPROM_ReadByte(uint16_t memAddress, uint8_t* data)
{
    uint8_t addr[2];
    addr[0] = (uint8_t)(memAddress >> 8);
    addr[1] = (uint8_t)(memAddress & 0xFF);

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(EEPROM_I2C, EEPROM_ADDR, addr, 2, 100);
    if (status != HAL_OK) return status;

    status = HAL_I2C_Master_Receive(EEPROM_I2C, EEPROM_ADDR, data, 1, 100);
    return status;
}

HAL_StatusTypeDef EEPROM_ReadBytes(uint16_t memAddress, uint8_t* data, uint16_t len)
{
    uint8_t addr[2];
    addr[0] = (uint8_t)(memAddress >> 8);
    addr[1] = (uint8_t)(memAddress & 0xFF);

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(EEPROM_I2C, EEPROM_ADDR, addr, 2, 100);
    if (status != HAL_OK) return status;

    status = HAL_I2C_Master_Receive(EEPROM_I2C, EEPROM_ADDR, data, len, 500);
    return status;
}

HAL_StatusTypeDef EEPROM_WriteUint32_BigEndian(uint16_t memAddress, uint32_t value)
{
    uint8_t data[4];
    data[0] = (uint8_t)((value >> 24) & 0xFF);
    data[1] = (uint8_t)((value >> 16) & 0xFF);
    data[2] = (uint8_t)((value >> 8) & 0xFF);
    data[3] = (uint8_t)(value & 0xFF);
    return EEPROM_WriteBytes(memAddress, data, 4);
}

HAL_StatusTypeDef EEPROM_ReadUint32_BigEndian(uint16_t memAddress, uint32_t* value)
{
    uint8_t data[4];
    HAL_StatusTypeDef status = EEPROM_ReadBytes(memAddress, data, 4);
    if (status != HAL_OK) return status;

    *value = ((uint32_t)data[0] << 24) |
             ((uint32_t)data[1] << 16) |
             ((uint32_t)data[2] << 8)  |
             ((uint32_t)data[3]);
    return HAL_OK;
}

#define TEST_BLOCK_SIZE   128      // 每次写入读取的数据块大小

uint8_t write_buffer[TEST_BLOCK_SIZE];
uint8_t read_buffer[TEST_BLOCK_SIZE] __attribute__((aligned(4)));

void EEPROM_Test(void)
{
//    u1_printf(">>> EEPROM 32KB 测试开始...\r\n");

//    // 初始化写入数据
//    for (uint16_t i = 0; i < TEST_BLOCK_SIZE; i++) {
//        write_buffer[i] = i;
//    }

//    // 遍历整个 EEPROM 地址空间，每 TEST_BLOCK_SIZE 字节写入一次
//    for (uint16_t addr = 0; addr < EEPROM_SIZE; addr += TEST_BLOCK_SIZE) {
//        // 写入数据
//        if (EEPROM_WriteBytes(addr, write_buffer, TEST_BLOCK_SIZE) != 0) {
//            u1_printf("写入失败: Addr = 0x%04X\r\n", addr);
//            return;
//        }

//        HAL_Delay(5);  // 适当延时，确保 EEPROM 写入完成

//        // 读取数据
//        memset(read_buffer, 0, TEST_BLOCK_SIZE);
//        if (EEPROM_ReadBytes(addr, read_buffer, TEST_BLOCK_SIZE) != 0) {
//            u1_printf("读取失败: Addr = 0x%04X\r\n", addr);
//            return;
//        }

//        // 数据校验
//        if (memcmp(write_buffer, read_buffer, TEST_BLOCK_SIZE) != 0) {
//            u1_printf("校验失败: Addr = 0x%04X\r\n", addr);
//            for (uint16_t i = 0; i < TEST_BLOCK_SIZE; i++) {
//                u1_printf("W: %02X R: %02X\r\n", write_buffer[i], read_buffer[i]);
//            }
//            return;
//        } else {
//            u1_printf("地址 0x%04X 校验通过\r\n", addr);
//        }
//    }

        if (EEPROM_ReadBytes(0x04cf5, read_buffer, TEST_BLOCK_SIZE) ==HAL_OK) {
            print_hex_with_tag("读取的字节是：",read_buffer,TEST_BLOCK_SIZE);
        }
				else u1_printf("读取失败\r\n");
}
