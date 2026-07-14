#ifndef __W5500_PORT_H__
#define __W5500_PORT_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "stm32f4xx_hal.h"
#include "wizchip_conf.h"
#include "socket.h"

#define W5500_CS_GPIO_Port NET1_CS_GPIO_Port
#define W5500_CS_Pin NET1_CS_Pin
#define W5500_RST_GPIO_Port NET1_RST_GPIO_Port
#define W5500_RST_Pin NET1_RST_Pin
#define MAX_LINK_CHK_CNT    50
#define W5500_PHYCFGR      0x002E
#define PHYCFGR_LNK  0x01


    extern SPI_HandleTypeDef hspi2;

    void W5500_HwReset(void);
    int W5500_DriverInit(void);
    void W5500_NetInfo_SetStatic(void); // 静态IP设置
    void W5500_PrintNetInfo(void);
    void W5500_RaiseSpiSpeed(void); // 初始化后提升 SPI 速率（可选）
    void W5500_TCP_EchoServer_Loop(void);
    int W5500_TCP_Connect_Debug(uint8_t sock, uint8_t* ip, uint16_t port, uint32_t timeout_ms);
    uint8_t W5500_Get_PHYCFGR(void);
    void W5500_PrintPhyStatus(const char* tag);
    void W5500_SoftReset(void);
    int W5500_WaitForLink(void);
    int W5500_Init(void);
    uint8_t get_w5500_init_status(void);
#ifdef __cplusplus
}
#endif
#endif
