#ifndef __W5500_PORT_HAL_H__
#define __W5500_PORT_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"
#include "wizchip_conf.h"
#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>

#define W5500_SPI_HANDLE hspi1
#define W5500_CS_PORT  NET1_CS_GPIO_Port
#define W5500_CS_PIN   NET1_CS_Pin
#define W5500_RST_PORT  NET1_RST_GPIO_Port
#define W5500_RST_PIN   NET1_RST_Pin

    /* 定义该宏则表示使用自动协商模式，取消则设置为100M全双工模式 */
#define USE_AUTONEGO

/* 定义该宏则表示在初始化网络信息时设置DHCP */
//#define USE_DHCP

    extern SPI_HandleTypeDef W5500_SPI_HANDLE;

    void do_tcpc(void);
    void W5500_ChipInit(void);

#ifdef __cplusplus
}
#endif

#endif