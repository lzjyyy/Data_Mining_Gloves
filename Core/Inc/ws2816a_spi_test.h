#ifndef __WS2816A_SPI_H
#define __WS2816A_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ========= 用户配置区 ========= */

/* 灯珠数量 */
#ifndef WS2816A_LED_NUM
#define WS2816A_LED_NUM    2
#endif

/* 你 CubeMX 生成的 SPI 句柄 */
extern SPI_HandleTypeDef hspi3;

/* 用哪个 SPI */
#define WS2816A_SPI_HANDLE   hspi3

/* reset 低电平时间 > 280us
   4MHz下，1字节=8bit=2us
   150字节约 300us，够用了 */
#define WS2816A_RESET_BYTES  150

/* ========= 数据结构 ========= */

typedef struct
{
    uint16_t r;
    uint16_t g;
    uint16_t b;
} WS2816A_Color_t;

typedef struct
{
    uint8_t ig;   /* Green gain: 0~31 */
    uint8_t ir;   /* Red gain:   0~31 */
    uint8_t ib;   /* Blue gain:  0~31 */
} WS2816A_Gain_t;

/* ========= 接口 ========= */

void WS2816A_SPI_Init(void);
void WS2816A_ClearAll(void);
void WS2816A_SetPixel(uint16_t index, uint16_t r, uint16_t g, uint16_t b);
void WS2816A_SetPixelGain(uint16_t index, uint8_t ig, uint8_t ir, uint8_t ib);
void WS2816A_Fill(uint16_t r, uint16_t g, uint16_t b);
void WS2816A_Show(void);

#ifdef __cplusplus
}
#endif

#endif


