#include "ws2816a_spi_test.h"
#include <string.h>

/*
 * 4MHz SPI:
 * 1 bit = 250ns
 *
 * WS2816A时序要求:
 * 0: T0H = 200~320ns, T0L = 800ns~1.2us
 * 1: T1H = 520~800ns, T1L = 480ns~1us
 * 周期 >= 1.25us
 *
 * 采用 5bit SPI 编码:
 * 0 -> 10000  => H=250ns, L=1000ns, total=1250ns
 * 1 -> 11100  => H=750ns, L=500ns,  total=1250ns
 */

/* 每个 WS 位编码成 5 个 SPI 位 */
#define WS_BIT0_PATTERN   0x10   /* 10000 */
#define WS_BIT1_PATTERN   0x1C   /* 11100 */

/* 每颗灯 64bit = 16bit gain + 48bit GRB */
#define WS2816A_BITS_PER_LED          64
#define WS2816A_SPI_BITS_PER_LED      (WS2816A_BITS_PER_LED * 5)

/* 向上取整成字节数 */
#define WS2816A_SPI_BYTES_PER_LED     ((WS2816A_SPI_BITS_PER_LED + 7) / 8)

/* 总发送缓冲:
 * 前导 reset + 数据区 + 后导 reset
 */
//#define WS2816A_TXBUF_SIZE  (WS2816A_RESET_BYTES + \
//                             (WS2816A_LED_NUM * WS2816A_SPI_BYTES_PER_LED) + \
//                             WS2816A_RESET_BYTES)


#define WS2816A_TOTAL_BITS  (16 + WS2816A_LED_NUM * 48)
#define WS2816A_TOTAL_SPI_BITS (WS2816A_TOTAL_BITS * 5)
#define WS2816A_DATA_BYTES ((WS2816A_TOTAL_SPI_BITS + 7) / 8)

#define WS2816A_TXBUF_SIZE  (WS2816A_RESET_BYTES + WS2816A_DATA_BYTES + WS2816A_RESET_BYTES)


static WS2816A_Color_t s_color[WS2816A_LED_NUM];
static WS2816A_Gain_t  s_gain[WS2816A_LED_NUM];
static uint8_t s_txbuf[WS2816A_TXBUF_SIZE];

/* ========== 内部函数 ========== */

/* 往 bit buffer 里写 1bit */
static void WS2816A_WriteOneBit(uint8_t *buf, uint32_t bit_pos, uint8_t bit_val)
{
    uint32_t byte_index = bit_pos / 8;
    uint32_t bit_index  = 7 - (bit_pos % 8);   // MSB first in byte

    if (bit_val)
    {
        buf[byte_index] |= (1U << bit_index);
    }
    else
    {
        buf[byte_index] &= ~(1U << bit_index);
    }
}

/* 把 5bit pattern 写入发送流 */
static void WS2816A_WritePattern5(uint8_t *buf, uint32_t *bit_pos, uint8_t pattern5)
{
    for (int i = 4; i >= 0; i--)
    {
        uint8_t b = (pattern5 >> i) & 0x01;
        WS2816A_WriteOneBit(buf, *bit_pos, b);
        (*bit_pos)++;
    }
}

/* 写一个 WS 数据位 */
static void WS2816A_EncodeWSBit(uint8_t *buf, uint32_t *bit_pos, uint8_t bit)
{
    if (bit)
    {
        WS2816A_WritePattern5(buf, bit_pos, WS_BIT1_PATTERN); // 11100
    }
    else
    {
        WS2816A_WritePattern5(buf, bit_pos, WS_BIT0_PATTERN); // 10000
    }
}

/* 写 16bit，高位先发 */
static void WS2816A_EncodeU16(uint8_t *buf, uint32_t *bit_pos, uint16_t value)
{
    for (int i = 15; i >= 0; i--)
    {
        WS2816A_EncodeWSBit(buf, bit_pos, (value >> i) & 0x01);
    }
}

/* 16bit gain 打包
 * 手册格式:
 * IG4 IG3 IG2 IG1 IG0 IR4 IR3 IR2 IR1 IR0 IB4 IB3 IB2 IB1 IB0 校验码
 * 校验码 0 或 1 均可
 */
static uint16_t WS2816A_PackGain(uint8_t ig, uint8_t ir, uint8_t ib, uint8_t check_bit)
{
    uint16_t v = 0;

    ig &= 0x1F;
    ir &= 0x1F;
    ib &= 0x1F;
    check_bit &= 0x01;

    v |= ((uint16_t)ig << 11);
    v |= ((uint16_t)ir << 6);
    v |= ((uint16_t)ib << 1);
    v |= check_bit;
	//	HAL_SPI_Transmit(&WS2816A_SPI_HANDLE, (uint8_t *)v, sizeof(v), HAL_MAX_DELAY);
    return v;
}

/* 组整帧:
 * reset + [LED0 gain + G + R + B] + [LED1 ...] + reset
 */
//static void WS2816A_BuildFrame(void)
//{
//    memset(s_txbuf, 0x00, sizeof(s_txbuf));

//    uint8_t *data_ptr = &s_txbuf[WS2816A_RESET_BYTES];
//    uint32_t bit_pos = 0;

//    for (uint16_t i = 0; i < WS2816A_LED_NUM; i++)
//    {
//        uint16_t gain_word = WS2816A_PackGain(s_gain[i].ig, s_gain[i].ir, s_gain[i].ib, 0);

//        /* 每颗灯: 16bit gain + 48bit GRB，高位先发 */
//        WS2816A_EncodeU16(data_ptr, &bit_pos, gain_word);
//        WS2816A_EncodeU16(data_ptr, &bit_pos, s_color[i].g);
//        WS2816A_EncodeU16(data_ptr, &bit_pos, s_color[i].r);
//        WS2816A_EncodeU16(data_ptr, &bit_pos, s_color[i].b);
//    }
//}

static void WS2816A_BuildFrame(void)
{
    memset(s_txbuf, 0x00, sizeof(s_txbuf));

    uint8_t *data_ptr = &s_txbuf[WS2816A_RESET_BYTES];
    uint32_t bit_pos = 0;

    /* 只发一次全局 gain */
    uint16_t gain_word = WS2816A_PackGain( s_gain[0].ir,s_gain[0].ig, s_gain[0].ib, 0);
    WS2816A_EncodeU16(data_ptr, &bit_pos, gain_word);

    for (uint16_t i = 0; i < WS2816A_LED_NUM; i++)
    {
        /* 这里只发颜色，不再发每颗灯自己的 gain */
        WS2816A_EncodeU16(data_ptr, &bit_pos, s_color[i].g);
        WS2816A_EncodeU16(data_ptr, &bit_pos, s_color[i].r);
        WS2816A_EncodeU16(data_ptr, &bit_pos, s_color[i].b);
    }
}
/* ========== 对外接口 ========== */

void WS2816A_SPI_Init(void)
{
    WS2816A_ClearAll();

    for (uint16_t i = 0; i < WS2816A_LED_NUM; i++)
    {
        s_gain[i].ig = 31;
        s_gain[i].ir = 31;
        s_gain[i].ib = 31;
    }

    WS2816A_Show();
}

void WS2816A_ClearAll(void)
{
    for (uint16_t i = 0; i < WS2816A_LED_NUM; i++)
    {
        s_color[i].r = 0;
        s_color[i].g = 0;
        s_color[i].b = 0;
    }
}

void WS2816A_SetPixel(uint16_t index, uint16_t r, uint16_t g, uint16_t b)
{
    if (index >= WS2816A_LED_NUM)
        return;

    s_color[index].r = r;
    s_color[index].g = g;
    s_color[index].b = b;
}

void WS2816A_SetPixelGain(uint16_t index, uint8_t ig, uint8_t ir, uint8_t ib)
{
    if (index >= WS2816A_LED_NUM)
        return;

    if (ig > 31) ig = 31;
    if (ir > 31) ir = 31;
    if (ib > 31) ib = 31;

    s_gain[index].ig = ig;
    s_gain[index].ir = ir;
    s_gain[index].ib = ib;
}

void WS2816A_Fill(uint16_t r, uint16_t g, uint16_t b)
{
    for (uint16_t i = 0; i < WS2816A_LED_NUM; i++)
    {
        s_color[i].r = r;
        s_color[i].g = g;
        s_color[i].b = b;
    }
}

void WS2816A_Show(void)
{
    WS2816A_BuildFrame();
//    uint8_t res = 0;
    HAL_SPI_Transmit(&WS2816A_SPI_HANDLE, s_txbuf, sizeof(s_txbuf), HAL_MAX_DELAY);
//	  for (int i = 0; i < 10; i++)
//    {
//        HAL_SPI_Transmit(&WS2816A_SPI_HANDLE, &res, 1, 0xFFFF);
//    }
}

