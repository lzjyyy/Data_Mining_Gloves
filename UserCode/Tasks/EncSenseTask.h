/* EncSenseTask.h */

#ifndef __ENCSENSETASK_H__
#define __ENCSENSETASK_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"

/*—— 编码器与缓冲配置 ——*/
#define ENCODER_CNT           6              /* 编码器数量 */
#define ENCODER_PPR           (256U * 4U)    /* 电机后出轴转一圈输出的脉冲计数值(4倍频) */
#define GEAR_RATIO_10MM_BAIDONG 51U          /* 大拇指摆动电机*/
#define GEAR_RATIO_10MM_WANQU   111U         /* 大拇指弯曲电机*/
#define GEAR_RATIO_10MM       80U            /* 10 mm 电机减速比 */
#define GEAR_RATIO_12MM       70U            /* 12 mm 电机减速比 */  //原来是96
#define SAMPLE_PERIOD_S       0.001f         /* 1 kHz 采样周期 */
#define ENCODER_BUF_SIZE      5              /* 5帧数据 */
#define PULSE_PER_REV         ((float)ENCODER_PPR/(float)10.f)/*1/10圈对应的脉冲计数*/

/*—— 按通道映射的每圈脉冲数 ——*/
extern const uint32_t pulsePerRev[ENCODER_CNT];

/*—— 定时器句柄数组（在 .c 中初始化） ——*/
extern TIM_HandleTypeDef  *htim_enc[ENCODER_CNT];

/*—— 环形缓冲元素：同时保存本次 delta 与累计 total ——*/
typedef struct {
    int32_t delta[ENCODER_CNT];
    int32_t total[ENCODER_CNT];
} EncBuf_t;

/*—— 环形缓冲 与 写索引 ——*/
extern volatile EncBuf_t  encBuf[ENCODER_BUF_SIZE];
extern volatile uint16_t  encWriteIdx;

/*—— 溢出补偿上次计数 ——*/
extern volatile int32_t    last_cnt[ENCODER_CNT];

/*—— 全局累计脉冲总量 ——*/
extern volatile int32_t    total_cnt[ENCODER_CNT];

/*—— 速度计算任务句柄（freertos.c 中定义） ——*/
extern osThreadId_t        SpdLoopTaskHandle;

/**
 * @brief 1 kHz 中断调用：
 *        — 读 CNT → 计算 delta → 溢出补偿 → 累加 total_count
 *        — 写入 encBuf[encWriteIdx]
 *        — 推进写索引 → 唤醒速度任务
 */
void EncDataHandle(void);
void StartEncSenseTask(void *argument);

#ifdef __cplusplus
 }
#endif

#endif /* __ENCSENSETASK_H__ */
