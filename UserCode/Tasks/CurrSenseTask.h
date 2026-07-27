#ifndef __CURRSENSETASK_H__
#define __CURRSENSETASK_H__

#include "stdint.h"
#include <stdio.h>
#include "cmsis_os2.h" 
#include "FreeRTOS.h"
#include "queue.h"
#include "adc.h"

#ifdef __cplusplus
extern "C" {
#endif


/* ADC 采集通道数量，使用 ADC1 的通道3、4、5、10、11、18 */
#define ADC_CHANNEL_NUM      6

/* 定义缓存池帧数，根据系统需求可以适当调整 */
#define ADC_FRAME_POOL_SIZE  400//20,

/**
  * @brief ADC 一帧数据结构
  *        保存一次 ADC 转换后所有通道的数据
  */
typedef struct {
    uint16_t data[ADC_CHANNEL_NUM];
} ADC_Frame_t;

/* 外部队列句柄，存放缓存池中最新数据帧的地址 */
extern QueueHandle_t CurSenseQueue;
extern ADC_Frame_t currBufferPool[ADC_FRAME_POOL_SIZE];

/* 函数原型 */

/**
  * @brief  ADC 数据预处理函数
  *         由 DMA 全传输中断中调用，将 DMA 传输到缓冲区的数据复制到缓存池中，
  *         并把当前数据帧地址放入队列，供数据处理任务使用。
  * @retval None
  */
void CurDataPreprocess(void);

/**
  * @brief  ADC 数据处理任务
  *         任务不断从队列中取出最新 ADC 数据帧地址，进行滤波处理，
  *         并将处理结果更新到全局传感器管理内存中供电流环控制使用。
  * @param  argument: 任务参数（本例未使用）
  * @retval None
  */
void StartCurrSenseTask(void *argument);
void CurDataPreprocess(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

#ifdef __cplusplus
}
#endif

#endif /* CURR_SENSE_H */



