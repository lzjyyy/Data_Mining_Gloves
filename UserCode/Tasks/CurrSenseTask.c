#include "CurrSenseTask.h"
#include "cmsis_os2.h" 
#include "adc.h"
#include "tim.h"
#include <string.h>
#include "filter.h"
#include "math.h"
#include "AT24C256.h"
#include "data_manager.h"

/*------------------------------------------------------------------------------------------------------------------变量声明*/
static volatile float currMotorPre=0.f;
volatile float currMotorNow=0.f;

extern osMessageQueueId_t CurrSenseTaskHandle; /*队列句柄-由CubeMX或手动创建*/ 

uint16_t adc_dma_buffer[ADC_CHANNEL_NUM];/* DMA 搬运 ADC 数据的缓冲区（由 DMA 自动填充）注意：此缓冲区大小必须与 ADC 通道数一致 */

ADC_Frame_t currBufferPool[ADC_FRAME_POOL_SIZE];/*电流数据缓存池：存储ADC_FRAME_POOL_SIZE帧ADC数据，防止DMA与数据处理任务同时访问同一内存区域*/

static float adcFramePoolRms[6]={0.f};                 //这里存储1帧由原始数据计算出的RMS值

static float currDataOriginal[ADC_FRAME_POOL_SIZE]={0};//x号电机的ADC_FRAME_POOL_SIZE帧原始数据

static float currDataOriginalRmsKalman[ADC_FRAME_POOL_SIZE]={0};


static float currDataOriginalRmsKalmanMulstage[ADC_FRAME_POOL_SIZE]={0};


/* 队列handle,去freertos.c文件里面找CubeMX给你创建好的*/
extern osMessageQueueId_t CurSenseQueueHandle;
extern DMA_HandleTypeDef hdma_adc1;




/*------------------------------------------------------------------------------------------------------------------400,000Hz中断处理*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)/*完全传输中断回调函数*/ 
{
	if((hadc->Instance) == ADC1)
	{
		/*①关闭ADC采集&DMA搬运*/
		HAL_TIM_Base_Stop(&htim6);/*关闭定时器中断*/
		HAL_ADC_Stop_DMA(&hadc1);/*停止ADC的DMA搬运*/
		
		/*②在中断上下文里通过通知唤醒数据处理任务 */
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(CurrSenseTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/*------------------------------------------------------------------------------------------------------------------333Hz任务处理*/
void StartCurrSenseTask(void *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 3;																					//线程周期，这里表示3个tick执行该线程一次
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		/*① 等待ADC传输完成通知，如果没收到，一直等，不执行后面代码*/
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);																//使用 Task Notification 等待 ADC 传输完成通知（无限等待）
		
		/*② 如果等到通知，取出数据，处理*/
		for(size_t k=0;k<ADC_CHANNEL_NUM;k++)																		//存储6通道的一帧RMS值
		{
			for (size_t i=0;i<ADC_FRAME_POOL_SIZE;i++)														//提取某一通道的所有帧数据
			{
				currDataOriginal[i] = (float)currBufferPool[i].data[k];
			}
			adcFramePoolRms[k] = computeRMS(currDataOriginal, ADC_FRAME_POOL_SIZE);//计算该通道数据的RMS值
		}
		KalmanFilter(adcFramePoolRms,currDataOriginalRmsKalman);			  				 //卡尔曼滤波
		multiStageFilter(currDataOriginalRmsKalman,currDataOriginalRmsKalmanMulstage,ADC_CHANNEL_NUM,3,0.2);//多阶级联一阶低通滤波器
		
		/*③ 更新数据到数管中心*/
		DataManager_SetAllMotorCurrentsMea(currDataOriginalRmsKalmanMulstage,6);
		DataManager_Commit();
		
		/*④ 启动ADC去采集，给下一个任务周期提前准备好数据*/
		HAL_TIM_Base_Start(&htim6);																							//启动定时器中断&&启动DMA
		HAL_ADC_Start_DMA(&hadc1,(uint32_t *)currBufferPool,ADC_CHANNEL_NUM*ADC_FRAME_POOL_SIZE);
		vTaskDelayUntil( &xLastWakeTime, xFrequency );													//任务周期
	}
}


