/* EncSenseTask.c */

#include "EncSenseTask.h"
#include "tim.h"
#include "stm32h7xx_hal.h"
#include "uart_redirect.h"
extern osThreadId_t PosLoopTaskHandle;

/* CubeMX 生成的定时器句柄 */
extern TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim5, htim8, htim7;

/* 定时器句柄数组 */
TIM_HandleTypeDef *htim_enc[ENCODER_CNT] = {&htim1, &htim2, &htim3,&htim4, &htim5, &htim8};

/* 环形缓冲 与 写索引 */
volatile EncBuf_t  	encBuf[ENCODER_BUF_SIZE] = {0};
volatile uint16_t  	encWriteIdx = 0;

/* 溢出补偿上次计数 */
volatile int32_t    last_cnt[ENCODER_CNT]  = {0};

/* 全局累计脉冲总量 */
volatile int32_t    total_cnt[ENCODER_CNT]= {0};

/* 每通道每圈脉冲数映射 */
const uint32_t pulsePerRev[ENCODER_CNT] = {
    /* 电机①～②：10mm */
    (ENCODER_PPR * GEAR_RATIO_10MM_WANQU),
	  (ENCODER_PPR * GEAR_RATIO_10MM_BAIDONG),
    /* 电机③～⑥：12mm */
    (ENCODER_PPR * GEAR_RATIO_12MM),
    (ENCODER_PPR * GEAR_RATIO_12MM),
    (ENCODER_PPR * GEAR_RATIO_12MM),
    (ENCODER_PPR * GEAR_RATIO_12MM)
};

/*------------------------------------------------------------------------------------------------------------------------------1KHz*/
/** @brief  读取编码器寄存器的值，以及一次中断产生的差值，并将它们放到环形缓冲区中；
  *  
**/
void EncDataHandle(void)
{
//printf("ret\r\n");
//	uint32_t t0=DWT->CYCCNT;
	static uint8_t isCount=0;
	if(__HAL_TIM_CLEAR_FLAG(&htim7,TIM_FLAG_UPDATE)!=RESET)			//成功清除中断标志位
	{
/*① 读 CNT→delta→溢出补偿→累加→写入结构体 */
		for(int i=0; i<ENCODER_CNT; i++)
		{
			int32_t cur_cnt = __HAL_TIM_GET_COUNTER(htim_enc[i]);
			int32_t delta   = cur_cnt - last_cnt[i];
			last_cnt[i]     = cur_cnt;
			if      (delta < -32768) delta += 65536;	  //16位定时器溢出补偿
			else if (delta >  32767) delta -= 65536;
			total_cnt[i] += delta;										  //更新全局累计量
			encBuf[encWriteIdx].delta[i] = delta;       //存入环形缓冲
			encBuf[encWriteIdx].total[i] = total_cnt[i];
		}
/*② 推进写索引（环回） */
		encWriteIdx = (encWriteIdx + 1) % ENCODER_BUF_SIZE;
/*③ 唤醒速度计算任务 */
		osThreadFlagsSet(SpdLoopTaskHandle, 1U);
/*④ 10分频，唤醒位置环(100Hz) */
//printf("Spd\r\n");
		if(++isCount>=10)
		{
			isCount=0;
			osThreadFlagsSet(PosLoopTaskHandle, 1U);
			//int32_t ret = osThreadFlagsSet(PosLoopTaskHandle, 1U);
//printf("ret is %d\r\n",ret);
//printf("EncDataHandle → PosLoop: handle=%p, ret=%ld\r\n",(void*)PosLoopTaskHandle, (long)ret);
		}
	__HAL_TIM_CLEAR_IT(&htim7,TIM_IT_UPDATE);									//清除定时器溢出中断标志位
	}
//	uint32_t t1=DWT->CYCCNT;
//	uint32_t cycles=t1-t0;
//	float us=(float)cycles*1e6f/SystemCoreClock;
//	printf("time is %.4f us",us);
}

/*--------------------------------------------------------------------------------------------------------------------------------*/
void StartEncSenseTask(void *argument)
{
	for(;;)
	{
		osDelay(100);
	}
}  
