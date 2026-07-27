#include "SysCtrlTask.h"
#include "cmsis_os2.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "LaunchTask.h"
#include "sysConfig.h"
#include "data_manager.h"
#include "AT24C256.h"
#include "motor_bsp.h"
#include "tim.h"
#include "EncSenseTask.h"
#include "data_manager.h"
#include <stdbool.h>
#include "sysConfig.h"

// static const uint32_t motorCurSafeLimit[MOTOR_COUNT]={10711,18639,17355,17355,17355,17355};   //参考的电流安全阈值
// static const uint32_t motorCurSafeLimit[MOTOR_COUNT]={15000,29000,23000,23000,23000,23000};   //参考的电流安全阈值

static const uint32_t motorCurSafeLimit[MOTOR_COUNT]={20000,20000,18000,18000,18000,18000};

#define ENCODER_ERROR_MAX   	0.1f    //编码器自身带来的误差最大值0.1mm
extern uint8_t startMotorPosUint8[2*MOTOR_COUNT];
extern volatile int32_t total_cnt[ENCODER_CNT];
volatile bool enc_reset_req[MOTOR_COUNT] = { false };

static float MotorCurMea[MOTOR_COUNT]={0.f};
static float MotorPosMea[MOTOR_COUNT]={0.f};

// --- 启动校准状态机变量 ---
static bool    startup_done[MOTOR_COUNT]={0};
static uint8_t startup_done_count=0;

volatile float sysctrl[MOTOR_COUNT] = {0};

volatile uint8_t system_reset_request = 0; //系统复位请求标志
//OTA标志位
volatile uint8_t ota_reply_complete_flag = 0;           //OTA升级回复完成标志
volatile uint8_t ota_write_eeprom_complete_flag = 0;    //OTA写EEPROM完成标志

//限流相关
static unsigned char fold_on[MOTOR_COUNT]; // 是否处于限流
// 可调参数
static const float RAMP_UP  = 0.5f;   // 每秒最大上升速率（0→1 约需 0.33s）
static const float GAIN_MIN = 0.10f;  // 最小系数底线，防止断崖

// 初始化一次
void sysctrl_init(void){
    for(int i=0;i<MOTOR_COUNT;++i){ sysctrl[i]=1.0f; fold_on[i]=0; }
}
// 每个控制周期调用，dt=控制周期(秒)
static inline float clampf(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }


void sysctrl_step(int i, float I_mea, float I_lim, float dt){
    float drop = 0.33f * I_lim / 20000.0f;
    drop = clampf(drop, GAIN_MIN, 1.0f);

    if(!fold_on[i] && I_mea >= I_lim){
        fold_on[i] = 1;
        sysctrl[i] = fminf(sysctrl[i], drop); 
    }else if(fold_on[i] && I_mea <= I_lim){
        fold_on[i] = 0;
    }

    // 脱离限流后才慢慢增大
    if(!fold_on[i]){
        float step = RAMP_UP * dt;
        sysctrl[i] = clampf(sysctrl[i] + step, GAIN_MIN, 1.0f);
    }
}

/*---------------------------------------------------------------------------------------------------------------20Hz线程处理*/
/**
* @brief  线程任务：系统安全控制&&位置实时标定
 */
void StartSysCtrlTask(void *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100;								   			    			// 线程周期，这里表示50个tick执行该线程一次
	xLastWakeTime = xTaskGetTickCount();
	sysctrl_init(); // 初始化限流控制器
	for(;;)
	{
		//判断OTA升级状态，升级过程中不进行系统控制
		if((ota_reply_complete_flag==1 && ota_write_eeprom_complete_flag==1) || system_reset_request==1)
		{
			//软复位 —— 复位后按 Option Bytes 从 Boot 起跑
			
			NVIC_SystemReset();
		}


		/*① 获取当前电流、位置值（20Hz）*/
		DataManager_GetAllMotorCurrentsMea(MotorCurMea,6); 			   				// 获取电机电流测量值，单位(1/10圈)
		DataManager_GetAllMotorPositionsMea(MotorPosMea,6);			   				// 获取电机位置测量值，单位(1/10圈)
//printf("Current3:%.4f\n",MotorCurMea[3]);
		
		/*② 取事件组的当前标志*/  
    uint32_t flags = osEventFlagsGet(LaunchEventsHandle);
		
		/*③ 校准模式，且还没发 LAUNCH_DONE_BIT*/ 
		if((flags & MODE_CALIB_BIT) &&!(flags & LAUNCH_DONE_BIT))
		{	
			for(uint8_t i = 0; i<MOTOR_COUNT; i++)													// 每路堵转一次就当零点校准
			{
				if(MotorCurMea[i]>=motorCurSafeLimit[i] && !startup_done[i]) 	// 避免重复计数
				{
					Motor_SetDuty((Motor_Id)i,0);																// 刹车
					startup_done[i] = true;
					startup_done_count++;
				}
			}
			if(startup_done_count >= 6)		//MOTOR_COUNT											// 全部路都堵转过一次？发 CALIB_DONE_BIT
			{	
				HAL_Delay(500);
				HAL_Delay(500);			//多加点延时，确保电机完全静止
				osEventFlagsSet(LaunchEventsHandle, CALIB_DONE_BIT);
				osEventFlagsSet(LaunchEventsHandle, MODE_NORMAL_BIT);            // [FIX] 进入正常模式（粘性位）
				osEventFlagsClear(LaunchEventsHandle, MODE_CALIB_BIT);           // [FIX] 退出校准模式（互斥）
			}
		}
		
		/*④ 正常模式，或完成校准后(LAUNCH_DONE_BIT 已发) */
		else if((flags & MODE_NORMAL_BIT) || (flags & LAUNCH_DONE_BIT))
		{
			for(uint8_t i = 0; i<MOTOR_COUNT; i++)
			{
				sysctrl_step(i, MotorCurMea[i], motorCurSafeLimit[i], (float)xFrequency / 1000.0f);
			}
			
    }
	vTaskDelayUntil( &xLastWakeTime, xFrequency );			        			 // 任务周期
	}
}



