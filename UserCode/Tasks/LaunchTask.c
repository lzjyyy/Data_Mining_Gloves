#include "cmsis_os2.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "tim.h"
#include "stm32h7xx_it.h"
#include "EncSenseTask.h"
/* 包含你的硬件初始化等头文件 */
#include "main.h"
#include "LaunchTask.h"
#include "sysConfig.h"
#include "Keys.h"
#include "data_manager.h"
#include "base_convert.h"
#include "AT24C256.h"
#include "motor_bsp.h"
#include "sysConfig.h"
#include "LedTask.h"

#define POSITION_CALIBRATION_MODE							        //复位模式控制

static  volatile uint8_t mode=2;
uint8_t startMotorPosUint8[2*MOTOR_COUNT]={0};
float   startMotorPosFloat[MOTOR_COUNT]={0.f};
static  volatile uint8_t processed[MOTOR_COUNT]={0};  // 0 表示还没比较，1 表示已超过阈值并计数
static  volatile uint8_t exceedCount=0;
static	volatile float	threshold=12000.f;           	// 电流阈值


/**
*@brief  A:根据按键来区分 是否为标定状态
				 B:检测上一次断电时刻的电机位置
**/
uint8_t GenshinStarted(void)
{
	/*① 获取上一次断电前存储的位置信息*/
	AT24Cxx_Read_Amount_Byte(24,startMotorPosUint8,12);
	bytes_to_floats(startMotorPosUint8,startMotorPosFloat,6);
		
	for(uint8_t j=0;j<6;j++)																				
	{
printf("Pos[%d]:%.4f\n",j,startMotorPosFloat[j]);              //测试读取的数据是否准确
	 }  
//while(1){}
	/*② 根据长按还是短按判断是否需要执行手指复位*/
	#ifdef POSITION_CALIBRATION_MODE
		//mode = Startup_KeyCheck();											        	 //检测按下模式
mode=1;//增加方便调试	 
	 
printf("mode:%d\n",mode);
		if(mode==1)																								 //长按进入复位模式
		{
			HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET); //亮灯LED4
			
			Motor_SetDuty((Motor_Id)0,-700);											   //设置大拇指①号电机反转	
			Motor_SetDuty((Motor_Id)1,-700);											   //设置大拇指②号电机反转
			Motor_SetDuty((Motor_Id)2,700);											 	 //设置食  指③号电机正传
			Motor_SetDuty((Motor_Id)3,700);											   //设置中  指④号电机正转
			Motor_SetDuty((Motor_Id)4,700);											   //设置无名指⑤号电机正转
			Motor_SetDuty((Motor_Id)5,700);											   //设置小拇指⑥号电机正转

//while(1){}
		}
		return mode;
	#else
		return 0; 																								 //不编译为标定模式时当短按处理
	#endif 
}






void StartLaunchTask(void *argument)
{
    /*① 先清理所有标志：上次残留的流程位+模式位*/
    osEventFlagsClear(LaunchEventsHandle, CALIB_DONE_BIT | LAUNCH_DONE_BIT | MODE_CALIB_BIT | MODE_NORMAL_BIT);
printf(">>> [Launch] begin mode select...\r\n");
	LedTask_SetMode(LED_MODE_BOOT);
	
	 /*② 读按键模式*/
    uint8_t mode1 = GenshinStarted();										//返回 1：校准模式；0：正常模式
printf(">>> [Launch] mode1:%d\n",mode1);
//while(1){} //调试用--人为阻塞
	
	 /*③ 根据 mode1 发布对应的模式位*/ 
    if(mode1==1) 
		{
			osEventFlagsSet(LaunchEventsHandle, MODE_CALIB_BIT);
printf(">>> [Launch] mode=CALIBRATION\r\n");
    } 
		else 
		{
			osEventFlagsSet(LaunchEventsHandle, MODE_NORMAL_BIT);
printf(">>> [Launch] mode=NORMAL\r\n");
    }
		
    /*④ 如果是校准模式，要等 SysCtrlTask 发 CALIB_DONE_BIT*/ 
    if(mode1==1) 
		{
			/*4.1 等待SysCtrlTask发出堵转校准完成*/ 
			osEventFlagsWait(LaunchEventsHandle,CALIB_DONE_BIT,osFlagsWaitAny,osWaitForever);
      /*4.2 因为是校准模式，所以接收到堵转完成事件标志后，需要将位置等数据进行标定，完成后方可放行*/
			for(uint8_t i=0;i<MOTOR_COUNT;i++)
			{
				__HAL_TIM_SET_COUNTER(htim_enc[i], 0);							// 清编码器计数为 0
				last_cnt[i]=0;
				total_cnt[i] = 0;
				startMotorPosUint8[i]=0;	  												// 因为编码器线程加上了startMotorPosFloat，所以将EEPROM也清零
				startMotorPosFloat[i]=0;
//				AT24Cxx_Write_Amount_Byte(24,startMotorPosUint8,12);
//				DataManager_SetAllMotorPositionsMea(startMotorPosFloat,6);
//				DataManager_Commit();
			}
			encWriteIdx=0;
			AT24Cxx_Write_Amount_Byte(24,startMotorPosUint8,12);
      /* 轮询直到 EEPROM 内部写入完成 */
      while(AT24C_IsDeviceReady(AT24CXX_ADDR_WRITE) != HAL_OK) 
			{
				HAL_Delay(20);
      }
			uint32_t ret=AT24C_IsDeviceReady(AT24CXX_ADDR_WRITE);
printf("ret is : %d\r\n",ret);
			DataManager_SetAllMotorPositionsMea(startMotorPosFloat,6);
			DataManager_Commit();
		}
		
printf(">>> [Launch] set LAUNCH_DONE_BIT, exiting\r\n");		

		/*⑤ 发布启动完成，放行位置控制线程*/
    osEventFlagsSet(LaunchEventsHandle, LAUNCH_DONE_BIT);
printf(">>> [Launch] done, other tasks may run now\r\n");
		LedTask_SetMode(LED_MODE_RUN);
		osThreadExit();
}





