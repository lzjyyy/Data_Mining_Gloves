#include "PosLoopTask.h"
#include "cmsis_os2.h" 
#include "motor_bsp.h"
#include "cmsis_os2.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "data_manager.h"
#include "motor_bsp.h"
#include "LaunchTask.h"
#include "sysConfig.h"
#include "EncSenseTask.h"
#include "TrapTrajPlanner.h"
#include "sysConfig.h"
#include "SpdLoopTask.h"
#include <math.h>
#include "Action.h"



extern const float spdMax[MOTOR_COUNT];  


/*--------------------------------------------------------------...--------------------------------------------------------------*/
/*位置最大值：(+102.8)<—>(+299.8)<—>(-293)<—>(-293)<—>(-293)<—>(-293)
  速度最大值：(+270.0)<—>(+270.0)<—>(+210)<—>(+190)<—>(+180)<—>(+200)
  速度和位置都是由485线程传值到这两个全局数组的
*/
float posRef[MOTOR_COUNT]={0.f,0.f,0.f,0.f,0.f,0.f};	
float spdTar[MOTOR_COUNT]={0.f,0.f,0.f,0.f,0.f,0.f};	//只给速度的绝对值，符号由位置决定

/*--------------------------------------------------------------...--------------------------------------------------------------*/
/**@bref: 位置环PID 参数列表与初始化
	*
	* 下面这部分保持你原有的思路：posPidConfigs[] 给 pidPos[]，
	* Kp/Ki/Kd、死区、积分限幅这些都不动。
	*/
typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float deadzone;
	float dt;
	float outMin;
	float outMax;
	float integralMin;
	float integralMax;
}PositionPID_Config_t;

static PID_HandleTypeDef pidPos[MOTOR_COUNT];

static const PositionPID_Config_t posPidConfigs[MOTOR_COUNT] =
{
	/* {Kp,    Ki,    Kd,   deadzone, dt,     outMin,   outMax,   iMin,     iMax} */
	{ 10.0f,  0.f, 0.0f, 0.f,      0.01f,  -100.f,   100.f,   -100.f,    100.f }, //拇指1(弯曲)
	{ 10.0f,  0.f, 0.0f, 0.f,      0.01f,  -299.f,   299.f,   -299.f,    299.f }, //拇指2(摆动)
	{ 10.0f,  0.f, 0.0f, 0.f,      0.01f,  -293.f,   293.f,   -293.f,    293.f }, //食指
	{ 10.0f,  0.f, 0.0f, 0.f,      0.01f,  -293.f,   293.f,   -293.f,    293.f }, //中指
	{ 10.0f,  0.f, 0.0f, 0.f,      0.01f,  -293.f,   293.f,   -293.f,    293.f }, //无名指
	{ 10.0f,  0.f, 0.0f, 0.f,      0.01f,  -293.f,   293.f,   -293.f,    293.f }  //小指
};

/* 每路电机允许的最大加速度，用于限斜率（deg/s^2 或类似单位） */
static const float Amax[MOTOR_COUNT] = { 1200.f,1200.f,1200.f,1200.f,1200.f,1200.f };

static const float CSP_Pos2Vel_Kp[MOTOR_COUNT] =
{
	5.5f,7.5f,8.2f,8.2f,8.2f,8.2f
	//20.0f,20.0f,20.0f,20.0f,20.0f,20.0f
	//30.0f,30.0f,30.0f,30.0f,30.0f,30.0f
	//40.0f,40.0f,40.0f,40.0f,40.0f,40.0f
	//50.0f,50.0f,50.0f,50.0f,50.0f,50.0f
	//60.0f,60.0f,60.0f,60.0f,60.0f,60.0f
	//70.0f,70.0f,70.0f,70.0f,70.0f,70.0f
	//80.0f,80.0f,80.0f,80.0f,80.0f,80.0f
	//90.0f,90.0f,90.0f,90.0f,90.0f,90.0f
};


/**
 * 电机圈数对应的度数（度每圈）:
 * 大拇指弯曲:		0.37
 * 大拇指侧摆:		0.46
 * 四指弯曲:		0.46
 */
static const float POS_deadzone_enter[MOTOR_COUNT] =
{
	0.8f,0.8f,0.4f,0.4f,0.4f,0.4f
};

static const float POS_deadzone_exit[MOTOR_COUNT] =
{
	5.f,3.8f,0.8f,0.8f,0.8f,0.8f
};
static float SpdCmdPrev[MOTOR_COUNT] = {0.f};

/* 原有：初始化位置环PID */
void PositionControl_Init(void)
{
	for(int i=0;i<MOTOR_COUNT;i++)
	{
		const PositionPID_Config_t *cfg = &posPidConfigs[i];
		PID_Init(&pidPos[i],
				 cfg->Kp, cfg->Ki, cfg->Kd,
				 cfg->deadzone, cfg->dt,
				 cfg->outMin, cfg->outMax,
				 cfg->integralMin, cfg->integralMax);
	}
}

uint8_t valid[MOTOR_COUNT]={0};
/*=======================================================================================
 * StartPosLoopTask
 * 周期：100 Hz (10ms)
 *=======================================================================================*/
void StartPosLoopTask(void *argument)
{
	for(;;)
	{
/*① 等待启动线程&&编码器中断100Hz通知*/		
		osThreadFlagsWait(1U,osFlagsWaitAny,osWaitForever);	//等待定时器的任务通知，100Hz

/*② 直接计算并上报位置*/
		float posMea[ENCODER_CNT];
		for(int i=0; i<ENCODER_CNT; i++)//计算出6路电机的位置(注意使用整除前面要加float，否则导致丢圈)
		{
			posMea[i] = (float)(total_cnt[i] / (float)ENCODER_PPR) + startMotorPosFloat[i]; //startMotorPosFloat[i]是读取上一次断电时的位置数据
		}                                                                                 

		DataManager_SetAllMotorPositionsMea(posMea, ENCODER_CNT);	//这里提交参考位置，是为了方便串口打印
		DataManager_Commit();		

		//获取模型计算的位置和速度参考
		DataManager_GetAllMotorPositionsRef(posRef,ENCODER_CNT);
		DataManager_GetSpeedTarAll(spdTar,ENCODER_CNT);
		// printf("spdtar:%.2f\r\n",spdTar[2]);


/*④ PID计算******/		
		float spdRef[MOTOR_COUNT]={0.f};
		for(int i=0;i<MOTOR_COUNT;i++)
		{
			/* 位置误差 */
			float e_pos = posRef[i] - posMea[i];
			/* 位置误差 -> 期望速度 (比例控制器) */
			//位置死区——滞回死区
			if(valid[i]){
				if(e_pos<POS_deadzone_enter[i] && e_pos>-POS_deadzone_enter[i]){
					valid[i]=0;
				}
			}
			else{
				if(e_pos>POS_deadzone_exit[i] || e_pos<-POS_deadzone_exit[i]){
					valid[i]=1;
				}
			}
			float v_cmd = (float)(valid[i] * CSP_Pos2Vel_Kp[i] * e_pos);
			
			// float v_cmd = PID_CalcPosition(&pidPos[i], posRef[i], posMea[i]);

			/* 限速幅度（用户spdTar[i]和硬件spdMax[i]二者取较小的那个） */
			float vmax_user = spdTar[i];
			if (vmax_user < 0.0f) vmax_user = -vmax_user; 	//只取幅值
			float vmax_hw   = spdMax[i];
			float vmax_allow = (vmax_user < vmax_hw) ? vmax_user : vmax_hw;

			if (v_cmd >  vmax_allow) v_cmd =  vmax_allow;
			if (v_cmd < -vmax_allow) v_cmd = -vmax_allow;

			/* 限加速度/斜率：Δv每10ms不能超过 Amax[i]*dt */
			float dv     = v_cmd - SpdCmdPrev[i];
			float dv_max = Amax[i] * POS_PERIOD_SEC;			//允许10ms内变化的最大速度增量
			if (dv >  dv_max) dv =  dv_max;
			if (dv < -dv_max) dv = -dv_max;

			SpdCmdPrev[i] += dv;										//平滑更新速度指令
			spdRef[i] = SpdCmdPrev[i];
		}
		DataManager_SetAllMotorSpeedsRef(spdRef,ENCODER_CNT);
		DataManager_Commit();
	} 
}
