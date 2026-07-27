#include "KinSolverTask.h"
#include "cmsis_os2.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "sysConfig.h"
#include "data_manager.h"
#include "Kin_Slover.h"

#include "stdint.h"
#include <stdio.h>


INPUT kinModelInput;
OUTPUT kinModelOutput;

#define DEBUG_MOTOR	  		1


void StartKinSolverTask(void *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 30;								   			    			// 线程周期，这里表示50个tick执行该线程一次
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		// printf("KinSolverTask is running...\n");
		//获取位置和速度的参考值和当前值
		float angle_cmd[MOTOR_COUNT];
		float angle_vel_cmd[MOTOR_COUNT];
		float angle_now[MOTOR_COUNT];
		float angle_vel_now[MOTOR_COUNT];

		DataManager_GetAllMotorPositionsMea(angle_now,MOTOR_COUNT);
		DataManager_GetAllMotorSpeedsMea(angle_vel_now,MOTOR_COUNT);
		DataManager_GetAngleCmdAll(angle_cmd,MOTOR_COUNT);
		DataManager_GetSpeedCmdAll(angle_vel_cmd,MOTOR_COUNT);

		//模型输入为绝对值
		for(uint8_t i=0;i<MOTOR_COUNT;i++)
		{
			if(angle_cmd[i]<0)
				angle_cmd[i] = -angle_cmd[i];
			if(angle_vel_cmd[i]<0)
				angle_vel_cmd[i] = -angle_vel_cmd[i];
			if(angle_now[i]<0)
				angle_now[i] = -angle_now[i];
			if(angle_vel_now[i]<0)
				angle_vel_now[i] = -angle_vel_now[i];
		}
		

		// printf("input_jiaodu,sudu:%.2f, %.2f, measure_jiaodu,sudu:%.2f, %.2f\r\n",
		// 	angle_cmd[DEBUG_MOTOR], angle_vel_cmd[DEBUG_MOTOR],
		// 	angle_now[DEBUG_MOTOR], angle_vel_now[DEBUG_MOTOR]);

		//将数据传递给运动学模型		
		kinModelInput.Tar_muzhi_wanqu_jiaodu_2   = angle_cmd[0];
		kinModelInput.Tar_muzhi_baidong_jiaodu_2 = angle_cmd[1];
		kinModelInput.Tar_shizhi_wanqu_jiaodu_2  = angle_cmd[2];
		kinModelInput.Tar_zhongzhi_wanqu_jiaodu_2= angle_cmd[3];
		kinModelInput.Tar_wumingzhi_wanqu_jiaodu_2= angle_cmd[4];
		kinModelInput.Tar_xiaomuzhi_wanqu_jiaodu_2= angle_cmd[5];
		kinModelInput.Tar_muzhi_wanqu_jiaosudu_2 = angle_vel_cmd[0];
		kinModelInput.Tar_muzhi_baidong_jiaosudu  = angle_vel_cmd[1];
		kinModelInput.Tar_shizhi_wanqu_jiaosudu_2= angle_vel_cmd[2];
		kinModelInput.Tar_zhongzhi_wanqu_jiaosudu_2= angle_vel_cmd[3];
		kinModelInput.Tar_wumingzhi_wanqu_jiaosudu_2= angle_vel_cmd[4];
		kinModelInput.Tar_xiaomuzhi_wanqu_jiaosudu_2= angle_vel_cmd[5];
		kinModelInput.now_M1_pos = kinModelInput.now_M1_pos1 = kinModelInput.now_M1_pos2 = angle_now[0];
		kinModelInput.now_M2_pos = kinModelInput.now_M2_pos1 = kinModelInput.now_M2_pos2 = angle_now[1];
		kinModelInput.now_M3_pos = kinModelInput.now_M3_pos1 = kinModelInput.now_M3_pos2 = angle_now[2];
		kinModelInput.now_M4_pos = kinModelInput.now_M4_pos1 = kinModelInput.now_M4_pos2 = angle_now[3];
		kinModelInput.now_M5_pos = kinModelInput.now_M5_pos1 = kinModelInput.now_M5_pos2 = angle_now[4];
		kinModelInput.now_M6_pos = kinModelInput.now_M6_pos1 = kinModelInput.now_M6_pos2 = angle_now[5];
		kinModelInput.now_M1_spd                  = angle_vel_now[0];
		kinModelInput.now_M2_spd                  = angle_vel_now[1];
		kinModelInput.now_M3_spd                  = angle_vel_now[2];
		kinModelInput.now_M4_spd                  = angle_vel_now[3];
		kinModelInput.now_M5_spd                  = angle_vel_now[4];
		kinModelInput.now_M6_spd                  = angle_vel_now[5];

		//调用运动学模型计算
		kinModelOutput = modelCalculate(kinModelInput);

		//将计算结果写回数据管理器。
		//注意：★★★模型输出为绝对值★★★
		float angelRef[MOTOR_COUNT], speedTar[MOTOR_COUNT], angle_du[MOTOR_COUNT], angle_sudu[MOTOR_COUNT];
		speedTar[0] = kinModelOutput.Tar_M1_spd;
		speedTar[1] = kinModelOutput.Tar_M2_spd;
		speedTar[2] = kinModelOutput.Tar_M3_spd;
		speedTar[3] = kinModelOutput.Tar_M4_spd;
		speedTar[4] = kinModelOutput.Tar_M5_spd;
		speedTar[5] = kinModelOutput.Tar_M6_spd;
		angelRef[0] = kinModelOutput.Tar_M1_pos;
		angelRef[1] = kinModelOutput.Tar_M2_pos;	
		angelRef[2] = -kinModelOutput.Tar_M3_pos;
		angelRef[3] = -kinModelOutput.Tar_M4_pos;
		angelRef[4] = -kinModelOutput.Tar_M5_pos;
		angelRef[5] = -kinModelOutput.Tar_M6_pos;
		angle_du[0] = kinModelOutput.now_muzhi_wanqu_jiaodu_2;
		angle_du[1] = kinModelOutput.now_muzhi_baidong_jiaodu_2;
		angle_du[2] = kinModelOutput.now_shizhi_wanqu_jiaodu_2;
		angle_du[3] = kinModelOutput.now_zhongzhi_wanqu_jiaodu_2;
		angle_du[4] = kinModelOutput.now_wumingzhi_wanqu_jiaodu_2;
		angle_du[5] = kinModelOutput.now_xiaomuzhi_wanqu_jiaodu_2;
		angle_sudu[0] = kinModelOutput.now_muzhi_wanqu_jiaosudu_2;
		angle_sudu[1] = kinModelOutput.now_muzhi_baidong_jiaosudu;
		angle_sudu[2] = kinModelOutput.now_shizhi_wanqu_jiaosudu_2;
		angle_sudu[3] = kinModelOutput.now_zhongzhi_wanqu_jiaosudu_2;
		angle_sudu[4] = kinModelOutput.now_wumingzhi_wanqu_jiaosudu_2;
		angle_sudu[5] = kinModelOutput.now_xiaomuzhi_wanqu_jiaosudu_2;

		//由于模型存在误差，当静止的时候，速度输出不为0，这里简单将其归零
		for(uint8_t i=0;i<MOTOR_COUNT;i++)
		{
			if(angle_sudu[i]<0.3f && angle_sudu[i]>-0.3f)
				angle_sudu[i] = 0.0f;
		}

		DataManager_SetAllMotorPositionsRef(angelRef,MOTOR_COUNT);
		DataManager_SetSpeedTarAll(speedTar,MOTOR_COUNT);
		DataManager_SetAngleDuAll(angle_du,MOTOR_COUNT);
		DataManager_SetAngleSuduAll(angle_sudu,MOTOR_COUNT);
		DataManager_Commit();
		DataManager_SetAllMotorPositionsRef(angelRef,MOTOR_COUNT);
		DataManager_SetSpeedTarAll(speedTar,MOTOR_COUNT);
		DataManager_SetAngleDuAll(angle_du,MOTOR_COUNT);
		DataManager_SetAngleSuduAll(angle_sudu,MOTOR_COUNT);
		DataManager_Commit();

		// printf("posref:%.2f, speedtar:%.2f, du:%.2f, sudu:%.2f\n",angelRef[DEBUG_MOTOR], speedTar[DEBUG_MOTOR], angle_du[DEBUG_MOTOR], angle_sudu[DEBUG_MOTOR]);

		//printf("speedtar:%.2f\n",speedTar[DEBUG_MOTOR]);

		vTaskDelayUntil( &xLastWakeTime, xFrequency );			        			 // 任务周期
	}
}

