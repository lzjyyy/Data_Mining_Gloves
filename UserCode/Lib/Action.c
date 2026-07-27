/*
▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁
▏▁▁▁▁▁▁▁▁▁传动比▁▁导程▁▁行程max(单位:mm)▁▁行程max(单位:1/10圈)▁速度max(单位：后出轴圈/s)	▏ 
▏大拇指①号电机：▏ 80		▏ 3mm	▏	3.8568mm     	▏     1028.48    	 ▏			 			250	▏
▏大拇指②号电机：▏ 80	   ▏ 3mm	▏	11.2412mm     	▏     2997.65			 ▏			 			270	▏
▏食指  ③号电机：▏ 96		▏ 5mm	▏	15.26mm       	▏     2929.92			 ▏		   			220	▏
▏中指  ④号电机：▏ 96		▏ 5mm	▏	15.26mm       	▏     2929.92			 ▏			 			220	▏
▏无名指⑤号电机：▏ 96		▏ 5mm	▏	15.26mm       	▏     2929.92			 ▏			 			220	▏
▏小拇指⑥号电机：▏ 96		▏ 5mm	▏	15.26mm       	▏     2929.92			 ▏			 			220	▏
▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔
*/
/*
▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁
▁▁▁▁▁▁▁▁▁电机旋转方向▁▁▁滑块位移方向▁▁▁关节运动方向   	▏
▏大拇指①号电机：   逆时针                           伸直(复位)	 	▏
▏						  顺时针                           弯曲			   ▏
▏大拇指②号电机：   逆时针                           远离掌心(复位)▏
▏						  顺时针            前移           靠近掌心		▏
▏③④⑤⑥号电机：   顺时针			      前移           伸直(复位)	▏
▏                  逆时针           	后移           弯曲			▏
▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔
*/
#include "Action.h"
#include <math.h>  // fabsf()
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"

static uint8_t currentStep[MOTOR_COUNT] = {0};

#define NUM_STEPS 15

//static const float stepTargetPos[MOTOR_COUNT][NUM_STEPS] = {
//					/*动作①：握拳*/   /*动作②：张开*/  /*动作③：数字6*/    /*动作④：OK*/   /*动作⑤：捏指*/	/*动作⑥：握拳*/	/*动作⑦：比耶*/
///*电机1*/{0.f, +040.0f,+100.0f,	+040.0f,+000.0f,	+000.0f,+000.0f,	+030.0f,+055.0f,	+055.0f,+065.0f,	+015.0f,+100.0f,	+015.0f,+100.0f}, //大拇指1
///*电机2*/{0.f, +105.0f,+270.0f,	+100.0f,+000.0f,	+000.0f,+000.0f,	+180.0f,+230.0f,	+240.0f,+299.0f,	+100.0f,+250.0f,	+100.0f,+299.0f}, //大拇指2
///*电机3*/{0.f, -090.0f,-290.0f,	-090.0f,+000.0f,	-090.0f,-290.0f,	-130.0f,-177.0f,	-100.0f,+000.0f,	-200.0f,-290.0f,	-200.0f,-000.0f}, //第2个手指
///*电机4*/{0.f, -090.0f,-290.0f,	-090.0f,+000.0f,	-090.0f,-290.0f,	-090.0f,+000.0f,	-090.0f,-180.0f,	-200.0f,-290.0f,	-200.0f,-000.0f}, //第3个手指
///*电机5*/{0.f, -090.0f,-290.0f,	-090.0f,+000.0f,	-090.0f,-290.0f,	-090.0f,+000.0f,	-000.0f,+000.0f,	-200.0f,-290.0f,	-290.0f,-290.0f}, //第4个手指
///*电机6*/{0.f, -090.0f,-290.0f,	-090.0f,+000.0f,	-000.0f,-000.0f,	-000.0f,+000.0f,	-000.0f,+000.0f,	-200.0f,-290.0f,	-290.0f,-290.0f}};//第5个手指
//static const float stepTargetSpd[MOTOR_COUNT][NUM_STEPS-1] = {	
///*电机1*/{ 		  030.0f,216.0f,	 193.0f,030.0f,	  000.0f,000.0f,	  034.0f,034.0f,	  034.0f,034.0f,	  084.0f,084.0f,	+106.0f,+106.0f}, //大拇指1
///*电机2*/{ 		  105.0f,270.0f,	 270.0f,095.0f,	  000.0f,000.0f,	  143.0f,143.0f,	  070.0f,070.0f,	  217.0f,217.0f,	+220.0f,+220.0f}, //大拇指2
///*电机3*/{		  180.0f,180.0f,	 180.0f,180.0f,	  180.0f,180.0f,	  160.0f,077.0f,	  180.0f,180.0f,	  180.0f,180.0f,	+180.0f,+180.0f}, //第2个手指
///*电机4*/{ 		  180.0f,180.0f,	 180.0f,180.0f,	  180.0f,180.0f,	  180.0f,180.0f,	  180.0f,180.0f,	  068.0f,068.0f,	+180.0f,+180.0f}, //第3个手指
///*电机5*/{ 		  180.0f,180.0f,	 180.0f,180.0f,	  180.0f,180.0f,	  180.0f,180.0f,	  000.0f,000.0f,	  180.0f,180.0f,	+000.0f,+000.0f}, //第4个手指
///*电机6*/{ 		  180.0f,180.0f,	 180.0f,180.0f,	  000.0f,000.0f,	  000.0f,000.0f,	  000.0f,000.0f,	  180.0f,180.0f,	+000.0f,+000.0f}};//第5个手指


#define NUM_STEPS 15
static const float stepTargetPos[MOTOR_COUNT][NUM_STEPS] = {
	
					/*动作①：握拳*/   /*动作②：张开*/  /*动作③：数字6*/  /*动作④：OK*/   /*动作⑤：捏指*/		/*动作⑥：握拳*/	/*动作⑦：比耶*/
/*电机1*/{0.f, +015.5f,+100.0f}, //大拇指1
/*电机2*/{0.f, +150.0f,+290.0f}, //大拇指2
/*电机3*/{0.f, -090.0f,-290.0f}, //第2个手指
/*电机4*/{0.f, -090.0f,-290.0f}, //第3个手指
/*电机5*/{0.f, -090.0f,-290.0f}, //第4个手指
/*电机6*/{0.f, -090.0f,-290.0f}};//第5个手指
static const float stepTargetSpd[MOTOR_COUNT][NUM_STEPS-1] = {	
/*电机1*/{ 		   9.6f, 255.0f}, //大拇指1
/*电机2*/{ 		  210.0f,210.0f}, //大拇指2
/*电机3*/{		  210.0f,210.0f}, //第2个手指
/*电机4*/{ 		  210.0f,210.0f}, //第3个手指
/*电机5*/{ 		  210.0f,210.0f}, //第4个手指
/*电机6*/{ 		  210.0f,210.0f}};//第5个手指


//float spdMax[MOTOR_COUNT]={255,  280,  220,  220,  220,   220  };







/**
 * @brief 检测 posRef 数组是否与上一次不同
 * @param posRef 本次的目标位置数组
 * @return true  如果发生了改变（新动作）
 *         false 如果没有改变
 */
bool hasPosRefChanged(const float posRef[MOTOR_COUNT]) 
{
	static float prevPosRef[MOTOR_COUNT] = {0};
	bool changed = false;
	for (int i = 0; i < MOTOR_COUNT; i++) 		//逐元素比较
	{
		if (posRef[i] != prevPosRef[i]) 
		{
			changed = true;
			break;
		}
	}
	if (changed) 															//如果变化了，就手动把新值拷贝到 prevPosRef
	{
		for (int i = 0; i < MOTOR_COUNT; i++) 
		{
			prevPosRef[i] = posRef[i];
		}
	}
	return changed;
}










/**
 * @brief  更新每路电机的子段参考位置和速度，支持动作间延时同步
 * @param  posMea[]  当前各电机的测量位置数组（单位同 stepTargetPos）
 * @param  posRef[]  输出的各电机参考位置数组
 * @param  spdTar[]  输出的各电机参考速度数组
 *
 * 逻辑流程：
 *  0. 群组检测：当所有“有速度”子段均已完成该动作的第二子段时，触发统一延时
 *  1. 各路电机常规推进：在非延时阶段，对非第二子段到达直接推进；在延时完成后推进第二子段
 *  2. 输出阶段：根据完成、延时、正常三种状态下发 posRef/spdTar
 */
void updateSubstepReferences(float posMea[], float posRef[], float spdTar[])
{
	// —— 静态状态变量 ——  
	static bool     doneAll[MOTOR_COUNT]  = {0};  // 每路是否完成所有子段
	static bool     inDelay[MOTOR_COUNT]  = {0};  // 每路是否处于动作边界延时状态
	static int      delayCnt[MOTOR_COUNT] = {0};  // 每路延时剩余 tick（100 tick @100Hz ≈1s）
	static uint8_t  idx[MOTOR_COUNT]      = {0};  // 每路当前子段索引 (0…NUM_STEPS-2)

	// 配置常量
	const int   SUBSTEPS = 2;                         // 每个动作 2 个子段
	const int   actions  = (NUM_STEPS - 1) / SUBSTEPS; // 动作总数 = 子段总数/2
	const float tol      = 5.0f;                      // 到达判定容差

// —— 0. 群组第二子段同步检测 ——  
	// 只有当所有“有速度”子段都到达它们的第二子段末尾，才统一触发延时
	bool allSecondReached = true;
	for (int j = 0; j < MOTOR_COUNT; j++) 
	{
		if (doneAll[j]) 
			continue;  // 已经完成的电机跳过
		// 若当前不是第二子段，则还未达到动作边界
		if ((idx[j] % SUBSTEPS) != 1) {
			allSecondReached = false;
			break;
		}
		// 对于该子段有速度的通道，检查位置是否到达终点
		float spd = stepTargetSpd[j][ idx[j] ];
		if (spd != 0.0f) 
		{
			float s = stepTargetPos[j][ idx[j]     ]; // 该子段起点
			float t = stepTargetPos[j][ idx[j] + 1 ]; // 该子段终点
			bool reached = (t >= s) ? (posMea[j] >= t - tol) : (posMea[j] <= t + tol);
			if (!reached) 
			{
				allSecondReached = false;
				break;
			}
		}
		// 对于 spd==0 的子段，我们视作自动到达，无需判断
	}
	if (allSecondReached) 								// 统一触发延时：所有未完成且未在延时中的通道都进入延时
	{
		for (int j = 0; j < MOTOR_COUNT; j++) 
		{
			if (!doneAll[j] && !inDelay[j]) 
			{
				 inDelay[j]  = true;
				 delayCnt[j] = 50;  					// 设置 100 tick 延时
			}
		}
	}

// —— 1. 每路电机常规推进 ——  
	for (int i = 0; i < MOTOR_COUNT; i++) 
	{
		/*1.1 已完成：锁定最终位置 & 零速度*/ 
		if (doneAll[i]) 
		{
			posRef[i] = stepTargetPos[i][NUM_STEPS - 1];
			spdTar[i] = 0;
			continue;
		}
		/*1.2 本子段起/终点及方向*/ 
		float start  = stepTargetPos[i][ idx[i]     ];
		float target = stepTargetPos[i][ idx[i] + 1 ];
		float delta  = target - start;
		float dir    = (delta > 0) ? +1 : -1;
		/*1.3 普通到达判定（不区分速度是否为零）*/ 
		bool reached = (posMea[i] >= target - tol && dir > 0) || (posMea[i] <= target + tol && dir < 0);
		/*1.4 子段推进逻辑*/ 
		if (!inDelay[i] && reached) 				// 如果不是第二子段，则可自行推进
		{
			if (((idx[i] + 1) % SUBSTEPS) != 0) 
			{
				idx[i]++;
				if (idx[i] >= NUM_STEPS - 1) 
				{
					doneAll[i] = true;  				// 最后一段结束后标记完成
				}
			}
		}													// 若 idx 位于第二子段，则由群组延时结束后统一推进，不在此处推进
		else if (inDelay[i]) 						// 延时期间逐 tick 递减
		{
			if (--delayCnt[i] <= 0) 
			{
				inDelay[i] = false;
				idx[i]++;  								// 延时结束后推进第二子段
				if (idx[i] >= NUM_STEPS - 1) 
				{
					doneAll[i] = true;
				}
			}
		}
// —— 2. 输出参考 ——  
		if (doneAll[i]) 
		{
			posRef[i] = stepTargetPos[i][NUM_STEPS - 1]; // 完成后锁定最终位置
			spdTar[i] = 0;
		}
		else if (inDelay[i]) 
		{
			posRef[i] = stepTargetPos[i][ idx[i] + 1 ];	// 延时中：锁定本子段的终点位置，速度清零
			spdTar[i] = 0;
		}
		else 
		{
			posRef[i] = stepTargetPos[i][ idx[i] + 1 ];  // 正常子段执行：给出本子段终点和对应速度
			spdTar[i] = stepTargetSpd[i][ idx[i] ];
		}
	}
}





















