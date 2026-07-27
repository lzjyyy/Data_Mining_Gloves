
/*************************************************************************************/
/********************************************************************* 2阶低通滤波器 */
/*************************************************************************************/
/* SpdLoopTask.c */
#include "SpdLoopTask.h"
#include "data_manager.h"
#include "pid.h"
#include "uart_redirect.h"
#include "motor_bsp.h"
#include "LaunchTask.h"
#include "eso.h"
#include "SysCtrlTask.h" 
#include "PosLoopTask.h"
// [DEL] 不再需要 <string.h>，因为不使用 memcpy 了
// #include <string.h>
// [ADD] 二阶滤波需要三角函数
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
███████████████████████████████████████████████████████████████████▉██
*/
/**
** @brief 
** @brief 
** @brief 
**/

float motor_duty[MOTOR_COUNT];
ESOState eso[MOTOR_COUNT];
static const float eso_T[MOTOR_COUNT]  = {0.001f,0.001f,0.001f,0.001f,0.001f,0.001f};
static const float eso_w[MOTOR_COUNT]  = {80.f,  80.f,  80.f,  80.f,  80.f,  80.f};
static const float eso_b0[MOTOR_COUNT] = {1000.f,1000.f,1000.f,1000.f,1000.f,1000.f};

/* -------------------- 参考速度滤波（由一阶改为二阶） -------------------- */
/* [MOD] 原：一阶LPF参数 τ=30ms → 现：用 τ 推导二阶巴特沃斯LPF的 fc */
#define REF_LPF_TAU_S  0.03f  // τ=30ms，可按需 0.01~0.05 调整

/* [ADD] 二阶LPF（Biquad Butterworth）结构与实现 */
typedef struct {
  float b0, b1, b2, a1, a2;
  float x1, x2, y1, y2;
} BiquadLPF;

static BiquadLPF  g_refLPF2[MOTOR_COUNT];        // 每路参考的二阶LPF
static float      spdRef_filt[MOTOR_COUNT]={0.0f}; // 滤波后的速度参考
static uint8_t    spdRef_filt_inited = 0;          // 首次初始化标志

/* [ADD] 设计二阶 Butterworth 低通（RBJ公式），f0=fc，Q=1/sqrt(2) */
static void BiquadLPF_Design(BiquadLPF* f, float sample_hz, float fc_hz, float x0_prime)
{
  /* 保护 */
  if (fc_hz < 0.5f) fc_hz = 0.5f;
  if (fc_hz > 0.5f * sample_hz - 10.0f) fc_hz = 0.5f * sample_hz - 10.0f;
  const float Q     = 0.70710678f;                      // Butterworth
  const float w0    = 2.0f * (float)M_PI * fc_hz / sample_hz;
  const float cw    = cosf(w0);
  const float sw    = sinf(w0);
  const float alpha = sw / (2.0f * Q);

  float a0 = 1.0f + alpha;
  f->b0 = (1.0f - cw) * 0.5f / a0;
  f->b1 = (1.0f - cw)        / a0;
  f->b2 = (1.0f - cw) * 0.5f / a0;
  f->a1 = (-2.0f * cw)       / a0;
  f->a2 = (1.0f - alpha)     / a0;

  /* 预充：把状态置到当前输入，避免首拍阶跃 */
  f->x1 = x0_prime; f->x2 = x0_prime;
  f->y1 = x0_prime; f->y2 = x0_prime;
}

/* [ADD] 二阶LPF 单步计算 */
static inline float BiquadLPF_Step(BiquadLPF* f, float x)
{
  float y = f->b0*x + f->b1*f->x1 + f->b2*f->x2 - f->a1*f->y1 - f->a2*f->y2;
  f->x2 = f->x1; f->x1 = x;
  f->y2 = f->y1; f->y1 = y;
  return y;
}

/*
███████████████████████████████████████████████████████████████████▉██
*/
/**@bref: 速度环PID参数列表与初始化
	*
	*
*/
float spdRef[MOTOR_COUNT]={0.f};
//static float spdRef[MOTOR_COUNT]={ 50,      125,    -200,   -200,    -200,   -200  };
float spdRefTest[MOTOR_COUNT]=   { 50,      125,    -200,   -200,    -200,   -200  };

/*注意一定要和参数列表匹配*/
float spdMax[MOTOR_COUNT]={250, 250, 225, 225,  225, 225 };

static PID_HandleTypeDef pidSpd[MOTOR_COUNT];

typedef struct {//专门用于存放每个电机参数的结构体
	float Kp, Ki, Kd;
	float deadzone, dt;
	float outMin, outMax;
	float integralMin, integralMax;
} SpeedPID_Config_t;                   

static const SpeedPID_Config_t spdPidConfigs[MOTOR_COUNT]={//对应每路电机的 PID 参数表
  //{  Kp,     Ki,     Kd,   deadzone,    dt,      outMin,   outMax,     iMin,    iMax  }
    { 50.0f,  0.50f,	0.02f,   0.5f,    0.001f,   -1250.f,   1250.f,   -1250.f,  1250.f },// 大拇指①号电机(弯曲)
    { 50.0f,  0.50f, 	0.02f,   0.5f,    0.001f,   -1250.f,   1250.f,   -1250.f,  1250.f },// 大拇指②号电机(摆动)
    { 55.0f,  0.50f, 	0.02f,   0.5f,    0.001f,   -1250.f,   1250.f,   -1250.f,  1250.f },// 食指  ③号电机
    { 55.0f,  0.50f, 	0.02f,   0.5f,    0.001f,   -1250.f,   1250.f,   -1250.f,  1250.f },// 中指  ④号电机
	 { 55.0f,  0.50f, 	0.02f,   0.5f,    0.001f,   -1250.f,   1250.f,   -1250.f,  1250.f },// 无名指⑤号电机
	 { 55.0f,  0.50f, 	0.02f,   0.5f,    0.001f,   -1250.f,   1250.f,   -1250.f,  1250.f } // 小拇指⑥号电机
};

void SpeedControl_Init(void)//初始化PID和ESO
{
	ESO_ALLInit(eso, MOTOR_COUNT, eso_T, eso_w, eso_b0);/*eso初始化*/
	for(int i=0; i<MOTOR_COUNT; i++) 
	{
		const SpeedPID_Config_t *cfg = &spdPidConfigs[i];
		PID_Init(&pidSpd[i],
							cfg->Kp,cfg->Ki,cfg->Kd,
							cfg->deadzone,cfg->dt,
							cfg->outMin,cfg->outMax,
							cfg->integralMin,cfg->integralMax);
	}
}

/*███████████████████████████████████████████████████████████████████▉*/
/**@bref: 速度环1KHz控制
	*
	*
*/
void StartSpdLoopTask(void *argument)
{
	osEventFlagsWait(LaunchEventsHandle,LAUNCH_DONE_BIT,osFlagsWaitAny|osFlagsNoClear,osWaitForever);//阻塞直到 STARTUP_DONE_BIT 被置位// 等到所有位，此例就一个位 // 永久阻塞  
	const float alpha = SAMPLE_PERIOD_S / (0.050f + SAMPLE_PERIOD_S);                 //spdMea指数滤波系数 α = Ts/(τ+Ts)
	// [DEL] 一阶参考滤波系数不再需要
	// const float alpha_ref = SAMPLE_PERIOD_S / (REF_LPF_TAU_S + SAMPLE_PERIOD_S);
	float velFilt[ENCODER_CNT] = {0};
	const float factor = (float)ENCODER_PPR * SAMPLE_PERIOD_S;			 					 //换算因子：PPR × 采样周期

	for(;;)
	{
/*① 等1 kHz */ 
		osThreadFlagsWait(1U, osFlagsWaitAny, osWaitForever);

/*② 读最新一帧 delta（readIdx 指向刚写入的那一帧）*/ 
		uint16_t readIdx = (encWriteIdx + ENCODER_BUF_SIZE - 1) % ENCODER_BUF_SIZE;

		float spdMea[ENCODER_CNT];
		for (int i = 0; i < ENCODER_CNT; i++)
		{
			float inst 	= encBuf[readIdx].delta[i] / factor;			  //后出轴(圈/s)即rev/s：delta ÷ (PPR * Ts)
			velFilt[i] 	= alpha * inst + (1 - alpha) * velFilt[i];     //一阶 IIR
			spdMea[i] 	= velFilt[i];
		}
		DataManager_SetAllMotorSpeedsMea(spdMea, ENCODER_CNT);
		DataManager_Commit();

		DataManager_GetAllMotorSpeedsRef(spdRef,ENCODER_CNT);

		/* [ADD] 首次进入：设计并初始化“二阶巴特沃斯低通”并用当前参考预充 */
		if(!spdRef_filt_inited) 
		{
			const float Fs = 1.0f / SAMPLE_PERIOD_S;                // 采样频率（约1kHz）
			float fc = 1.0f / (2.0f * (float)M_PI * REF_LPF_TAU_S); // 用 τ 推导 fc
			if (fc > 0.45f * Fs) fc = 0.45f * Fs;                   // 保护，避免过高
			for(uint8_t i = 0; i < MOTOR_COUNT; ++i) 
			{
				spdRef_filt[i] = spdRef[i];
				BiquadLPF_Design(&g_refLPF2[i], Fs, fc, spdRef[i]);
			}
			spdRef_filt_inited = 1u;
		}

		/* [MOD] 把原来的一阶： y += α*(x - y)  改为  二阶Biquad：y = f(x) */
		for(uint8_t i = 0; i < MOTOR_COUNT; ++i) 
		{
			spdRef_filt[i] = BiquadLPF_Step(&g_refLPF2[i], spdRef[i]);
		}

/*③ ESO+PID实现速度闭环*/
		for(uint8_t k=0;k<MOTOR_COUNT;k++) 
		{
			//PID反馈  
			// [MOD] 用二阶滤波后的参考 spdRef_filt[k] 驱动 PID
			float fb = PID_CalcPosition(&pidSpd[k], spdRef_filt[k], spdMea[k]);
			//float fb = PID_CalcPosition(&pidSpd[k], spdRef[k], spdMea[k]);
			motor_duty[k] = fb;

			//  测量速度换成 rad/s 
			float y_rad_s = spdMea[k] * 6.2831853f;
			// z2 ≈ b0 * d  （d 为总扰动），所以电压补偿可取  u_comp_V = z2 / b0
			float u_comp_V   = eso[k].z2 / eso_b0[k];
			float duty_comp  = u_comp_V / 12.f * 1250.f;
			// 施加补偿并限幅 
			float duty_cmd = motor_duty[k] - (float)(duty_comp * valid[k]);
			if (duty_cmd >  1250.f) duty_cmd =  1250.f;
			if (duty_cmd < -1250.f) duty_cmd = -1250.f;
			//加入电流限幅
			duty_cmd = duty_cmd * sysctrl[k];
			Motor_SetDuty((Motor_Id)k, duty_cmd);
			//  把补偿后的占空比换算成等效电压，单位 V 
			float u_cmd_V =  duty_cmd / 1250.f * 12.f;
			ESO_Step(&eso[k], u_cmd_V, y_rad_s);
		}			
	}
}