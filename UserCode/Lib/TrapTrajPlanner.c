#include "TrapTrajPlanner.h"
#include <math.h>  // sqrtf, fabsf


/*
███████████████████████████████████████████████████████████████████▉█████
*/
/**
 * @brief  初始化梯形轨迹规划器，支持任意初速度 v0。
 *
 * @param planner  [in,out]  待初始化的规划器结构体
 * @param x0       [in]      当前测得位置 x0
 * @param v0       [in]      当前测得速度 v0（可正、可负、或零）
 * @param xf       [in]      目标位置 xf
 * @param vt       [in]      期望匀速大小 vt（正值，仅表示大小）
 * @param amax     [in]      最大加速度 a_max（正值）
 * @param dt       [in]      轨迹更新周期（秒），例如 0.01f 对应 100 Hz
 */
void TrapTrajPlanner_InitWithV0(TrapTrajPlanner *planner,float x0,float v0,float xf,float vt,float amax,float dt)
{
    planner->x0    = x0;      																	// 记录初始位置 x0
    planner->v0    = v0;      																	// 记录初始速度 v0
    planner->xf    = xf;      																	// 记录目标位置 xf
    planner->vt    = (vt >= 0.0f ? vt : -vt);   								// 保证 vt 是正值
    planner->amax  = (amax >= 0.0f ? amax : -amax); 						// 保证 amax 是正值
    planner->dt    = dt;      																	// 记录更新周期 dt

    float delta = xf - x0;																			// 计算“到目标方向”sign_target;如果 xf>x0，说明目标在当前点正方向，sign_target=+1；否则=-1
    float sign_target = (delta >= 0.0f ? +1.0f : -1.0f);
    float abs_v0 = fabsf(v0);
    float abs_vt = planner->vt;  																// 已经是正值
    planner->init_phase = 0;  																	// 默认：无初始调整
    planner->t_init     = 0.0f;
    planner->x_init     = x0; 																	// 缺省主阶段起点就是 x0
    planner->v_init     = v0; 																	// 缺省主阶段起点速度就是 v0
	
/*①初始调整阶段*/
		/*A：当前速度方向与目标方向相反，需要先“刹车到 0”*/
    if(v0 * sign_target < 0.0f) 																//用条件(v0 * sign_target < 0)判断：“速度 v0”与“到目标的方向”相反
    {
			planner->init_phase = 1;  																//刹车到 0
			planner->t_brake = abs_v0 / planner->amax;								//计算刹车时间： t_brake = |v0| / a_max
			planner->d_brake = (abs_v0 * abs_v0)/(2.0f*planner->amax);//计算刹车距离： d_brake = (v02) / (2 · a_max)
			planner->v_init  = 0.0f;													     		//刹车后的速度 v_init
			planner->x_init  = x0 - sign_target * planner->d_brake;		//刹车后的位置 x_init
			planner->t_init  = planner->t_brake;							     		//刹车所用时间 t_init
    }
		/*B：当前速度同向且“比期望匀速跑得更快”→先“减速到vt”*/
    else if((v0 * sign_target > 0.0f) && (abs_v0 > abs_vt))
    {
			planner->init_phase = 2;  																//减速到 vt 
			planner->t_decel0 = (abs_v0 - abs_vt) / planner->amax;		//从 |v0| → abs_vt 所需时间： t_decel0 = (|v0| - abs_vt)/a_max
			planner->d_decel0 = (abs_v0 * abs_v0 - abs_vt * abs_vt)		//相应的减速距离： d_decel0 = (v02 - vt2) / (2 · a_max)
													/ (2.0f * planner->amax);
			planner->v_init = sign_target * abs_vt;										//减速结束后，速度 = sign_target·abs_vt																											
			planner->x_init = x0 + sign_target * planner->d_decel0;   //因此  x_after_decel0 = x0 + sign_target·d_decel0
			planner->t_init = planner->t_decel0;											//初始调整阶段总用时 = 减速到 vt 时间
    }
		/*C：否则 init_phase=0（无需初始调整），x_init=x0, v_init=v0, t_init=0*/ 
		
		
/*②主动运动阶段*/ 
		/*A：先计算剩余距离，调整后的速度，加速时间与距离，减速时间与距离*/
    float abs_dx_remain = fabsf(xf - planner->x_init);					//先计算出剩余距离
    float abs_vinit = fabsf(planner->v_init);										//调整后的速度大小
    if(abs_vinit < abs_vt)                                      //如果调整后的速度小于期望速度，计算加速时间与距离
    {
			planner->t_acc = (abs_vt - abs_vinit) / planner->amax;		//加速时间：t_acc = (|vt| - |v_init|) / a_max
			planner->d_acc = (abs_vt * abs_vt - abs_vinit * abs_vinit)//加速距离：d_acc = (vt2 - v_init2) / (2 · a_max)
											 / (2.0f * planner->amax);
    }
    else
    {    
			planner->t_acc = 0.0f;																		//已经 v_init 的绝对值 ≥ vt，则无需“主阶段加速到 vt”
			planner->d_acc = 0.0f;
    }
    planner->t_dec = abs_vt / planner->amax;										//计算“从 |vt| → 0”的减速所需距离 d_dec 和时间 t_dec：
    planner->d_dec = (abs_vt * abs_vt) / (2.0f * planner->amax);
		
		/*B：采用梯形*/
    if (abs_dx_remain >= (planner->d_acc + planner->d_dec))			//如果 abs_dx_remain ≥ (d_acc + d_dec)，就分三段：
    {
			planner->main_phase = 1;																	//主阶段采用三段式
			float d_const = abs_dx_remain - (planner->d_acc + planner->d_dec);
			planner->t_cruise = d_const / abs_vt;  										//计算匀速阶段所需时间 t_cruise = (残余距离 - (d_acc + d_dec)) / vt
			planner->d_cruise = abs_vt * planner->t_cruise;  					//匀速距离
			planner->x_after_acc = planner->x_init + sign_target * planner->d_acc;				 //记录加速结束时的位置 
			planner->x_before_dec = planner->x_after_acc + sign_target * planner->d_cruise;//记录减速开始时的位置
    }
		/*C：采用三角形*/
    else
    {
			planner->main_phase = 2;																	//主阶段采用三角式（Triangle）
			float numerator = 2.0f * planner->amax * abs_dx_remain + (abs_vinit * abs_vinit);//(v_peak2-v_init2)/(2a)+(v_peak2)/(2a)=abs_dx_remain
			float vpeak_sq = numerator / 2.0f;
			if (vpeak_sq < 0.0f) 																			//理论上不该出现，这里防止数值误差
			{
					vpeak_sq = 0.0f;
			}
			planner->v_peak = sqrtf(vpeak_sq);
			if (planner->v_peak > abs_vt) 														//理论上不该出现，因为 abs_dx_remain < d_acc + d_dec → v_peak < vt
			{
					planner->v_peak = abs_vt;
			}
			planner->t_acc_peak = (planner->v_peak - abs_vinit) / planner->amax;						 //计算加速到 v_peak 所需时间
			planner->d_acc_peak = (planner->v_peak * planner->v_peak - abs_vinit * abs_vinit)//计算加速距离
														/ (2.0f * planner->amax);	
			planner->t_dec_peak = planner->v_peak / planner->amax;		//计算减速到 0 所需时间 t_dec_peak = v_peak / a_max

																																// 峰值位置 x_peak = x_init + sign_target·d_acc_peak
																																// （后面 Update 时可用此 x_peak 辅助验证）
																																// 不必单独存储为成员，遇到需要可按此公式再算。
    }
    planner->idx = 0U;																				  //初始化 idx=0，等待后续 Update(...) 调用
}

/*
███████████████████████████████████████████████████████████████████▉█████
*/
/**
 * @brief  更新一次轨迹，输出当下 (x_ref, v_ref, a_ref)。
 *
 * @param planner  [in,out]  已初始化的规划器
 * @param x_ref    [out]     本次参考位置
 * @param v_ref    [out]     本次参考速度
 * @param a_ref    [out]     本次参考加速度
 *
 * @return  true：轨迹已完成（之后始终保持 x_ref=xf, v_ref=0, a_ref=0）  
 *          false：轨迹尚未结束，请继续调用
 */
bool TrapTrajPlanner_Update(TrapTrajPlanner *planner,float *x_ref,float *v_ref,float *a_ref)
{
/*① 计算全局时间t=idx·dt*/
	float t = planner->idx * planner->dt;

/*② 判断是否处于“初始调整阶段”*/
	if(planner->init_phase == 1)
	{
		if(t < planner->t_brake)																 //如果 t < t_brake，就在刹车阶段
		{
			float sign_v0 = (planner->v0 >= 0.0f ? +1.0f : -1.0f);
			*a_ref = -sign_v0 * planner->amax;                     //计算刹车加速度 a_ref
			*v_ref = planner->v0 + (*a_ref) * t;									 //计算刹车过程中的 v_ref(t)
			*x_ref = planner->x0 + planner->v0*t+0.5f*(*a_ref)*t*t;//计算刹车过程中的 x_ref(t)
			planner->idx++;																				 // 增加 idx，结束本次 Update
			return false;
		}
		t -= planner->t_brake;																		 //如果刹车时间已过，扣掉刹车时间，进入主阶段
	}
	else if(planner->init_phase == 2)
	{
		if (t < planner->t_decel0)														 	 //减速到 vt
		{
			float sign_target = (planner->xf >= planner->x0 ? +1.0f : -1.0f);
			*a_ref = -planner->amax * sign_target;								 //减速加速度 a_ref = ?amax·sign_target（因为与目标同向但要减速）
			*v_ref = planner->v0 + (*a_ref) * t;									 //当前速度 v_ref(t) = v0 + a_ref·t
			*x_ref = planner->x0 + planner->v0 * t + 0.5f * (*a_ref) * t * t;//当前位置 x_ref(t)
			planner->idx++;
			return false;
		}
		t -= planner->t_decel0;																		 // 如果“减速到 vt”阶段已过，扣掉减速时间，进入主阶段
	}
	
/*③ 进入主动阶段*/
	float sign_target = (planner->xf >= planner->x_init ? +1.0f : -1.0f);
	
	/*A：三段梯形*/
	if(planner->main_phase == 1)
  {
float abs_vt = planner->vt;//期望匀速（正值）
		float t1 = planner->t_acc;      // 加速用时
		float t2 = planner->t_cruise;   // 匀速用时
		float t3 = planner->t_dec;      // 减速用时

		if(t < t1)															//阶段 1：加速段
		{
			*a_ref = +planner->amax * sign_target;//加速度 a_ref = +amax·sign_target
			float abs_vinit = fabsf(planner->v_init);//速度 v_ref(t) = |v_init| + a_max·t，同号
      *v_ref = (abs_vinit + planner->amax * t) * sign_target;
			*x_ref = planner->x_init+ sign_target * (abs_vinit * t + 0.5f * planner->amax * t * t);//位置 x_ref(t) = x_init + sign_target·[ |v_init|·t + 0.5·a_max·t2 ]
			planner->idx++;
			return false;
		}
		else if(t < (t1 + t2))									//阶段 2：匀速段
		{
			float tau = t - t1;  									// 相对匀速阶段起点的时间 
			*a_ref = 0.0f;												//匀速加速度 a_ref = 0
			*v_ref = planner->vt * sign_target;		//匀速速度 v_ref = vt·sign_target 
			*x_ref = planner->x_after_acc + sign_target * (planner->vt * tau);//位置 x_ref = x_after_acc + sign_target·(vt·tau)
			planner->idx++;
			return false;
		}
		else if (t < (t1 + t2 + t3))						//阶段 3：减速段
		{
			float t_d = t - (t1 + t2);  					// 相对减速阶段起点的时间
			*a_ref = -planner->amax * sign_target;//减速加速度 a_ref = ?amax·sign_target
	    *v_ref = (planner->vt - planner->amax * t_d) * sign_target;//v_ref(t) = vt·sign_target ? a_max·t_d·sign_target
			float rem = t3 - t_d;
			*x_ref = planner->xf - sign_target * (0.5f * planner->amax * rem * rem);//位置 x_ref = xf ? sign_target·[0.5·a_max·(rem)2]，其中 rem = t3 ? t_d
			planner->idx++;
			return false;
		}
		else																		//阶段 4：轨迹完全结束
		{
			*a_ref = 0.0f;
			*v_ref = 0.0f;
			*x_ref = planner->xf;                 // 始终保持 f 点
			return true;
		}
	}
	/*B：三角形*/
	else
	{
float abs_vinit = fabsf(planner->v_init);
		float t1 = planner->t_acc_peak;   			// 加速到 v_peak 时间
		float t2 = planner->t_dec_peak;   			// 从 v_peak → 0 时间
		float t_tot = t1 + t2;            			// 三角式总时长

		if (t < t1)															//阶段 1：加速段
		{
			*a_ref = +planner->amax * sign_target;//加速度 a_ref = +amax·sign_target
			float abs_vinit = fabsf(planner->v_init);
			*v_ref = (abs_vinit + planner->amax * t) * sign_target; //速度 v_ref(t) = |v_init| + a_max·t，同号
			*x_ref = planner->x_init+ sign_target * (abs_vinit * t + 0.5f * planner->amax * t * t);//位置 x_ref(t) = x_init + sign_target·[ |v_init|·t + 0.5·a_max·t2 ]
			planner->idx++;
			return false;
		}
		else if (t < t_tot)											//阶段 2：减速段
		{
			float t_d = t - t1;  									// 相对减速阶段起点的时间
			*a_ref = -planner->amax * sign_target;//减速加速度 a_ref = ?amax·sign_target
			*v_ref = (planner->v_peak - planner->amax * t_d) * sign_target;//速度 v_ref(t) = v_peak·sign_target ? a_max·t_d·sign_target
			float rem = t2 - t_d;
			*x_ref = planner->xf - sign_target * (0.5f * planner->amax * rem * rem);//位置 x_ref = xf ? sign_target·[0.5·a_max·(rem)2]，其中 rem = t2 ? t_d
			planner->idx++;
			return false;
		}
		else                      							//阶段 3：轨迹结束
		{
			*a_ref = 0.0f;
			*v_ref = 0.0f;
			*x_ref = planner->xf;
			return true;
		}
	}
}
