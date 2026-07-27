#ifndef TRAP_TRAJ_PLANNER_H
#define TRAP_TRAJ_PLANNER_H

#include <stdbool.h>

/**
 * @brief 轨迹规划器的数据结构（带“中途换目标”时的初速度 v0 处理）
 *
 * 这个结构体支持以下几阶段：
 *   1. 【初始调整阶段】（init_phase）：
 *       - 场景 A：当前速度与目标方向相反 → “刹车到 0”（Brake to Zero）
 *       - 场景 B：当前速度与目标方向同向，但绝对值大于期望匀速 → “减速到 vt”（Decel to vt）
 *       - 否则（当前速度与目标方向同向且绝对值 ≤ vt，或 v0=0），直接进入“主阶段”
 *
 *   2. 【主运动阶段】：
 *       - 若剩余距离能支持“三段式”（加速到 vt → 匀速 vt → 减速到 0），则 main_phase = 1（Trapezoid）
 *       - 否则主阶段退化为“三角式”（先加速到峰值 v_peak < vt → 再减速到 0），则 main_phase = 2（Triangle）
 *
 * 下面各个成员保存了上述各阶段所需的时间、距离、拐点位置等参数，供 Update(...) 逐周期计算 x_ref, v_ref, a_ref。
 */
typedef struct
{
    /* ---------------- 用户输入参数 ---------------- */
    float x0;       		 ///< 初始位置 x0（单位：与编码器映射位置相同）
    float v0;       		 ///< 初始速度 v0（可以正、负或 0，单位：同 x/秒）
    float xf;       		 ///< 目标位置 xf（单位：同 x）
    float vt;       		 ///< 目标匀速大小 vt（正值，仅表示大小，方向由 sign(xf-x0) 决定）
    float amax;     		 ///< 最大加速度 a_max（正值，单位：同 x/秒2）
    float dt;       		 ///< 轨迹更新周期（秒），例如 0.01f 表示 100 Hz

    /* ---------------- 初始调整阶段（init_phase） ---------------- */
    int    init_phase;   ///< 0=无初始调整，1=“刹车到 0”，2=“减速到 vt”
    float  t_init;       ///< 初始阶段总用时（秒），若 init_phase=0 则为 0
    float  x_init;       ///< 初始阶段结束后的位置（作为主阶段起点）
    float  v_init;       ///< 初始阶段结束后速度（=0 if “刹车到 0”；=±vt if “减速到 vt”；否则 = v0）

    /* 如果 init_phase=1（刹车到 0）需要： */
    float  t_brake;      ///< 刹车到 0 所需时间 t_brake = |v0|/a_max
    float  d_brake;      ///< 刹车距离       d_brake = v02/(2·a_max)
    /* 如果 init_phase=2（减速到 vt）需要： */
    float  t_decel0;     ///< 从 |v0| 减速到 |vt| 所需时间 t_decel0 = (|v0| - vt)/a_max
    float  d_decel0;     ///< 减速距离          d_decel0 = (v02 - vt2)/(2·a_max)

    /* ---------------- 主运动阶段 ---------------- */
    int    main_phase;   ///< 1=三段式（Trapezoid），2=三角式（Triangle）
    /* 如果 main_phase=1（三段式），需要这些： */
    float  t_acc;        ///< 从 v_init → vt 的加速时间
    float  d_acc;        ///< 从 v_init → vt 的加速距离
    float  t_cruise;     ///< 匀速阶段时间（纯 vt 匀速），可能 = 0
    float  d_cruise;     ///< 匀速阶段距离 = vt · t_cruise
    float  t_dec;        ///< 从 vt → 0 的减速时间
    float  d_dec;        ///< 从 vt → 0 的减速距离
    /* 关键位置标记（用于 Update 中计算） */
    float  x_after_acc;  ///< 加速结束时的位置 x_i + sign·d_acc
    float  x_before_dec; ///< 减速开始时的位置 = x_after_acc + sign·d_cruise

    /* 如果 main_phase=2（三角式），需要这些： */
    float  v_peak;       ///< 峰值速度：v_peak = sqrt((2·a_max·|Δx_remain| + v_init2)/2)
    float  t_acc_peak;   ///< v_init → v_peak 所需时间 = (|v_peak| - |v_init|)/a_max
    float  d_acc_peak;   ///< v_init → v_peak 的加速距离 = (v_peak2 - v_init2)/(2·a_max)
    float  t_dec_peak;   ///< v_peak → 0 所需时间 = |v_peak|/a_max
    /* 峰值位置 x_peak = x_init + sign·d_acc_peak */

    /* ---------------- 运行时状态 ---------------- */
    unsigned int idx;    ///< 更新周期计数器，从 0 开始，Update 每次调用后 idx++
} TrapTrajPlanner;


/**
 * @brief  初始化梯形轨迹规划器（支持任意初速度 v0）。
 *
 * @param planner  [in,out]  待初始化的规划器地址
 * @param x0       [in]      当前测得位置 x0
 * @param v0       [in]      当前测得速度 v0（可正、可负、或零）
 * @param xf       [in]      目标位置 xf
 * @param vt       [in]      期望匀速大小 vt（正值，仅表示大小）
 * @param amax     [in]      最大加速度 a_max（正值）
 * @param dt       [in]      轨迹更新周期（秒），例如 0.01f 对应 100 Hz
 *
 * 本函数会按以下逻辑来构建“从 (x0, v0) → (xf, 0)”的轨迹（分 2 大阶段）：
 * 
 * 一、初始调整阶段 init_phase：
 *   1) 计算 方向 sign_target = sign(xf - x0)：若 xf>x0 则 +1，否则 -1。
 *   2) 判断 v0 与 sign_target 的关系：
 *      a) 如果 v0 * sign_target < 0（即 v0 与去目标的方向相反），
 *         则 init_phase=1（“刹车到 0”）：
 *           - t_brake = |v0|/a_max；  
 *           - d_brake = v02/(2·a_max)；  
 *           - x_init = x0 + (?sign(v0))·d_brake，v_init = 0；  
 *           - 剩余阶段从 (x_init, 0) → (xf, 0)。
 *      b) 否则如果 v0 * sign_target > 0 且 |v0| > vt（同方向但“跑得比期望匀速快”），
 *         则 init_phase=2（“减速到 vt”）：
 *           - t_decel0 = (|v0| - vt)/a_max；  
 *           - d_decel0 = (v02 - vt2)/(2·a_max)；  
 *           - x_init = x0 + sign_target·d_decel0；  
 *           - v_init = sign_target·vt；  
 *           - 剩余阶段从 (x_init, v_init) → (xf, 0)。
 *      c) 否则 init_phase=0（无需初始调整，直接用 v_init=v0, x_init=x0）。
 *
 * 二、主运动阶段 main_phase（在做完初始调整后）：
 *   1) 先计算剩余距离 abs_dx_remain = |xf - x_init|。
 *   2) 计算从 |v_init| → vt 的加速所需距离 d_acc、时间 t_acc：
 *        if |v_init| < vt:  
 *           t_acc = (vt - |v_init|)/a_max；  
 *           d_acc = (vt2 - v_init2)/(2·a_max)；  
 *        else （|v_init| == vt）: t_acc=0, d_acc=0。
 *   3) 计算从 vt → 0 的减速所需距离 d_dec = vt2/(2·a_max)，时间 t_dec = vt/a_max。
 *   4) 如果 abs_dx_remain ≥ (d_acc + d_dec)，则 main_phase=1（三段式）：
 *        - t_cruise = (abs_dx_remain - (d_acc + d_dec)) / vt；  
 *        - d_cruise = vt * t_cruise；  
 *        - 三段式总时长 = t_acc + t_cruise + t_dec；  
 *        - 关键位置 x_after_acc = x_init + sign_target·d_acc；  
 *        - 减速开始时位置 x_before_dec = x_after_acc + sign_target·d_cruise。
 *   5) 否则 main_phase=2（三角式）：
 *        - 计算峰值速度 v_peak = sqrt( (2·a_max·abs_dx_remain + v_init2)/2 )；  
 *        - t_acc_peak = (|v_peak| - |v_init|)/a_max；  
 *        - d_acc_peak = (v_peak2 - v_init2)/(2·a_max)；  
 *        - t_dec_peak = |v_peak|/a_max；  
 *        - 三角式总时长 = t_acc_peak + t_dec_peak；  
 *        - 峰值位置 x_peak = x_init + sign_target·d_acc_peak。
 * 
 * 三、初始化 idx=0，等待外部每周期调用 Update(...) 输出 (x_ref, v_ref, a_ref)。
 */
void TrapTrajPlanner_InitWithV0(
    TrapTrajPlanner *planner,
    float x0,
    float v0,
    float xf,
    float vt,
    float amax,
    float dt);


/**
 * @brief  更新一次轨迹，输出当前时刻的 (x_ref, v_ref, a_ref)。
 *
 * @param planner  [in,out]  已初始化的规划器结构体
 * @param x_ref    [out]     当前时刻参考位置
 * @param v_ref    [out]     当前时刻参考速度
 * @param a_ref    [out]     当前时刻参考加速度
 *
 * @return  true：轨迹已完成（后续保持 x_ref=xf, v_ref=0, a_ref=0）；  
 *          false：轨迹尚未结束，请继续调用。
 *
 * @note  本函数根据 idx 计算“全局时间” t = idx·dt，先判断是否处于“初始调整阶段”：
 *         - 如果 init_phase==1（刹车到 0），且 t < t_brake，则输出“刹车段”公式  
 *         - 如果 init_phase==2（减速到 vt），且 t < t_decel0，则输出“减速到 vt”公式  
 *         - 否则扣去初始阶段时间后进入“主运动阶段”，根据 main_phase=1（三段式）或=2（三角式）选择对应公式  
 *         - idx++ 并返回相应的标志。
 */
bool TrapTrajPlanner_Update(
    TrapTrajPlanner *planner,
    float *x_ref,
    float *v_ref,
    float *a_ref);

#endif // TRAP_TRAJ_PLANNER_H
