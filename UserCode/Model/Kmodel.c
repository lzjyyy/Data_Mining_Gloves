/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Kmodel.c
 *
 * Code generated for Simulink model 'Kmodel'.
 *
 * Model version                  : 1.33
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Tue Nov 25 11:22:12 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Kmodel.h"
#include "rtwtypes.h"
#include <math.h>

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void MATLABFunction(real_T rtu_u, real_T *rty_y);
static void MATLABFunction1(real_T rtu_u, real_T *rty_y);

/*
 * Output and update for atomic system:
 *    '<S1>/MATLAB Function'
 *    '<S1>/MATLAB Function2'
 *    '<S1>/MATLAB Function3'
 *    '<S1>/MATLAB Function4'
 *    '<S1>/MATLAB Function5'
 *    '<S1>/MATLAB Function6'
 */
static void MATLABFunction(real_T rtu_u, real_T *rty_y)
{
  *rty_y = rtu_u;
  if (rtu_u <= 0.0) {
    *rty_y = 0.0;
  }
}

/*
 * Output and update for atomic system:
 *    '<S1>/MATLAB Function1'
 *    '<S1>/MATLAB Function10'
 *    '<S1>/MATLAB Function11'
 *    '<S1>/MATLAB Function7'
 *    '<S1>/MATLAB Function8'
 *    '<S1>/MATLAB Function9'
 */
static void MATLABFunction1(real_T rtu_u, real_T *rty_y)
{
  *rty_y = rtu_u - 0.0014;
}

/* Model step function */
void Kmodel_step(void)
{
  real_T rtb_Exp_af[6];
  real_T rtb_Exp_ay[6];
  real_T rtb_Exp_fn[6];
  real_T rtb_Exp_gn[6];
  real_T rtb_Exp_o2[6];
  real_T tmp[6];
  real_T tmp_5[5];
  real_T rtb_DotProduct_i;
  real_T rtb_Sum1_a_idx_0;
  real_T rtb_Sum1_a_idx_1;
  real_T rtb_Sum1_p_0;
  real_T tmp_0;
  real_T tmp_1;
  real_T tmp_2;
  real_T tmp_3;
  real_T tmp_4;
  real_T u;
  real_T u_0;
  real_T u_1;
  real_T u_2;
  real_T y;
  real_T y_0;
  real_T y_1;
  real_T y_2;
  real_T y_3;
  int32_T i;

  /* Outputs for Atomic SubSystem: '<Root>/Kmodel' */
  /* Bias: '<S293>/Add min y' incorporates:
   *  Gain: '<S293>/range y // range x'
   *  Inport: '<Root>/Tar_muzhi_baidong_jiaosudu'
   *  Inport: '<Root>/now_M2_pos'
   */
  rtb_Sum1_a_idx_0 = 0.011994683715430247 * rtU.Tar_muzhi_baidong_jiaosudu - 1.0;
  rtb_Sum1_a_idx_1 = 0.010450205283362327 * rtU.now_M2_pos - 1.0;

  /* Sum: '<S254>/netsum' incorporates:
   *  Constant: '<S254>/b{1}'
   *  Constant: '<S261>/IW{1,1}(1,:)''
   *  Constant: '<S261>/IW{1,1}(2,:)''
   *  Constant: '<S261>/IW{1,1}(3,:)''
   *  Constant: '<S261>/IW{1,1}(4,:)''
   *  Constant: '<S261>/IW{1,1}(5,:)''
   *  Constant: '<S261>/IW{1,1}(6,:)''
   *  DotProduct: '<S263>/Dot Product'
   *  DotProduct: '<S264>/Dot Product'
   *  DotProduct: '<S265>/Dot Product'
   *  DotProduct: '<S266>/Dot Product'
   *  DotProduct: '<S267>/Dot Product'
   *  DotProduct: '<S268>/Dot Product'
   *  Gain: '<S262>/Gain'
   */
  tmp[0] = ((1.2831152658900689 * rtb_Sum1_a_idx_0 + -2.6464820346052766 *
             rtb_Sum1_a_idx_1) - 3.2223862763874949) * -2.0;
  tmp[1] = ((-2.0978112416178472 * rtb_Sum1_a_idx_0 + 0.97610882280670774 *
             rtb_Sum1_a_idx_1) + 2.9017492344818456) * -2.0;
  tmp[2] = ((-1.7611079282434368 * rtb_Sum1_a_idx_0 + -0.65523946770868147 *
             rtb_Sum1_a_idx_1) + 1.0214603067374655) * -2.0;
  tmp[3] = ((-0.32882152739519044 * rtb_Sum1_a_idx_0 + 0.93034260513546974 *
             rtb_Sum1_a_idx_1) - 0.34920401285894687) * -2.0;
  tmp[4] = ((-2.4894604333138233 * rtb_Sum1_a_idx_0 + -0.994997239089603 *
             rtb_Sum1_a_idx_1) - 1.285329428802773) * -2.0;
  tmp[5] = ((2.6208925368637779 * rtb_Sum1_a_idx_0 + 1.2258775465050498 *
             rtb_Sum1_a_idx_1) + 3.50051230424322) * -2.0;

  /* DotProduct: '<S272>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S273>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S274>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S275>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S276>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S277>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S278>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;

  /* DotProduct: '<S279>/Dot Product' */
  y = 0.0;
  for (i = 0; i < 6; i++) {
    /* DotProduct: '<S272>/Dot Product' incorporates:
     *  Constant: '<S262>/one'
     *  Constant: '<S262>/one1'
     *  Constant: '<S270>/IW{2,1}(1,:)''
     *  DotProduct: '<S273>/Dot Product'
     *  DotProduct: '<S274>/Dot Product'
     *  DotProduct: '<S275>/Dot Product'
     *  DotProduct: '<S276>/Dot Product'
     *  DotProduct: '<S277>/Dot Product'
     *  DotProduct: '<S278>/Dot Product'
     *  DotProduct: '<S279>/Dot Product'
     *  Gain: '<S262>/Gain1'
     *  Math: '<S262>/Exp'
     *  Math: '<S262>/Reciprocal'
     *  Sum: '<S262>/Sum'
     *  Sum: '<S262>/Sum1'
     *
     * About '<S262>/Exp':
     *  Operator: exp
     *
     * About '<S262>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_p_0 = 1.0 / (exp(tmp[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += rtb_Sum1_p_0 * rtConstP.IW211_Value_k[i];

    /* DotProduct: '<S273>/Dot Product' incorporates:
     *  Constant: '<S270>/IW{2,1}(2,:)''
     */
    tmp_1 += rtb_Sum1_p_0 * rtConstP.IW212_Value_f[i];

    /* DotProduct: '<S274>/Dot Product' incorporates:
     *  Constant: '<S270>/IW{2,1}(3,:)''
     */
    tmp_2 += rtb_Sum1_p_0 * rtConstP.IW213_Value_b[i];

    /* DotProduct: '<S275>/Dot Product' incorporates:
     *  Constant: '<S270>/IW{2,1}(4,:)''
     */
    tmp_3 += rtb_Sum1_p_0 * rtConstP.IW214_Value_p[i];

    /* DotProduct: '<S276>/Dot Product' incorporates:
     *  Constant: '<S270>/IW{2,1}(5,:)''
     */
    tmp_4 += rtb_Sum1_p_0 * rtConstP.IW215_Value_k[i];

    /* DotProduct: '<S277>/Dot Product' incorporates:
     *  Constant: '<S270>/IW{2,1}(6,:)''
     */
    rtb_DotProduct_i += rtb_Sum1_p_0 * rtConstP.IW216_Value_e[i];

    /* DotProduct: '<S278>/Dot Product' incorporates:
     *  Constant: '<S270>/IW{2,1}(7,:)''
     */
    rtb_Sum1_a_idx_0 += rtb_Sum1_p_0 * rtConstP.IW217_Value_j[i];

    /* DotProduct: '<S279>/Dot Product' incorporates:
     *  Constant: '<S270>/IW{2,1}(8,:)''
     */
    y += rtb_Sum1_p_0 * rtConstP.IW218_Value[i];
  }

  /* Sum: '<S255>/netsum' incorporates:
   *  Constant: '<S255>/b{2}'
   *  Gain: '<S271>/Gain'
   */
  rtDW.dv[0] = (tmp_0 - 2.3512422964212965) * -2.0;
  rtDW.dv[1] = (tmp_1 - 1.2181206442580335) * -2.0;
  rtDW.dv[2] = (tmp_2 + 0.53656833067199072) * -2.0;
  rtDW.dv[3] = (tmp_3 + 0.47116193213709434) * -2.0;
  rtDW.dv[4] = (tmp_4 - 0.97734253965696738) * -2.0;
  rtDW.dv[5] = (rtb_DotProduct_i + 0.63226050291102165) * -2.0;
  rtDW.dv[6] = (rtb_Sum1_a_idx_0 - 1.561181270744717) * -2.0;
  rtDW.dv[7] = (y + 1.8144378384189934) * -2.0;

  /* DotProduct: '<S283>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S284>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S285>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S286>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S287>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S288>/Dot Product' */
  rtb_DotProduct_i = 0.0;
  for (i = 0; i < 8; i++) {
    /* Sum: '<S271>/Sum1' incorporates:
     *  Constant: '<S271>/one'
     *  Constant: '<S271>/one1'
     *  Gain: '<S271>/Gain1'
     *  Math: '<S271>/Exp'
     *  Math: '<S271>/Reciprocal'
     *  Sum: '<S271>/Sum'
     *
     * About '<S271>/Exp':
     *  Operator: exp
     *
     * About '<S271>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_p_0 = 1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0;
    rtDW.Sum1_p[i] = rtb_Sum1_p_0;

    /* DotProduct: '<S283>/Dot Product' incorporates:
     *  Constant: '<S281>/IW{3,2}(1,:)''
     */
    tmp_0 += rtConstP.IW321_Value_i[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S284>/Dot Product' incorporates:
     *  Constant: '<S281>/IW{3,2}(2,:)''
     */
    tmp_1 += rtConstP.IW322_Value_e[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S285>/Dot Product' incorporates:
     *  Constant: '<S281>/IW{3,2}(3,:)''
     */
    tmp_2 += rtConstP.IW323_Value_i[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S286>/Dot Product' incorporates:
     *  Constant: '<S281>/IW{3,2}(4,:)''
     */
    tmp_3 += rtConstP.IW324_Value_e[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S287>/Dot Product' incorporates:
     *  Constant: '<S281>/IW{3,2}(5,:)''
     */
    tmp_4 += rtConstP.IW325_Value_n[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S288>/Dot Product' incorporates:
     *  Constant: '<S281>/IW{3,2}(6,:)''
     */
    rtb_DotProduct_i += rtConstP.IW326_Value_n[i] * rtb_Sum1_p_0;
  }

  /* Sum: '<S256>/netsum' incorporates:
   *  Constant: '<S256>/b{3}'
   */
  rtDW.Sum1_p[0] = tmp_0 + 1.7948802403510855;
  rtDW.Sum1_p[1] = tmp_1 - 1.4680665765989145;
  rtDW.Sum1_p[2] = tmp_2 + 0.40390749240746204;
  rtDW.Sum1_p[3] = tmp_3 - 0.6830399269702091;
  rtDW.Sum1_p[4] = tmp_4 + 0.48690996974435347;
  rtDW.Sum1_p[5] = rtb_DotProduct_i - 1.7497042699102594;

  /* DotProduct: '<S292>/Dot Product' */
  tmp_0 = 0.0;
  for (i = 0; i < 6; i++) {
    /* Sum: '<S282>/Sum1' incorporates:
     *  Constant: '<S282>/one'
     *  Constant: '<S282>/one1'
     *  Gain: '<S282>/Gain'
     *  Gain: '<S282>/Gain1'
     *  Math: '<S282>/Exp'
     *  Math: '<S282>/Reciprocal'
     *  Sum: '<S282>/Sum'
     *
     * About '<S282>/Exp':
     *  Operator: exp
     *
     * About '<S282>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_p_0 = 1.0 / (exp(-2.0 * rtDW.Sum1_p[i]) + 1.0) * 2.0 - 1.0;
    rtDW.Sum1_p[i] = rtb_Sum1_p_0;

    /* DotProduct: '<S292>/Dot Product' incorporates:
     *  Constant: '<S290>/IW{4,3}(1,:)''
     */
    tmp_0 += rtConstP.IW431_Value_m[i] * rtb_Sum1_p_0;
  }

  /* MATLAB Function: '<S1>/MATLAB Function2' incorporates:
   *  Bias: '<S294>/Subtract min y'
   *  Gain: '<S294>/Divide by range y'
   *  Outport: '<Root>/Tar_M2_spd'
   *  Sum: '<S257>/netsum'
   */
  MATLABFunction(124.9249849969994 * ((tmp_0 - 0.81804251753119861) + 1.0),
                 &rtY.Tar_M2_spd);

  /* Bias: '<S324>/Add min y' incorporates:
   *  Gain: '<S324>/range y // range x'
   *  Inport: '<Root>/now_M1_pos1'
   *  Inport: '<Root>/now_M1_spd'
   */
  rtb_Sum1_a_idx_0 = 0.012223966159175996 * rtU.now_M1_pos1 - 1.0;
  rtb_Sum1_a_idx_1 = 0.0080048038430744588 * rtU.now_M1_spd - 1.0;

  /* Sum: '<S295>/netsum' incorporates:
   *  Constant: '<S295>/b{1}'
   *  Constant: '<S301>/IW{1,1}(1,:)''
   *  Constant: '<S301>/IW{1,1}(2,:)''
   *  Constant: '<S301>/IW{1,1}(3,:)''
   *  Constant: '<S301>/IW{1,1}(4,:)''
   *  Constant: '<S301>/IW{1,1}(5,:)''
   *  Constant: '<S301>/IW{1,1}(6,:)''
   *  DotProduct: '<S303>/Dot Product'
   *  DotProduct: '<S304>/Dot Product'
   *  DotProduct: '<S305>/Dot Product'
   *  DotProduct: '<S306>/Dot Product'
   *  DotProduct: '<S307>/Dot Product'
   *  DotProduct: '<S308>/Dot Product'
   *  Gain: '<S302>/Gain'
   */
  tmp[0] = ((0.67683595451560119 * rtb_Sum1_a_idx_0 + -2.9380673272354554 *
             rtb_Sum1_a_idx_1) - 3.085487284121796) * -2.0;
  tmp[1] = ((1.2165197263566212 * rtb_Sum1_a_idx_0 + 0.26163863489506856 *
             rtb_Sum1_a_idx_1) - 1.959856387444167) * -2.0;
  tmp[2] = ((-0.19035176635435158 * rtb_Sum1_a_idx_0 + 1.3898891779729434 *
             rtb_Sum1_a_idx_1) + 0.60834832793651028) * -2.0;
  tmp[3] = ((0.14573705322393621 * rtb_Sum1_a_idx_0 + -1.8467188583305731 *
             rtb_Sum1_a_idx_1) + 1.5139321445547844) * -2.0;
  tmp[4] = ((0.74000612857164394 * rtb_Sum1_a_idx_0 + 0.027569019900167538 *
             rtb_Sum1_a_idx_1) + 0.58195610141587828) * -2.0;
  tmp[5] = ((-2.2623523207625338 * rtb_Sum1_a_idx_0 + -1.3973092750121079 *
             rtb_Sum1_a_idx_1) - 3.7618341743216814) * -2.0;

  /* DotProduct: '<S312>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S313>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S314>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S315>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S316>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S317>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S318>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;

  /* DotProduct: '<S319>/Dot Product' */
  y = 0.0;
  for (i = 0; i < 6; i++) {
    /* Sum: '<S302>/Sum1' incorporates:
     *  Constant: '<S302>/one'
     *  Constant: '<S302>/one1'
     *  Gain: '<S302>/Gain1'
     *  Math: '<S302>/Exp'
     *  Math: '<S302>/Reciprocal'
     *  Sum: '<S302>/Sum'
     *
     * About '<S302>/Exp':
     *  Operator: exp
     *
     * About '<S302>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_p_0 = 1.0 / (exp(tmp[i]) + 1.0) * 2.0 - 1.0;

    /* DotProduct: '<S312>/Dot Product' incorporates:
     *  Constant: '<S310>/IW{2,1}(1,:)''
     */
    tmp_0 += rtConstP.IW211_Value_m[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S313>/Dot Product' incorporates:
     *  Constant: '<S310>/IW{2,1}(2,:)''
     */
    tmp_1 += rtConstP.IW212_Value_j[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S314>/Dot Product' incorporates:
     *  Constant: '<S310>/IW{2,1}(3,:)''
     */
    tmp_2 += rtConstP.IW213_Value_n[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S315>/Dot Product' incorporates:
     *  Constant: '<S310>/IW{2,1}(4,:)''
     */
    tmp_3 += rtConstP.IW214_Value_o[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S316>/Dot Product' incorporates:
     *  Constant: '<S310>/IW{2,1}(5,:)''
     */
    tmp_4 += rtConstP.IW215_Value_i[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S317>/Dot Product' incorporates:
     *  Constant: '<S310>/IW{2,1}(6,:)''
     */
    rtb_DotProduct_i += rtConstP.IW216_Value_k[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S318>/Dot Product' incorporates:
     *  Constant: '<S310>/IW{2,1}(7,:)''
     */
    rtb_Sum1_a_idx_0 += rtConstP.IW217_Value_k[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S319>/Dot Product' incorporates:
     *  Constant: '<S310>/IW{2,1}(8,:)''
     */
    y += rtConstP.IW218_Value_d[i] * rtb_Sum1_p_0;
  }

  /* Sum: '<S296>/netsum' incorporates:
   *  Constant: '<S296>/b{2}'
   *  Gain: '<S311>/Gain'
   */
  rtDW.dv[0] = (tmp_0 - 1.827239642293484) * -2.0;
  rtDW.dv[1] = (tmp_1 + 1.5880460808073849) * -2.0;
  rtDW.dv[2] = (tmp_2 - 1.090932621247201) * -2.0;
  rtDW.dv[3] = (tmp_3 + 0.31713442811987247) * -2.0;
  rtDW.dv[4] = (tmp_4 + 0.41275656628127244) * -2.0;
  rtDW.dv[5] = (rtb_DotProduct_i + 0.26537777996287032) * -2.0;
  rtDW.dv[6] = (rtb_Sum1_a_idx_0 + 1.4799932488953793) * -2.0;
  rtDW.dv[7] = (y - 2.1915168727308854) * -2.0;

  /* DotProduct: '<S323>/Dot Product' incorporates:
   *  Constant: '<S311>/one'
   *  Constant: '<S311>/one1'
   *  Constant: '<S321>/IW{3,2}(1,:)''
   *  Gain: '<S311>/Gain1'
   *  Math: '<S311>/Exp'
   *  Math: '<S311>/Reciprocal'
   *  Sum: '<S311>/Sum'
   *  Sum: '<S311>/Sum1'
   *
   * About '<S311>/Exp':
   *  Operator: exp
   *
   * About '<S311>/Reciprocal':
   *  Operator: reciprocal
   */
  tmp_0 = 0.0;
  for (i = 0; i < 8; i++) {
    tmp_0 += (1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0) *
      rtConstP.IW321_Value_m[i];
  }

  /* End of DotProduct: '<S323>/Dot Product' */

  /* MATLAB Function: '<S1>/MATLAB Function1' incorporates:
   *  Bias: '<S325>/Subtract min y'
   *  Constant: '<S297>/b{3}'
   *  Gain: '<S325>/Divide by range y'
   *  Outport: '<Root>/now_muzhi_wanqu_jiaosudu_2'
   *  Sum: '<S297>/netsum'
   */
  MATLABFunction1(61.314015555217296 * ((tmp_0 + 0.70446715982399777) + 1.0),
                  &rtY.now_muzhi_wanqu_jiaosudu_2);

  /* Bias: '<S543>/Add min y' incorporates:
   *  Gain: '<S543>/range y // range x'
   *  Inport: '<Root>/now_M3_pos1'
   *  Inport: '<Root>/now_M3_spd'
   */
  rtb_Sum1_a_idx_0 = 0.011442940347073955 * rtU.now_M3_pos1 - 1.0;
  rtb_Sum1_a_idx_1 = 0.0088948200869909053 * rtU.now_M3_spd - 1.0;

  /* Sum: '<S500>/netsum' incorporates:
   *  Constant: '<S500>/b{1}'
   *  Constant: '<S507>/IW{1,1}(1,:)''
   *  Constant: '<S507>/IW{1,1}(2,:)''
   *  Constant: '<S507>/IW{1,1}(3,:)''
   *  Constant: '<S507>/IW{1,1}(4,:)''
   *  Constant: '<S507>/IW{1,1}(5,:)''
   *  Constant: '<S507>/IW{1,1}(6,:)''
   *  Constant: '<S507>/IW{1,1}(7,:)''
   *  Constant: '<S507>/IW{1,1}(8,:)''
   *  DotProduct: '<S509>/Dot Product'
   *  DotProduct: '<S510>/Dot Product'
   *  DotProduct: '<S511>/Dot Product'
   *  DotProduct: '<S512>/Dot Product'
   *  DotProduct: '<S513>/Dot Product'
   *  DotProduct: '<S514>/Dot Product'
   *  DotProduct: '<S515>/Dot Product'
   *  DotProduct: '<S516>/Dot Product'
   *  Gain: '<S508>/Gain'
   */
  rtDW.dv[0] = ((-1.8883213107036583 * rtb_Sum1_a_idx_0 + 2.7264405419151014 *
                 rtb_Sum1_a_idx_1) + 3.9372713672464874) * -2.0;
  rtDW.dv[1] = ((2.3434620813347893 * rtb_Sum1_a_idx_0 + -0.56861021889814367 *
                 rtb_Sum1_a_idx_1) - 2.2568037376489092) * -2.0;
  rtDW.dv[2] = ((-1.9225325006688625 * rtb_Sum1_a_idx_0 + 2.0591606516062329 *
                 rtb_Sum1_a_idx_1) + 1.6123622829928963) * -2.0;
  rtDW.dv[3] = ((1.9182925442906105 * rtb_Sum1_a_idx_0 + -1.0440186024819671 *
                 rtb_Sum1_a_idx_1) - 0.69544501103659129) * -2.0;
  rtDW.dv[4] = ((-0.79579906702584513 * rtb_Sum1_a_idx_0 + 1.2263520013200968 *
                 rtb_Sum1_a_idx_1) - 0.66057142133070812) * -2.0;
  rtDW.dv[5] = ((-1.9426523141840821 * rtb_Sum1_a_idx_0 + 1.6738162619001831 *
                 rtb_Sum1_a_idx_1) - 1.8347480668887592) * -2.0;
  rtDW.dv[6] = ((-2.6525139739129369 * rtb_Sum1_a_idx_0 + -0.081371677390578476 *
                 rtb_Sum1_a_idx_1) - 1.6683402663806088) * -2.0;
  rtDW.dv[7] = ((-3.3680624132745285 * rtb_Sum1_a_idx_0 + 0.75059375984035459 *
                 rtb_Sum1_a_idx_1) - 3.8789520269820388) * -2.0;

  /* DotProduct: '<S520>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S521>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S522>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S523>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S524>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S525>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S526>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;

  /* DotProduct: '<S527>/Dot Product' */
  y = 0.0;
  for (i = 0; i < 8; i++) {
    /* Sum: '<S508>/Sum1' incorporates:
     *  Constant: '<S508>/one'
     *  Constant: '<S508>/one1'
     *  Gain: '<S508>/Gain1'
     *  Math: '<S508>/Exp'
     *  Math: '<S508>/Reciprocal'
     *  Sum: '<S508>/Sum'
     *
     * About '<S508>/Exp':
     *  Operator: exp
     *
     * About '<S508>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_p_0 = 1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0;

    /* DotProduct: '<S520>/Dot Product' incorporates:
     *  Constant: '<S518>/IW{2,1}(1,:)''
     */
    tmp_0 += rtConstP.pooled54[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S521>/Dot Product' incorporates:
     *  Constant: '<S518>/IW{2,1}(2,:)''
     */
    tmp_1 += rtConstP.pooled55[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S522>/Dot Product' incorporates:
     *  Constant: '<S518>/IW{2,1}(3,:)''
     */
    tmp_2 += rtConstP.pooled56[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S523>/Dot Product' incorporates:
     *  Constant: '<S518>/IW{2,1}(4,:)''
     */
    tmp_3 += rtConstP.pooled57[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S524>/Dot Product' incorporates:
     *  Constant: '<S518>/IW{2,1}(5,:)''
     */
    tmp_4 += rtConstP.pooled58[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S525>/Dot Product' incorporates:
     *  Constant: '<S518>/IW{2,1}(6,:)''
     */
    rtb_DotProduct_i += rtConstP.pooled59[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S526>/Dot Product' incorporates:
     *  Constant: '<S518>/IW{2,1}(7,:)''
     */
    rtb_Sum1_a_idx_0 += rtConstP.pooled60[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S527>/Dot Product' incorporates:
     *  Constant: '<S518>/IW{2,1}(8,:)''
     */
    y += rtConstP.pooled61[i] * rtb_Sum1_p_0;
  }

  /* Sum: '<S501>/netsum' incorporates:
   *  Constant: '<S501>/b{2}'
   *  Gain: '<S519>/Gain'
   */
  rtDW.dv[0] = (tmp_0 + 1.6046475204164423) * -2.0;
  rtDW.dv[1] = (tmp_1 + 1.4606242966108713) * -2.0;
  rtDW.dv[2] = (tmp_2 + 0.58015229277762848) * -2.0;
  rtDW.dv[3] = (tmp_3 + 0.30730899286732177) * -2.0;
  rtDW.dv[4] = (tmp_4 - 0.22510633750660691) * -2.0;
  rtDW.dv[5] = (rtb_DotProduct_i - 0.90331239580027856) * -2.0;
  rtDW.dv[6] = (rtb_Sum1_a_idx_0 - 0.916851803006424) * -2.0;
  rtDW.dv[7] = (y - 1.3426230187338815) * -2.0;

  /* DotProduct: '<S531>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S532>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S533>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S534>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S535>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S536>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S537>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;

  /* DotProduct: '<S538>/Dot Product' */
  y = 0.0;
  for (i = 0; i < 8; i++) {
    /* Sum: '<S519>/Sum1' incorporates:
     *  Constant: '<S519>/one'
     *  Constant: '<S519>/one1'
     *  Gain: '<S519>/Gain1'
     *  Math: '<S519>/Exp'
     *  Math: '<S519>/Reciprocal'
     *  Sum: '<S519>/Sum'
     *
     * About '<S519>/Exp':
     *  Operator: exp
     *
     * About '<S519>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_p_0 = 1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0;

    /* DotProduct: '<S531>/Dot Product' incorporates:
     *  Constant: '<S529>/IW{3,2}(1,:)''
     */
    tmp_0 += rtConstP.pooled63[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S532>/Dot Product' incorporates:
     *  Constant: '<S529>/IW{3,2}(2,:)''
     */
    tmp_1 += rtConstP.pooled64[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S533>/Dot Product' incorporates:
     *  Constant: '<S529>/IW{3,2}(3,:)''
     */
    tmp_2 += rtConstP.pooled65[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S534>/Dot Product' incorporates:
     *  Constant: '<S529>/IW{3,2}(4,:)''
     */
    tmp_3 += rtConstP.pooled66[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S535>/Dot Product' incorporates:
     *  Constant: '<S529>/IW{3,2}(5,:)''
     */
    tmp_4 += rtConstP.pooled67[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S536>/Dot Product' incorporates:
     *  Constant: '<S529>/IW{3,2}(6,:)''
     */
    rtb_DotProduct_i += rtConstP.pooled68[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S537>/Dot Product' incorporates:
     *  Constant: '<S529>/IW{3,2}(7,:)''
     */
    rtb_Sum1_a_idx_0 += rtConstP.pooled69[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S538>/Dot Product' incorporates:
     *  Constant: '<S529>/IW{3,2}(8,:)''
     */
    y += rtConstP.pooled70[i] * rtb_Sum1_p_0;
  }

  /* Sum: '<S502>/netsum' incorporates:
   *  Constant: '<S502>/b{3}'
   *  Gain: '<S530>/Gain'
   */
  rtDW.dv[0] = (tmp_0 - 1.7933447210666922) * -2.0;
  rtDW.dv[1] = (tmp_1 - 1.4021822232818775) * -2.0;
  rtDW.dv[2] = (tmp_2 + 0.65489251567190065) * -2.0;
  rtDW.dv[3] = (tmp_3 + 0.33556321053808896) * -2.0;
  rtDW.dv[4] = (tmp_4 - 0.3596047527117644) * -2.0;
  rtDW.dv[5] = (rtb_DotProduct_i - 0.97282066824493807) * -2.0;
  rtDW.dv[6] = (rtb_Sum1_a_idx_0 + 1.1981280377566486) * -2.0;
  rtDW.dv[7] = (y - 1.6711500696070511) * -2.0;

  /* DotProduct: '<S542>/Dot Product' incorporates:
   *  Constant: '<S530>/one'
   *  Constant: '<S530>/one1'
   *  Constant: '<S540>/IW{4,3}(1,:)''
   *  Gain: '<S530>/Gain1'
   *  Math: '<S530>/Exp'
   *  Math: '<S530>/Reciprocal'
   *  Sum: '<S530>/Sum'
   *  Sum: '<S530>/Sum1'
   *
   * About '<S530>/Exp':
   *  Operator: exp
   *
   * About '<S530>/Reciprocal':
   *  Operator: reciprocal
   */
  tmp_0 = 0.0;
  for (i = 0; i < 8; i++) {
    tmp_0 += (1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0) * rtConstP.pooled72[i];
  }

  /* End of DotProduct: '<S542>/Dot Product' */

  /* MATLAB Function: '<S1>/MATLAB Function8' incorporates:
   *  Bias: '<S544>/Subtract min y'
   *  Gain: '<S544>/Divide by range y'
   *  Outport: '<Root>/now_shizhi_wanqu_jiaosudu_2'
   *  Sum: '<S503>/netsum'
   */
  MATLABFunction1(72.449016187362631 * ((tmp_0 - 0.72799606024092589) + 1.0),
                  &rtY.now_shizhi_wanqu_jiaosudu_2);

  /* Bias: '<S123>/Add min y' incorporates:
   *  Gain: '<S123>/range y // range x'
   *  Inport: '<Root>/now_M4_pos1'
   *  Inport: '<Root>/now_M4_spd'
   */
  rtb_Sum1_a_idx_0 = 0.011442940347073955 * rtU.now_M4_pos1 - 1.0;
  rtb_Sum1_a_idx_1 = 0.0088948200869909053 * rtU.now_M4_spd - 1.0;

  /* Sum: '<S80>/netsum' incorporates:
   *  Constant: '<S80>/b{1}'
   *  Constant: '<S87>/IW{1,1}(1,:)''
   *  Constant: '<S87>/IW{1,1}(2,:)''
   *  Constant: '<S87>/IW{1,1}(3,:)''
   *  Constant: '<S87>/IW{1,1}(4,:)''
   *  Constant: '<S87>/IW{1,1}(5,:)''
   *  Constant: '<S87>/IW{1,1}(6,:)''
   *  Constant: '<S87>/IW{1,1}(7,:)''
   *  Constant: '<S87>/IW{1,1}(8,:)''
   *  DotProduct: '<S89>/Dot Product'
   *  DotProduct: '<S90>/Dot Product'
   *  DotProduct: '<S91>/Dot Product'
   *  DotProduct: '<S92>/Dot Product'
   *  DotProduct: '<S93>/Dot Product'
   *  DotProduct: '<S94>/Dot Product'
   *  DotProduct: '<S95>/Dot Product'
   *  DotProduct: '<S96>/Dot Product'
   *  Gain: '<S88>/Gain'
   */
  rtDW.dv[0] = ((-1.8883213107036583 * rtb_Sum1_a_idx_0 + 2.7264405419151014 *
                 rtb_Sum1_a_idx_1) + 3.9372713672464874) * -2.0;
  rtDW.dv[1] = ((2.3434620813347893 * rtb_Sum1_a_idx_0 + -0.56861021889814367 *
                 rtb_Sum1_a_idx_1) - 2.2568037376489092) * -2.0;
  rtDW.dv[2] = ((-1.9225325006688625 * rtb_Sum1_a_idx_0 + 2.0591606516062329 *
                 rtb_Sum1_a_idx_1) + 1.6123622829928963) * -2.0;
  rtDW.dv[3] = ((1.9182925442906105 * rtb_Sum1_a_idx_0 + -1.0440186024819671 *
                 rtb_Sum1_a_idx_1) - 0.69544501103659129) * -2.0;
  rtDW.dv[4] = ((-0.79579906702584513 * rtb_Sum1_a_idx_0 + 1.2263520013200968 *
                 rtb_Sum1_a_idx_1) - 0.66057142133070812) * -2.0;
  rtDW.dv[5] = ((-1.9426523141840821 * rtb_Sum1_a_idx_0 + 1.6738162619001831 *
                 rtb_Sum1_a_idx_1) - 1.8347480668887592) * -2.0;
  rtDW.dv[6] = ((-2.6525139739129369 * rtb_Sum1_a_idx_0 + -0.081371677390578476 *
                 rtb_Sum1_a_idx_1) - 1.6683402663806088) * -2.0;
  rtDW.dv[7] = ((-3.3680624132745285 * rtb_Sum1_a_idx_0 + 0.75059375984035459 *
                 rtb_Sum1_a_idx_1) - 3.8789520269820388) * -2.0;

  /* DotProduct: '<S100>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S101>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S102>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S103>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S104>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S105>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S106>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;

  /* DotProduct: '<S107>/Dot Product' */
  y = 0.0;
  for (i = 0; i < 8; i++) {
    /* Sum: '<S88>/Sum1' incorporates:
     *  Constant: '<S88>/one'
     *  Constant: '<S88>/one1'
     *  Gain: '<S88>/Gain1'
     *  Math: '<S88>/Exp'
     *  Math: '<S88>/Reciprocal'
     *  Sum: '<S88>/Sum'
     *
     * About '<S88>/Exp':
     *  Operator: exp
     *
     * About '<S88>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_p_0 = 1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0;

    /* DotProduct: '<S100>/Dot Product' incorporates:
     *  Constant: '<S98>/IW{2,1}(1,:)''
     */
    tmp_0 += rtConstP.pooled54[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S101>/Dot Product' incorporates:
     *  Constant: '<S98>/IW{2,1}(2,:)''
     */
    tmp_1 += rtConstP.pooled55[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S102>/Dot Product' incorporates:
     *  Constant: '<S98>/IW{2,1}(3,:)''
     */
    tmp_2 += rtConstP.pooled56[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S103>/Dot Product' incorporates:
     *  Constant: '<S98>/IW{2,1}(4,:)''
     */
    tmp_3 += rtConstP.pooled57[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S104>/Dot Product' incorporates:
     *  Constant: '<S98>/IW{2,1}(5,:)''
     */
    tmp_4 += rtConstP.pooled58[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S105>/Dot Product' incorporates:
     *  Constant: '<S98>/IW{2,1}(6,:)''
     */
    rtb_DotProduct_i += rtConstP.pooled59[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S106>/Dot Product' incorporates:
     *  Constant: '<S98>/IW{2,1}(7,:)''
     */
    rtb_Sum1_a_idx_0 += rtConstP.pooled60[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S107>/Dot Product' incorporates:
     *  Constant: '<S98>/IW{2,1}(8,:)''
     */
    y += rtConstP.pooled61[i] * rtb_Sum1_p_0;
  }

  /* Sum: '<S81>/netsum' incorporates:
   *  Constant: '<S81>/b{2}'
   *  Gain: '<S99>/Gain'
   */
  rtDW.dv[0] = (tmp_0 + 1.6046475204164423) * -2.0;
  rtDW.dv[1] = (tmp_1 + 1.4606242966108713) * -2.0;
  rtDW.dv[2] = (tmp_2 + 0.58015229277762848) * -2.0;
  rtDW.dv[3] = (tmp_3 + 0.30730899286732177) * -2.0;
  rtDW.dv[4] = (tmp_4 - 0.22510633750660691) * -2.0;
  rtDW.dv[5] = (rtb_DotProduct_i - 0.90331239580027856) * -2.0;
  rtDW.dv[6] = (rtb_Sum1_a_idx_0 - 0.916851803006424) * -2.0;
  rtDW.dv[7] = (y - 1.3426230187338815) * -2.0;

  /* DotProduct: '<S111>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S112>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S113>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S114>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S115>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S116>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S117>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;

  /* DotProduct: '<S118>/Dot Product' */
  y = 0.0;
  for (i = 0; i < 8; i++) {
    /* Sum: '<S99>/Sum1' incorporates:
     *  Constant: '<S99>/one'
     *  Constant: '<S99>/one1'
     *  Gain: '<S99>/Gain1'
     *  Math: '<S99>/Exp'
     *  Math: '<S99>/Reciprocal'
     *  Sum: '<S99>/Sum'
     *
     * About '<S99>/Exp':
     *  Operator: exp
     *
     * About '<S99>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_p_0 = 1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0;

    /* DotProduct: '<S111>/Dot Product' incorporates:
     *  Constant: '<S109>/IW{3,2}(1,:)''
     */
    tmp_0 += rtConstP.pooled63[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S112>/Dot Product' incorporates:
     *  Constant: '<S109>/IW{3,2}(2,:)''
     */
    tmp_1 += rtConstP.pooled64[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S113>/Dot Product' incorporates:
     *  Constant: '<S109>/IW{3,2}(3,:)''
     */
    tmp_2 += rtConstP.pooled65[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S114>/Dot Product' incorporates:
     *  Constant: '<S109>/IW{3,2}(4,:)''
     */
    tmp_3 += rtConstP.pooled66[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S115>/Dot Product' incorporates:
     *  Constant: '<S109>/IW{3,2}(5,:)''
     */
    tmp_4 += rtConstP.pooled67[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S116>/Dot Product' incorporates:
     *  Constant: '<S109>/IW{3,2}(6,:)''
     */
    rtb_DotProduct_i += rtConstP.pooled68[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S117>/Dot Product' incorporates:
     *  Constant: '<S109>/IW{3,2}(7,:)''
     */
    rtb_Sum1_a_idx_0 += rtConstP.pooled69[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S118>/Dot Product' incorporates:
     *  Constant: '<S109>/IW{3,2}(8,:)''
     */
    y += rtConstP.pooled70[i] * rtb_Sum1_p_0;
  }

  /* Sum: '<S82>/netsum' incorporates:
   *  Constant: '<S82>/b{3}'
   *  Gain: '<S110>/Gain'
   */
  rtDW.dv[0] = (tmp_0 - 1.7933447210666922) * -2.0;
  rtDW.dv[1] = (tmp_1 - 1.4021822232818775) * -2.0;
  rtDW.dv[2] = (tmp_2 + 0.65489251567190065) * -2.0;
  rtDW.dv[3] = (tmp_3 + 0.33556321053808896) * -2.0;
  rtDW.dv[4] = (tmp_4 - 0.3596047527117644) * -2.0;
  rtDW.dv[5] = (rtb_DotProduct_i - 0.97282066824493807) * -2.0;
  rtDW.dv[6] = (rtb_Sum1_a_idx_0 + 1.1981280377566486) * -2.0;
  rtDW.dv[7] = (y - 1.6711500696070511) * -2.0;

  /* DotProduct: '<S122>/Dot Product' incorporates:
   *  Constant: '<S110>/one'
   *  Constant: '<S110>/one1'
   *  Constant: '<S120>/IW{4,3}(1,:)''
   *  Gain: '<S110>/Gain1'
   *  Math: '<S110>/Exp'
   *  Math: '<S110>/Reciprocal'
   *  Sum: '<S110>/Sum'
   *  Sum: '<S110>/Sum1'
   *
   * About '<S110>/Exp':
   *  Operator: exp
   *
   * About '<S110>/Reciprocal':
   *  Operator: reciprocal
   */
  tmp_0 = 0.0;
  for (i = 0; i < 8; i++) {
    tmp_0 += (1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0) * rtConstP.pooled72[i];
  }

  /* End of DotProduct: '<S122>/Dot Product' */

  /* MATLAB Function: '<S1>/MATLAB Function9' incorporates:
   *  Bias: '<S124>/Subtract min y'
   *  Gain: '<S124>/Divide by range y'
   *  Outport: '<Root>/now_zhongzhi_wanqu_jiaosudu_2'
   *  Sum: '<S83>/netsum'
   */
  MATLABFunction1(72.449016187362631 * ((tmp_0 - 0.72799606024092589) + 1.0),
                  &rtY.now_zhongzhi_wanqu_jiaosudu_2);

  /* Bias: '<S456>/Add min y' incorporates:
   *  Gain: '<S456>/range y // range x'
   *  Inport: '<Root>/now_M5_pos1'
   *  Inport: '<Root>/now_M5_spd'
   */
  rtb_Sum1_a_idx_0 = 0.011442940347073955 * rtU.now_M5_pos1 - 1.0;
  rtb_Sum1_a_idx_1 = 0.0088948200869909053 * rtU.now_M5_spd - 1.0;

  /* Sum: '<S413>/netsum' incorporates:
   *  Constant: '<S413>/b{1}'
   *  Constant: '<S420>/IW{1,1}(1,:)''
   *  Constant: '<S420>/IW{1,1}(2,:)''
   *  Constant: '<S420>/IW{1,1}(3,:)''
   *  Constant: '<S420>/IW{1,1}(4,:)''
   *  Constant: '<S420>/IW{1,1}(5,:)''
   *  Constant: '<S420>/IW{1,1}(6,:)''
   *  Constant: '<S420>/IW{1,1}(7,:)''
   *  Constant: '<S420>/IW{1,1}(8,:)''
   *  DotProduct: '<S422>/Dot Product'
   *  DotProduct: '<S423>/Dot Product'
   *  DotProduct: '<S424>/Dot Product'
   *  DotProduct: '<S425>/Dot Product'
   *  DotProduct: '<S426>/Dot Product'
   *  DotProduct: '<S427>/Dot Product'
   *  DotProduct: '<S428>/Dot Product'
   *  DotProduct: '<S429>/Dot Product'
   *  Gain: '<S421>/Gain'
   */
  rtDW.dv[0] = ((-1.8883213107036583 * rtb_Sum1_a_idx_0 + 2.7264405419151014 *
                 rtb_Sum1_a_idx_1) + 3.9372713672464874) * -2.0;
  rtDW.dv[1] = ((2.3434620813347893 * rtb_Sum1_a_idx_0 + -0.56861021889814367 *
                 rtb_Sum1_a_idx_1) - 2.2568037376489092) * -2.0;
  rtDW.dv[2] = ((-1.9225325006688625 * rtb_Sum1_a_idx_0 + 2.0591606516062329 *
                 rtb_Sum1_a_idx_1) + 1.6123622829928963) * -2.0;
  rtDW.dv[3] = ((1.9182925442906105 * rtb_Sum1_a_idx_0 + -1.0440186024819671 *
                 rtb_Sum1_a_idx_1) - 0.69544501103659129) * -2.0;
  rtDW.dv[4] = ((-0.79579906702584513 * rtb_Sum1_a_idx_0 + 1.2263520013200968 *
                 rtb_Sum1_a_idx_1) - 0.66057142133070812) * -2.0;
  rtDW.dv[5] = ((-1.9426523141840821 * rtb_Sum1_a_idx_0 + 1.6738162619001831 *
                 rtb_Sum1_a_idx_1) - 1.8347480668887592) * -2.0;
  rtDW.dv[6] = ((-2.6525139739129369 * rtb_Sum1_a_idx_0 + -0.081371677390578476 *
                 rtb_Sum1_a_idx_1) - 1.6683402663806088) * -2.0;
  rtDW.dv[7] = ((-3.3680624132745285 * rtb_Sum1_a_idx_0 + 0.75059375984035459 *
                 rtb_Sum1_a_idx_1) - 3.8789520269820388) * -2.0;

  /* DotProduct: '<S433>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S434>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S435>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S436>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S437>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S438>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S439>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;

  /* DotProduct: '<S440>/Dot Product' */
  y = 0.0;
  for (i = 0; i < 8; i++) {
    /* Sum: '<S421>/Sum1' incorporates:
     *  Constant: '<S421>/one'
     *  Constant: '<S421>/one1'
     *  Gain: '<S421>/Gain1'
     *  Math: '<S421>/Exp'
     *  Math: '<S421>/Reciprocal'
     *  Sum: '<S421>/Sum'
     *
     * About '<S421>/Exp':
     *  Operator: exp
     *
     * About '<S421>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_p_0 = 1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0;

    /* DotProduct: '<S433>/Dot Product' incorporates:
     *  Constant: '<S431>/IW{2,1}(1,:)''
     */
    tmp_0 += rtConstP.pooled54[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S434>/Dot Product' incorporates:
     *  Constant: '<S431>/IW{2,1}(2,:)''
     */
    tmp_1 += rtConstP.pooled55[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S435>/Dot Product' incorporates:
     *  Constant: '<S431>/IW{2,1}(3,:)''
     */
    tmp_2 += rtConstP.pooled56[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S436>/Dot Product' incorporates:
     *  Constant: '<S431>/IW{2,1}(4,:)''
     */
    tmp_3 += rtConstP.pooled57[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S437>/Dot Product' incorporates:
     *  Constant: '<S431>/IW{2,1}(5,:)''
     */
    tmp_4 += rtConstP.pooled58[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S438>/Dot Product' incorporates:
     *  Constant: '<S431>/IW{2,1}(6,:)''
     */
    rtb_DotProduct_i += rtConstP.pooled59[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S439>/Dot Product' incorporates:
     *  Constant: '<S431>/IW{2,1}(7,:)''
     */
    rtb_Sum1_a_idx_0 += rtConstP.pooled60[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S440>/Dot Product' incorporates:
     *  Constant: '<S431>/IW{2,1}(8,:)''
     */
    y += rtConstP.pooled61[i] * rtb_Sum1_p_0;
  }

  /* Sum: '<S414>/netsum' incorporates:
   *  Constant: '<S414>/b{2}'
   *  Gain: '<S432>/Gain'
   */
  rtDW.dv[0] = (tmp_0 + 1.6046475204164423) * -2.0;
  rtDW.dv[1] = (tmp_1 + 1.4606242966108713) * -2.0;
  rtDW.dv[2] = (tmp_2 + 0.58015229277762848) * -2.0;
  rtDW.dv[3] = (tmp_3 + 0.30730899286732177) * -2.0;
  rtDW.dv[4] = (tmp_4 - 0.22510633750660691) * -2.0;
  rtDW.dv[5] = (rtb_DotProduct_i - 0.90331239580027856) * -2.0;
  rtDW.dv[6] = (rtb_Sum1_a_idx_0 - 0.916851803006424) * -2.0;
  rtDW.dv[7] = (y - 1.3426230187338815) * -2.0;

  /* DotProduct: '<S444>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S445>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S446>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S447>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S448>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S449>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S450>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;

  /* DotProduct: '<S451>/Dot Product' */
  y = 0.0;
  for (i = 0; i < 8; i++) {
    /* Sum: '<S432>/Sum1' incorporates:
     *  Constant: '<S432>/one'
     *  Constant: '<S432>/one1'
     *  Gain: '<S432>/Gain1'
     *  Math: '<S432>/Exp'
     *  Math: '<S432>/Reciprocal'
     *  Sum: '<S432>/Sum'
     *
     * About '<S432>/Exp':
     *  Operator: exp
     *
     * About '<S432>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_p_0 = 1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0;

    /* DotProduct: '<S444>/Dot Product' incorporates:
     *  Constant: '<S442>/IW{3,2}(1,:)''
     */
    tmp_0 += rtConstP.pooled63[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S445>/Dot Product' incorporates:
     *  Constant: '<S442>/IW{3,2}(2,:)''
     */
    tmp_1 += rtConstP.pooled64[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S446>/Dot Product' incorporates:
     *  Constant: '<S442>/IW{3,2}(3,:)''
     */
    tmp_2 += rtConstP.pooled65[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S447>/Dot Product' incorporates:
     *  Constant: '<S442>/IW{3,2}(4,:)''
     */
    tmp_3 += rtConstP.pooled66[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S448>/Dot Product' incorporates:
     *  Constant: '<S442>/IW{3,2}(5,:)''
     */
    tmp_4 += rtConstP.pooled67[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S449>/Dot Product' incorporates:
     *  Constant: '<S442>/IW{3,2}(6,:)''
     */
    rtb_DotProduct_i += rtConstP.pooled68[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S450>/Dot Product' incorporates:
     *  Constant: '<S442>/IW{3,2}(7,:)''
     */
    rtb_Sum1_a_idx_0 += rtConstP.pooled69[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S451>/Dot Product' incorporates:
     *  Constant: '<S442>/IW{3,2}(8,:)''
     */
    y += rtConstP.pooled70[i] * rtb_Sum1_p_0;
  }

  /* Sum: '<S415>/netsum' incorporates:
   *  Constant: '<S415>/b{3}'
   *  Gain: '<S443>/Gain'
   */
  rtDW.dv[0] = (tmp_0 - 1.7933447210666922) * -2.0;
  rtDW.dv[1] = (tmp_1 - 1.4021822232818775) * -2.0;
  rtDW.dv[2] = (tmp_2 + 0.65489251567190065) * -2.0;
  rtDW.dv[3] = (tmp_3 + 0.33556321053808896) * -2.0;
  rtDW.dv[4] = (tmp_4 - 0.3596047527117644) * -2.0;
  rtDW.dv[5] = (rtb_DotProduct_i - 0.97282066824493807) * -2.0;
  rtDW.dv[6] = (rtb_Sum1_a_idx_0 + 1.1981280377566486) * -2.0;
  rtDW.dv[7] = (y - 1.6711500696070511) * -2.0;

  /* DotProduct: '<S455>/Dot Product' incorporates:
   *  Constant: '<S443>/one'
   *  Constant: '<S443>/one1'
   *  Constant: '<S453>/IW{4,3}(1,:)''
   *  Gain: '<S443>/Gain1'
   *  Math: '<S443>/Exp'
   *  Math: '<S443>/Reciprocal'
   *  Sum: '<S443>/Sum'
   *  Sum: '<S443>/Sum1'
   *
   * About '<S443>/Exp':
   *  Operator: exp
   *
   * About '<S443>/Reciprocal':
   *  Operator: reciprocal
   */
  tmp_0 = 0.0;
  for (i = 0; i < 8; i++) {
    tmp_0 += (1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0) * rtConstP.pooled72[i];
  }

  /* End of DotProduct: '<S455>/Dot Product' */

  /* MATLAB Function: '<S1>/MATLAB Function10' incorporates:
   *  Bias: '<S457>/Subtract min y'
   *  Gain: '<S457>/Divide by range y'
   *  Outport: '<Root>/now_wumingzhi_wanqu_jiaosudu_2'
   *  Sum: '<S416>/netsum'
   */
  MATLABFunction1(72.449016187362631 * ((tmp_0 - 0.72799606024092589) + 1.0),
                  &rtY.now_wumingzhi_wanqu_jiaosudu_2);

  /* Bias: '<S210>/Add min y' incorporates:
   *  Gain: '<S210>/range y // range x'
   *  Inport: '<Root>/now_M6_pos1'
   *  Inport: '<Root>/now_M6_spd'
   */
  rtb_Sum1_a_idx_0 = 0.011442940347073955 * rtU.now_M6_pos1 - 1.0;
  rtb_Sum1_a_idx_1 = 0.0088948200869909053 * rtU.now_M6_spd - 1.0;

  /* Sum: '<S167>/netsum' incorporates:
   *  Constant: '<S167>/b{1}'
   *  Constant: '<S174>/IW{1,1}(1,:)''
   *  Constant: '<S174>/IW{1,1}(2,:)''
   *  Constant: '<S174>/IW{1,1}(3,:)''
   *  Constant: '<S174>/IW{1,1}(4,:)''
   *  Constant: '<S174>/IW{1,1}(5,:)''
   *  Constant: '<S174>/IW{1,1}(6,:)''
   *  Constant: '<S174>/IW{1,1}(7,:)''
   *  Constant: '<S174>/IW{1,1}(8,:)''
   *  DotProduct: '<S176>/Dot Product'
   *  DotProduct: '<S177>/Dot Product'
   *  DotProduct: '<S178>/Dot Product'
   *  DotProduct: '<S179>/Dot Product'
   *  DotProduct: '<S180>/Dot Product'
   *  DotProduct: '<S181>/Dot Product'
   *  DotProduct: '<S182>/Dot Product'
   *  DotProduct: '<S183>/Dot Product'
   *  Gain: '<S175>/Gain'
   */
  rtDW.dv[0] = ((-1.8883213107036583 * rtb_Sum1_a_idx_0 + 2.7264405419151014 *
                 rtb_Sum1_a_idx_1) + 3.9372713672464874) * -2.0;
  rtDW.dv[1] = ((2.3434620813347893 * rtb_Sum1_a_idx_0 + -0.56861021889814367 *
                 rtb_Sum1_a_idx_1) - 2.2568037376489092) * -2.0;
  rtDW.dv[2] = ((-1.9225325006688625 * rtb_Sum1_a_idx_0 + 2.0591606516062329 *
                 rtb_Sum1_a_idx_1) + 1.6123622829928963) * -2.0;
  rtDW.dv[3] = ((1.9182925442906105 * rtb_Sum1_a_idx_0 + -1.0440186024819671 *
                 rtb_Sum1_a_idx_1) - 0.69544501103659129) * -2.0;
  rtDW.dv[4] = ((-0.79579906702584513 * rtb_Sum1_a_idx_0 + 1.2263520013200968 *
                 rtb_Sum1_a_idx_1) - 0.66057142133070812) * -2.0;
  rtDW.dv[5] = ((-1.9426523141840821 * rtb_Sum1_a_idx_0 + 1.6738162619001831 *
                 rtb_Sum1_a_idx_1) - 1.8347480668887592) * -2.0;
  rtDW.dv[6] = ((-2.6525139739129369 * rtb_Sum1_a_idx_0 + -0.081371677390578476 *
                 rtb_Sum1_a_idx_1) - 1.6683402663806088) * -2.0;
  rtDW.dv[7] = ((-3.3680624132745285 * rtb_Sum1_a_idx_0 + 0.75059375984035459 *
                 rtb_Sum1_a_idx_1) - 3.8789520269820388) * -2.0;

  /* DotProduct: '<S187>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S188>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S189>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S190>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S191>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S192>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S193>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;

  /* DotProduct: '<S194>/Dot Product' */
  y = 0.0;
  for (i = 0; i < 8; i++) {
    /* Sum: '<S175>/Sum1' incorporates:
     *  Constant: '<S175>/one'
     *  Constant: '<S175>/one1'
     *  Gain: '<S175>/Gain1'
     *  Math: '<S175>/Exp'
     *  Math: '<S175>/Reciprocal'
     *  Sum: '<S175>/Sum'
     *
     * About '<S175>/Exp':
     *  Operator: exp
     *
     * About '<S175>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_p_0 = 1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0;

    /* DotProduct: '<S187>/Dot Product' incorporates:
     *  Constant: '<S185>/IW{2,1}(1,:)''
     */
    tmp_0 += rtConstP.pooled54[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S188>/Dot Product' incorporates:
     *  Constant: '<S185>/IW{2,1}(2,:)''
     */
    tmp_1 += rtConstP.pooled55[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S189>/Dot Product' incorporates:
     *  Constant: '<S185>/IW{2,1}(3,:)''
     */
    tmp_2 += rtConstP.pooled56[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S190>/Dot Product' incorporates:
     *  Constant: '<S185>/IW{2,1}(4,:)''
     */
    tmp_3 += rtConstP.pooled57[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S191>/Dot Product' incorporates:
     *  Constant: '<S185>/IW{2,1}(5,:)''
     */
    tmp_4 += rtConstP.pooled58[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S192>/Dot Product' incorporates:
     *  Constant: '<S185>/IW{2,1}(6,:)''
     */
    rtb_DotProduct_i += rtConstP.pooled59[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S193>/Dot Product' incorporates:
     *  Constant: '<S185>/IW{2,1}(7,:)''
     */
    rtb_Sum1_a_idx_0 += rtConstP.pooled60[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S194>/Dot Product' incorporates:
     *  Constant: '<S185>/IW{2,1}(8,:)''
     */
    y += rtConstP.pooled61[i] * rtb_Sum1_p_0;
  }

  /* Sum: '<S168>/netsum' incorporates:
   *  Constant: '<S168>/b{2}'
   *  Gain: '<S186>/Gain'
   */
  rtDW.dv[0] = (tmp_0 + 1.6046475204164423) * -2.0;
  rtDW.dv[1] = (tmp_1 + 1.4606242966108713) * -2.0;
  rtDW.dv[2] = (tmp_2 + 0.58015229277762848) * -2.0;
  rtDW.dv[3] = (tmp_3 + 0.30730899286732177) * -2.0;
  rtDW.dv[4] = (tmp_4 - 0.22510633750660691) * -2.0;
  rtDW.dv[5] = (rtb_DotProduct_i - 0.90331239580027856) * -2.0;
  rtDW.dv[6] = (rtb_Sum1_a_idx_0 - 0.916851803006424) * -2.0;
  rtDW.dv[7] = (y - 1.3426230187338815) * -2.0;

  /* DotProduct: '<S198>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S199>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S200>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S201>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S202>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S203>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S204>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;

  /* DotProduct: '<S205>/Dot Product' */
  y = 0.0;
  for (i = 0; i < 8; i++) {
    /* Sum: '<S186>/Sum1' incorporates:
     *  Constant: '<S186>/one'
     *  Constant: '<S186>/one1'
     *  Gain: '<S186>/Gain1'
     *  Math: '<S186>/Exp'
     *  Math: '<S186>/Reciprocal'
     *  Sum: '<S186>/Sum'
     *
     * About '<S186>/Exp':
     *  Operator: exp
     *
     * About '<S186>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_p_0 = 1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0;

    /* DotProduct: '<S198>/Dot Product' incorporates:
     *  Constant: '<S196>/IW{3,2}(1,:)''
     */
    tmp_0 += rtConstP.pooled63[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S199>/Dot Product' incorporates:
     *  Constant: '<S196>/IW{3,2}(2,:)''
     */
    tmp_1 += rtConstP.pooled64[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S200>/Dot Product' incorporates:
     *  Constant: '<S196>/IW{3,2}(3,:)''
     */
    tmp_2 += rtConstP.pooled65[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S201>/Dot Product' incorporates:
     *  Constant: '<S196>/IW{3,2}(4,:)''
     */
    tmp_3 += rtConstP.pooled66[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S202>/Dot Product' incorporates:
     *  Constant: '<S196>/IW{3,2}(5,:)''
     */
    tmp_4 += rtConstP.pooled67[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S203>/Dot Product' incorporates:
     *  Constant: '<S196>/IW{3,2}(6,:)''
     */
    rtb_DotProduct_i += rtConstP.pooled68[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S204>/Dot Product' incorporates:
     *  Constant: '<S196>/IW{3,2}(7,:)''
     */
    rtb_Sum1_a_idx_0 += rtConstP.pooled69[i] * rtb_Sum1_p_0;

    /* DotProduct: '<S205>/Dot Product' incorporates:
     *  Constant: '<S196>/IW{3,2}(8,:)''
     */
    y += rtConstP.pooled70[i] * rtb_Sum1_p_0;
  }

  /* Sum: '<S169>/netsum' incorporates:
   *  Constant: '<S169>/b{3}'
   *  Gain: '<S197>/Gain'
   */
  rtDW.dv[0] = (tmp_0 - 1.7933447210666922) * -2.0;
  rtDW.dv[1] = (tmp_1 - 1.4021822232818775) * -2.0;
  rtDW.dv[2] = (tmp_2 + 0.65489251567190065) * -2.0;
  rtDW.dv[3] = (tmp_3 + 0.33556321053808896) * -2.0;
  rtDW.dv[4] = (tmp_4 - 0.3596047527117644) * -2.0;
  rtDW.dv[5] = (rtb_DotProduct_i - 0.97282066824493807) * -2.0;
  rtDW.dv[6] = (rtb_Sum1_a_idx_0 + 1.1981280377566486) * -2.0;
  rtDW.dv[7] = (y - 1.6711500696070511) * -2.0;

  /* DotProduct: '<S209>/Dot Product' incorporates:
   *  Constant: '<S197>/one'
   *  Constant: '<S197>/one1'
   *  Constant: '<S207>/IW{4,3}(1,:)''
   *  Gain: '<S197>/Gain1'
   *  Math: '<S197>/Exp'
   *  Math: '<S197>/Reciprocal'
   *  Sum: '<S197>/Sum'
   *  Sum: '<S197>/Sum1'
   *
   * About '<S197>/Exp':
   *  Operator: exp
   *
   * About '<S197>/Reciprocal':
   *  Operator: reciprocal
   */
  tmp_0 = 0.0;
  for (i = 0; i < 8; i++) {
    tmp_0 += (1.0 / (exp(rtDW.dv[i]) + 1.0) * 2.0 - 1.0) * rtConstP.pooled72[i];
  }

  /* End of DotProduct: '<S209>/Dot Product' */

  /* MATLAB Function: '<S1>/MATLAB Function11' incorporates:
   *  Bias: '<S211>/Subtract min y'
   *  Gain: '<S211>/Divide by range y'
   *  Outport: '<Root>/now_xiaomuzhi_wanqu_jiaosudu_2'
   *  Sum: '<S170>/netsum'
   */
  MATLABFunction1(72.449016187362631 * ((tmp_0 - 0.72799606024092589) + 1.0),
                  &rtY.now_xiaomuzhi_wanqu_jiaosudu_2);

  /* Bias: '<S252>/Add min y' incorporates:
   *  Gain: '<S252>/range y // range x'
   *  Inport: '<Root>/Tar_muzhi_wanqu_jiaosudu_2'
   *  Inport: '<Root>/now_M1_pos'
   */
  rtb_Sum1_a_idx_0 = 0.016309484722941272 * rtU.Tar_muzhi_wanqu_jiaosudu_2 - 1.0;
  rtb_Sum1_a_idx_1 = 0.012223966159175996 * rtU.now_M1_pos - 1.0;

  /* Sum: '<S212>/netsum' incorporates:
   *  Constant: '<S212>/b{1}'
   *  Constant: '<S219>/IW{1,1}(1,:)''
   *  Constant: '<S219>/IW{1,1}(2,:)''
   *  Constant: '<S219>/IW{1,1}(3,:)''
   *  Constant: '<S219>/IW{1,1}(4,:)''
   *  Constant: '<S219>/IW{1,1}(5,:)''
   *  Constant: '<S219>/IW{1,1}(6,:)''
   *  Constant: '<S219>/IW{1,1}(7,:)''
   *  DotProduct: '<S221>/Dot Product'
   *  DotProduct: '<S222>/Dot Product'
   *  DotProduct: '<S223>/Dot Product'
   *  DotProduct: '<S224>/Dot Product'
   *  DotProduct: '<S225>/Dot Product'
   *  DotProduct: '<S226>/Dot Product'
   *  DotProduct: '<S227>/Dot Product'
   *  Gain: '<S220>/Gain'
   */
  rtDW.dv1[0] = ((-3.2136732212618191 * rtb_Sum1_a_idx_0 + 0.731468293515356 *
                  rtb_Sum1_a_idx_1) + 4.1290563441764467) * -2.0;
  rtDW.dv1[1] = ((-1.5163055257061189 * rtb_Sum1_a_idx_0 + -2.2025842184908533 *
                  rtb_Sum1_a_idx_1) + 2.6398137873152034) * -2.0;
  rtDW.dv1[2] = ((-2.4291057870722637 * rtb_Sum1_a_idx_0 + 1.1858490854011612 *
                  rtb_Sum1_a_idx_1) + 1.3501742790860161) * -2.0;
  rtDW.dv1[3] = ((1.616222445802199 * rtb_Sum1_a_idx_0 + 0.45587460948195607 *
                  rtb_Sum1_a_idx_1) + 0.42044414506085181) * -2.0;
  rtDW.dv1[4] = ((-1.5015693009690569 * rtb_Sum1_a_idx_0 + 0.9664248128148113 *
                  rtb_Sum1_a_idx_1) - 0.2346160675763945) * -2.0;
  rtDW.dv1[5] = ((1.9365689806167377 * rtb_Sum1_a_idx_0 + -1.5034374919610403 *
                  rtb_Sum1_a_idx_1) + 2.1974397093968157) * -2.0;
  rtDW.dv1[6] = ((-2.9691267974906115 * rtb_Sum1_a_idx_0 + -1.068123715247739 *
                  rtb_Sum1_a_idx_1) - 3.2676826925480951) * -2.0;

  /* DotProduct: '<S231>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S232>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S233>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S234>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S235>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S236>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S237>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S231>/Dot Product' incorporates:
     *  Constant: '<S220>/one'
     *  Constant: '<S220>/one1'
     *  Constant: '<S229>/IW{2,1}(1,:)''
     *  DotProduct: '<S232>/Dot Product'
     *  DotProduct: '<S233>/Dot Product'
     *  DotProduct: '<S234>/Dot Product'
     *  DotProduct: '<S235>/Dot Product'
     *  DotProduct: '<S236>/Dot Product'
     *  DotProduct: '<S237>/Dot Product'
     *  Gain: '<S220>/Gain1'
     *  Math: '<S220>/Exp'
     *  Math: '<S220>/Reciprocal'
     *  Sum: '<S220>/Sum'
     *  Sum: '<S220>/Sum1'
     *
     * About '<S220>/Exp':
     *  Operator: exp
     *
     * About '<S220>/Reciprocal':
     *  Operator: reciprocal
     */
    y = 1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += y * rtConstP.IW211_Value_l[i];

    /* DotProduct: '<S232>/Dot Product' incorporates:
     *  Constant: '<S229>/IW{2,1}(2,:)''
     */
    tmp_1 += y * rtConstP.IW212_Value_d[i];

    /* DotProduct: '<S233>/Dot Product' incorporates:
     *  Constant: '<S229>/IW{2,1}(3,:)''
     */
    tmp_2 += y * rtConstP.IW213_Value_m[i];

    /* DotProduct: '<S234>/Dot Product' incorporates:
     *  Constant: '<S229>/IW{2,1}(4,:)''
     */
    tmp_3 += y * rtConstP.IW214_Value[i];

    /* DotProduct: '<S235>/Dot Product' incorporates:
     *  Constant: '<S229>/IW{2,1}(5,:)''
     */
    tmp_4 += y * rtConstP.IW215_Value[i];

    /* DotProduct: '<S236>/Dot Product' incorporates:
     *  Constant: '<S229>/IW{2,1}(6,:)''
     */
    rtb_DotProduct_i += y * rtConstP.IW216_Value[i];

    /* DotProduct: '<S237>/Dot Product' incorporates:
     *  Constant: '<S229>/IW{2,1}(7,:)''
     */
    rtb_Sum1_a_idx_0 += y * rtConstP.IW217_Value[i];
  }

  /* Sum: '<S213>/netsum' incorporates:
   *  Constant: '<S213>/b{2}'
   *  Gain: '<S230>/Gain'
   */
  rtDW.dv1[0] = (tmp_0 + 1.4198519675205874) * -2.0;
  rtDW.dv1[1] = (tmp_1 - 1.0863544001636412) * -2.0;
  rtDW.dv1[2] = (tmp_2 + 0.90035116465631382) * -2.0;
  rtDW.dv1[3] = (tmp_3 - 0.035681276649163284) * -2.0;
  rtDW.dv1[4] = (tmp_4 + 0.22352532679977666) * -2.0;
  rtDW.dv1[5] = (rtb_DotProduct_i + 1.24146051265779) * -2.0;
  rtDW.dv1[6] = (rtb_Sum1_a_idx_0 - 1.8810188167600921) * -2.0;

  /* DotProduct: '<S241>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S242>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S243>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S244>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S245>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S246>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S247>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S241>/Dot Product' incorporates:
     *  Constant: '<S230>/one'
     *  Constant: '<S230>/one1'
     *  Constant: '<S239>/IW{3,2}(1,:)''
     *  DotProduct: '<S242>/Dot Product'
     *  DotProduct: '<S243>/Dot Product'
     *  DotProduct: '<S244>/Dot Product'
     *  DotProduct: '<S245>/Dot Product'
     *  DotProduct: '<S246>/Dot Product'
     *  DotProduct: '<S247>/Dot Product'
     *  Gain: '<S230>/Gain1'
     *  Math: '<S230>/Exp'
     *  Math: '<S230>/Reciprocal'
     *  Sum: '<S230>/Sum'
     *  Sum: '<S230>/Sum1'
     *
     * About '<S230>/Exp':
     *  Operator: exp
     *
     * About '<S230>/Reciprocal':
     *  Operator: reciprocal
     */
    y = 1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += y * rtConstP.IW321_Value_p[i];

    /* DotProduct: '<S242>/Dot Product' incorporates:
     *  Constant: '<S239>/IW{3,2}(2,:)''
     */
    tmp_1 += y * rtConstP.IW322_Value_k[i];

    /* DotProduct: '<S243>/Dot Product' incorporates:
     *  Constant: '<S239>/IW{3,2}(3,:)''
     */
    tmp_2 += y * rtConstP.IW323_Value[i];

    /* DotProduct: '<S244>/Dot Product' incorporates:
     *  Constant: '<S239>/IW{3,2}(4,:)''
     */
    tmp_3 += y * rtConstP.IW324_Value[i];

    /* DotProduct: '<S245>/Dot Product' incorporates:
     *  Constant: '<S239>/IW{3,2}(5,:)''
     */
    tmp_4 += y * rtConstP.IW325_Value[i];

    /* DotProduct: '<S246>/Dot Product' incorporates:
     *  Constant: '<S239>/IW{3,2}(6,:)''
     */
    rtb_DotProduct_i += y * rtConstP.IW326_Value[i];

    /* DotProduct: '<S247>/Dot Product' incorporates:
     *  Constant: '<S239>/IW{3,2}(7,:)''
     */
    rtb_Sum1_a_idx_0 += y * rtConstP.IW327_Value[i];
  }

  /* Sum: '<S214>/netsum' incorporates:
   *  Constant: '<S214>/b{3}'
   *  Gain: '<S240>/Gain'
   */
  rtDW.dv1[0] = (tmp_0 + 1.6968734376265606) * -2.0;
  rtDW.dv1[1] = (tmp_1 + 1.0484580087859259) * -2.0;
  rtDW.dv1[2] = (tmp_2 + 0.44568785796289123) * -2.0;
  rtDW.dv1[3] = (tmp_3 + 0.14799065102119538) * -2.0;
  rtDW.dv1[4] = (tmp_4 - 0.29832514503545216) * -2.0;
  rtDW.dv1[5] = (rtb_DotProduct_i + 1.3168365973290797) * -2.0;
  rtDW.dv1[6] = (rtb_Sum1_a_idx_0 + 1.8504953559671447) * -2.0;

  /* DotProduct: '<S251>/Dot Product' */
  tmp_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S251>/Dot Product' incorporates:
     *  Constant: '<S240>/one'
     *  Constant: '<S240>/one1'
     *  Constant: '<S249>/IW{4,3}(1,:)''
     *  Gain: '<S240>/Gain1'
     *  Math: '<S240>/Exp'
     *  Math: '<S240>/Reciprocal'
     *  Sum: '<S240>/Sum'
     *  Sum: '<S240>/Sum1'
     *
     * About '<S240>/Exp':
     *  Operator: exp
     *
     * About '<S240>/Reciprocal':
     *  Operator: reciprocal
     */
    tmp_0 += (1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0) *
      rtConstP.IW431_Value_b[i];
  }

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  Bias: '<S253>/Subtract min y'
   *  Gain: '<S253>/Divide by range y'
   *  Outport: '<Root>/Tar_M1_spd'
   *  Sum: '<S215>/netsum'
   */
  MATLABFunction(124.9249849969994 * ((tmp_0 - 0.21300100418446177) + 1.0),
                 &rtY.Tar_M1_spd);

  /* Bias: '<S498>/Add min y' incorporates:
   *  Gain: '<S498>/range y // range x'
   *  Inport: '<Root>/Tar_shizhi_wanqu_jiaosudu_2'
   *  Inport: '<Root>/now_M3_pos'
   */
  rtb_Sum1_a_idx_0 = 0.013802809929314557 * rtU.Tar_shizhi_wanqu_jiaosudu_2 -
    1.0;
  rtb_Sum1_a_idx_1 = 0.011442940347073955 * rtU.now_M3_pos - 1.0;

  /* Sum: '<S458>/netsum' incorporates:
   *  Constant: '<S458>/b{1}'
   *  Constant: '<S465>/IW{1,1}(1,:)''
   *  Constant: '<S465>/IW{1,1}(2,:)''
   *  Constant: '<S465>/IW{1,1}(3,:)''
   *  Constant: '<S465>/IW{1,1}(4,:)''
   *  Constant: '<S465>/IW{1,1}(5,:)''
   *  Constant: '<S465>/IW{1,1}(6,:)''
   *  Constant: '<S465>/IW{1,1}(7,:)''
   *  DotProduct: '<S467>/Dot Product'
   *  DotProduct: '<S468>/Dot Product'
   *  DotProduct: '<S469>/Dot Product'
   *  DotProduct: '<S470>/Dot Product'
   *  DotProduct: '<S471>/Dot Product'
   *  DotProduct: '<S472>/Dot Product'
   *  DotProduct: '<S473>/Dot Product'
   *  Gain: '<S466>/Gain'
   */
  rtDW.dv1[0] = ((-2.7119273351817843 * rtb_Sum1_a_idx_0 + 0.44747157470388466 *
                  rtb_Sum1_a_idx_1) + 4.3912795586119442) * -2.0;
  rtDW.dv1[1] = ((-1.8321903911205828 * rtb_Sum1_a_idx_0 + -2.9020703800423036 *
                  rtb_Sum1_a_idx_1) + 2.1151409130516212) * -2.0;
  rtDW.dv1[2] = ((-1.7890478813626052 * rtb_Sum1_a_idx_0 + 0.95481263822193607 *
                  rtb_Sum1_a_idx_1) + 1.4629575044399497) * -2.0;
  rtDW.dv1[3] = ((1.1106836063599648 * rtb_Sum1_a_idx_0 + 1.2723813034298581 *
                  rtb_Sum1_a_idx_1) + 0.29848557709419393) * -2.0;
  rtDW.dv1[4] = ((-1.2399859063766674 * rtb_Sum1_a_idx_0 + 0.8273926707601007 *
                  rtb_Sum1_a_idx_1) - 0.31477835330976894) * -2.0;
  rtDW.dv1[5] = ((1.3170560435883034 * rtb_Sum1_a_idx_0 + -1.1855843485287676 *
                  rtb_Sum1_a_idx_1) + 2.06338795838314) * -2.0;
  rtDW.dv1[6] = ((-1.9914124966733338 * rtb_Sum1_a_idx_0 + -2.0789387628368923 *
                  rtb_Sum1_a_idx_1) - 3.1625304334615212) * -2.0;

  /* DotProduct: '<S477>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S478>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S479>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S480>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S481>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S482>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S483>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S477>/Dot Product' incorporates:
     *  Constant: '<S466>/one'
     *  Constant: '<S466>/one1'
     *  Constant: '<S475>/IW{2,1}(1,:)''
     *  DotProduct: '<S478>/Dot Product'
     *  DotProduct: '<S479>/Dot Product'
     *  DotProduct: '<S480>/Dot Product'
     *  DotProduct: '<S481>/Dot Product'
     *  DotProduct: '<S482>/Dot Product'
     *  DotProduct: '<S483>/Dot Product'
     *  Gain: '<S466>/Gain1'
     *  Math: '<S466>/Exp'
     *  Math: '<S466>/Reciprocal'
     *  Sum: '<S466>/Sum'
     *  Sum: '<S466>/Sum1'
     *
     * About '<S466>/Exp':
     *  Operator: exp
     *
     * About '<S466>/Reciprocal':
     *  Operator: reciprocal
     */
    y = 1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += y * rtConstP.pooled18[i];

    /* DotProduct: '<S478>/Dot Product' incorporates:
     *  Constant: '<S475>/IW{2,1}(2,:)''
     */
    tmp_1 += y * rtConstP.pooled19[i];

    /* DotProduct: '<S479>/Dot Product' incorporates:
     *  Constant: '<S475>/IW{2,1}(3,:)''
     */
    tmp_2 += y * rtConstP.pooled20[i];

    /* DotProduct: '<S480>/Dot Product' incorporates:
     *  Constant: '<S475>/IW{2,1}(4,:)''
     */
    tmp_3 += y * rtConstP.pooled21[i];

    /* DotProduct: '<S481>/Dot Product' incorporates:
     *  Constant: '<S475>/IW{2,1}(5,:)''
     */
    tmp_4 += y * rtConstP.pooled22[i];

    /* DotProduct: '<S482>/Dot Product' incorporates:
     *  Constant: '<S475>/IW{2,1}(6,:)''
     */
    rtb_DotProduct_i += y * rtConstP.pooled23[i];

    /* DotProduct: '<S483>/Dot Product' incorporates:
     *  Constant: '<S475>/IW{2,1}(7,:)''
     */
    rtb_Sum1_a_idx_0 += y * rtConstP.pooled24[i];
  }

  /* Sum: '<S459>/netsum' incorporates:
   *  Constant: '<S459>/b{2}'
   *  Gain: '<S476>/Gain'
   */
  rtDW.dv1[0] = (tmp_0 + 1.3728525719786651) * -2.0;
  rtDW.dv1[1] = (tmp_1 - 1.1736901051043762) * -2.0;
  rtDW.dv1[2] = (tmp_2 + 0.55600983842864748) * -2.0;
  rtDW.dv1[3] = (tmp_3 + 0.16302991841207706) * -2.0;
  rtDW.dv1[4] = (tmp_4 + 0.6330461767913923) * -2.0;
  rtDW.dv1[5] = (rtb_DotProduct_i + 1.2265049835315) * -2.0;
  rtDW.dv1[6] = (rtb_Sum1_a_idx_0 - 1.9603788241706528) * -2.0;

  /* DotProduct: '<S487>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S488>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S489>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S490>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S491>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S492>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S493>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S487>/Dot Product' incorporates:
     *  Constant: '<S476>/one'
     *  Constant: '<S476>/one1'
     *  Constant: '<S485>/IW{3,2}(1,:)''
     *  DotProduct: '<S488>/Dot Product'
     *  DotProduct: '<S489>/Dot Product'
     *  DotProduct: '<S490>/Dot Product'
     *  DotProduct: '<S491>/Dot Product'
     *  DotProduct: '<S492>/Dot Product'
     *  DotProduct: '<S493>/Dot Product'
     *  Gain: '<S476>/Gain1'
     *  Math: '<S476>/Exp'
     *  Math: '<S476>/Reciprocal'
     *  Sum: '<S476>/Sum'
     *  Sum: '<S476>/Sum1'
     *
     * About '<S476>/Exp':
     *  Operator: exp
     *
     * About '<S476>/Reciprocal':
     *  Operator: reciprocal
     */
    y = 1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += y * rtConstP.pooled26[i];

    /* DotProduct: '<S488>/Dot Product' incorporates:
     *  Constant: '<S485>/IW{3,2}(2,:)''
     */
    tmp_1 += y * rtConstP.pooled27[i];

    /* DotProduct: '<S489>/Dot Product' incorporates:
     *  Constant: '<S485>/IW{3,2}(3,:)''
     */
    tmp_2 += y * rtConstP.pooled28[i];

    /* DotProduct: '<S490>/Dot Product' incorporates:
     *  Constant: '<S485>/IW{3,2}(4,:)''
     */
    tmp_3 += y * rtConstP.pooled29[i];

    /* DotProduct: '<S491>/Dot Product' incorporates:
     *  Constant: '<S485>/IW{3,2}(5,:)''
     */
    tmp_4 += y * rtConstP.pooled30[i];

    /* DotProduct: '<S492>/Dot Product' incorporates:
     *  Constant: '<S485>/IW{3,2}(6,:)''
     */
    rtb_DotProduct_i += y * rtConstP.pooled31[i];

    /* DotProduct: '<S493>/Dot Product' incorporates:
     *  Constant: '<S485>/IW{3,2}(7,:)''
     */
    rtb_Sum1_a_idx_0 += y * rtConstP.pooled32[i];
  }

  /* Sum: '<S460>/netsum' incorporates:
   *  Constant: '<S460>/b{3}'
   *  Gain: '<S486>/Gain'
   */
  rtDW.dv1[0] = (tmp_0 + 1.8054912293314147) * -2.0;
  rtDW.dv1[1] = (tmp_1 + 1.1443737624177923) * -2.0;
  rtDW.dv1[2] = (tmp_2 + 0.6074819532213086) * -2.0;
  rtDW.dv1[3] = (tmp_3 + 0.1111786603260108) * -2.0;
  rtDW.dv1[4] = (tmp_4 + 0.018904421515564152) * -2.0;
  rtDW.dv1[5] = (rtb_DotProduct_i + 1.2687208073238621) * -2.0;
  rtDW.dv1[6] = (rtb_Sum1_a_idx_0 + 1.8097340802078581) * -2.0;

  /* DotProduct: '<S497>/Dot Product' */
  tmp_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S497>/Dot Product' incorporates:
     *  Constant: '<S486>/one'
     *  Constant: '<S486>/one1'
     *  Constant: '<S495>/IW{4,3}(1,:)''
     *  Gain: '<S486>/Gain1'
     *  Math: '<S486>/Exp'
     *  Math: '<S486>/Reciprocal'
     *  Sum: '<S486>/Sum'
     *  Sum: '<S486>/Sum1'
     *
     * About '<S486>/Exp':
     *  Operator: exp
     *
     * About '<S486>/Reciprocal':
     *  Operator: reciprocal
     */
    tmp_0 += (1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0) * rtConstP.pooled34[i];
  }

  /* MATLAB Function: '<S1>/MATLAB Function3' incorporates:
   *  Bias: '<S499>/Subtract min y'
   *  Constant: '<S461>/b{4}'
   *  Gain: '<S499>/Divide by range y'
   *  Outport: '<Root>/Tar_M3_spd'
   *  Sum: '<S461>/netsum'
   */
  MATLABFunction(112.4249833296288 * ((tmp_0 + 0.0525359417320578) + 1.0),
                 &rtY.Tar_M3_spd);

  /* Bias: '<S78>/Add min y' incorporates:
   *  Gain: '<S78>/range y // range x'
   *  Inport: '<Root>/Tar_zhongzhi_wanqu_jiaosudu_2'
   *  Inport: '<Root>/now_M4_pos'
   */
  rtb_Sum1_a_idx_0 = 0.013802809929314557 * rtU.Tar_zhongzhi_wanqu_jiaosudu_2 -
    1.0;
  rtb_Sum1_a_idx_1 = 0.011442940347073955 * rtU.now_M4_pos - 1.0;

  /* Sum: '<S38>/netsum' incorporates:
   *  Constant: '<S38>/b{1}'
   *  Constant: '<S45>/IW{1,1}(1,:)''
   *  Constant: '<S45>/IW{1,1}(2,:)''
   *  Constant: '<S45>/IW{1,1}(3,:)''
   *  Constant: '<S45>/IW{1,1}(4,:)''
   *  Constant: '<S45>/IW{1,1}(5,:)''
   *  Constant: '<S45>/IW{1,1}(6,:)''
   *  Constant: '<S45>/IW{1,1}(7,:)''
   *  DotProduct: '<S47>/Dot Product'
   *  DotProduct: '<S48>/Dot Product'
   *  DotProduct: '<S49>/Dot Product'
   *  DotProduct: '<S50>/Dot Product'
   *  DotProduct: '<S51>/Dot Product'
   *  DotProduct: '<S52>/Dot Product'
   *  DotProduct: '<S53>/Dot Product'
   *  Gain: '<S46>/Gain'
   */
  rtDW.dv1[0] = ((-2.7119273351817843 * rtb_Sum1_a_idx_0 + 0.44747157470388466 *
                  rtb_Sum1_a_idx_1) + 4.3912795586119442) * -2.0;
  rtDW.dv1[1] = ((-1.8321903911205828 * rtb_Sum1_a_idx_0 + -2.9020703800423036 *
                  rtb_Sum1_a_idx_1) + 2.1151409130516212) * -2.0;
  rtDW.dv1[2] = ((-1.7890478813626052 * rtb_Sum1_a_idx_0 + 0.95481263822193607 *
                  rtb_Sum1_a_idx_1) + 1.4629575044399497) * -2.0;
  rtDW.dv1[3] = ((1.1106836063599648 * rtb_Sum1_a_idx_0 + 1.2723813034298581 *
                  rtb_Sum1_a_idx_1) + 0.29848557709419393) * -2.0;
  rtDW.dv1[4] = ((-1.2399859063766674 * rtb_Sum1_a_idx_0 + 0.8273926707601007 *
                  rtb_Sum1_a_idx_1) - 0.31477835330976894) * -2.0;
  rtDW.dv1[5] = ((1.3170560435883034 * rtb_Sum1_a_idx_0 + -1.1855843485287676 *
                  rtb_Sum1_a_idx_1) + 2.06338795838314) * -2.0;
  rtDW.dv1[6] = ((-1.9914124966733338 * rtb_Sum1_a_idx_0 + -2.0789387628368923 *
                  rtb_Sum1_a_idx_1) - 3.1625304334615212) * -2.0;

  /* DotProduct: '<S57>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S58>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S59>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S60>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S61>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S62>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S63>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S57>/Dot Product' incorporates:
     *  Constant: '<S46>/one'
     *  Constant: '<S46>/one1'
     *  Constant: '<S55>/IW{2,1}(1,:)''
     *  DotProduct: '<S58>/Dot Product'
     *  DotProduct: '<S59>/Dot Product'
     *  DotProduct: '<S60>/Dot Product'
     *  DotProduct: '<S61>/Dot Product'
     *  DotProduct: '<S62>/Dot Product'
     *  DotProduct: '<S63>/Dot Product'
     *  Gain: '<S46>/Gain1'
     *  Math: '<S46>/Exp'
     *  Math: '<S46>/Reciprocal'
     *  Sum: '<S46>/Sum'
     *  Sum: '<S46>/Sum1'
     *
     * About '<S46>/Exp':
     *  Operator: exp
     *
     * About '<S46>/Reciprocal':
     *  Operator: reciprocal
     */
    y = 1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += y * rtConstP.pooled18[i];

    /* DotProduct: '<S58>/Dot Product' incorporates:
     *  Constant: '<S55>/IW{2,1}(2,:)''
     */
    tmp_1 += y * rtConstP.pooled19[i];

    /* DotProduct: '<S59>/Dot Product' incorporates:
     *  Constant: '<S55>/IW{2,1}(3,:)''
     */
    tmp_2 += y * rtConstP.pooled20[i];

    /* DotProduct: '<S60>/Dot Product' incorporates:
     *  Constant: '<S55>/IW{2,1}(4,:)''
     */
    tmp_3 += y * rtConstP.pooled21[i];

    /* DotProduct: '<S61>/Dot Product' incorporates:
     *  Constant: '<S55>/IW{2,1}(5,:)''
     */
    tmp_4 += y * rtConstP.pooled22[i];

    /* DotProduct: '<S62>/Dot Product' incorporates:
     *  Constant: '<S55>/IW{2,1}(6,:)''
     */
    rtb_DotProduct_i += y * rtConstP.pooled23[i];

    /* DotProduct: '<S63>/Dot Product' incorporates:
     *  Constant: '<S55>/IW{2,1}(7,:)''
     */
    rtb_Sum1_a_idx_0 += y * rtConstP.pooled24[i];
  }

  /* Sum: '<S39>/netsum' incorporates:
   *  Constant: '<S39>/b{2}'
   *  Gain: '<S56>/Gain'
   */
  rtDW.dv1[0] = (tmp_0 + 1.3728525719786651) * -2.0;
  rtDW.dv1[1] = (tmp_1 - 1.1736901051043762) * -2.0;
  rtDW.dv1[2] = (tmp_2 + 0.55600983842864748) * -2.0;
  rtDW.dv1[3] = (tmp_3 + 0.16302991841207706) * -2.0;
  rtDW.dv1[4] = (tmp_4 + 0.6330461767913923) * -2.0;
  rtDW.dv1[5] = (rtb_DotProduct_i + 1.2265049835315) * -2.0;
  rtDW.dv1[6] = (rtb_Sum1_a_idx_0 - 1.9603788241706528) * -2.0;

  /* DotProduct: '<S67>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S68>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S69>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S70>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S71>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S72>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S73>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S67>/Dot Product' incorporates:
     *  Constant: '<S56>/one'
     *  Constant: '<S56>/one1'
     *  Constant: '<S65>/IW{3,2}(1,:)''
     *  DotProduct: '<S68>/Dot Product'
     *  DotProduct: '<S69>/Dot Product'
     *  DotProduct: '<S70>/Dot Product'
     *  DotProduct: '<S71>/Dot Product'
     *  DotProduct: '<S72>/Dot Product'
     *  DotProduct: '<S73>/Dot Product'
     *  Gain: '<S56>/Gain1'
     *  Math: '<S56>/Exp'
     *  Math: '<S56>/Reciprocal'
     *  Sum: '<S56>/Sum'
     *  Sum: '<S56>/Sum1'
     *
     * About '<S56>/Exp':
     *  Operator: exp
     *
     * About '<S56>/Reciprocal':
     *  Operator: reciprocal
     */
    y = 1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += y * rtConstP.pooled26[i];

    /* DotProduct: '<S68>/Dot Product' incorporates:
     *  Constant: '<S65>/IW{3,2}(2,:)''
     */
    tmp_1 += y * rtConstP.pooled27[i];

    /* DotProduct: '<S69>/Dot Product' incorporates:
     *  Constant: '<S65>/IW{3,2}(3,:)''
     */
    tmp_2 += y * rtConstP.pooled28[i];

    /* DotProduct: '<S70>/Dot Product' incorporates:
     *  Constant: '<S65>/IW{3,2}(4,:)''
     */
    tmp_3 += y * rtConstP.pooled29[i];

    /* DotProduct: '<S71>/Dot Product' incorporates:
     *  Constant: '<S65>/IW{3,2}(5,:)''
     */
    tmp_4 += y * rtConstP.pooled30[i];

    /* DotProduct: '<S72>/Dot Product' incorporates:
     *  Constant: '<S65>/IW{3,2}(6,:)''
     */
    rtb_DotProduct_i += y * rtConstP.pooled31[i];

    /* DotProduct: '<S73>/Dot Product' incorporates:
     *  Constant: '<S65>/IW{3,2}(7,:)''
     */
    rtb_Sum1_a_idx_0 += y * rtConstP.pooled32[i];
  }

  /* Sum: '<S40>/netsum' incorporates:
   *  Constant: '<S40>/b{3}'
   *  Gain: '<S66>/Gain'
   */
  rtDW.dv1[0] = (tmp_0 + 1.8054912293314147) * -2.0;
  rtDW.dv1[1] = (tmp_1 + 1.1443737624177923) * -2.0;
  rtDW.dv1[2] = (tmp_2 + 0.6074819532213086) * -2.0;
  rtDW.dv1[3] = (tmp_3 + 0.1111786603260108) * -2.0;
  rtDW.dv1[4] = (tmp_4 + 0.018904421515564152) * -2.0;
  rtDW.dv1[5] = (rtb_DotProduct_i + 1.2687208073238621) * -2.0;
  rtDW.dv1[6] = (rtb_Sum1_a_idx_0 + 1.8097340802078581) * -2.0;

  /* DotProduct: '<S77>/Dot Product' */
  tmp_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S77>/Dot Product' incorporates:
     *  Constant: '<S66>/one'
     *  Constant: '<S66>/one1'
     *  Constant: '<S75>/IW{4,3}(1,:)''
     *  Gain: '<S66>/Gain1'
     *  Math: '<S66>/Exp'
     *  Math: '<S66>/Reciprocal'
     *  Sum: '<S66>/Sum'
     *  Sum: '<S66>/Sum1'
     *
     * About '<S66>/Exp':
     *  Operator: exp
     *
     * About '<S66>/Reciprocal':
     *  Operator: reciprocal
     */
    tmp_0 += (1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0) * rtConstP.pooled34[i];
  }

  /* MATLAB Function: '<S1>/MATLAB Function4' incorporates:
   *  Bias: '<S79>/Subtract min y'
   *  Constant: '<S41>/b{4}'
   *  Gain: '<S79>/Divide by range y'
   *  Outport: '<Root>/Tar_M4_spd'
   *  Sum: '<S41>/netsum'
   */
  MATLABFunction(112.4249833296288 * ((tmp_0 + 0.0525359417320578) + 1.0),
                 &rtY.Tar_M4_spd);

  /* Bias: '<S411>/Add min y' incorporates:
   *  Gain: '<S411>/range y // range x'
   *  Inport: '<Root>/Tar_wumingzhi_wanqu_jiaosudu_2'
   *  Inport: '<Root>/now_M5_pos'
   */
  rtb_Sum1_a_idx_0 = 0.013802809929314557 * rtU.Tar_wumingzhi_wanqu_jiaosudu_2 -
    1.0;
  rtb_Sum1_a_idx_1 = 0.011442940347073955 * rtU.now_M5_pos - 1.0;

  /* Sum: '<S371>/netsum' incorporates:
   *  Constant: '<S371>/b{1}'
   *  Constant: '<S378>/IW{1,1}(1,:)''
   *  Constant: '<S378>/IW{1,1}(2,:)''
   *  Constant: '<S378>/IW{1,1}(3,:)''
   *  Constant: '<S378>/IW{1,1}(4,:)''
   *  Constant: '<S378>/IW{1,1}(5,:)''
   *  Constant: '<S378>/IW{1,1}(6,:)''
   *  Constant: '<S378>/IW{1,1}(7,:)''
   *  DotProduct: '<S380>/Dot Product'
   *  DotProduct: '<S381>/Dot Product'
   *  DotProduct: '<S382>/Dot Product'
   *  DotProduct: '<S383>/Dot Product'
   *  DotProduct: '<S384>/Dot Product'
   *  DotProduct: '<S385>/Dot Product'
   *  DotProduct: '<S386>/Dot Product'
   *  Gain: '<S379>/Gain'
   */
  rtDW.dv1[0] = ((-2.7119273351817843 * rtb_Sum1_a_idx_0 + 0.44747157470388466 *
                  rtb_Sum1_a_idx_1) + 4.3912795586119442) * -2.0;
  rtDW.dv1[1] = ((-1.8321903911205828 * rtb_Sum1_a_idx_0 + -2.9020703800423036 *
                  rtb_Sum1_a_idx_1) + 2.1151409130516212) * -2.0;
  rtDW.dv1[2] = ((-1.7890478813626052 * rtb_Sum1_a_idx_0 + 0.95481263822193607 *
                  rtb_Sum1_a_idx_1) + 1.4629575044399497) * -2.0;
  rtDW.dv1[3] = ((1.1106836063599648 * rtb_Sum1_a_idx_0 + 1.2723813034298581 *
                  rtb_Sum1_a_idx_1) + 0.29848557709419393) * -2.0;
  rtDW.dv1[4] = ((-1.2399859063766674 * rtb_Sum1_a_idx_0 + 0.8273926707601007 *
                  rtb_Sum1_a_idx_1) - 0.31477835330976894) * -2.0;
  rtDW.dv1[5] = ((1.3170560435883034 * rtb_Sum1_a_idx_0 + -1.1855843485287676 *
                  rtb_Sum1_a_idx_1) + 2.06338795838314) * -2.0;
  rtDW.dv1[6] = ((-1.9914124966733338 * rtb_Sum1_a_idx_0 + -2.0789387628368923 *
                  rtb_Sum1_a_idx_1) - 3.1625304334615212) * -2.0;

  /* DotProduct: '<S390>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S391>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S392>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S393>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S394>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S395>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S396>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S390>/Dot Product' incorporates:
     *  Constant: '<S379>/one'
     *  Constant: '<S379>/one1'
     *  Constant: '<S388>/IW{2,1}(1,:)''
     *  DotProduct: '<S391>/Dot Product'
     *  DotProduct: '<S392>/Dot Product'
     *  DotProduct: '<S393>/Dot Product'
     *  DotProduct: '<S394>/Dot Product'
     *  DotProduct: '<S395>/Dot Product'
     *  DotProduct: '<S396>/Dot Product'
     *  Gain: '<S379>/Gain1'
     *  Math: '<S379>/Exp'
     *  Math: '<S379>/Reciprocal'
     *  Sum: '<S379>/Sum'
     *  Sum: '<S379>/Sum1'
     *
     * About '<S379>/Exp':
     *  Operator: exp
     *
     * About '<S379>/Reciprocal':
     *  Operator: reciprocal
     */
    y = 1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += y * rtConstP.pooled18[i];

    /* DotProduct: '<S391>/Dot Product' incorporates:
     *  Constant: '<S388>/IW{2,1}(2,:)''
     */
    tmp_1 += y * rtConstP.pooled19[i];

    /* DotProduct: '<S392>/Dot Product' incorporates:
     *  Constant: '<S388>/IW{2,1}(3,:)''
     */
    tmp_2 += y * rtConstP.pooled20[i];

    /* DotProduct: '<S393>/Dot Product' incorporates:
     *  Constant: '<S388>/IW{2,1}(4,:)''
     */
    tmp_3 += y * rtConstP.pooled21[i];

    /* DotProduct: '<S394>/Dot Product' incorporates:
     *  Constant: '<S388>/IW{2,1}(5,:)''
     */
    tmp_4 += y * rtConstP.pooled22[i];

    /* DotProduct: '<S395>/Dot Product' incorporates:
     *  Constant: '<S388>/IW{2,1}(6,:)''
     */
    rtb_DotProduct_i += y * rtConstP.pooled23[i];

    /* DotProduct: '<S396>/Dot Product' incorporates:
     *  Constant: '<S388>/IW{2,1}(7,:)''
     */
    rtb_Sum1_a_idx_0 += y * rtConstP.pooled24[i];
  }

  /* Sum: '<S372>/netsum' incorporates:
   *  Constant: '<S372>/b{2}'
   *  Gain: '<S389>/Gain'
   */
  rtDW.dv1[0] = (tmp_0 + 1.3728525719786651) * -2.0;
  rtDW.dv1[1] = (tmp_1 - 1.1736901051043762) * -2.0;
  rtDW.dv1[2] = (tmp_2 + 0.55600983842864748) * -2.0;
  rtDW.dv1[3] = (tmp_3 + 0.16302991841207706) * -2.0;
  rtDW.dv1[4] = (tmp_4 + 0.6330461767913923) * -2.0;
  rtDW.dv1[5] = (rtb_DotProduct_i + 1.2265049835315) * -2.0;
  rtDW.dv1[6] = (rtb_Sum1_a_idx_0 - 1.9603788241706528) * -2.0;

  /* DotProduct: '<S400>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S401>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S402>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S403>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S404>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S405>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S406>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S400>/Dot Product' incorporates:
     *  Constant: '<S389>/one'
     *  Constant: '<S389>/one1'
     *  Constant: '<S398>/IW{3,2}(1,:)''
     *  DotProduct: '<S401>/Dot Product'
     *  DotProduct: '<S402>/Dot Product'
     *  DotProduct: '<S403>/Dot Product'
     *  DotProduct: '<S404>/Dot Product'
     *  DotProduct: '<S405>/Dot Product'
     *  DotProduct: '<S406>/Dot Product'
     *  Gain: '<S389>/Gain1'
     *  Math: '<S389>/Exp'
     *  Math: '<S389>/Reciprocal'
     *  Sum: '<S389>/Sum'
     *  Sum: '<S389>/Sum1'
     *
     * About '<S389>/Exp':
     *  Operator: exp
     *
     * About '<S389>/Reciprocal':
     *  Operator: reciprocal
     */
    y = 1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += y * rtConstP.pooled26[i];

    /* DotProduct: '<S401>/Dot Product' incorporates:
     *  Constant: '<S398>/IW{3,2}(2,:)''
     */
    tmp_1 += y * rtConstP.pooled27[i];

    /* DotProduct: '<S402>/Dot Product' incorporates:
     *  Constant: '<S398>/IW{3,2}(3,:)''
     */
    tmp_2 += y * rtConstP.pooled28[i];

    /* DotProduct: '<S403>/Dot Product' incorporates:
     *  Constant: '<S398>/IW{3,2}(4,:)''
     */
    tmp_3 += y * rtConstP.pooled29[i];

    /* DotProduct: '<S404>/Dot Product' incorporates:
     *  Constant: '<S398>/IW{3,2}(5,:)''
     */
    tmp_4 += y * rtConstP.pooled30[i];

    /* DotProduct: '<S405>/Dot Product' incorporates:
     *  Constant: '<S398>/IW{3,2}(6,:)''
     */
    rtb_DotProduct_i += y * rtConstP.pooled31[i];

    /* DotProduct: '<S406>/Dot Product' incorporates:
     *  Constant: '<S398>/IW{3,2}(7,:)''
     */
    rtb_Sum1_a_idx_0 += y * rtConstP.pooled32[i];
  }

  /* Sum: '<S373>/netsum' incorporates:
   *  Constant: '<S373>/b{3}'
   *  Gain: '<S399>/Gain'
   */
  rtDW.dv1[0] = (tmp_0 + 1.8054912293314147) * -2.0;
  rtDW.dv1[1] = (tmp_1 + 1.1443737624177923) * -2.0;
  rtDW.dv1[2] = (tmp_2 + 0.6074819532213086) * -2.0;
  rtDW.dv1[3] = (tmp_3 + 0.1111786603260108) * -2.0;
  rtDW.dv1[4] = (tmp_4 + 0.018904421515564152) * -2.0;
  rtDW.dv1[5] = (rtb_DotProduct_i + 1.2687208073238621) * -2.0;
  rtDW.dv1[6] = (rtb_Sum1_a_idx_0 + 1.8097340802078581) * -2.0;

  /* DotProduct: '<S410>/Dot Product' */
  tmp_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S410>/Dot Product' incorporates:
     *  Constant: '<S399>/one'
     *  Constant: '<S399>/one1'
     *  Constant: '<S408>/IW{4,3}(1,:)''
     *  Gain: '<S399>/Gain1'
     *  Math: '<S399>/Exp'
     *  Math: '<S399>/Reciprocal'
     *  Sum: '<S399>/Sum'
     *  Sum: '<S399>/Sum1'
     *
     * About '<S399>/Exp':
     *  Operator: exp
     *
     * About '<S399>/Reciprocal':
     *  Operator: reciprocal
     */
    tmp_0 += (1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0) * rtConstP.pooled34[i];
  }

  /* MATLAB Function: '<S1>/MATLAB Function5' incorporates:
   *  Bias: '<S412>/Subtract min y'
   *  Constant: '<S374>/b{4}'
   *  Gain: '<S412>/Divide by range y'
   *  Outport: '<Root>/Tar_M5_spd'
   *  Sum: '<S374>/netsum'
   */
  MATLABFunction(112.4249833296288 * ((tmp_0 + 0.0525359417320578) + 1.0),
                 &rtY.Tar_M5_spd);

  /* Bias: '<S165>/Add min y' incorporates:
   *  Gain: '<S165>/range y // range x'
   *  Inport: '<Root>/Tar_xiaomuzhi_wanqu_jiaosudu_2'
   *  Inport: '<Root>/now_M6_pos'
   */
  rtb_Sum1_a_idx_0 = 0.013802809929314557 * rtU.Tar_xiaomuzhi_wanqu_jiaosudu_2 -
    1.0;
  rtb_Sum1_a_idx_1 = 0.011442940347073955 * rtU.now_M6_pos - 1.0;

  /* Sum: '<S125>/netsum' incorporates:
   *  Constant: '<S125>/b{1}'
   *  Constant: '<S132>/IW{1,1}(1,:)''
   *  Constant: '<S132>/IW{1,1}(2,:)''
   *  Constant: '<S132>/IW{1,1}(3,:)''
   *  Constant: '<S132>/IW{1,1}(4,:)''
   *  Constant: '<S132>/IW{1,1}(5,:)''
   *  Constant: '<S132>/IW{1,1}(6,:)''
   *  Constant: '<S132>/IW{1,1}(7,:)''
   *  DotProduct: '<S134>/Dot Product'
   *  DotProduct: '<S135>/Dot Product'
   *  DotProduct: '<S136>/Dot Product'
   *  DotProduct: '<S137>/Dot Product'
   *  DotProduct: '<S138>/Dot Product'
   *  DotProduct: '<S139>/Dot Product'
   *  DotProduct: '<S140>/Dot Product'
   *  Gain: '<S133>/Gain'
   */
  rtDW.dv1[0] = ((-2.7119273351817843 * rtb_Sum1_a_idx_0 + 0.44747157470388466 *
                  rtb_Sum1_a_idx_1) + 4.3912795586119442) * -2.0;
  rtDW.dv1[1] = ((-1.8321903911205828 * rtb_Sum1_a_idx_0 + -2.9020703800423036 *
                  rtb_Sum1_a_idx_1) + 2.1151409130516212) * -2.0;
  rtDW.dv1[2] = ((-1.7890478813626052 * rtb_Sum1_a_idx_0 + 0.95481263822193607 *
                  rtb_Sum1_a_idx_1) + 1.4629575044399497) * -2.0;
  rtDW.dv1[3] = ((1.1106836063599648 * rtb_Sum1_a_idx_0 + 1.2723813034298581 *
                  rtb_Sum1_a_idx_1) + 0.29848557709419393) * -2.0;
  rtDW.dv1[4] = ((-1.2399859063766674 * rtb_Sum1_a_idx_0 + 0.8273926707601007 *
                  rtb_Sum1_a_idx_1) - 0.31477835330976894) * -2.0;
  rtDW.dv1[5] = ((1.3170560435883034 * rtb_Sum1_a_idx_0 + -1.1855843485287676 *
                  rtb_Sum1_a_idx_1) + 2.06338795838314) * -2.0;
  rtDW.dv1[6] = ((-1.9914124966733338 * rtb_Sum1_a_idx_0 + -2.0789387628368923 *
                  rtb_Sum1_a_idx_1) - 3.1625304334615212) * -2.0;

  /* DotProduct: '<S144>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S145>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S146>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S147>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S148>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S149>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S150>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S144>/Dot Product' incorporates:
     *  Constant: '<S133>/one'
     *  Constant: '<S133>/one1'
     *  Constant: '<S142>/IW{2,1}(1,:)''
     *  DotProduct: '<S145>/Dot Product'
     *  DotProduct: '<S146>/Dot Product'
     *  DotProduct: '<S147>/Dot Product'
     *  DotProduct: '<S148>/Dot Product'
     *  DotProduct: '<S149>/Dot Product'
     *  DotProduct: '<S150>/Dot Product'
     *  Gain: '<S133>/Gain1'
     *  Math: '<S133>/Exp'
     *  Math: '<S133>/Reciprocal'
     *  Sum: '<S133>/Sum'
     *  Sum: '<S133>/Sum1'
     *
     * About '<S133>/Exp':
     *  Operator: exp
     *
     * About '<S133>/Reciprocal':
     *  Operator: reciprocal
     */
    y = 1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += y * rtConstP.pooled18[i];

    /* DotProduct: '<S145>/Dot Product' incorporates:
     *  Constant: '<S142>/IW{2,1}(2,:)''
     */
    tmp_1 += y * rtConstP.pooled19[i];

    /* DotProduct: '<S146>/Dot Product' incorporates:
     *  Constant: '<S142>/IW{2,1}(3,:)''
     */
    tmp_2 += y * rtConstP.pooled20[i];

    /* DotProduct: '<S147>/Dot Product' incorporates:
     *  Constant: '<S142>/IW{2,1}(4,:)''
     */
    tmp_3 += y * rtConstP.pooled21[i];

    /* DotProduct: '<S148>/Dot Product' incorporates:
     *  Constant: '<S142>/IW{2,1}(5,:)''
     */
    tmp_4 += y * rtConstP.pooled22[i];

    /* DotProduct: '<S149>/Dot Product' incorporates:
     *  Constant: '<S142>/IW{2,1}(6,:)''
     */
    rtb_DotProduct_i += y * rtConstP.pooled23[i];

    /* DotProduct: '<S150>/Dot Product' incorporates:
     *  Constant: '<S142>/IW{2,1}(7,:)''
     */
    rtb_Sum1_a_idx_0 += y * rtConstP.pooled24[i];
  }

  /* Sum: '<S126>/netsum' incorporates:
   *  Constant: '<S126>/b{2}'
   *  Gain: '<S143>/Gain'
   */
  rtDW.dv1[0] = (tmp_0 + 1.3728525719786651) * -2.0;
  rtDW.dv1[1] = (tmp_1 - 1.1736901051043762) * -2.0;
  rtDW.dv1[2] = (tmp_2 + 0.55600983842864748) * -2.0;
  rtDW.dv1[3] = (tmp_3 + 0.16302991841207706) * -2.0;
  rtDW.dv1[4] = (tmp_4 + 0.6330461767913923) * -2.0;
  rtDW.dv1[5] = (rtb_DotProduct_i + 1.2265049835315) * -2.0;
  rtDW.dv1[6] = (rtb_Sum1_a_idx_0 - 1.9603788241706528) * -2.0;

  /* DotProduct: '<S154>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S155>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S156>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S157>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S158>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S159>/Dot Product' */
  rtb_DotProduct_i = 0.0;

  /* DotProduct: '<S160>/Dot Product' */
  rtb_Sum1_a_idx_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S154>/Dot Product' incorporates:
     *  Constant: '<S143>/one'
     *  Constant: '<S143>/one1'
     *  Constant: '<S152>/IW{3,2}(1,:)''
     *  DotProduct: '<S155>/Dot Product'
     *  DotProduct: '<S156>/Dot Product'
     *  DotProduct: '<S157>/Dot Product'
     *  DotProduct: '<S158>/Dot Product'
     *  DotProduct: '<S159>/Dot Product'
     *  DotProduct: '<S160>/Dot Product'
     *  Gain: '<S143>/Gain1'
     *  Math: '<S143>/Exp'
     *  Math: '<S143>/Reciprocal'
     *  Sum: '<S143>/Sum'
     *  Sum: '<S143>/Sum1'
     *
     * About '<S143>/Exp':
     *  Operator: exp
     *
     * About '<S143>/Reciprocal':
     *  Operator: reciprocal
     */
    y = 1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += y * rtConstP.pooled26[i];

    /* DotProduct: '<S155>/Dot Product' incorporates:
     *  Constant: '<S152>/IW{3,2}(2,:)''
     */
    tmp_1 += y * rtConstP.pooled27[i];

    /* DotProduct: '<S156>/Dot Product' incorporates:
     *  Constant: '<S152>/IW{3,2}(3,:)''
     */
    tmp_2 += y * rtConstP.pooled28[i];

    /* DotProduct: '<S157>/Dot Product' incorporates:
     *  Constant: '<S152>/IW{3,2}(4,:)''
     */
    tmp_3 += y * rtConstP.pooled29[i];

    /* DotProduct: '<S158>/Dot Product' incorporates:
     *  Constant: '<S152>/IW{3,2}(5,:)''
     */
    tmp_4 += y * rtConstP.pooled30[i];

    /* DotProduct: '<S159>/Dot Product' incorporates:
     *  Constant: '<S152>/IW{3,2}(6,:)''
     */
    rtb_DotProduct_i += y * rtConstP.pooled31[i];

    /* DotProduct: '<S160>/Dot Product' incorporates:
     *  Constant: '<S152>/IW{3,2}(7,:)''
     */
    rtb_Sum1_a_idx_0 += y * rtConstP.pooled32[i];
  }

  /* Sum: '<S127>/netsum' incorporates:
   *  Constant: '<S127>/b{3}'
   *  Gain: '<S153>/Gain'
   */
  rtDW.dv1[0] = (tmp_0 + 1.8054912293314147) * -2.0;
  rtDW.dv1[1] = (tmp_1 + 1.1443737624177923) * -2.0;
  rtDW.dv1[2] = (tmp_2 + 0.6074819532213086) * -2.0;
  rtDW.dv1[3] = (tmp_3 + 0.1111786603260108) * -2.0;
  rtDW.dv1[4] = (tmp_4 + 0.018904421515564152) * -2.0;
  rtDW.dv1[5] = (rtb_DotProduct_i + 1.2687208073238621) * -2.0;
  rtDW.dv1[6] = (rtb_Sum1_a_idx_0 + 1.8097340802078581) * -2.0;

  /* DotProduct: '<S164>/Dot Product' */
  tmp_0 = 0.0;
  for (i = 0; i < 7; i++) {
    /* DotProduct: '<S164>/Dot Product' incorporates:
     *  Constant: '<S153>/one'
     *  Constant: '<S153>/one1'
     *  Constant: '<S162>/IW{4,3}(1,:)''
     *  Gain: '<S153>/Gain1'
     *  Math: '<S153>/Exp'
     *  Math: '<S153>/Reciprocal'
     *  Sum: '<S153>/Sum'
     *  Sum: '<S153>/Sum1'
     *
     * About '<S153>/Exp':
     *  Operator: exp
     *
     * About '<S153>/Reciprocal':
     *  Operator: reciprocal
     */
    tmp_0 += (1.0 / (exp(rtDW.dv1[i]) + 1.0) * 2.0 - 1.0) * rtConstP.pooled34[i];
  }

  /* MATLAB Function: '<S1>/MATLAB Function6' incorporates:
   *  Bias: '<S166>/Subtract min y'
   *  Constant: '<S128>/b{4}'
   *  Gain: '<S166>/Divide by range y'
   *  Outport: '<Root>/Tar_M6_spd'
   *  Sum: '<S128>/netsum'
   */
  MATLABFunction(112.4249833296288 * ((tmp_0 + 0.0525359417320578) + 1.0),
                 &rtY.Tar_M6_spd);

  /* Bias: '<S693>/Add min y' incorporates:
   *  Gain: '<S693>/range y // range x'
   *  Inport: '<Root>/now_M1_pos2'
   */
  rtb_DotProduct_i = 0.01222620826595501 * rtU.now_M1_pos2 - 1.0;

  /* Math: '<S682>/Exp' incorporates:
   *  Constant: '<S676>/b{1}'
   *  Constant: '<S681>/IW{1,1}(1,:)''
   *  Constant: '<S681>/IW{1,1}(2,:)''
   *  Constant: '<S681>/IW{1,1}(3,:)''
   *  Constant: '<S681>/IW{1,1}(4,:)''
   *  Constant: '<S681>/IW{1,1}(5,:)''
   *  Constant: '<S681>/IW{1,1}(6,:)''
   *  DotProduct: '<S683>/Dot Product'
   *  DotProduct: '<S684>/Dot Product'
   *  DotProduct: '<S685>/Dot Product'
   *  DotProduct: '<S686>/Dot Product'
   *  DotProduct: '<S687>/Dot Product'
   *  DotProduct: '<S688>/Dot Product'
   *  Gain: '<S682>/Gain'
   *  Sum: '<S676>/netsum'
   *
   * About '<S682>/Exp':
   *  Operator: exp
   */
  rtDW.Exp[0] = exp((7.1738203374863083 * rtb_DotProduct_i - 6.9217205801174311)
                    * -2.0);
  rtDW.Exp[1] = exp((5.0783178211844877 * rtb_DotProduct_i - 3.7096872734534312)
                    * -2.0);
  rtDW.Exp[2] = exp((-2.1329149563556213 * rtb_DotProduct_i +
                     0.86138796328916767) * -2.0);
  rtDW.Exp[3] = exp((1.9381030892372919 * rtb_DotProduct_i + 0.63443294113914184)
                    * -2.0);
  rtDW.Exp[4] = exp((4.9445371410763741 * rtb_DotProduct_i + 3.5080595403073294)
                    * -2.0);
  rtDW.Exp[5] = exp((-7.8242436813961014 * rtb_DotProduct_i - 7.3801754832242263)
                    * -2.0);

  /* Bias: '<S712>/Add min y' incorporates:
   *  Gain: '<S712>/range y // range x'
   *  Inport: '<Root>/now_M2_pos2'
   */
  rtb_DotProduct_i = 0.0104518435889565 * rtU.now_M2_pos2 - 1.0;

  /* Math: '<S701>/Exp' incorporates:
   *  Constant: '<S695>/b{1}'
   *  Constant: '<S700>/IW{1,1}(1,:)''
   *  Constant: '<S700>/IW{1,1}(2,:)''
   *  Constant: '<S700>/IW{1,1}(3,:)''
   *  Constant: '<S700>/IW{1,1}(4,:)''
   *  Constant: '<S700>/IW{1,1}(5,:)''
   *  Constant: '<S700>/IW{1,1}(6,:)''
   *  DotProduct: '<S702>/Dot Product'
   *  DotProduct: '<S703>/Dot Product'
   *  DotProduct: '<S704>/Dot Product'
   *  DotProduct: '<S705>/Dot Product'
   *  DotProduct: '<S706>/Dot Product'
   *  DotProduct: '<S707>/Dot Product'
   *  Gain: '<S701>/Gain'
   *  Sum: '<S695>/netsum'
   *
   * About '<S701>/Exp':
   *  Operator: exp
   */
  rtb_Exp_gn[0] = exp((7.1991835015158161 * rtb_DotProduct_i -
                       6.8385448075550981) * -2.0);
  rtb_Exp_gn[1] = exp((5.1105588559995807 * rtb_DotProduct_i - 3.593374102520924)
                      * -2.0);
  rtb_Exp_gn[2] = exp((-2.4043357476804021 * rtb_DotProduct_i +
                       0.893279792186059) * -2.0);
  rtb_Exp_gn[3] = exp((1.4318191299453014 * rtb_DotProduct_i +
                       0.61309848769164954) * -2.0);
  rtb_Exp_gn[4] = exp((4.1595707437845713 * rtb_DotProduct_i +
                       3.9411499506278806) * -2.0);
  rtb_Exp_gn[5] = exp((-6.8539567136055517 * rtb_DotProduct_i -
                       9.9697294453240257) * -2.0);

  /* Bias: '<S794>/Add min y' incorporates:
   *  Gain: '<S794>/range y // range x'
   *  Inport: '<Root>/now_M3_pos2'
   */
  rtb_DotProduct_i = 0.011444904687943789 * rtU.now_M3_pos2 - 1.0;

  /* Math: '<S783>/Exp' incorporates:
   *  Constant: '<S777>/b{1}'
   *  Constant: '<S782>/IW{1,1}(1,:)''
   *  Constant: '<S782>/IW{1,1}(2,:)''
   *  Constant: '<S782>/IW{1,1}(3,:)''
   *  Constant: '<S782>/IW{1,1}(4,:)''
   *  Constant: '<S782>/IW{1,1}(5,:)''
   *  Constant: '<S782>/IW{1,1}(6,:)''
   *  DotProduct: '<S784>/Dot Product'
   *  DotProduct: '<S785>/Dot Product'
   *  DotProduct: '<S786>/Dot Product'
   *  DotProduct: '<S787>/Dot Product'
   *  DotProduct: '<S788>/Dot Product'
   *  DotProduct: '<S789>/Dot Product'
   *  Gain: '<S783>/Gain'
   *  Sum: '<S777>/netsum'
   *
   * About '<S783>/Exp':
   *  Operator: exp
   */
  rtb_Exp_o2[0] = exp((7.3676055941048455 * rtb_DotProduct_i -
                       6.9028414594374992) * -2.0);
  rtb_Exp_o2[1] = exp((5.3424515392326164 * rtb_DotProduct_i -
                       3.6906179090820173) * -2.0);
  rtb_Exp_o2[2] = exp((-2.4764865492010677 * rtb_DotProduct_i +
                       0.84924460775240007) * -2.0);
  rtb_Exp_o2[3] = exp((2.0462451403506248 * rtb_DotProduct_i +
                       0.69387175541234292) * -2.0);
  rtb_Exp_o2[4] = exp((4.9043403101144083 * rtb_DotProduct_i +
                       3.5188058707067662) * -2.0);
  rtb_Exp_o2[5] = exp((-7.5091073001763071 * rtb_DotProduct_i -
                       7.1656131292885119) * -2.0);

  /* Bias: '<S584>/Add min y' incorporates:
   *  Gain: '<S584>/range y // range x'
   *  Inport: '<Root>/now_M4_pos2'
   */
  rtb_DotProduct_i = 0.011444904687943789 * rtU.now_M4_pos2 - 1.0;

  /* Math: '<S573>/Exp' incorporates:
   *  Constant: '<S567>/b{1}'
   *  Constant: '<S572>/IW{1,1}(1,:)''
   *  Constant: '<S572>/IW{1,1}(2,:)''
   *  Constant: '<S572>/IW{1,1}(3,:)''
   *  Constant: '<S572>/IW{1,1}(4,:)''
   *  Constant: '<S572>/IW{1,1}(5,:)''
   *  Constant: '<S572>/IW{1,1}(6,:)''
   *  DotProduct: '<S574>/Dot Product'
   *  DotProduct: '<S575>/Dot Product'
   *  DotProduct: '<S576>/Dot Product'
   *  DotProduct: '<S577>/Dot Product'
   *  DotProduct: '<S578>/Dot Product'
   *  DotProduct: '<S579>/Dot Product'
   *  Gain: '<S573>/Gain'
   *  Sum: '<S567>/netsum'
   *
   * About '<S573>/Exp':
   *  Operator: exp
   */
  rtb_Exp_fn[0] = exp((7.3676055941048455 * rtb_DotProduct_i -
                       6.9028414594374992) * -2.0);
  rtb_Exp_fn[1] = exp((5.3424515392326164 * rtb_DotProduct_i -
                       3.6906179090820173) * -2.0);
  rtb_Exp_fn[2] = exp((-2.4764865492010677 * rtb_DotProduct_i +
                       0.84924460775240007) * -2.0);
  rtb_Exp_fn[3] = exp((2.0462451403506248 * rtb_DotProduct_i +
                       0.69387175541234292) * -2.0);
  rtb_Exp_fn[4] = exp((4.9043403101144083 * rtb_DotProduct_i +
                       3.5188058707067662) * -2.0);
  rtb_Exp_fn[5] = exp((-7.5091073001763071 * rtb_DotProduct_i -
                       7.1656131292885119) * -2.0);

  /* Bias: '<S753>/Add min y' incorporates:
   *  Gain: '<S753>/range y // range x'
   *  Inport: '<Root>/now_M5_pos2'
   */
  rtb_DotProduct_i = 0.011444904687943789 * rtU.now_M5_pos2 - 1.0;

  /* Math: '<S742>/Exp' incorporates:
   *  Constant: '<S736>/b{1}'
   *  Constant: '<S741>/IW{1,1}(1,:)''
   *  Constant: '<S741>/IW{1,1}(2,:)''
   *  Constant: '<S741>/IW{1,1}(3,:)''
   *  Constant: '<S741>/IW{1,1}(4,:)''
   *  Constant: '<S741>/IW{1,1}(5,:)''
   *  Constant: '<S741>/IW{1,1}(6,:)''
   *  DotProduct: '<S743>/Dot Product'
   *  DotProduct: '<S744>/Dot Product'
   *  DotProduct: '<S745>/Dot Product'
   *  DotProduct: '<S746>/Dot Product'
   *  DotProduct: '<S747>/Dot Product'
   *  DotProduct: '<S748>/Dot Product'
   *  Gain: '<S742>/Gain'
   *  Sum: '<S736>/netsum'
   *
   * About '<S742>/Exp':
   *  Operator: exp
   */
  rtb_Exp_af[0] = exp((7.3676055941048455 * rtb_DotProduct_i -
                       6.9028414594374992) * -2.0);
  rtb_Exp_af[1] = exp((5.3424515392326164 * rtb_DotProduct_i -
                       3.6906179090820173) * -2.0);
  rtb_Exp_af[2] = exp((-2.4764865492010677 * rtb_DotProduct_i +
                       0.84924460775240007) * -2.0);
  rtb_Exp_af[3] = exp((2.0462451403506248 * rtb_DotProduct_i +
                       0.69387175541234292) * -2.0);
  rtb_Exp_af[4] = exp((4.9043403101144083 * rtb_DotProduct_i +
                       3.5188058707067662) * -2.0);
  rtb_Exp_af[5] = exp((-7.5091073001763071 * rtb_DotProduct_i -
                       7.1656131292885119) * -2.0);

  /* Bias: '<S625>/Add min y' incorporates:
   *  Gain: '<S625>/range y // range x'
   *  Inport: '<Root>/now_M6_pos2'
   */
  rtb_DotProduct_i = 0.011444904687943789 * rtU.now_M6_pos2 - 1.0;

  /* Math: '<S614>/Exp' incorporates:
   *  Constant: '<S608>/b{1}'
   *  Constant: '<S613>/IW{1,1}(1,:)''
   *  Constant: '<S613>/IW{1,1}(2,:)''
   *  Constant: '<S613>/IW{1,1}(3,:)''
   *  Constant: '<S613>/IW{1,1}(4,:)''
   *  Constant: '<S613>/IW{1,1}(5,:)''
   *  Constant: '<S613>/IW{1,1}(6,:)''
   *  DotProduct: '<S615>/Dot Product'
   *  DotProduct: '<S616>/Dot Product'
   *  DotProduct: '<S617>/Dot Product'
   *  DotProduct: '<S618>/Dot Product'
   *  DotProduct: '<S619>/Dot Product'
   *  DotProduct: '<S620>/Dot Product'
   *  Gain: '<S614>/Gain'
   *  Sum: '<S608>/netsum'
   *
   * About '<S614>/Exp':
   *  Operator: exp
   */
  rtb_Exp_ay[0] = exp((7.3676055941048455 * rtb_DotProduct_i -
                       6.9028414594374992) * -2.0);
  rtb_Exp_ay[1] = exp((5.3424515392326164 * rtb_DotProduct_i -
                       3.6906179090820173) * -2.0);
  rtb_Exp_ay[2] = exp((-2.4764865492010677 * rtb_DotProduct_i +
                       0.84924460775240007) * -2.0);
  rtb_Exp_ay[3] = exp((2.0462451403506248 * rtb_DotProduct_i +
                       0.69387175541234292) * -2.0);
  rtb_Exp_ay[4] = exp((4.9043403101144083 * rtb_DotProduct_i +
                       3.5188058707067662) * -2.0);
  rtb_Exp_ay[5] = exp((-7.5091073001763071 * rtb_DotProduct_i -
                       7.1656131292885119) * -2.0);

  /* Bias: '<S369>/Add min y' incorporates:
   *  Gain: '<S369>/range y // range x'
   *  Inport: '<Root>/now_M2_pos1'
   *  Inport: '<Root>/now_M2_spd'
   */
  rtb_Sum1_a_idx_0 = 0.010450205283362327 * rtU.now_M2_pos1 - 1.0;
  rtb_Sum1_a_idx_1 = 0.0080048038430744588 * rtU.now_M2_spd - 1.0;

  /* Sum: '<S326>/netsum' incorporates:
   *  Constant: '<S326>/b{1}'
   *  Constant: '<S334>/IW{1,1}(1,:)''
   *  Constant: '<S334>/IW{1,1}(2,:)''
   *  Constant: '<S334>/IW{1,1}(3,:)''
   *  Constant: '<S334>/IW{1,1}(4,:)''
   *  Constant: '<S334>/IW{1,1}(5,:)''
   *  DotProduct: '<S336>/Dot Product'
   *  DotProduct: '<S337>/Dot Product'
   *  DotProduct: '<S338>/Dot Product'
   *  DotProduct: '<S339>/Dot Product'
   *  DotProduct: '<S340>/Dot Product'
   *  Gain: '<S335>/Gain'
   */
  tmp_5[0] = ((-1.3742770601439496 * rtb_Sum1_a_idx_0 + -0.63893779308470855 *
               rtb_Sum1_a_idx_1) + 2.3030484149951636) * -2.0;
  tmp_5[1] = ((-0.036958975003470104 * rtb_Sum1_a_idx_0 + 0.765620669761754 *
               rtb_Sum1_a_idx_1) + 1.0641864500707539) * -2.0;
  tmp_5[2] = ((-0.94019688853344807 * rtb_Sum1_a_idx_0 + -0.28372098806797291 *
               rtb_Sum1_a_idx_1) - 0.73398219227795425) * -2.0;
  tmp_5[3] = ((0.529358579540111 * rtb_Sum1_a_idx_0 + 0.61461875669896038 *
               rtb_Sum1_a_idx_1) + 0.5359408997417684) * -2.0;
  tmp_5[4] = ((0.637585415696151 * rtb_Sum1_a_idx_0 + 1.6944195550927097 *
               rtb_Sum1_a_idx_1) - 2.42264062131828) * -2.0;

  /* DotProduct: '<S344>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S345>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S346>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S347>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S348>/Dot Product' */
  tmp_4 = 0.0;
  for (i = 0; i < 5; i++) {
    /* DotProduct: '<S344>/Dot Product' incorporates:
     *  Constant: '<S335>/one'
     *  Constant: '<S335>/one1'
     *  Constant: '<S342>/IW{2,1}(1,:)''
     *  DotProduct: '<S345>/Dot Product'
     *  DotProduct: '<S346>/Dot Product'
     *  DotProduct: '<S347>/Dot Product'
     *  DotProduct: '<S348>/Dot Product'
     *  Gain: '<S335>/Gain1'
     *  Math: '<S335>/Exp'
     *  Math: '<S335>/Reciprocal'
     *  Sum: '<S335>/Sum'
     *  Sum: '<S335>/Sum1'
     *
     * About '<S335>/Exp':
     *  Operator: exp
     *
     * About '<S335>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_DotProduct_i = 1.0 / (exp(tmp_5[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += rtb_DotProduct_i * rtConstP.IW211_Value_kh[i];

    /* DotProduct: '<S345>/Dot Product' incorporates:
     *  Constant: '<S342>/IW{2,1}(2,:)''
     */
    tmp_1 += rtb_DotProduct_i * rtConstP.IW212_Value_dw[i];

    /* DotProduct: '<S346>/Dot Product' incorporates:
     *  Constant: '<S342>/IW{2,1}(3,:)''
     */
    tmp_2 += rtb_DotProduct_i * rtConstP.IW213_Value_k[i];

    /* DotProduct: '<S347>/Dot Product' incorporates:
     *  Constant: '<S342>/IW{2,1}(4,:)''
     */
    tmp_3 += rtb_DotProduct_i * rtConstP.IW214_Value_e[i];

    /* DotProduct: '<S348>/Dot Product' incorporates:
     *  Constant: '<S342>/IW{2,1}(5,:)''
     */
    tmp_4 += rtb_DotProduct_i * rtConstP.IW215_Value_e[i];
  }

  /* Sum: '<S327>/netsum' incorporates:
   *  Constant: '<S327>/b{2}'
   *  Gain: '<S343>/Gain'
   */
  tmp_5[0] = (tmp_0 + 1.6567247179481805) * -2.0;
  tmp_5[1] = (tmp_1 - 0.76336268603368906) * -2.0;
  tmp_5[2] = (tmp_2 - 0.33987808135442693) * -2.0;
  tmp_5[3] = (tmp_3 - 0.99121916053871073) * -2.0;
  tmp_5[4] = (tmp_4 + 1.5314004473196035) * -2.0;

  /* DotProduct: '<S352>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S353>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S354>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S355>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S356>/Dot Product' */
  tmp_4 = 0.0;
  for (i = 0; i < 5; i++) {
    /* DotProduct: '<S352>/Dot Product' incorporates:
     *  Constant: '<S343>/one'
     *  Constant: '<S343>/one1'
     *  Constant: '<S350>/IW{3,2}(1,:)''
     *  DotProduct: '<S353>/Dot Product'
     *  DotProduct: '<S354>/Dot Product'
     *  DotProduct: '<S355>/Dot Product'
     *  DotProduct: '<S356>/Dot Product'
     *  Gain: '<S343>/Gain1'
     *  Math: '<S343>/Exp'
     *  Math: '<S343>/Reciprocal'
     *  Sum: '<S343>/Sum'
     *  Sum: '<S343>/Sum1'
     *
     * About '<S343>/Exp':
     *  Operator: exp
     *
     * About '<S343>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_DotProduct_i = 1.0 / (exp(tmp_5[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += rtb_DotProduct_i * rtConstP.IW321_Value_mv[i];

    /* DotProduct: '<S353>/Dot Product' incorporates:
     *  Constant: '<S350>/IW{3,2}(2,:)''
     */
    tmp_1 += rtb_DotProduct_i * rtConstP.IW322_Value_ks[i];

    /* DotProduct: '<S354>/Dot Product' incorporates:
     *  Constant: '<S350>/IW{3,2}(3,:)''
     */
    tmp_2 += rtb_DotProduct_i * rtConstP.IW323_Value_b[i];

    /* DotProduct: '<S355>/Dot Product' incorporates:
     *  Constant: '<S350>/IW{3,2}(4,:)''
     */
    tmp_3 += rtb_DotProduct_i * rtConstP.IW324_Value_h[i];

    /* DotProduct: '<S356>/Dot Product' incorporates:
     *  Constant: '<S350>/IW{3,2}(5,:)''
     */
    tmp_4 += rtb_DotProduct_i * rtConstP.IW325_Value_p[i];
  }

  /* Sum: '<S328>/netsum' incorporates:
   *  Constant: '<S328>/b{3}'
   *  Gain: '<S351>/Gain'
   */
  tmp_5[0] = (tmp_0 + 1.8536507051960618) * -2.0;
  tmp_5[1] = (tmp_1 + 0.82182157037305448) * -2.0;
  tmp_5[2] = (tmp_2 + 0.18710174317255837) * -2.0;
  tmp_5[3] = (tmp_3 + 0.979100222468924) * -2.0;
  tmp_5[4] = (tmp_4 - 1.9498590115814665) * -2.0;

  /* DotProduct: '<S360>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S361>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S362>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S363>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S364>/Dot Product' */
  tmp_4 = 0.0;
  for (i = 0; i < 5; i++) {
    /* DotProduct: '<S360>/Dot Product' incorporates:
     *  Constant: '<S351>/one'
     *  Constant: '<S351>/one1'
     *  Constant: '<S358>/IW{4,3}(1,:)''
     *  DotProduct: '<S361>/Dot Product'
     *  DotProduct: '<S362>/Dot Product'
     *  DotProduct: '<S363>/Dot Product'
     *  DotProduct: '<S364>/Dot Product'
     *  Gain: '<S351>/Gain1'
     *  Math: '<S351>/Exp'
     *  Math: '<S351>/Reciprocal'
     *  Sum: '<S351>/Sum'
     *  Sum: '<S351>/Sum1'
     *
     * About '<S351>/Exp':
     *  Operator: exp
     *
     * About '<S351>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_DotProduct_i = 1.0 / (exp(tmp_5[i]) + 1.0) * 2.0 - 1.0;
    tmp_0 += rtb_DotProduct_i * rtConstP.IW431_Value_o[i];

    /* DotProduct: '<S361>/Dot Product' incorporates:
     *  Constant: '<S358>/IW{4,3}(2,:)''
     */
    tmp_1 += rtb_DotProduct_i * rtConstP.IW432_Value[i];

    /* DotProduct: '<S362>/Dot Product' incorporates:
     *  Constant: '<S358>/IW{4,3}(3,:)''
     */
    tmp_2 += rtb_DotProduct_i * rtConstP.IW433_Value[i];

    /* DotProduct: '<S363>/Dot Product' incorporates:
     *  Constant: '<S358>/IW{4,3}(4,:)''
     */
    tmp_3 += rtb_DotProduct_i * rtConstP.IW434_Value[i];

    /* DotProduct: '<S364>/Dot Product' incorporates:
     *  Constant: '<S358>/IW{4,3}(5,:)''
     */
    tmp_4 += rtb_DotProduct_i * rtConstP.IW435_Value[i];
  }

  /* Sum: '<S329>/netsum' incorporates:
   *  Constant: '<S329>/b{4}'
   *  Gain: '<S359>/Gain'
   */
  tmp_5[0] = (tmp_0 + 1.8354684875398206) * -2.0;
  tmp_5[1] = (tmp_1 - 0.98710192935211594) * -2.0;
  tmp_5[2] = (tmp_2 - 0.18700491095656485) * -2.0;
  tmp_5[3] = (tmp_3 + 0.88038912985964624) * -2.0;
  tmp_5[4] = (tmp_4 - 2.2207479163367712) * -2.0;

  /* DotProduct: '<S368>/Dot Product' */
  tmp_0 = 0.0;
  for (i = 0; i < 5; i++) {
    /* DotProduct: '<S368>/Dot Product' incorporates:
     *  Constant: '<S359>/one'
     *  Constant: '<S359>/one1'
     *  Constant: '<S366>/IW{5,4}(1,:)''
     *  Gain: '<S359>/Gain1'
     *  Math: '<S359>/Exp'
     *  Math: '<S359>/Reciprocal'
     *  Sum: '<S359>/Sum'
     *  Sum: '<S359>/Sum1'
     *
     * About '<S359>/Exp':
     *  Operator: exp
     *
     * About '<S359>/Reciprocal':
     *  Operator: reciprocal
     */
    tmp_0 += (1.0 / (exp(tmp_5[i]) + 1.0) * 2.0 - 1.0) * rtConstP.IW541_Value[i];
  }

  /* MATLAB Function: '<S1>/MATLAB Function7' incorporates:
   *  Bias: '<S370>/Subtract min y'
   *  Gain: '<S370>/Divide by range y'
   *  Outport: '<Root>/now_muzhi_baidong_jiaosudu'
   *  Sum: '<S330>/netsum'
   */
  MATLABFunction1(83.370268339262353 * ((tmp_0 - 0.059313151445844564) + 1.0),
                  &rtY.now_muzhi_baidong_jiaosudu);

  /* Bias: '<S647>/Add min y' incorporates:
   *  Bias: '<S647>/Subtract min x'
   *  Gain: '<S647>/range y // range x'
   *  Inport: '<Root>/Tar_muzhi_wanqu_jiaodu_2'
   */
  rtb_DotProduct_i = (rtU.Tar_muzhi_wanqu_jiaodu_2 - 83.224093107371417) *
    0.036292745817769763 - 1.0;

  /* Sum: '<S634>/Sum1' incorporates:
   *  Constant: '<S627>/b{1}'
   *  Constant: '<S633>/IW{1,1}(1,:)''
   *  Constant: '<S633>/IW{1,1}(2,:)''
   *  Constant: '<S634>/one'
   *  Constant: '<S634>/one1'
   *  DotProduct: '<S635>/Dot Product'
   *  DotProduct: '<S636>/Dot Product'
   *  Gain: '<S634>/Gain'
   *  Gain: '<S634>/Gain1'
   *  Math: '<S634>/Exp'
   *  Math: '<S634>/Reciprocal'
   *  Sum: '<S627>/netsum'
   *  Sum: '<S634>/Sum'
   *
   * About '<S634>/Exp':
   *  Operator: exp
   *
   * About '<S634>/Reciprocal':
   *  Operator: reciprocal
   */
  rtb_Sum1_a_idx_0 = 1.0 / (exp((-0.42003171632599218 * rtb_DotProduct_i +
    0.20884149830522561) * -2.0) + 1.0) * 2.0 - 1.0;
  rtb_Sum1_a_idx_1 = 1.0 / (exp((1.4064861548849663 * rtb_DotProduct_i +
    4.3155475595063413) * -2.0) + 1.0) * 2.0 - 1.0;

  /* DotProduct: '<S640>/Dot Product' incorporates:
   *  Constant: '<S638>/IW{2,1}(1,:)''
   */
  tmp_0 = 1.1949331363646636 * rtb_Sum1_a_idx_0 + 1.3887464852995792 *
    rtb_Sum1_a_idx_1;

  /* Math: '<S639>/Reciprocal' incorporates:
   *  Constant: '<S628>/b{2}'
   *  Constant: '<S638>/IW{2,1}(2,:)''
   *  Constant: '<S638>/IW{2,1}(3,:)''
   *  Constant: '<S639>/one'
   *  DotProduct: '<S640>/Dot Product'
   *  DotProduct: '<S641>/Dot Product'
   *  DotProduct: '<S642>/Dot Product'
   *  Gain: '<S639>/Gain'
   *  Math: '<S639>/Exp'
   *  Sum: '<S628>/netsum'
   *  Sum: '<S639>/Sum'
   *
   * About '<S639>/Reciprocal':
   *  Operator: reciprocal
   *
   * About '<S639>/Exp':
   *  Operator: exp
   */
  y = 1.0 / (exp(((1.2314685953760245 * rtb_Sum1_a_idx_0 + 2.326664405371329 *
                   rtb_Sum1_a_idx_1) + 0.58468111200442929) * -2.0) + 1.0);
  rtb_Sum1_p_0 = exp(((-1.5726017474913363 * rtb_Sum1_a_idx_0 +
                       0.78202445375613128 * rtb_Sum1_a_idx_1) -
                      1.5407131443615525) * -2.0) + 1.0;

  /* Bias: '<S775>/Add min y' incorporates:
   *  Bias: '<S775>/Subtract min x'
   *  Gain: '<S775>/range y // range x'
   *  Inport: '<Root>/Tar_shizhi_wanqu_jiaodu_2'
   */
  rtb_DotProduct_i = (rtU.Tar_shizhi_wanqu_jiaodu_2 - 92.420101710378077) *
    0.024848851334981964 - 1.0;

  /* Sum: '<S762>/Sum1' incorporates:
   *  Constant: '<S755>/b{1}'
   *  Constant: '<S761>/IW{1,1}(1,:)''
   *  Constant: '<S761>/IW{1,1}(2,:)''
   *  Constant: '<S762>/one'
   *  Constant: '<S762>/one1'
   *  DotProduct: '<S763>/Dot Product'
   *  DotProduct: '<S764>/Dot Product'
   *  Gain: '<S762>/Gain'
   *  Gain: '<S762>/Gain1'
   *  Math: '<S762>/Exp'
   *  Math: '<S762>/Reciprocal'
   *  Sum: '<S755>/netsum'
   *  Sum: '<S762>/Sum'
   *
   * About '<S762>/Exp':
   *  Operator: exp
   *
   * About '<S762>/Reciprocal':
   *  Operator: reciprocal
   */
  rtb_Sum1_a_idx_0 = 1.0 / (exp((-0.40679407706252757 * rtb_DotProduct_i -
    0.255020423164783) * -2.0) + 1.0) * 2.0 - 1.0;
  rtb_Sum1_a_idx_1 = 1.0 / (exp((1.4020030186972001 * rtb_DotProduct_i +
    4.2457481269126429) * -2.0) + 1.0) * 2.0 - 1.0;

  /* DotProduct: '<S768>/Dot Product' incorporates:
   *  Constant: '<S766>/IW{2,1}(1,:)''
   */
  tmp_1 = 1.1138385669076818 * rtb_Sum1_a_idx_0 + 1.6365565034808855 *
    rtb_Sum1_a_idx_1;

  /* Math: '<S767>/Reciprocal' incorporates:
   *  Constant: '<S756>/b{2}'
   *  Constant: '<S766>/IW{2,1}(2,:)''
   *  Constant: '<S766>/IW{2,1}(3,:)''
   *  Constant: '<S767>/one'
   *  DotProduct: '<S768>/Dot Product'
   *  DotProduct: '<S769>/Dot Product'
   *  DotProduct: '<S770>/Dot Product'
   *  Gain: '<S767>/Gain'
   *  Math: '<S767>/Exp'
   *  Sum: '<S756>/netsum'
   *  Sum: '<S767>/Sum'
   *
   * About '<S767>/Reciprocal':
   *  Operator: reciprocal
   *
   * About '<S767>/Exp':
   *  Operator: exp
   */
  y_0 = 1.0 / (exp(((1.2481833684937178 * rtb_Sum1_a_idx_0 + 2.2528758327074248 *
                     rtb_Sum1_a_idx_1) + 0.52785840610568768) * -2.0) + 1.0);
  u = exp(((-1.5332366304465124 * rtb_Sum1_a_idx_0 + 0.66227122156968432 *
            rtb_Sum1_a_idx_1) - 1.6595440422476724) * -2.0) + 1.0;

  /* Bias: '<S565>/Add min y' incorporates:
   *  Bias: '<S565>/Subtract min x'
   *  Gain: '<S565>/range y // range x'
   *  Inport: '<Root>/Tar_zhongzhi_wanqu_jiaodu_2'
   */
  rtb_DotProduct_i = (rtU.Tar_zhongzhi_wanqu_jiaodu_2 - 92.420101710378077) *
    0.024848851334981964 - 1.0;

  /* Sum: '<S552>/Sum1' incorporates:
   *  Constant: '<S545>/b{1}'
   *  Constant: '<S551>/IW{1,1}(1,:)''
   *  Constant: '<S551>/IW{1,1}(2,:)''
   *  Constant: '<S552>/one'
   *  Constant: '<S552>/one1'
   *  DotProduct: '<S553>/Dot Product'
   *  DotProduct: '<S554>/Dot Product'
   *  Gain: '<S552>/Gain'
   *  Gain: '<S552>/Gain1'
   *  Math: '<S552>/Exp'
   *  Math: '<S552>/Reciprocal'
   *  Sum: '<S545>/netsum'
   *  Sum: '<S552>/Sum'
   *
   * About '<S552>/Exp':
   *  Operator: exp
   *
   * About '<S552>/Reciprocal':
   *  Operator: reciprocal
   */
  rtb_Sum1_a_idx_0 = 1.0 / (exp((-0.40679407706252757 * rtb_DotProduct_i -
    0.255020423164783) * -2.0) + 1.0) * 2.0 - 1.0;
  rtb_Sum1_a_idx_1 = 1.0 / (exp((1.4020030186972001 * rtb_DotProduct_i +
    4.2457481269126429) * -2.0) + 1.0) * 2.0 - 1.0;

  /* DotProduct: '<S558>/Dot Product' incorporates:
   *  Constant: '<S556>/IW{2,1}(1,:)''
   */
  tmp_2 = 1.1138385669076818 * rtb_Sum1_a_idx_0 + 1.6365565034808855 *
    rtb_Sum1_a_idx_1;

  /* Math: '<S557>/Reciprocal' incorporates:
   *  Constant: '<S546>/b{2}'
   *  Constant: '<S556>/IW{2,1}(2,:)''
   *  Constant: '<S556>/IW{2,1}(3,:)''
   *  Constant: '<S557>/one'
   *  DotProduct: '<S558>/Dot Product'
   *  DotProduct: '<S559>/Dot Product'
   *  DotProduct: '<S560>/Dot Product'
   *  Gain: '<S557>/Gain'
   *  Math: '<S557>/Exp'
   *  Sum: '<S546>/netsum'
   *  Sum: '<S557>/Sum'
   *
   * About '<S557>/Reciprocal':
   *  Operator: reciprocal
   *
   * About '<S557>/Exp':
   *  Operator: exp
   */
  y_1 = 1.0 / (exp(((1.2481833684937178 * rtb_Sum1_a_idx_0 + 2.2528758327074248 *
                     rtb_Sum1_a_idx_1) + 0.52785840610568768) * -2.0) + 1.0);
  u_0 = exp(((-1.5332366304465124 * rtb_Sum1_a_idx_0 + 0.66227122156968432 *
              rtb_Sum1_a_idx_1) - 1.6595440422476724) * -2.0) + 1.0;

  /* Bias: '<S734>/Add min y' incorporates:
   *  Bias: '<S734>/Subtract min x'
   *  Gain: '<S734>/range y // range x'
   *  Inport: '<Root>/Tar_wumingzhi_wanqu_jiaodu_2'
   */
  rtb_DotProduct_i = (rtU.Tar_wumingzhi_wanqu_jiaodu_2 - 92.420101710378077) *
    0.024848851334981964 - 1.0;

  /* Sum: '<S721>/Sum1' incorporates:
   *  Constant: '<S714>/b{1}'
   *  Constant: '<S720>/IW{1,1}(1,:)''
   *  Constant: '<S720>/IW{1,1}(2,:)''
   *  Constant: '<S721>/one'
   *  Constant: '<S721>/one1'
   *  DotProduct: '<S722>/Dot Product'
   *  DotProduct: '<S723>/Dot Product'
   *  Gain: '<S721>/Gain'
   *  Gain: '<S721>/Gain1'
   *  Math: '<S721>/Exp'
   *  Math: '<S721>/Reciprocal'
   *  Sum: '<S714>/netsum'
   *  Sum: '<S721>/Sum'
   *
   * About '<S721>/Exp':
   *  Operator: exp
   *
   * About '<S721>/Reciprocal':
   *  Operator: reciprocal
   */
  rtb_Sum1_a_idx_0 = 1.0 / (exp((-0.40679407706252757 * rtb_DotProduct_i -
    0.255020423164783) * -2.0) + 1.0) * 2.0 - 1.0;
  rtb_Sum1_a_idx_1 = 1.0 / (exp((1.4020030186972001 * rtb_DotProduct_i +
    4.2457481269126429) * -2.0) + 1.0) * 2.0 - 1.0;

  /* DotProduct: '<S727>/Dot Product' incorporates:
   *  Constant: '<S725>/IW{2,1}(1,:)''
   */
  tmp_3 = 1.1138385669076818 * rtb_Sum1_a_idx_0 + 1.6365565034808855 *
    rtb_Sum1_a_idx_1;

  /* Math: '<S726>/Reciprocal' incorporates:
   *  Constant: '<S715>/b{2}'
   *  Constant: '<S725>/IW{2,1}(2,:)''
   *  Constant: '<S725>/IW{2,1}(3,:)''
   *  Constant: '<S726>/one'
   *  DotProduct: '<S727>/Dot Product'
   *  DotProduct: '<S728>/Dot Product'
   *  DotProduct: '<S729>/Dot Product'
   *  Gain: '<S726>/Gain'
   *  Math: '<S726>/Exp'
   *  Sum: '<S715>/netsum'
   *  Sum: '<S726>/Sum'
   *
   * About '<S726>/Reciprocal':
   *  Operator: reciprocal
   *
   * About '<S726>/Exp':
   *  Operator: exp
   */
  y_2 = 1.0 / (exp(((1.2481833684937178 * rtb_Sum1_a_idx_0 + 2.2528758327074248 *
                     rtb_Sum1_a_idx_1) + 0.52785840610568768) * -2.0) + 1.0);
  u_1 = exp(((-1.5332366304465124 * rtb_Sum1_a_idx_0 + 0.66227122156968432 *
              rtb_Sum1_a_idx_1) - 1.6595440422476724) * -2.0) + 1.0;

  /* Bias: '<S606>/Add min y' incorporates:
   *  Bias: '<S606>/Subtract min x'
   *  Gain: '<S606>/range y // range x'
   *  Inport: '<Root>/Tar_xiaomuzhi_wanqu_jiaodu_2'
   */
  rtb_DotProduct_i = (rtU.Tar_xiaomuzhi_wanqu_jiaodu_2 - 92.420101710378077) *
    0.024848851334981964 - 1.0;

  /* Sum: '<S593>/Sum1' incorporates:
   *  Constant: '<S586>/b{1}'
   *  Constant: '<S592>/IW{1,1}(1,:)''
   *  Constant: '<S592>/IW{1,1}(2,:)''
   *  Constant: '<S593>/one'
   *  Constant: '<S593>/one1'
   *  DotProduct: '<S594>/Dot Product'
   *  DotProduct: '<S595>/Dot Product'
   *  Gain: '<S593>/Gain'
   *  Gain: '<S593>/Gain1'
   *  Math: '<S593>/Exp'
   *  Math: '<S593>/Reciprocal'
   *  Sum: '<S586>/netsum'
   *  Sum: '<S593>/Sum'
   *
   * About '<S593>/Exp':
   *  Operator: exp
   *
   * About '<S593>/Reciprocal':
   *  Operator: reciprocal
   */
  rtb_Sum1_a_idx_0 = 1.0 / (exp((-0.40679407706252757 * rtb_DotProduct_i -
    0.255020423164783) * -2.0) + 1.0) * 2.0 - 1.0;
  rtb_Sum1_a_idx_1 = 1.0 / (exp((1.4020030186972001 * rtb_DotProduct_i +
    4.2457481269126429) * -2.0) + 1.0) * 2.0 - 1.0;

  /* DotProduct: '<S599>/Dot Product' incorporates:
   *  Constant: '<S597>/IW{2,1}(1,:)''
   */
  tmp_4 = 1.1138385669076818 * rtb_Sum1_a_idx_0 + 1.6365565034808855 *
    rtb_Sum1_a_idx_1;

  /* Math: '<S598>/Reciprocal' incorporates:
   *  Constant: '<S587>/b{2}'
   *  Constant: '<S597>/IW{2,1}(2,:)''
   *  Constant: '<S597>/IW{2,1}(3,:)''
   *  Constant: '<S598>/one'
   *  DotProduct: '<S599>/Dot Product'
   *  DotProduct: '<S600>/Dot Product'
   *  DotProduct: '<S601>/Dot Product'
   *  Gain: '<S598>/Gain'
   *  Math: '<S598>/Exp'
   *  Sum: '<S587>/netsum'
   *  Sum: '<S598>/Sum'
   *
   * About '<S598>/Reciprocal':
   *  Operator: reciprocal
   *
   * About '<S598>/Exp':
   *  Operator: exp
   */
  y_3 = 1.0 / (exp(((1.2481833684937178 * rtb_Sum1_a_idx_0 + 2.2528758327074248 *
                     rtb_Sum1_a_idx_1) + 0.52785840610568768) * -2.0) + 1.0);
  u_2 = exp(((-1.5332366304465124 * rtb_Sum1_a_idx_0 + 0.66227122156968432 *
              rtb_Sum1_a_idx_1) - 1.6595440422476724) * -2.0) + 1.0;

  /* Bias: '<S674>/Add min y' incorporates:
   *  Bias: '<S674>/Subtract min x'
   *  Gain: '<S674>/range y // range x'
   *  Inport: '<Root>/Tar_muzhi_baidong_jiaodu_2'
   */
  rtb_DotProduct_i = (rtU.Tar_muzhi_baidong_jiaodu_2 - 6.46095314) *
    0.022506419504021224 - 1.0;

  /* Sum: '<S657>/Sum1' incorporates:
   *  Constant: '<S649>/b{1}'
   *  Constant: '<S656>/IW{1,1}(1,:)''
   *  Constant: '<S656>/IW{1,1}(2,:)''
   *  Constant: '<S657>/one'
   *  Constant: '<S657>/one1'
   *  DotProduct: '<S658>/Dot Product'
   *  DotProduct: '<S659>/Dot Product'
   *  Gain: '<S657>/Gain'
   *  Gain: '<S657>/Gain1'
   *  Math: '<S657>/Exp'
   *  Math: '<S657>/Reciprocal'
   *  Sum: '<S649>/netsum'
   *  Sum: '<S657>/Sum'
   *
   * About '<S657>/Exp':
   *  Operator: exp
   *
   * About '<S657>/Reciprocal':
   *  Operator: reciprocal
   */
  rtb_Sum1_a_idx_0 = 1.0 / (exp((0.80766694146296725 * rtb_DotProduct_i -
    1.5255666936706596) * -2.0) + 1.0) * 2.0 - 1.0;
  rtb_Sum1_a_idx_1 = 1.0 / (exp((1.2964695953340646 * rtb_DotProduct_i +
    0.481759059748135) * -2.0) + 1.0) * 2.0 - 1.0;

  /* DotProduct: '<S664>/Dot Product' incorporates:
   *  Constant: '<S661>/IW{2,1}(2,:)''
   *  DotProduct: '<S663>/Dot Product'
   */
  rtb_DotProduct_i = 2.0053906128660768 * rtb_Sum1_a_idx_0 + 0.72191025726872127
    * rtb_Sum1_a_idx_1;

  /* Sum: '<S662>/Sum1' incorporates:
   *  Constant: '<S650>/b{2}'
   *  Constant: '<S661>/IW{2,1}(1,:)''
   *  Constant: '<S662>/one'
   *  Constant: '<S662>/one1'
   *  DotProduct: '<S663>/Dot Product'
   *  Gain: '<S662>/Gain'
   *  Gain: '<S662>/Gain1'
   *  Math: '<S662>/Exp'
   *  Math: '<S662>/Reciprocal'
   *  Sum: '<S650>/netsum'
   *  Sum: '<S662>/Sum'
   *
   * About '<S662>/Exp':
   *  Operator: exp
   *
   * About '<S662>/Reciprocal':
   *  Operator: reciprocal
   */
  rtb_Sum1_a_idx_0 = 1.0 / (exp(((-0.95139723922618435 * rtb_Sum1_a_idx_0 +
    1.9072340125816372 * rtb_Sum1_a_idx_1) + 1.8277375216264486) * -2.0) + 1.0) *
    2.0 - 1.0;
  rtb_Sum1_a_idx_1 = 1.0 / (exp((rtb_DotProduct_i + 1.8694648284576796) * -2.0)
    + 1.0) * 2.0 - 1.0;

  /* Outport: '<Root>/Tar_M1_pos' incorporates:
   *  Bias: '<S648>/Subtract min y'
   *  Constant: '<S628>/b{2}'
   *  Constant: '<S629>/b{3}'
   *  Constant: '<S639>/one'
   *  Constant: '<S639>/one1'
   *  Constant: '<S644>/IW{3,2}(1,:)''
   *  DotProduct: '<S646>/Dot Product'
   *  Gain: '<S639>/Gain'
   *  Gain: '<S639>/Gain1'
   *  Gain: '<S648>/Divide by range y'
   *  Math: '<S639>/Exp'
   *  Math: '<S639>/Reciprocal'
   *  Sum: '<S628>/netsum'
   *  Sum: '<S629>/netsum'
   *  Sum: '<S639>/Sum'
   *  Sum: '<S639>/Sum1'
   *
   * About '<S639>/Exp':
   *  Operator: exp
   *
   * About '<S639>/Reciprocal':
   *  Operator: reciprocal
   */
  rtY.Tar_M1_pos = (((((1.0 / (exp((tmp_0 - 2.0285961960247354) * -2.0) + 1.0) *
                        2.0 - 1.0) * 1.882966712983938 + (2.0 * y - 1.0) *
                       -0.57200332290903122) + (1.0 / rtb_Sum1_p_0 * 2.0 - 1.0) *
                      -1.2145352604925903) + 0.41115749113220973) + 1.0) *
    81.7915070843829;

  /* Outport: '<Root>/Tar_M2_pos' incorporates:
   *  Bias: '<S675>/Subtract min y'
   *  Constant: '<S651>/b{3}'
   *  Constant: '<S652>/b{4}'
   *  Constant: '<S666>/IW{3,2}(1,:)''
   *  Constant: '<S666>/IW{3,2}(2,:)''
   *  Constant: '<S667>/one'
   *  Constant: '<S667>/one1'
   *  Constant: '<S671>/IW{4,3}(1,:)''
   *  DotProduct: '<S668>/Dot Product'
   *  DotProduct: '<S669>/Dot Product'
   *  DotProduct: '<S673>/Dot Product'
   *  Gain: '<S667>/Gain'
   *  Gain: '<S667>/Gain1'
   *  Gain: '<S675>/Divide by range y'
   *  Math: '<S667>/Exp'
   *  Math: '<S667>/Reciprocal'
   *  Sum: '<S651>/netsum'
   *  Sum: '<S652>/netsum'
   *  Sum: '<S667>/Sum'
   *  Sum: '<S667>/Sum1'
   *
   * About '<S667>/Exp':
   *  Operator: exp
   *
   * About '<S667>/Reciprocal':
   *  Operator: reciprocal
   */
  rtY.Tar_M2_pos = ((((1.0 / (exp(((2.0857045653533164 * rtb_Sum1_a_idx_0 +
    0.65905415975412063 * rtb_Sum1_a_idx_1) - 1.8720571598758049) * -2.0) + 1.0)
                       * 2.0 - 1.0) * 1.2925651884013727 + (1.0 / (exp
    (((-0.72744816130766676 * rtb_Sum1_a_idx_0 + 2.1448397479700985 *
       rtb_Sum1_a_idx_1) - 1.3854557881285987) * -2.0) + 1.0) * 2.0 - 1.0) *
                      0.97917707537802856) + 0.397664497770017) + 1.0) *
    95.676900585903127;

  /* Outport: '<Root>/Tar_M3_pos' incorporates:
   *  Bias: '<S776>/Subtract min y'
   *  Constant: '<S756>/b{2}'
   *  Constant: '<S757>/b{3}'
   *  Constant: '<S767>/one'
   *  Constant: '<S767>/one1'
   *  Constant: '<S772>/IW{3,2}(1,:)''
   *  DotProduct: '<S774>/Dot Product'
   *  Gain: '<S767>/Gain'
   *  Gain: '<S767>/Gain1'
   *  Gain: '<S776>/Divide by range y'
   *  Math: '<S767>/Exp'
   *  Math: '<S767>/Reciprocal'
   *  Sum: '<S756>/netsum'
   *  Sum: '<S757>/netsum'
   *  Sum: '<S767>/Sum'
   *  Sum: '<S767>/Sum1'
   *
   * About '<S767>/Exp':
   *  Operator: exp
   *
   * About '<S767>/Reciprocal':
   *  Operator: reciprocal
   */
  rtY.Tar_M3_pos = (((((1.0 / (exp((tmp_1 - 1.8723440515697127) * -2.0) + 1.0) *
                        2.0 - 1.0) * 1.9758848462153804 + (2.0 * y_0 - 1.0) *
                       -0.34498823131310025) + (1.0 / u * 2.0 - 1.0) *
                      -1.0446994168509822) + 0.61780986147611661) + 1.0) *
    87.375126946527828;

  /* Outport: '<Root>/Tar_M4_pos' incorporates:
   *  Bias: '<S566>/Subtract min y'
   *  Constant: '<S546>/b{2}'
   *  Constant: '<S547>/b{3}'
   *  Constant: '<S557>/one'
   *  Constant: '<S557>/one1'
   *  Constant: '<S562>/IW{3,2}(1,:)''
   *  DotProduct: '<S564>/Dot Product'
   *  Gain: '<S557>/Gain'
   *  Gain: '<S557>/Gain1'
   *  Gain: '<S566>/Divide by range y'
   *  Math: '<S557>/Exp'
   *  Math: '<S557>/Reciprocal'
   *  Sum: '<S546>/netsum'
   *  Sum: '<S547>/netsum'
   *  Sum: '<S557>/Sum'
   *  Sum: '<S557>/Sum1'
   *
   * About '<S557>/Exp':
   *  Operator: exp
   *
   * About '<S557>/Reciprocal':
   *  Operator: reciprocal
   */
  rtY.Tar_M4_pos = (((((1.0 / (exp((tmp_2 - 1.8723440515697127) * -2.0) + 1.0) *
                        2.0 - 1.0) * 1.9758848462153804 + (2.0 * y_1 - 1.0) *
                       -0.34498823131310025) + (1.0 / u_0 * 2.0 - 1.0) *
                      -1.0446994168509822) + 0.61780986147611661) + 1.0) *
    87.375126946527828;

  /* Outport: '<Root>/Tar_M5_pos' incorporates:
   *  Bias: '<S735>/Subtract min y'
   *  Constant: '<S715>/b{2}'
   *  Constant: '<S716>/b{3}'
   *  Constant: '<S726>/one'
   *  Constant: '<S726>/one1'
   *  Constant: '<S731>/IW{3,2}(1,:)''
   *  DotProduct: '<S733>/Dot Product'
   *  Gain: '<S726>/Gain'
   *  Gain: '<S726>/Gain1'
   *  Gain: '<S735>/Divide by range y'
   *  Math: '<S726>/Exp'
   *  Math: '<S726>/Reciprocal'
   *  Sum: '<S715>/netsum'
   *  Sum: '<S716>/netsum'
   *  Sum: '<S726>/Sum'
   *  Sum: '<S726>/Sum1'
   *
   * About '<S726>/Exp':
   *  Operator: exp
   *
   * About '<S726>/Reciprocal':
   *  Operator: reciprocal
   */
  rtY.Tar_M5_pos = (((((1.0 / (exp((tmp_3 - 1.8723440515697127) * -2.0) + 1.0) *
                        2.0 - 1.0) * 1.9758848462153804 + (2.0 * y_2 - 1.0) *
                       -0.34498823131310025) + (1.0 / u_1 * 2.0 - 1.0) *
                      -1.0446994168509822) + 0.61780986147611661) + 1.0) *
    87.375126946527828;

  /* Outport: '<Root>/Tar_M6_pos' incorporates:
   *  Bias: '<S607>/Subtract min y'
   *  Constant: '<S587>/b{2}'
   *  Constant: '<S588>/b{3}'
   *  Constant: '<S598>/one'
   *  Constant: '<S598>/one1'
   *  Constant: '<S603>/IW{3,2}(1,:)''
   *  DotProduct: '<S605>/Dot Product'
   *  Gain: '<S598>/Gain'
   *  Gain: '<S598>/Gain1'
   *  Gain: '<S607>/Divide by range y'
   *  Math: '<S598>/Exp'
   *  Math: '<S598>/Reciprocal'
   *  Sum: '<S587>/netsum'
   *  Sum: '<S588>/netsum'
   *  Sum: '<S598>/Sum'
   *  Sum: '<S598>/Sum1'
   *
   * About '<S598>/Exp':
   *  Operator: exp
   *
   * About '<S598>/Reciprocal':
   *  Operator: reciprocal
   */
  rtY.Tar_M6_pos = (((((1.0 / (exp((tmp_4 - 1.8723440515697127) * -2.0) + 1.0) *
                        2.0 - 1.0) * 1.9758848462153804 + (2.0 * y_3 - 1.0) *
                       -0.34498823131310025) + (1.0 / u_2 * 2.0 - 1.0) *
                      -1.0446994168509822) + 0.61780986147611661) + 1.0) *
    87.375126946527828;

  /* DotProduct: '<S692>/Dot Product' */
  tmp_0 = 0.0;

  /* DotProduct: '<S711>/Dot Product' */
  tmp_1 = 0.0;

  /* DotProduct: '<S793>/Dot Product' */
  tmp_2 = 0.0;

  /* DotProduct: '<S583>/Dot Product' */
  tmp_3 = 0.0;

  /* DotProduct: '<S752>/Dot Product' */
  tmp_4 = 0.0;

  /* DotProduct: '<S624>/Dot Product' */
  rtb_DotProduct_i = 0.0;
  for (i = 0; i < 6; i++) {
    /* DotProduct: '<S692>/Dot Product' incorporates:
     *  Constant: '<S682>/one'
     *  Constant: '<S682>/one1'
     *  Constant: '<S690>/IW{2,1}(1,:)''
     *  Gain: '<S682>/Gain1'
     *  Math: '<S682>/Reciprocal'
     *  Sum: '<S682>/Sum'
     *  Sum: '<S682>/Sum1'
     *
     * About '<S682>/Reciprocal':
     *  Operator: reciprocal
     */
    tmp_0 += (1.0 / (rtDW.Exp[i] + 1.0) * 2.0 - 1.0) * rtConstP.IW211_Value_kl[i];

    /* DotProduct: '<S711>/Dot Product' incorporates:
     *  Constant: '<S701>/one'
     *  Constant: '<S701>/one1'
     *  Constant: '<S709>/IW{2,1}(1,:)''
     *  Gain: '<S701>/Gain1'
     *  Math: '<S701>/Reciprocal'
     *  Sum: '<S701>/Sum'
     *  Sum: '<S701>/Sum1'
     *
     * About '<S701>/Reciprocal':
     *  Operator: reciprocal
     */
    tmp_1 += (1.0 / (rtb_Exp_gn[i] + 1.0) * 2.0 - 1.0) *
      rtConstP.IW211_Value_n[i];

    /* DotProduct: '<S793>/Dot Product' incorporates:
     *  Constant: '<S783>/one'
     *  Constant: '<S783>/one1'
     *  Constant: '<S791>/IW{2,1}(1,:)''
     *  Gain: '<S783>/Gain1'
     *  Math: '<S783>/Reciprocal'
     *  Sum: '<S783>/Sum'
     *  Sum: '<S783>/Sum1'
     *
     * About '<S783>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_Sum1_a_idx_0 = rtConstP.pooled43[i];
    tmp_2 += (1.0 / (rtb_Exp_o2[i] + 1.0) * 2.0 - 1.0) * rtb_Sum1_a_idx_0;

    /* DotProduct: '<S583>/Dot Product' incorporates:
     *  Constant: '<S573>/one'
     *  Constant: '<S573>/one1'
     *  Constant: '<S581>/IW{2,1}(1,:)''
     *  Gain: '<S573>/Gain1'
     *  Math: '<S573>/Reciprocal'
     *  Sum: '<S573>/Sum'
     *  Sum: '<S573>/Sum1'
     *
     * About '<S573>/Reciprocal':
     *  Operator: reciprocal
     */
    tmp_3 += (1.0 / (rtb_Exp_fn[i] + 1.0) * 2.0 - 1.0) * rtb_Sum1_a_idx_0;

    /* DotProduct: '<S752>/Dot Product' incorporates:
     *  Constant: '<S742>/one'
     *  Constant: '<S742>/one1'
     *  Constant: '<S750>/IW{2,1}(1,:)''
     *  Gain: '<S742>/Gain1'
     *  Math: '<S742>/Reciprocal'
     *  Sum: '<S742>/Sum'
     *  Sum: '<S742>/Sum1'
     *
     * About '<S742>/Reciprocal':
     *  Operator: reciprocal
     */
    tmp_4 += (1.0 / (rtb_Exp_af[i] + 1.0) * 2.0 - 1.0) * rtb_Sum1_a_idx_0;

    /* DotProduct: '<S624>/Dot Product' incorporates:
     *  Constant: '<S614>/one'
     *  Constant: '<S614>/one1'
     *  Constant: '<S622>/IW{2,1}(1,:)''
     *  Gain: '<S614>/Gain1'
     *  Math: '<S614>/Reciprocal'
     *  Sum: '<S614>/Sum'
     *  Sum: '<S614>/Sum1'
     *
     * About '<S614>/Reciprocal':
     *  Operator: reciprocal
     */
    rtb_DotProduct_i += (1.0 / (rtb_Exp_ay[i] + 1.0) * 2.0 - 1.0) *
      rtb_Sum1_a_idx_0;
  }

  /* Outport: '<Root>/now_muzhi_wanqu_jiaodu_2' incorporates:
   *  Bias: '<S694>/Add min x'
   *  Bias: '<S694>/Subtract min y'
   *  Gain: '<S694>/Divide by range y'
   *  Sum: '<S677>/netsum'
   */
  rtY.now_muzhi_wanqu_jiaodu_2 = ((tmp_0 - 0.032661555117449718) + 1.0) *
    27.553715693519585 + 83.224093107371417;

  /* Outport: '<Root>/now_muzhi_baidong_jiaodu_2' incorporates:
   *  Bias: '<S713>/Add min x'
   *  Bias: '<S713>/Subtract min y'
   *  Constant: '<S696>/b{2}'
   *  Gain: '<S713>/Divide by range y'
   *  Sum: '<S696>/netsum'
   */
  rtY.now_muzhi_baidong_jiaodu_2 = ((tmp_1 + 0.0039554745784309365) + 1.0) *
    44.43176755953251 + 6.46095314;

  /* Outport: '<Root>/now_shizhi_wanqu_jiaodu_2' incorporates:
   *  Bias: '<S795>/Add min x'
   *  Bias: '<S795>/Subtract min y'
   *  Constant: '<S778>/b{2}'
   *  Gain: '<S795>/Divide by range y'
   *  Sum: '<S778>/netsum'
   */
  rtY.now_shizhi_wanqu_jiaodu_2 = ((tmp_2 + 0.030726560714447439) + 1.0) *
    40.243308896625337 + 92.420101710378077;

  /* Outport: '<Root>/now_zhongzhi_wanqu_jiaodu_2' incorporates:
   *  Bias: '<S585>/Add min x'
   *  Bias: '<S585>/Subtract min y'
   *  Constant: '<S568>/b{2}'
   *  Gain: '<S585>/Divide by range y'
   *  Sum: '<S568>/netsum'
   */
  rtY.now_zhongzhi_wanqu_jiaodu_2 = ((tmp_3 + 0.030726560714447439) + 1.0) *
    40.243308896625337 + 92.420101710378077;

  /* Outport: '<Root>/now_wumingzhi_wanqu_jiaodu_2' incorporates:
   *  Bias: '<S754>/Add min x'
   *  Bias: '<S754>/Subtract min y'
   *  Constant: '<S737>/b{2}'
   *  Gain: '<S754>/Divide by range y'
   *  Sum: '<S737>/netsum'
   */
  rtY.now_wumingzhi_wanqu_jiaodu_2 = ((tmp_4 + 0.030726560714447439) + 1.0) *
    40.243308896625337 + 92.420101710378077;

  /* Outport: '<Root>/now_xiaomuzhi_wanqu_jiaodu_2' incorporates:
   *  Bias: '<S626>/Add min x'
   *  Bias: '<S626>/Subtract min y'
   *  Constant: '<S609>/b{2}'
   *  Gain: '<S626>/Divide by range y'
   *  Sum: '<S609>/netsum'
   */
  rtY.now_xiaomuzhi_wanqu_jiaodu_2 = ((rtb_DotProduct_i + 0.030726560714447439)
    + 1.0) * 40.243308896625337 + 92.420101710378077;

  /* End of Outputs for SubSystem: '<Root>/Kmodel' */
}

/* Model initialize function */
void Kmodel_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
