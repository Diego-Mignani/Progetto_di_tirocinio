/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: mcb_pmsm_foc_hall_S32K144EVB_private.h
 *
 * Code generated for Simulink model 'mcb_pmsm_foc_hall_S32K144EVB'.
 *
 * Model version                   : 10.0
 * Simulink Coder version          : 24.2 (R2024b) 21-Jun-2024
 * MBDT for S32K1xx Series Version : 4.2.0 (R2016a-R2020a) 20-Jul-2020
 * C/C++ source code generated on  : Thu Oct 10 11:47:14 2024
 *
 * Target selection: mbd_s32k.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef mcb_pmsm_foc_hall_S32K144EVB_private_h_
#define mcb_pmsm_foc_hall_S32K144EVB_private_h_
#include "rtwtypes.h"
#include "mcb_pmsm_foc_hall_S32K144EVB_types.h"
#include "mcb_pmsm_foc_hall_S32K144EVB.h"
#include "gt_pf.h"
#include "profile_buffer.h"
#include "clock_manager.h"
#include "lpit_hw_access.h"
#include "pcc_hw_access.h"
#if defined(__MWERKS__)

double fmod (double x, double y);
double fabs (double);

#endif

extern void mcb_pmsm_foc_FaultDetection(uint32_T rtu_Vdc, uint32_T rtu_Idc);
extern void mcb_pmsm_foc_h_HallValueof1(real32_T *rty_position);
extern void mcb_pmsm_foc_h_HallValueof2(real32_T *rty_position);
extern void mcb_pmsm_foc_h_HallValueof3(real32_T *rty_position);
extern void mcb_pmsm_foc_h_HallValueof4(real32_T *rty_position);
extern void mcb_pmsm_foc_h_HallValueof5(real32_T *rty_position);
extern void mcb_pmsm_foc_h_HallValueof7(real32_T *rty_position);
extern void mcb_pms_CurrentControl_Init(void);
extern void mcb_pm_CurrentControl_Reset(void);
extern void mcb_pmsm_foc_CurrentControl(void);
extern void mcb_pmsm_foc_h_SpeedControl(void);

#endif                             /* mcb_pmsm_foc_hall_S32K144EVB_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
