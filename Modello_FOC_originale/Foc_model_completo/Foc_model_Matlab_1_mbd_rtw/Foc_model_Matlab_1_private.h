/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Foc_model_Matlab_1_private.h
 *
 * Code generated for Simulink model 'Foc_model_Matlab_1'.
 *
 * Model version                   : 10.30
 * Simulink Coder version          : 24.2 (R2024b) 21-Jun-2024
 * MBDT for S32K1xx Series Version : 4.2.0 (R2016a-R2020a) 20-Jul-2020
 * C/C++ source code generated on  : Wed Nov 27 16:31:33 2024
 *
 * Target selection: mbd_s32k.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef Foc_model_Matlab_1_private_h_
#define Foc_model_Matlab_1_private_h_
#include "rtwtypes.h"
#include "Foc_model_Matlab_1_types.h"
#include "Foc_model_Matlab_1.h"
#include "gt_pf.h"
#include "profile_buffer.h"
#include "clock_manager.h"
#include "lpit_hw_access.h"
#include "pcc_hw_access.h"

extern real32_T rt_modf_snf(real32_T u0, real32_T u1);

#if defined(__MWERKS__)

double fmod (double x, double y);
double fabs (double);

#endif

extern void Foc_model_Ma_FaultDetection(uint32_T rtu_Vdc, uint32_T rtu_Idc);
extern void Foc_model_Matl_HallValueof1(real32_T *rty_position);
extern void Foc_model_Matl_HallValueof2(real32_T *rty_position);
extern void Foc_model_Matl_HallValueof3(real32_T *rty_position);
extern void Foc_model_Matl_HallValueof4(real32_T *rty_position);
extern void Foc_model_Matl_HallValueof5(real32_T *rty_position);
extern void Foc_model_Matl_HallValueof7(real32_T *rty_position);
extern void Foc_mod_CurrentControl_Init(void);
extern void Foc_mo_CurrentControl_Reset(void);
extern void Foc_m_CurrentControl_Enable(void);
extern void Foc_model_Ma_CurrentControl(void);
extern void Foc_model_Matl_SpeedControl(void);

#endif                                 /* Foc_model_Matlab_1_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */