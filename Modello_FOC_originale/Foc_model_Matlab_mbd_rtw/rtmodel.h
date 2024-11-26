/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: rtmodel.h
 *
 * Code generated for Simulink model 'Foc_model_Matlab'.
 *
 * Model version                   : 10.22
 * Simulink Coder version          : 24.2 (R2024b) 21-Jun-2024
 * MBDT for S32K1xx Series Version : 4.2.0 (R2016a-R2020a) 20-Jul-2020
 * C/C++ source code generated on  : Tue Nov 26 12:30:44 2024
 *
 * Target selection: mbd_s32k.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef rtmodel_h_
#define rtmodel_h_
#include "Foc_model_Matlab.h"
#define GRTINTERFACE                   0

/* Macros generated for backwards compatibility  */
#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((void*) 0)
#endif

/* Model wrapper function */
/* Use this function only if you need to maintain compatibility with an existing static main program. */
extern void Foc_model_Matlab_step(int_T tid);

#endif                                 /* rtmodel_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
