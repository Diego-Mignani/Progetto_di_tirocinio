/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: rtmodel.c
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

#include "rtmodel.h"

/* Use this function only if you need to maintain compatibility with an existing static main program. */
void mcb_pmsm_foc_hall_S32K144EVB_step(int_T tid)
{
  switch (tid) {
   case 0 :
    mcb_pmsm_foc_hall_S32K144EVB_step0();
    break;

   case 1 :
    mcb_pmsm_foc_hall_S32K144EVB_step1();
    break;

   case 2 :
    mcb_pmsm_foc_hall_S32K144EVB_step2();
    break;

   default :
    /* do nothing */
    break;
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */