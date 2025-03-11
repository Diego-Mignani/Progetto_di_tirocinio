/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Foc_model_Matlab_ultimo.c
 *
 * Code generated for Simulink model 'Foc_model_Matlab_ultimo'.
 *
 * Model version                   : 10.39
 * Simulink Coder version          : 24.2 (R2024b) 21-Jun-2024
 * MBDT for S32K1xx Series Version : 4.2.0 (R2016a-R2020a) 20-Jul-2020
 * C/C++ source code generated on  : Thu Dec 12 12:27:24 2024
 *
 * Target selection: mbd_s32k.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Foc_model_Matlab_ultimo.h"
#include "rtwtypes.h"
#include "Foc_model_Matlab_ultimo_private.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <float.h>

/* Named constants for Chart: '<S4>/Enable PDB and start FTM' */
#define Foc_model_Matlab_ultimo_IN_A   ((uint8_T)1U)
#define Foc_model_Matlab_ultimo_IN_END ((uint8_T)2U)

lpspi_state_t lpspiMasterState0;
void lpspi_master_transfer_callback0(void *driverState, spi_event_t event, void *
  userData) __attribute__((weak));

/* Exported data definition */

/* Volatile memory section */
/* Definition for custom storage class: Volatile */
volatile real32_T ADC_A;               /* '<Root>/Data Store Memory11' */
volatile real32_T ADC_B;               /* '<Root>/Data Store Memory12' */
volatile uint32_T ADC_IA;              /* '<S209>/ADC_AD4_IA' */
volatile uint32_T ADC_IB;              /* '<S209>/ADC_IB' */
volatile uint32_T ADC_IDC;             /* '<S209>/ADC_AD6_IDC' */
volatile uint32_T ADC_VDC;             /* '<S209>/ADC_AD7_VDC' */
volatile uint32_T CH0S_ERR;            /* '<S208>/PDB1_ISR' */
volatile uint32_T CH1S_ERR;            /* '<S208>/PDB1_ISR' */
volatile real32_T COS;                 /* '<S53>/Sum6' */
volatile uint16_T CntHall;             /* '<S3>/FTM_Hall_Sensor' */
volatile uint32_T CntHallDecoder;      /* '<S70>/Read_Register' */
volatile uint16_T CntHallValidityIn;
                                /* '<S2>/SigConvForSigProp_Variant_Source2_0' */
volatile real32_T DesiredSpeed;        /* '<Root>/Data Store Memory7' */
volatile boolean_T Enable;             /* '<Root>/Data Store Memory29' */
volatile real32_T Epsilon;             /* '<Root>/Data Store Memory13' */
volatile boolean_T FAULT;              /* '<Root>/I_MAX Scalling3' */
volatile real32_T Gamma;               /* '<Root>/Data Store Memory8' */
volatile int16_T GlobalDirection;      /* '<Root>/Data Store Memory3' */
volatile uint32_T GlobalHallState;     /* '<Root>/Data Store Memory4' */
volatile uint16_T GlobalSpeedCount;    /* '<Root>/Data Store Memory1' */
volatile uint16_T GlobalSpeedValidity; /* '<Root>/Data Store Memory2' */
volatile uint32_T HALL_A;              /* '<S191>/bit_shift' */
volatile uint32_T HALL_A_controller;   /* '<S73>/Data Type Conversion6' */
volatile uint32_T HALL_B;              /* '<S192>/bit_shift' */
volatile uint32_T HALL_B_controller;   /* '<S75>/bit_shift' */
volatile uint32_T HALL_C;              /* '<S189>/Data Type Conversion6' */
volatile uint32_T HALL_C_controller;   /* '<S74>/bit_shift' */
volatile uint16_T HallCntActual;       /* '<Root>/Data Store Memory25' */
volatile uint16_T HallCntPrev;         /* '<Root>/Data Store Memory24' */
volatile uint16_T HallStateChangeFlag; /* '<Root>/Data Store Memory' */
volatile uint32_T HallVal;             /* '<S73>/Add1' */
volatile uint16_T HallValididyInvalid; /* '<S195>/Merge' */
volatile real32_T I_ab_afterOffset[2]; /* '<S69>/Add' */
volatile real32_T IaOffset;            /* '<Root>/Data Store Memory5' */
volatile real32_T Iab_fb[2];           /* '<S69>/Multiply' */
volatile real32_T IbOffset;            /* '<Root>/Data Store Memory6' */
volatile real32_T Id_err;              /* '<S28>/Sum' */
volatile real32_T Id_fb;               /* '<S18>/Signal Copy1' */
volatile real32_T Idc_afterOffset;     /* '<S211>/Sum' */
volatile real32_T Idq_ref_PU[2];       /* '<Root>/RT11' */
volatile real32_T Iq_err;              /* '<S29>/Sum' */
volatile real32_T Iq_fb;               /* '<S18>/Signal Copy' */
volatile real32_T Lambda;              /* '<Root>/Data Store Memory9' */
volatile real32_T PWM[3];              /* '<S11>/Switch1' */
volatile real32_T PWM_Duty_Cycles[3];  /* '<S13>/Gain' */
volatile real32_T PWM_Enable;          /* '<S13>/Data Type Conversion' */
volatile real32_T Pos_PU;              /* '<S116>/Add' */
volatile uint32_T SC_PDBIF;            /* '<S208>/PDB1_ISR' */
volatile real32_T SIN;                 /* '<S53>/Sum4' */
volatile real32_T SpeedMeasured;       /* '<S1>/Input Scaling' */
volatile real32_T Speed_Ref;           /* '<S222>/Switch' */
volatile real32_T Speed_Ref_PU;        /* '<Root>/RT2' */
volatile real32_T Speed_fb;            /* '<Root>/RT1' */
volatile real32_T ThetaHalls;          /* '<S71>/Merge1' */
volatile real32_T Vd_ref_beforeLimiter;/* '<S28>/MATLAB Function1' */
volatile real32_T Vq_ref_beforeLimiter;/* '<S29>/MATLAB Function1' */

/* Block signals (default storage) */
B_Foc_model_Matlab_ultimo_T Foc_model_Matlab_ultimo_B;

/* Block states (default storage) */
DW_Foc_model_Matlab_ultimo_T Foc_model_Matlab_ultimo_DW;

/* Real-time model */
static RT_MODEL_Foc_model_Matlab_ult_T Foc_model_Matlab_ultimo_M_;
RT_MODEL_Foc_model_Matlab_ult_T *const Foc_model_Matlab_ultimo_M =
  &Foc_model_Matlab_ultimo_M_;
static void rate_monotonic_scheduler(void);
trgmux_inout_mapping_config_t trgmuxAllMappingConfig[2];
const trgmux_user_config_t pdbTrgmuxUserConfig = {
  .numInOutMappingConfigs = 2,
  .inOutMappingConfig = trgmuxAllMappingConfig
};

void pdb0_isr(void)
{
  uint32_t fPDBIF = (uint32_t)PDB_DRV_GetTimerIntFlag(0);
  uint32_t errCh0 = PDB_DRV_GetAdcPreTriggerSeqErrFlags(0, 0, 0xFF);
  uint32_t errCh1 = PDB_DRV_GetAdcPreTriggerSeqErrFlags(0, 1, 0xFF);
  SC_PDBIF = fPDBIF;
  CH0S_ERR = errCh0;
  CH1S_ERR = errCh1;

  /* Output and update for function-call system: '<S208>/PDB1_IRQHandler' */

  /* S-Function (ftm_s32k_init_disen): '<S210>/FTM_Init_Trigger_Enable' */

  /* FTM PWM Initialization Trigger Enable Disable*/
  FTM_DRV_SetInitTriggerCmd(FTM3, true);
  if (fPDBIF) {
    PDB_DRV_ClearTimerIntFlag(0);
  }

  if (errCh0) {
    PDB_DRV_ClearAdcPreTriggerSeqErrFlags(0, 0, errCh0);
  }

  if (errCh1) {
    PDB_DRV_ClearAdcPreTriggerSeqErrFlags(0, 1, errCh1);
  }
}

void ADC1_SC1reg2U_callback(void)
{
  adc_chan_config_t config;
  uint16_t result;
  ADC_DRV_GetChanResult(1, 2U, &result);
  Foc_model_Matlab_ultimo_B.ADC1_ISR_o2 = result;
  ADC_DRV_GetChanConfig(1, 2U, &config);
  Foc_model_Matlab_ultimo_B.ADC1_ISR_o3 = config.channel;

  /* Output and update for function-call system: '<S208>/ADC1_IRQHandler' */

  /* user code (Output function Body) */
  {
    /* Start of Profile Code */
    uint32_t tmp1;
    uint32_t tmp2;
    tmp1 = profiler_get_cnt();

    /* Start Profiling This Function.*/

    /* S-Function (ftm_s32k_init_disen): '<S209>/FTM_Init_Trigger_Disable' */

    /* FTM PWM Initialization Trigger Enable Disable*/
    FTM_DRV_SetInitTriggerCmd(FTM3, false);

    /* S-Function (adc_s32k_start): '<S209>/ADC_AD4_IA' */
    {
      uint16_t result;

      /* Get conversion result of ADC0 */
      ADC_DRV_WaitConvDone(0);
      ADC_DRV_GetChanResult(0, 0, &result);
      ADC_IA = result;
    }

    /* SignalConversion generated from: '<S209>/ADC_IB' */
    ADC_IB = Foc_model_Matlab_ultimo_B.ADC1_ISR_o2;

    /* S-Function (adc_s32k_start): '<S209>/ADC_AD7_VDC' */
    {
      uint16_t result;

      /* Get conversion result of ADC1 */
      ADC_DRV_WaitConvDone(1);
      ADC_DRV_GetChanResult(1, 0, &result);
      ADC_VDC = result;
    }

    /* S-Function (adc_s32k_start): '<S209>/ADC_AD6_IDC' */
    {
      uint16_t result;

      /* Get conversion result of ADC1 */
      ADC_DRV_WaitConvDone(1);
      ADC_DRV_GetChanResult(1, 1, &result);
      ADC_IDC = result;
    }

    /* Outputs for Atomic SubSystem: '<S209>/FaultDetection' */
    Foc_model_Ma_FaultDetection(ADC_VDC, ADC_IDC);

    /* End of Outputs for SubSystem: '<S209>/FaultDetection' */

    /* S-Function (fcgen): '<S209>/Function-Call Generator' incorporates:
     *  SubSystem: '<Root>/CurrentControl'
     */
    Foc_model_Ma_CurrentControl();

    /* End of Outputs for S-Function (fcgen): '<S209>/Function-Call Generator' */

    /* user code (Output function Trailer) */

    /* Profile Code : Compute function execution time in us. */
    tmp2 = profiler_get_cnt();
    profile_buffer[2] = gt_pf(tmp1, tmp2);

    /* End of Profile Code */
  }
}

void GPIPORTE10_callback (void)
{
  /* Output and update for function-call system: '<S4>/GD3000_interrupt' */

  /* DataStoreWrite: '<S216>/FAULT_write' incorporates:
   *  Constant: '<S216>/NOK'
   */
  FAULT = true;

  /* S-Function (ftm_s32k_pwm_disen): '<S216>/FTM_PWM_Disable_Enable' */
  FTM_DRV_DeinitPwm(FTM_PWM3);

  /* S-Function (tpp_s32k_func_mode): '<S216>/TPP_Functional_Mode' */
  TPP_SetOperationalMode(&tppDrvConfig, tppModeSleep);

  /* Clear interrupt flag */
  PINS_DRV_ClearPinIntFlagCmd(PORTE, 10);
}

/*
 * Set which subrates need to run this base step (base rate always runs).
 * This function must be called prior to calling the model step function
 * in order to remember which rates need to run this base step.  The
 * buffering of events allows for overlapping preemption.
 */
void Foc_model_Matlab_ultimo_SetEventsForThisBaseStep(boolean_T *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(Foc_model_Matlab_ultimo_M, 1));
  eventFlags[2] = ((boolean_T)rtmStepTask(Foc_model_Matlab_ultimo_M, 2));
  eventFlags[3] = ((boolean_T)rtmStepTask(Foc_model_Matlab_ultimo_M, 3));
}

/*
 *         This function updates active task flag for each subrate
 *         and rate transition flags for tasks that exchange data.
 *         The function assumes rate-monotonic multitasking scheduler.
 *         The function must be called at model base rate so that
 *         the generated code self-manages all its subrates and rate
 *         transition flags.
 */
static void rate_monotonic_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (Foc_model_Matlab_ultimo_M->Timing.TaskCounters.TID[1])++;
  if ((Foc_model_Matlab_ultimo_M->Timing.TaskCounters.TID[1]) > 1) {/* Sample time: [0.0001s, 0.0s] */
    Foc_model_Matlab_ultimo_M->Timing.TaskCounters.TID[1] = 0;
  }

  (Foc_model_Matlab_ultimo_M->Timing.TaskCounters.TID[2])++;
  if ((Foc_model_Matlab_ultimo_M->Timing.TaskCounters.TID[2]) > 19) {/* Sample time: [0.001s, 0.0s] */
    Foc_model_Matlab_ultimo_M->Timing.TaskCounters.TID[2] = 0;
  }

  (Foc_model_Matlab_ultimo_M->Timing.TaskCounters.TID[3])++;
  if ((Foc_model_Matlab_ultimo_M->Timing.TaskCounters.TID[3]) > 1999) {/* Sample time: [0.1s, 0.0s] */
    Foc_model_Matlab_ultimo_M->Timing.TaskCounters.TID[3] = 0;
  }
}

/* Output and update for atomic system: '<S209>/FaultDetection' */
void Foc_model_Ma_FaultDetection(uint32_T rtu_Vdc, uint32_T rtu_Idc)
{
  real32_T rtb_Product1_f;

  /* Product: '<S211>/Product1' incorporates:
   *  Constant: '<S211>/ADC1_AD7_Offset'
   *  Constant: '<S211>/bits2volts'
   *  DataTypeConversion: '<S211>/Data Type Conversion1'
   *  Sum: '<S211>/Add'
   */
  rtb_Product1_f = ((real32_T)rtu_Vdc - 17.0F) * 0.0109890113F;

  /* Sum: '<S211>/Sum' incorporates:
   *  Constant: '<S211>/ADC_AD6 offset  and  Logic power supply compensation'
   *  DataTypeConversion: '<S211>/Data Type Conversion'
   */
  Idc_afterOffset = (real32_T)rtu_Idc - 2090.0F;

  /* If: '<S211>/Check_Voltage_Current_Limits' incorporates:
   *  Constant: '<S211>/bits2amps'
   *  Product: '<S211>/Product'
   */
  if ((rtb_Product1_f < 8.0F) || (rtb_Product1_f > 16.0F) || (Idc_afterOffset *
       0.00805664062F > 2.3F)) {
    /* Outputs for IfAction SubSystem: '<S211>/FAILURE' incorporates:
     *  ActionPort: '<S212>/Action Port'
     */
    /* If: '<S212>/If' incorporates:
     *  DataStoreRead: '<S212>/FAULT_read'
     */
    if (!FAULT) {
      /* Outputs for IfAction SubSystem: '<S212>/Failed Subsystem' incorporates:
       *  ActionPort: '<S213>/Action Port'
       */
      /* S-Function (ftm_s32k_pwm_disen): '<S213>/FTM_PWM_Disable_Enable' */
      FTM_DRV_DeinitPwm(FTM_PWM3);

      /* DataStoreWrite: '<S213>/FAULT_write' incorporates:
       *  Constant: '<S213>/NOK'
       */
      FAULT = true;

      /* S-Function (tpp_s32k_func_mode): '<S213>/TPP_Functional_Mode' */
      TPP_SetOperationalMode(&tppDrvConfig, tppModeSleep);

      /* End of Outputs for SubSystem: '<S212>/Failed Subsystem' */
    }

    /* End of If: '<S212>/If' */
    /* End of Outputs for SubSystem: '<S211>/FAILURE' */
  }

  /* End of If: '<S211>/Check_Voltage_Current_Limits' */
}

/*
 * Output and update for action system:
 *    '<S96>/Hall Value of 1'
 *    '<S95>/Hall Value of 2'
 */
void Foc_model_Matl_HallValueof1(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S106>/position' incorporates:
   *  Constant: '<S106>/Constant'
   */
  *rty_position = 0.16667F;
}

/*
 * Output and update for action system:
 *    '<S96>/Hall Value of 2'
 *    '<S95>/Hall Value of 3'
 */
void Foc_model_Matl_HallValueof2(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S107>/position' incorporates:
   *  Constant: '<S107>/Constant'
   */
  *rty_position = 0.33333F;
}

/*
 * Output and update for action system:
 *    '<S96>/Hall Value of 3'
 *    '<S95>/Hall Value of 4'
 */
void Foc_model_Matl_HallValueof3(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S108>/position' incorporates:
   *  Constant: '<S108>/Constant'
   */
  *rty_position = 0.5F;
}

/*
 * Output and update for action system:
 *    '<S96>/Hall Value of 4'
 *    '<S95>/Hall Value of 5'
 */
void Foc_model_Matl_HallValueof4(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S109>/position' incorporates:
   *  Constant: '<S109>/Constant'
   */
  *rty_position = 0.66667F;
}

/*
 * Output and update for action system:
 *    '<S96>/Hall Value of 5'
 *    '<S95>/Hall Value of 6'
 */
void Foc_model_Matl_HallValueof5(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S110>/position' incorporates:
   *  Constant: '<S110>/Constant'
   */
  *rty_position = 0.83333F;
}

/*
 * Output and update for action system:
 *    '<S96>/Hall Value of 7'
 *    '<S95>/Hall Value of 1'
 *    '<S95>/Hall Value of 7'
 *    '<S84>/Hall Value of 7'
 */
void Foc_model_Matl_HallValueof7(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S112>/position' incorporates:
   *  Constant: '<S112>/Constant'
   */
  *rty_position = 0.0F;
}

real32_T rt_modf_snf(real32_T u0, real32_T u1)
{
  real32_T q;
  real32_T y;
  boolean_T yEq;
  y = u0;
  if (u1 == 0.0F) {
    if (u0 == 0.0F) {
      y = u1;
    }
  } else if (rtIsNaNF(u0) || rtIsNaNF(u1) || rtIsInfF(u0)) {
    y = (rtNaNF);
  } else if (u0 == 0.0F) {
    y = 0.0F / u1;
  } else if (rtIsInfF(u1)) {
    if ((u1 < 0.0F) != (u0 < 0.0F)) {
      y = u1;
    }
  } else {
    y = (real32_T)fmod(u0, u1);
    yEq = (y == 0.0F);
    if ((!yEq) && (u1 > (real32_T)floor(u1))) {
      q = (real32_T)fabs(u0 / u1);
      yEq = !((real32_T)fabs(q - (real32_T)floor(q + 0.5F)) > FLT_EPSILON * q);
    }

    if (yEq) {
      y = u1 * 0.0F;
    } else if ((u0 < 0.0F) != (u1 < 0.0F)) {
      y += u1;
    }
  }

  return y;
}

/* System initialize for function-call system: '<Root>/CurrentControl' */
void Foc_mod_CurrentControl_Init(void)
{
  /* Start for S-Function (ftm_s32k_pwm_config): '<S11>/FTM_PWM_Config' */

  /* Enable clock for PORTB */
  PCC_SetClockMode (PCC, PCC_PORTB_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel (PORTB, 8, PORT_MUX_ALT2);

  /* Enable clock for PORTB */
  PCC_SetClockMode (PCC, PCC_PORTB_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel (PORTB, 9, PORT_MUX_ALT2);

  /* Enable clock for PORTB */
  PCC_SetClockMode (PCC, PCC_PORTB_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel (PORTB, 10, PORT_MUX_ALT2);

  /* Enable clock for PORTB */
  PCC_SetClockMode (PCC, PCC_PORTB_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel (PORTB, 11, PORT_MUX_ALT2);

  /* Enable clock for PORTC */
  PCC_SetClockMode (PCC, PCC_PORTC_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel (PORTC, 10, PORT_MUX_ALT2);

  /* Enable clock for PORTC */
  PCC_SetClockMode (PCC, PCC_PORTC_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel (PORTC, 11, PORT_MUX_ALT2);

  /* Set FTM_3 clock source */
  PCC_SetPeripheralClockControl (PCC, FTM3_CLK, true, CLK_SRC_SPLL, 0, 0);

  /* Enable clock for FTM_3 */
  PCC_SetClockMode (PCC, FTM3_CLK, true);

  /* PWM3 initialization */
  FTM_DRV_Init (FTM_PWM3, &flexTimer_pwm3_InitConfig, &ftmStateStruct3);
  FTM_DRV_SetChnTriggerCmd(FTM3, 1, false);
  FTM_DRV_SetChnTriggerCmd(FTM3, 3, false);
  FTM_DRV_SetChnTriggerCmd(FTM3, 5, false);

  /* Start for S-Function (profiler_s32k_fcn): '<S1>/FOCProfiler' */
  {
    /* Un-gate pit clock*/
    PCC_SetPeripheralClockControl(PCC, LPIT0_CLK, true, CLK_SRC_SPLL, 0, 0);
  }

  profiler_init();

  /* InitializeConditions for Delay: '<S79>/Delay One Step1' */
  Foc_model_Matlab_ultimo_DW.DelayOneStep1_DSTATE = true;

  /* InitializeConditions for Delay: '<S79>/Delay One Step' */
  Foc_model_Matlab_ultimo_DW.DelayOneStep_DSTATE = 500U;

  /* InitializeConditions for DiscreteIntegrator: '<S168>/Integrator' */
  Foc_model_Matlab_ultimo_DW.Integrator_DSTATE = Foc_model_Matlab_ultimo_B.Ki2;

  /* SystemInitialize for Atomic SubSystem: '<S68>/Atomic Hall Reading' */

  /* Start for S-Function (register_s32k_read): '<S70>/Read_Register' */
  PCC_SetClockMode(PCC, ADC0_CLK, true);

  /* End of SystemInitialize for SubSystem: '<S68>/Atomic Hall Reading' */
}

/* System reset for function-call system: '<Root>/CurrentControl' */
void Foc_mo_CurrentControl_Reset(void)
{
  /* InitializeConditions for Delay: '<S79>/Delay One Step1' */
  Foc_model_Matlab_ultimo_DW.DelayOneStep1_DSTATE = true;

  /* InitializeConditions for Delay: '<S79>/Delay One Step' */
  Foc_model_Matlab_ultimo_DW.DelayOneStep_DSTATE = 500U;

  /* InitializeConditions for DiscreteIntegrator: '<S14>/Discrete-Time Integrator4' */
  Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator4_PREV_U = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S14>/Discrete-Time Integrator2' */
  Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator2_PREV_U = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S14>/Discrete-Time Integrator3' */
  Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator3_PREV_U = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S14>/Discrete-Time Integrator1' */
  Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator1_PREV_U = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S14>/Discrete-Time Integrator' */
  Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator_PREV_U = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S168>/Integrator' */
  Foc_model_Matlab_ultimo_DW.Integrator_DSTATE = Foc_model_Matlab_ultimo_B.Ki2;
  Foc_model_Matlab_ultimo_DW.Integrator_PrevResetState = 0;

  /* InitializeConditions for Delay: '<S16>/Delay' */
  Foc_model_Matlab_ultimo_DW.CircBufIdx = 0U;

  /* SystemReset for MATLAB Function: '<S29>/MATLAB Function1' */
  Foc_model_Matlab_ultimo_DW.integral_not_empty = false;

  /* SystemReset for MATLAB Function: '<S28>/MATLAB Function1' */
  Foc_model_Matlab_ultimo_DW.integral_not_empty_p = false;
}

/* Enable for function-call system: '<Root>/CurrentControl' */
void Foc_m_CurrentControl_Enable(void)
{
  Foc_model_Matlab_ultimo_DW.CurrentControl_RESET_ELAPS_T = true;

  /* Enable for DiscreteIntegrator: '<S14>/Discrete-Time Integrator4' */
  Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator4_SYSTEM_ = 1U;

  /* Enable for DiscreteIntegrator: '<S14>/Discrete-Time Integrator2' */
  Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator2_SYSTEM_ = 1U;

  /* Enable for DiscreteIntegrator: '<S14>/Discrete-Time Integrator3' */
  Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator3_SYSTEM_ = 1U;

  /* Enable for DiscreteIntegrator: '<S14>/Discrete-Time Integrator1' */
  Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator1_SYSTEM_ = 1U;

  /* Enable for DiscreteIntegrator: '<S14>/Discrete-Time Integrator' */
  Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator_SYSTEM_E = 1U;
}

/* Output and update for function-call system: '<Root>/CurrentControl' */
void Foc_model_Ma_CurrentControl(void)
{
  int32_T rtb_Step;
  real32_T P;
  real32_T int_sat_ia;
  real32_T int_sat_ib;
  real32_T rtb_DTC;
  real32_T rtb_DTC_p;
  real32_T rtb_DeadZone;
  real32_T rtb_Integrator;
  real32_T rtb_Merge1;
  real32_T rtb_Merge1_p;
  real32_T rtb_Merge_ik;
  real32_T rtb_Sum_o;
  real32_T rtb_algDD_o1_f;
  real32_T wnq;
  real32_T z_alpha;
  real32_T z_beta;
  uint32_T CurrentControl_ELAPS_T;
  uint16_T rtb_DataStoreRead5;
  int8_T tmp;
  int8_T tmp_0;
  boolean_T OR;
  boolean_T rtb_Compare;
  boolean_T rtb_NOT;

  /* user code (Output function Body) */
  {
    /* Start of Profile Code */
    uint32_t tmp1;
    uint32_t tmp2;
    tmp1 = profiler_get_cnt();

    /* Start Profiling This Function.*/
    if (Foc_model_Matlab_ultimo_DW.CurrentControl_RESET_ELAPS_T) {
      CurrentControl_ELAPS_T = 0U;
    } else {
      CurrentControl_ELAPS_T = Foc_model_Matlab_ultimo_M->Timing.clockTick1 -
        Foc_model_Matlab_ultimo_DW.CurrentControl_PREV_T;
    }

    Foc_model_Matlab_ultimo_DW.CurrentControl_PREV_T =
      Foc_model_Matlab_ultimo_M->Timing.clockTick1;
    Foc_model_Matlab_ultimo_DW.CurrentControl_RESET_ELAPS_T = false;

    /* Outputs for Atomic SubSystem: '<S68>/Atomic Hall Reading' */
    /* S-Function (register_s32k_read): '<S70>/Read_Register' */

    /* read from ADC0_SC1A register */
    CntHallDecoder = *((uint32_t *) 0x4003B000);

    /* DataStoreRead: '<S70>/Data Store Read5' */
    rtb_DataStoreRead5 = HallStateChangeFlag;

    /* S-Function (gpio_s32k_input): '<S73>/Digital_Input_HALL_C' */

    /* GPIPORTA1 signal update */
    Foc_model_Matlab_ultimo_B.Digital_Input_HALL_C = (PINS_DRV_ReadPins(PTA) >>
      1) & 0x01;

    /* Outputs for Atomic SubSystem: '<S73>/Bit Shift' */
    /* MATLAB Function: '<S74>/bit_shift' incorporates:
     *  DataTypeConversion: '<S73>/Data Type Conversion3'
     */
    /* MATLAB Function 'Logic and Bit Operations/Bit Shift/bit_shift': '<S76>:1' */
    /* '<S76>:1:6' */
    HALL_C_controller = (uint32_T)Foc_model_Matlab_ultimo_B.Digital_Input_HALL_C
      << 2;

    /* End of Outputs for SubSystem: '<S73>/Bit Shift' */

    /* S-Function (gpio_s32k_input): '<S73>/Digital_Input_HALL_B' */

    /* GPIPORTD10 signal update */
    Foc_model_Matlab_ultimo_B.Digital_Input_HALL_B = (PINS_DRV_ReadPins(PTD) >>
      10) & 0x01;

    /* Outputs for Atomic SubSystem: '<S73>/Bit Shift1' */
    /* MATLAB Function: '<S75>/bit_shift' incorporates:
     *  DataTypeConversion: '<S73>/Data Type Conversion2'
     */
    /* MATLAB Function 'Logic and Bit Operations/Bit Shift/bit_shift': '<S77>:1' */
    /* '<S77>:1:6' */
    HALL_B_controller = (uint32_T)Foc_model_Matlab_ultimo_B.Digital_Input_HALL_B
      << 1;

    /* End of Outputs for SubSystem: '<S73>/Bit Shift1' */

    /* S-Function (gpio_s32k_input): '<S73>/Digital_Input_HALL_A' */

    /* GPIPORTD11 signal update */
    Foc_model_Matlab_ultimo_B.Digital_Input_HALL_A = (PINS_DRV_ReadPins(PTD) >>
      11) & 0x01;

    /* DataTypeConversion: '<S73>/Data Type Conversion6' */
    HALL_A_controller = (uint32_T)Foc_model_Matlab_ultimo_B.Digital_Input_HALL_A;

    /* Sum: '<S73>/Add1' */
    HallVal = (HALL_C_controller + HALL_B_controller) + HALL_A_controller;

    /* Switch: '<S71>/Switch' incorporates:
     *  Constant: '<S71>/WatchDog'
     *  DataStoreRead: '<S70>/Data Store Read5'
     *  DataStoreWrite: '<S68>/Data Store Write2'
     */
    if (HallStateChangeFlag != 0) {
      HallStateChangeFlag = 0U;
    }

    /* End of Switch: '<S71>/Switch' */
    /* End of Outputs for SubSystem: '<S68>/Atomic Hall Reading' */

    /* Logic: '<S79>/OR' incorporates:
     *  DataTypeConversion: '<S71>/Data Type Conversion4'
     *  Delay: '<S79>/Delay One Step1'
     */
    OR = (Foc_model_Matlab_ultimo_DW.DelayOneStep1_DSTATE || (rtb_DataStoreRead5
           != 0));

    /* Delay: '<S79>/Delay One Step' incorporates:
     *  DataTypeConversion: '<S71>/Data Type Conversion4'
     */
    if (OR) {
      if (rtb_DataStoreRead5 != 0) {
        Foc_model_Matlab_ultimo_DW.DelayOneStep_DSTATE = 500U;
      }

      /* Delay: '<S79>/Delay One Step' incorporates:
       *  DataTypeConversion: '<S71>/Data Type Conversion4'
       */
      Foc_model_Matlab_ultimo_B.DelayOneStep =
        Foc_model_Matlab_ultimo_DW.DelayOneStep_DSTATE;
    }

    /* End of Delay: '<S79>/Delay One Step' */

    /* RelationalOperator: '<S83>/Compare' incorporates:
     *  Constant: '<S83>/Constant'
     */
    rtb_Compare = (Foc_model_Matlab_ultimo_B.DelayOneStep > 0);

    /* Switch: '<S82>/watchdog check' incorporates:
     *  Constant: '<S82>/Constant'
     */
    if (rtb_Compare) {
      /* MinMax: '<S82>/Max' incorporates:
       *  DataStoreRead: '<S70>/Data Store Read2'
       *  DataTypeConversion: '<S71>/counterSize1'
       */
      if (GlobalSpeedCount >= (uint16_T)CntHallDecoder) {
        rtb_DataStoreRead5 = GlobalSpeedCount;
      } else {
        rtb_DataStoreRead5 = (uint16_T)CntHallDecoder;
      }

      /* Switch: '<S82>/speed check' incorporates:
       *  Constant: '<S82>/Constant'
       *  DataStoreRead: '<S70>/Data Store Read4'
       *  DataTypeConversion: '<S78>/Data Type Conversion'
       *  Logic: '<S78>/Logical Operator'
       *  MinMax: '<S82>/Max'
       */
      if (rtb_DataStoreRead5 >= 31250) {
        rtb_DataStoreRead5 = 0U;
      } else {
        rtb_DataStoreRead5 = (uint16_T)((GlobalSpeedValidity != 0) ||
          Foc_model_Matlab_ultimo_B.validityDelay);
      }

      /* End of Switch: '<S82>/speed check' */
    } else {
      rtb_DataStoreRead5 = 0U;
    }

    /* End of Switch: '<S82>/watchdog check' */

    /* If: '<S71>/If' */
    if (rtb_DataStoreRead5 != 0) {
      /* Outputs for IfAction SubSystem: '<S71>/Speed and direction are valid Use speed to extrapolate position' incorporates:
       *  ActionPort: '<S81>/Action Port'
       */
      /* If: '<S81>/If' incorporates:
       *  DataStoreRead: '<S70>/Data Store Read3'
       */
      if (GlobalDirection > 0) {
        /* Outputs for IfAction SubSystem: '<S81>/If Action Subsystem' incorporates:
         *  ActionPort: '<S92>/Action Port'
         */
        /* SignalConversion generated from: '<S92>/In1' incorporates:
         *  DataStoreRead: '<S70>/Data Store Read2'
         *  DataTypeConversion: '<S81>/currentSpeedData'
         *  Gain: '<S81>/SpeedGain'
         *  Product: '<S81>/Divide'
         */
        rtb_Merge_ik = Foc_model_Matlab_ultimo_ConstB.SpeedConstData / (real32_T)
          GlobalSpeedCount * 0.05F;

        /* End of Outputs for SubSystem: '<S81>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S81>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S93>/Action Port'
         */
        /* UnaryMinus: '<S93>/Unary Minus' incorporates:
         *  DataStoreRead: '<S70>/Data Store Read2'
         *  DataTypeConversion: '<S81>/currentSpeedData'
         *  Gain: '<S81>/SpeedGain'
         *  Product: '<S81>/Divide'
         */
        rtb_Merge_ik = -(Foc_model_Matlab_ultimo_ConstB.SpeedConstData /
                         (real32_T)GlobalSpeedCount * 0.05F);

        /* End of Outputs for SubSystem: '<S81>/If Action Subsystem1' */
      }

      /* End of If: '<S81>/If' */

      /* Outputs for Enabled SubSystem: '<S81>/Subsystem1' incorporates:
       *  EnablePort: '<S94>/Enable'
       */
      /* Outputs for IfAction SubSystem: '<S94>/first_order' incorporates:
       *  ActionPort: '<S97>/Action Port'
       */
      /* If: '<S94>/If1' incorporates:
       *  DataStoreRead: '<S70>/Data Store Read2'
       *  DataTypeConversion: '<S71>/counterSize1'
       *  DataTypeConversion: '<S97>/countData'
       *  DataTypeConversion: '<S97>/currentSpeedData'
       *  Gain: '<S97>/Gain'
       *  Product: '<S97>/Divide'
       */
      rtb_Merge1 = (real32_T)(uint16_T)CntHallDecoder / (real32_T)
        GlobalSpeedCount * 0.166666672F;

      /* End of Outputs for SubSystem: '<S94>/first_order' */

      /* Saturate: '<S94>/Saturation' */
      if (rtb_Merge1 > 0.16667F) {
        rtb_Merge1 = 0.16667F;
      }

      /* End of Saturate: '<S94>/Saturation' */

      /* If: '<S94>/If' incorporates:
       *  DataStoreRead: '<S70>/Data Store Read3'
       */
      if (GlobalDirection != 1) {
        /* Outputs for IfAction SubSystem: '<S94>/-ve Direction' incorporates:
         *  ActionPort: '<S96>/Action Port'
         */
        /* SwitchCase: '<S96>/Switch Case' */
        switch ((int32_T)HallVal) {
         case 6:
          /* Outputs for IfAction SubSystem: '<S96>/Hall Value of 1' incorporates:
           *  ActionPort: '<S106>/Action Port'
           */
          Foc_model_Matl_HallValueof1(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S96>/Hall Value of 1' */
          break;

         case 4:
          /* Outputs for IfAction SubSystem: '<S96>/Hall Value of 2' incorporates:
           *  ActionPort: '<S107>/Action Port'
           */
          Foc_model_Matl_HallValueof2(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S96>/Hall Value of 2' */
          break;

         case 5:
          /* Outputs for IfAction SubSystem: '<S96>/Hall Value of 3' incorporates:
           *  ActionPort: '<S108>/Action Port'
           */
          Foc_model_Matl_HallValueof3(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S96>/Hall Value of 3' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S96>/Hall Value of 4' incorporates:
           *  ActionPort: '<S109>/Action Port'
           */
          Foc_model_Matl_HallValueof4(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S96>/Hall Value of 4' */
          break;

         case 3:
          /* Outputs for IfAction SubSystem: '<S96>/Hall Value of 5' incorporates:
           *  ActionPort: '<S110>/Action Port'
           */
          Foc_model_Matl_HallValueof5(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S96>/Hall Value of 5' */
          break;

         case 2:
          /* Outputs for IfAction SubSystem: '<S96>/Hall Value of 6' incorporates:
           *  ActionPort: '<S111>/Action Port'
           */
          /* SignalConversion generated from: '<S111>/position' incorporates:
           *  Constant: '<S111>/Constant'
           */
          rtb_Merge1_p = 1.0F;

          /* End of Outputs for SubSystem: '<S96>/Hall Value of 6' */
          break;

         default:
          /* Outputs for IfAction SubSystem: '<S96>/Hall Value of 7' incorporates:
           *  ActionPort: '<S112>/Action Port'
           */
          Foc_model_Matl_HallValueof7(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S96>/Hall Value of 7' */
          break;
        }

        /* End of SwitchCase: '<S96>/Switch Case' */

        /* Merge: '<S94>/Merge' incorporates:
         *  Sum: '<S96>/Sum'
         */
        Foc_model_Matlab_ultimo_B.Merge = rtb_Merge1_p - rtb_Merge1;

        /* End of Outputs for SubSystem: '<S94>/-ve Direction' */
      } else {
        /* Outputs for IfAction SubSystem: '<S94>/+ve Direction' incorporates:
         *  ActionPort: '<S95>/Action Port'
         */
        /* SwitchCase: '<S95>/Switch Case' */
        switch ((int32_T)HallVal) {
         case 6:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 1' incorporates:
           *  ActionPort: '<S99>/Action Port'
           */
          Foc_model_Matl_HallValueof7(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 1' */
          break;

         case 4:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 2' incorporates:
           *  ActionPort: '<S100>/Action Port'
           */
          Foc_model_Matl_HallValueof1(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 2' */
          break;

         case 5:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 3' incorporates:
           *  ActionPort: '<S101>/Action Port'
           */
          Foc_model_Matl_HallValueof2(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 3' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 4' incorporates:
           *  ActionPort: '<S102>/Action Port'
           */
          Foc_model_Matl_HallValueof3(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 4' */
          break;

         case 3:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 5' incorporates:
           *  ActionPort: '<S103>/Action Port'
           */
          Foc_model_Matl_HallValueof4(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 5' */
          break;

         case 2:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 6' incorporates:
           *  ActionPort: '<S104>/Action Port'
           */
          Foc_model_Matl_HallValueof5(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 6' */
          break;

         default:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 7' incorporates:
           *  ActionPort: '<S105>/Action Port'
           */
          Foc_model_Matl_HallValueof7(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 7' */
          break;
        }

        /* End of SwitchCase: '<S95>/Switch Case' */

        /* Merge: '<S94>/Merge' incorporates:
         *  Sum: '<S95>/Sum'
         */
        Foc_model_Matlab_ultimo_B.Merge = rtb_Merge1_p + rtb_Merge1;

        /* End of Outputs for SubSystem: '<S94>/+ve Direction' */
      }

      /* End of If: '<S94>/If' */
      /* End of Outputs for SubSystem: '<S81>/Subsystem1' */

      /* Merge: '<S71>/Merge1' incorporates:
       *  SignalConversion generated from: '<S81>/rawPosition'
       */
      ThetaHalls = Foc_model_Matlab_ultimo_B.Merge;

      /* End of Outputs for SubSystem: '<S71>/Speed and direction are valid Use speed to extrapolate position' */
    } else {
      /* Outputs for IfAction SubSystem: '<S71>/Speed and direction are not valid Position will be set to the middle of the Hall quadrant' incorporates:
       *  ActionPort: '<S80>/Action Port'
       */
      /* SwitchCase: '<S84>/Switch Case' */
      switch ((int32_T)HallVal) {
       case 6:
        /* Outputs for IfAction SubSystem: '<S84>/Hall Value of 1' incorporates:
         *  ActionPort: '<S85>/Action Port'
         */
        /* Merge: '<S71>/Merge1' incorporates:
         *  Constant: '<S85>/Constant'
         *  SignalConversion generated from: '<S85>/position'
         */
        ThetaHalls = 0.083333F;

        /* End of Outputs for SubSystem: '<S84>/Hall Value of 1' */
        break;

       case 4:
        /* Outputs for IfAction SubSystem: '<S84>/Hall Value of 2' incorporates:
         *  ActionPort: '<S86>/Action Port'
         */
        /* Merge: '<S71>/Merge1' incorporates:
         *  Constant: '<S86>/Constant'
         *  SignalConversion generated from: '<S86>/position'
         */
        ThetaHalls = 0.25F;

        /* End of Outputs for SubSystem: '<S84>/Hall Value of 2' */
        break;

       case 5:
        /* Outputs for IfAction SubSystem: '<S84>/Hall Value of 3' incorporates:
         *  ActionPort: '<S87>/Action Port'
         */
        /* Merge: '<S71>/Merge1' incorporates:
         *  Constant: '<S87>/Constant'
         *  SignalConversion generated from: '<S87>/position'
         */
        ThetaHalls = 0.41667F;

        /* End of Outputs for SubSystem: '<S84>/Hall Value of 3' */
        break;

       case 1:
        /* Outputs for IfAction SubSystem: '<S84>/Hall Value of 4' incorporates:
         *  ActionPort: '<S88>/Action Port'
         */
        /* Merge: '<S71>/Merge1' incorporates:
         *  Constant: '<S88>/Constant'
         *  SignalConversion generated from: '<S88>/position'
         */
        ThetaHalls = 0.58333F;

        /* End of Outputs for SubSystem: '<S84>/Hall Value of 4' */
        break;

       case 3:
        /* Outputs for IfAction SubSystem: '<S84>/Hall Value of 5' incorporates:
         *  ActionPort: '<S89>/Action Port'
         */
        /* Merge: '<S71>/Merge1' incorporates:
         *  Constant: '<S89>/Constant'
         *  SignalConversion generated from: '<S89>/position'
         */
        ThetaHalls = 0.75F;

        /* End of Outputs for SubSystem: '<S84>/Hall Value of 5' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S84>/Hall Value of 6' incorporates:
         *  ActionPort: '<S90>/Action Port'
         */
        /* Merge: '<S71>/Merge1' incorporates:
         *  Constant: '<S90>/Constant'
         *  SignalConversion generated from: '<S90>/position'
         */
        ThetaHalls = 0.91667F;

        /* End of Outputs for SubSystem: '<S84>/Hall Value of 6' */
        break;

       default:
        /* Outputs for IfAction SubSystem: '<S84>/Hall Value of 7' incorporates:
         *  ActionPort: '<S91>/Action Port'
         */
        Foc_model_Matl_HallValueof7((real32_T *)&ThetaHalls);

        /* End of Outputs for SubSystem: '<S84>/Hall Value of 7' */
        break;
      }

      /* End of SwitchCase: '<S84>/Switch Case' */

      /* SignalConversion generated from: '<S80>/Speed(r.p.m)' incorporates:
       *  Constant: '<S80>/Constant'
       */
      rtb_Merge_ik = 0.0F;

      /* End of Outputs for SubSystem: '<S71>/Speed and direction are not valid Position will be set to the middle of the Hall quadrant' */
    }

    /* End of If: '<S71>/If' */

    /* Sum: '<S79>/Sum' */
    rtb_DataStoreRead5 = Foc_model_Matlab_ultimo_B.DelayOneStep;

    /* If: '<S115>/If' incorporates:
     *  Constant: '<S114>/Constant1'
     *  Switch: '<S114>/Switch'
     */
    if (ThetaHalls <= 0.7061F) {
      /* Outputs for IfAction SubSystem: '<S115>/If Action Subsystem' incorporates:
       *  ActionPort: '<S117>/Action Port'
       */
      /* Sum: '<S117>/Add' incorporates:
       *  Constant: '<S117>/Constant'
       */
      rtb_Merge1 = (ThetaHalls + 1.0F) - 0.7061F;

      /* End of Outputs for SubSystem: '<S115>/If Action Subsystem' */
    } else {
      /* Outputs for IfAction SubSystem: '<S115>/If Action Subsystem1' incorporates:
       *  ActionPort: '<S118>/Action Port'
       */
      /* Sum: '<S118>/Add' */
      rtb_Merge1 = ThetaHalls - 0.7061F;

      /* End of Outputs for SubSystem: '<S115>/If Action Subsystem1' */
    }

    /* End of If: '<S115>/If' */

    /* Sum: '<S116>/Add' incorporates:
     *  Rounding: '<S116>/Floor'
     */
    Pos_PU = rtb_Merge1 - (real32_T)floor(rtb_Merge1);

    /* Sum: '<S69>/Add' incorporates:
     *  DataStoreRead: '<S186>/Data Store Read'
     *  DataStoreRead: '<S186>/Data Store Read1'
     *  DataStoreRead: '<S69>/Data Store Read'
     *  DataStoreRead: '<S69>/Data Store Read1'
     */
    I_ab_afterOffset[0] = IaOffset - ADC_A;
    I_ab_afterOffset[1] = IbOffset - ADC_B;

    /* Gain: '<S69>/Multiply' */
    Iab_fb[0] = 0.00048828125F * I_ab_afterOffset[0];
    Iab_fb[1] = 0.00048828125F * I_ab_afterOffset[1];

    /* Outputs for Atomic SubSystem: '<S23>/Two phase CRL wrap' */
    /* Gain: '<S24>/one_by_sqrt3' incorporates:
     *  Sum: '<S24>/a_plus_2b'
     */
    rtb_DTC_p = ((Iab_fb[0] + Iab_fb[1]) + Iab_fb[1]) * 0.577350259F;

    /* AlgorithmDescriptorDelegate generated from: '<S24>/a16' */
    rtb_algDD_o1_f = Iab_fb[0];

    /* End of Outputs for SubSystem: '<S23>/Two phase CRL wrap' */

    /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator4' */
    if (Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator4_SYSTEM_ != 0) {
      /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator4' */
      rtb_Merge1_p = Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator4_DSTATE;
    } else {
      /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator4' */
      rtb_Merge1_p = 0.0001F * (real32_T)CurrentControl_ELAPS_T
        * Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator4_PREV_U +
        Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator4_DSTATE;
    }

    /* End of DiscreteIntegrator: '<S14>/Discrete-Time Integrator4' */

    /* Gain: '<S14>/Gain' */
    rtb_Merge1 = 6.28318548F * rtb_Merge1_p;

    /* If: '<S54>/If' incorporates:
     *  Constant: '<S56>/Constant'
     *  DataTypeConversion: '<S57>/Convert_back'
     *  DataTypeConversion: '<S57>/Convert_uint16'
     *  DataTypeConversion: '<S58>/Convert_back'
     *  DataTypeConversion: '<S58>/Convert_uint16'
     *  Gain: '<S52>/indexing'
     *  RelationalOperator: '<S56>/Compare'
     *  Sum: '<S57>/Sum'
     *  Sum: '<S58>/Sum'
     *  Switch: '<S1>/Switch'
     */
    if (Pos_PU < 0.0F) {
      /* Outputs for IfAction SubSystem: '<S54>/If Action Subsystem' incorporates:
       *  ActionPort: '<S57>/Action Port'
       */
      rtb_DeadZone = Pos_PU - (real32_T)(int16_T)(real32_T)floor(Pos_PU);

      /* End of Outputs for SubSystem: '<S54>/If Action Subsystem' */
    } else {
      /* Outputs for IfAction SubSystem: '<S54>/If Action Subsystem1' incorporates:
       *  ActionPort: '<S58>/Action Port'
       */
      rtb_DeadZone = Pos_PU - (real32_T)(int16_T)Pos_PU;

      /* End of Outputs for SubSystem: '<S54>/If Action Subsystem1' */
    }

    rtb_DeadZone *= 800.0F;

    /* End of If: '<S54>/If' */

    /* Sum: '<S52>/Sum2' incorporates:
     *  DataTypeConversion: '<S52>/Data Type Conversion1'
     *  DataTypeConversion: '<S52>/Get_Integer'
     */
    rtb_Integrator = rtb_DeadZone - (real32_T)(uint16_T)rtb_DeadZone;

    /* Selector: '<S52>/Lookup' incorporates:
     *  Constant: '<S52>/sine_table_values'
     *  DataTypeConversion: '<S52>/Get_Integer'
     */
    wnq = Foc_model_Matlab_ultimo_ConstP.sine_table_values_Value[(uint16_T)
      rtb_DeadZone];

    /* Sum: '<S53>/Sum4' incorporates:
     *  Constant: '<S52>/offset'
     *  Constant: '<S52>/sine_table_values'
     *  DataTypeConversion: '<S52>/Get_Integer'
     *  Product: '<S53>/Product'
     *  Selector: '<S52>/Lookup'
     *  Sum: '<S52>/Sum'
     *  Sum: '<S53>/Sum3'
     */
    SIN = (Foc_model_Matlab_ultimo_ConstP.sine_table_values_Value[(int32_T)
           ((uint16_T)rtb_DeadZone + 1U)] - wnq) * rtb_Integrator + wnq;

    /* Selector: '<S52>/Lookup' incorporates:
     *  Constant: '<S52>/offset'
     *  Constant: '<S52>/sine_table_values'
     *  DataTypeConversion: '<S52>/Get_Integer'
     *  Sum: '<S52>/Sum'
     *  Sum: '<S53>/Sum5'
     */
    wnq = Foc_model_Matlab_ultimo_ConstP.sine_table_values_Value[(int32_T)
      ((uint16_T)rtb_DeadZone + 200U)];

    /* Sum: '<S53>/Sum6' incorporates:
     *  Constant: '<S52>/offset'
     *  Constant: '<S52>/sine_table_values'
     *  DataTypeConversion: '<S52>/Get_Integer'
     *  Product: '<S53>/Product1'
     *  Selector: '<S52>/Lookup'
     *  Sum: '<S52>/Sum'
     *  Sum: '<S53>/Sum5'
     */
    COS = (Foc_model_Matlab_ultimo_ConstP.sine_table_values_Value[(int32_T)
           ((uint16_T)rtb_DeadZone + 201U)] - wnq) * rtb_Integrator + wnq;

    /* Outputs for Atomic SubSystem: '<S20>/Two inputs CRL' */
    /* Outputs for Atomic SubSystem: '<S23>/Two phase CRL wrap' */
    /* SignalConversion: '<S18>/Signal Copy' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S24>/a16'
     *  Product: '<S50>/asin'
     *  Product: '<S50>/bcos'
     *  Sum: '<S50>/sum_Qs'
     */
    Iq_fb = rtb_DTC_p * COS - Iab_fb[0] * SIN;

    /* End of Outputs for SubSystem: '<S23>/Two phase CRL wrap' */
    /* End of Outputs for SubSystem: '<S20>/Two inputs CRL' */

    /* Sum: '<S29>/Sum' */
    Iq_err = Idq_ref_PU[1] - Iq_fb;

    /* MATLAB Function: '<S29>/MATLAB Function' incorporates:
     *  Constant: '<S29>/Constant'
     *  Constant: '<S29>/Constant1'
     *  DataStoreRead: '<S29>/Data Store Read4'
     *  MATLAB Function: '<S28>/MATLAB Function'
     */
    /* MATLAB Function 'CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/MATLAB Function': '<S46>:1' */
    /* '<S46>:1:3' */
    rtb_DeadZone = 1.0F / (1.0F - Gamma);
    wnq = rtb_DeadZone * 1463.60156F;

    /* '<S46>:1:4' */
    P = 1.414F * wnq * 0.000435F - 0.636666656F;

    /* '<S46>:1:5' */
    wnq = P / (wnq * wnq * 0.000435F);

    /* '<S46>:1:6' */
    /* '<S46>:1:7' */
    wnq *= P / wnq;

    /* Logic: '<S29>/Logical Operator' incorporates:
     *  DataStoreRead: '<S29>/Data Store Read1'
     *  Logic: '<S28>/Logical Operator'
     */
    /* MATLAB Function 'CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/MATLAB Function1': '<S47>:1' */
    /* '<S47>:1:6' */
    /* '<S47>:1:7' */
    /* '<S47>:1:8' */
    /* '<S47>:1:9' */
    /* '<S47>:1:10' */
    /* '<S47>:1:11' */
    rtb_NOT = !Enable;

    /* MATLAB Function: '<S29>/MATLAB Function1' incorporates:
     *  Constant: '<S29>/Kp1'
     *  Constant: '<S29>/Kp2'
     *  Logic: '<S29>/Logical Operator'
     *  MATLAB Function: '<S29>/MATLAB Function'
     */
    if ((!Foc_model_Matlab_ultimo_DW.integral_not_empty) || rtb_NOT) {
      /* '<S47>:1:14' */
      /* '<S47>:1:15' */
      Foc_model_Matlab_ultimo_DW.integral = 0.0F;
      Foc_model_Matlab_ultimo_DW.integral_not_empty = true;
    }

    /* '<S47>:1:19' */
    P *= Iq_err;
    rtb_Integrator = wnq * Foc_model_Matlab_ultimo_DW.integral + P;
    if (((rtb_Integrator < 1.0F) && (rtb_Integrator > -1.0F)) ||
        ((rtb_Integrator >= 1.0F) && (Iq_err < 0.0F)) || ((rtb_Integrator <=
          -1.0F) && (Iq_err > 0.0F))) {
      /* '<S47>:1:22' */
      /* '<S47>:1:23' */
      /* '<S47>:1:24' */
      /* '<S47>:1:25' */
      Foc_model_Matlab_ultimo_DW.integral += Iq_err;
    }

    /* '<S47>:1:29' */
    /* '<S47>:1:32' */
    Vq_ref_beforeLimiter = wnq * Foc_model_Matlab_ultimo_DW.integral + P;
    if (!(Vq_ref_beforeLimiter >= -1.0F)) {
      Vq_ref_beforeLimiter = -1.0F;
    }

    if (!(Vq_ref_beforeLimiter <= 1.0F)) {
      Vq_ref_beforeLimiter = 1.0F;
    }

    /* End of MATLAB Function: '<S29>/MATLAB Function1' */

    /* Gain: '<S27>/Gain1' */
    rtb_Integrator = 2.0F * rtb_Merge_ik;

    /* Outputs for Atomic SubSystem: '<S20>/Two inputs CRL' */
    /* Outputs for Atomic SubSystem: '<S23>/Two phase CRL wrap' */
    /* SignalConversion: '<S18>/Signal Copy1' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S24>/a16'
     *  Product: '<S50>/acos'
     *  Product: '<S50>/bsin'
     *  Sum: '<S50>/sum_Ds'
     */
    Id_fb = Iab_fb[0] * COS + rtb_DTC_p * SIN;

    /* End of Outputs for SubSystem: '<S23>/Two phase CRL wrap' */
    /* End of Outputs for SubSystem: '<S20>/Two inputs CRL' */

    /* Sum: '<S18>/Sum1' incorporates:
     *  Constant: '<S27>/Constant4'
     *  DataTypeConversion: '<S27>/Cast To Single'
     *  Gain: '<S27>/Gain'
     *  Product: '<S27>/Product'
     *  Product: '<S27>/Product1'
     */
    wnq = ((real32_T)(0.0079942689836159844 * rtb_Integrator) +
           Vq_ref_beforeLimiter) + rtb_Integrator * Id_fb * 0.000375F;

    /* Sum: '<S28>/Sum' */
    Id_err = Idq_ref_PU[0] - Id_fb;

    /* MATLAB Function: '<S28>/MATLAB Function' incorporates:
     *  Constant: '<S28>/Constant'
     *  Constant: '<S28>/Constant1'
     */
    /* MATLAB Function 'CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/MATLAB Function': '<S44>:1' */
    /* '<S44>:1:3' */
    rtb_DeadZone *= 1697.77783F;

    /* '<S44>:1:4' */
    P = 1.414F * rtb_DeadZone * 0.000375F - 0.636666656F;

    /* '<S44>:1:5' */
    rtb_DeadZone = P / (rtb_DeadZone * rtb_DeadZone * 0.000375F);

    /* '<S44>:1:6' */
    /* '<S44>:1:7' */
    rtb_DeadZone *= P / rtb_DeadZone;

    /* MATLAB Function: '<S28>/MATLAB Function1' incorporates:
     *  Constant: '<S28>/Ki1'
     *  Constant: '<S28>/Ki2'
     *  MATLAB Function: '<S28>/MATLAB Function'
     */
    /* MATLAB Function 'CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/MATLAB Function1': '<S45>:1' */
    /* '<S45>:1:6' */
    /* '<S45>:1:7' */
    /* '<S45>:1:8' */
    /* '<S45>:1:9' */
    /* '<S45>:1:10' */
    /* '<S45>:1:11' */
    if ((!Foc_model_Matlab_ultimo_DW.integral_not_empty_p) || rtb_NOT) {
      /* '<S45>:1:14' */
      /* '<S45>:1:15' */
      Foc_model_Matlab_ultimo_DW.integral_j = 0.0F;
      Foc_model_Matlab_ultimo_DW.integral_not_empty_p = true;
    }

    /* '<S45>:1:19' */
    P *= Id_err;
    rtb_Integrator = rtb_DeadZone * Foc_model_Matlab_ultimo_DW.integral_j + P;
    if (((rtb_Integrator < 1.0F) && (rtb_Integrator > -1.0F)) ||
        ((rtb_Integrator >= 1.0F) && (Id_err < 0.0F)) || ((rtb_Integrator <=
          -1.0F) && (Id_err > 0.0F))) {
      /* '<S45>:1:22' */
      /* '<S45>:1:23' */
      /* '<S45>:1:24' */
      /* '<S45>:1:25' */
      Foc_model_Matlab_ultimo_DW.integral_j += Id_err;
    }

    /* '<S45>:1:29' */
    /* '<S45>:1:32' */
    Vd_ref_beforeLimiter = rtb_DeadZone * Foc_model_Matlab_ultimo_DW.integral_j
      + P;
    if (!(Vd_ref_beforeLimiter >= -1.0F)) {
      Vd_ref_beforeLimiter = -1.0F;
    }

    if (!(Vd_ref_beforeLimiter <= 1.0F)) {
      Vd_ref_beforeLimiter = 1.0F;
    }

    /* End of MATLAB Function: '<S28>/MATLAB Function1' */

    /* Sum: '<S18>/Sum' incorporates:
     *  Gain: '<S26>/Gain'
     *  Gain: '<S26>/Gain1'
     *  Product: '<S26>/Product'
     */
    rtb_DeadZone = Vd_ref_beforeLimiter - 2.0F * rtb_Merge_ik * Iq_fb *
      0.000435F;

    /* Sum: '<S33>/Sum1' incorporates:
     *  Product: '<S33>/Product'
     *  Product: '<S33>/Product1'
     */
    P = rtb_DeadZone * rtb_DeadZone + wnq * wnq;

    /* Outputs for IfAction SubSystem: '<S25>/D-Q Equivalence' incorporates:
     *  ActionPort: '<S30>/Action Port'
     */
    /* If: '<S30>/If' incorporates:
     *  If: '<S25>/If'
     *  RelationalOperator: '<S30>/Relational Operator'
     */
    if (P > 0.9025F) {
      /* Outputs for IfAction SubSystem: '<S30>/Limiter' incorporates:
       *  ActionPort: '<S34>/Action Port'
       */
      /* Sqrt: '<S34>/Square Root' */
      P = (real32_T)sqrt(P);

      /* Product: '<S34>/Divide' incorporates:
       *  Constant: '<S32>/Constant3'
       *  Product: '<S34>/Product'
       *  Switch: '<S32>/Switch'
       *  Switch: '<S34>/Switch'
       */
      rtb_DeadZone = rtb_DeadZone * 0.95F / P;
      wnq = wnq * 0.95F / P;

      /* End of Outputs for SubSystem: '<S30>/Limiter' */
    }

    /* End of If: '<S30>/If' */
    /* End of Outputs for SubSystem: '<S25>/D-Q Equivalence' */

    /* Outputs for Atomic SubSystem: '<S19>/Two inputs CRL' */
    /* Switch: '<S49>/Switch' incorporates:
     *  Product: '<S48>/dcos'
     *  Product: '<S48>/dsin'
     *  Product: '<S48>/qcos'
     *  Product: '<S48>/qsin'
     *  Sum: '<S48>/sum_alpha'
     *  Sum: '<S48>/sum_beta'
     */
    P = rtb_DeadZone * COS - wnq * SIN;
    wnq = wnq * COS + rtb_DeadZone * SIN;

    /* Gain: '<S66>/one_by_two' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S48>/a16'
     */
    rtb_Integrator = 0.5F * P;

    /* Gain: '<S66>/sqrt3_by_two' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S48>/a16'
     */
    rtb_Sum_o = 0.866025388F * wnq;

    /* End of Outputs for SubSystem: '<S19>/Two inputs CRL' */

    /* Sum: '<S66>/add_b' */
    rtb_DeadZone = rtb_Sum_o - rtb_Integrator;

    /* Sum: '<S66>/add_c' */
    rtb_Integrator = (0.0F - rtb_Integrator) - rtb_Sum_o;

    /* MinMax: '<S63>/Max' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S48>/a16'
     *  MinMax: '<S63>/Min'
     */
    rtb_NOT = rtIsNaNF(rtb_DeadZone);

    /* Outputs for Atomic SubSystem: '<S19>/Two inputs CRL' */
    if ((P >= rtb_DeadZone) || rtb_NOT) {
      rtb_Sum_o = P;
    } else {
      rtb_Sum_o = rtb_DeadZone;
    }

    /* MinMax: '<S63>/Min' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S48>/a16'
     */
    if ((P <= rtb_DeadZone) || rtb_NOT) {
      rtb_DTC = P;
    } else {
      rtb_DTC = rtb_DeadZone;
    }

    /* End of Outputs for SubSystem: '<S19>/Two inputs CRL' */

    /* MinMax: '<S63>/Max' incorporates:
     *  MinMax: '<S63>/Min'
     */
    rtb_NOT = !rtIsNaNF(rtb_Integrator);
    if ((!(rtb_Sum_o >= rtb_Integrator)) && rtb_NOT) {
      rtb_Sum_o = rtb_Integrator;
    }

    /* MinMax: '<S63>/Min' */
    if ((!(rtb_DTC <= rtb_Integrator)) && rtb_NOT) {
      rtb_DTC = rtb_Integrator;
    }

    /* Gain: '<S63>/one_by_two' incorporates:
     *  MinMax: '<S63>/Max'
     *  MinMax: '<S63>/Min'
     *  Sum: '<S63>/Add'
     */
    rtb_DTC = (rtb_Sum_o + rtb_DTC) * -0.5F;

    /* Outputs for Atomic SubSystem: '<S19>/Two inputs CRL' */
    /* Gain: '<S13>/Gain' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S48>/a16'
     *  Constant: '<S13>/Constant1'
     *  Gain: '<S62>/Gain'
     *  Sum: '<S13>/Sum1'
     *  Sum: '<S62>/Add1'
     *  Sum: '<S62>/Add2'
     *  Sum: '<S62>/Add3'
     */
    PWM_Duty_Cycles[0] = ((P + rtb_DTC) * 1.15470052F + 1.0F) * 0.5F;

    /* End of Outputs for SubSystem: '<S19>/Two inputs CRL' */
    PWM_Duty_Cycles[1] = ((rtb_DeadZone + rtb_DTC) * 1.15470052F + 1.0F) * 0.5F;
    PWM_Duty_Cycles[2] = ((rtb_DTC + rtb_Integrator) * 1.15470052F + 1.0F) *
      0.5F;

    /* DataTypeConversion: '<S13>/Data Type Conversion' incorporates:
     *  DataStoreRead: '<S13>/Enable'
     */
    PWM_Enable = (real32_T)Enable;

    /* Switch: '<S11>/Switch1' */
    if (PWM_Enable >= 0.5F) {
      /* Switch: '<S11>/Switch1' */
      PWM[0] = PWM_Duty_Cycles[0];
      PWM[1] = PWM_Duty_Cycles[1];
      PWM[2] = PWM_Duty_Cycles[2];
    } else {
      /* Switch: '<S11>/Switch1' incorporates:
       *  Constant: '<S11>/stop'
       */
      PWM[0] = 0.0F;
      PWM[1] = 0.0F;
      PWM[2] = 0.0F;
    }

    /* End of Switch: '<S11>/Switch1' */

    /* S-Function (ftm_s32k_pwm_config): '<S11>/FTM_PWM_Config' */
    {
      uint16_t dutyA = FTM_MAX_DUTY_CYCLE * PWM[0];
      FTM_DRV_UpdatePwmChannel(FTM_PWM3, 0U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, dutyA,
        0, true);
    }

    {
      uint16_t dutyA = FTM_MAX_DUTY_CYCLE * PWM[1];
      FTM_DRV_UpdatePwmChannel(FTM_PWM3, 2U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, dutyA,
        0, true);
    }

    {
      uint16_t dutyA = FTM_MAX_DUTY_CYCLE * PWM[2];
      FTM_DRV_UpdatePwmChannel(FTM_PWM3, 4U, FTM_PWM_UPDATE_IN_DUTY_CYCLE, dutyA,
        0, true);
    }

    /* S-Function (fm_s32k_recorder_call): '<S11>/FreeMaster_Recorder_Call' */
    FMSTR_Recorder();

    /* SignalConversion generated from: '<S1>/Speed_fb' */
    SpeedMeasured = rtb_Merge_ik;

    /* Step: '<S1>/Step' incorporates:
     *  Step: '<S1>/Step1'
     */
    rtb_Step = !(((Foc_model_Matlab_ultimo_M->Timing.clockTick1) * 0.0001) < 0.0);

    /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator2' */
    if (Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator2_SYSTEM_ != 0) {
      /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator2' */
      rtb_Merge_ik = Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator2_DSTATE;
    } else {
      /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator2' */
      rtb_Merge_ik = 0.0001F * (real32_T)CurrentControl_ELAPS_T
        * Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator2_PREV_U +
        Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator2_DSTATE;
    }

    /* End of DiscreteIntegrator: '<S14>/Discrete-Time Integrator2' */

    /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator3' */
    if (Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator3_SYSTEM_ != 0) {
      /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator3' */
      rtb_Integrator = Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator3_DSTATE;
    } else {
      /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator3' */
      rtb_Integrator = 0.0001F * (real32_T)CurrentControl_ELAPS_T
        * Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator3_PREV_U +
        Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator3_DSTATE;
    }

    /* End of DiscreteIntegrator: '<S14>/Discrete-Time Integrator3' */

    /* MATLAB Function: '<S14>/Saturation ' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S24>/a16'
     *  Constant: '<S14>/epsilona'
     *  Constant: '<S14>/epsilonb'
     *  MATLAB Function: '<S14>/Back-EMF Estimation'
     *  Product: '<S1>/Product'
     */
    /* MATLAB Function 'CurrentControl/STSMO/Saturation ': '<S131>:1' */
    /* '<S131>:1:7' */
    rtb_Sum_o = rtb_Merge_ik - (real32_T)(rtb_algDD_o1_f * (real_T)rtb_Step);

    /* Outputs for Atomic SubSystem: '<S23>/Two phase CRL wrap' */
    /* '<S131>:1:8' */
    rtb_algDD_o1_f = rtb_Integrator - (real32_T)(rtb_DTC_p * (real_T)rtb_Step);

    /* End of Outputs for SubSystem: '<S23>/Two phase CRL wrap' */
    rtb_DeadZone = (real32_T)fabs(rtb_Sum_o);
    if (rtb_DeadZone > 0.0001) {
      /* '<S131>:1:10' */
      /* '<S131>:1:11' */
      if (rtIsNaNF(rtb_Sum_o)) {
        rtb_DTC_p = (rtNaNF);
      } else if (rtb_Sum_o < 0.0F) {
        rtb_DTC_p = -1.0F;
      } else {
        rtb_DTC_p = (real32_T)(rtb_Sum_o > 0.0F);
      }
    } else {
      /* '<S131>:1:13' */
      rtb_DTC_p = rtb_DeadZone / 0.0001F;
    }

    rtb_Sum_o = (real32_T)fabs(rtb_algDD_o1_f);
    if (rtb_Sum_o > 0.0001) {
      /* '<S131>:1:16' */
      /* '<S131>:1:17' */
      if (rtIsNaNF(rtb_algDD_o1_f)) {
        rtb_algDD_o1_f = (rtNaNF);
      } else if (rtb_algDD_o1_f < 0.0F) {
        rtb_algDD_o1_f = -1.0F;
      } else {
        rtb_algDD_o1_f = (real32_T)(rtb_algDD_o1_f > 0.0F);
      }
    } else {
      /* '<S131>:1:19' */
      rtb_algDD_o1_f = rtb_Sum_o / 0.0001F;
    }

    /* End of MATLAB Function: '<S14>/Saturation ' */

    /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator1' */
    if (Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator1_SYSTEM_ != 0) {
      /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator1' */
      int_sat_ia = Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator1_DSTATE;
    } else {
      /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator1' */
      int_sat_ia = 0.0001F * (real32_T)CurrentControl_ELAPS_T
        * Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator1_PREV_U +
        Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator1_DSTATE;
    }

    /* End of DiscreteIntegrator: '<S14>/Discrete-Time Integrator1' */

    /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator' */
    if (Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator_SYSTEM_E != 0) {
      /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator' */
      int_sat_ib = Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator_DSTATE_l;
    } else {
      /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator' */
      int_sat_ib = 0.0001F * (real32_T)CurrentControl_ELAPS_T
        * Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator_PREV_U +
        Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator_DSTATE_l;
    }

    /* End of DiscreteIntegrator: '<S14>/Discrete-Time Integrator' */

    /* MATLAB Function: '<S14>/Back-EMF Estimation' */
    /* MATLAB Function 'CurrentControl/STSMO/Back-EMF Estimation': '<S129>:1' */
    /* '<S129>:1:9' */
    z_alpha = Foc_model_Matlab_ultimo_ConstB.CastToSingle6 * (real32_T)sqrt
      (rtb_DeadZone) * rtb_DTC_p + Foc_model_Matlab_ultimo_ConstB.CastToSingle7 *
      int_sat_ia;

    /* '<S129>:1:10' */
    z_beta = Foc_model_Matlab_ultimo_ConstB.CastToSingle6 * (real32_T)sqrt
      (rtb_Sum_o) * rtb_algDD_o1_f +
      Foc_model_Matlab_ultimo_ConstB.CastToSingle7 * int_sat_ib;

    /* Sum: '<S14>/Sum' incorporates:
     *  MATLAB Function: '<S14>/Back-EMF Estimation'
     *  Product: '<S14>/Product'
     *  Product: '<S14>/Product1'
     *  Trigonometry: '<S14>/Cos'
     *  Trigonometry: '<S14>/Sin'
     */
    /* '<S129>:1:12' */
    /* '<S129>:1:13' */
    /* '<S129>:1:15' */
    /* '<S129>:1:16' */
    rtb_DTC = (0.0F - z_alpha * (real32_T)cos(rtb_Merge1)) - z_beta * (real32_T)
      sin(rtb_Merge1);

    /* Logic: '<S14>/NOT' incorporates:
     *  DataStoreRead: '<S14>/Data Store Read'
     */
    rtb_NOT = !Enable;

    /* Constant: '<S14>/Ki2' */
    Foc_model_Matlab_ultimo_B.Ki2 = 0.0F;

    /* DiscreteIntegrator: '<S168>/Integrator' incorporates:
     *  Logic: '<S14>/NOT'
     */
    if (rtb_NOT || (Foc_model_Matlab_ultimo_DW.Integrator_PrevResetState != 0))
    {
      Foc_model_Matlab_ultimo_DW.Integrator_DSTATE =
        Foc_model_Matlab_ultimo_B.Ki2;
    }

    /* Sum: '<S177>/Sum' incorporates:
     *  DiscreteIntegrator: '<S168>/Integrator'
     *  Product: '<S173>/PProd Out'
     */
    rtb_Sum_o = rtb_DTC * Foc_model_Matlab_ultimo_ConstB.CastToSingle1 +
      Foc_model_Matlab_ultimo_DW.Integrator_DSTATE;

    /* DeadZone: '<S160>/DeadZone' */
    if (rtb_Sum_o > 1.0F) {
      rtb_DeadZone = rtb_Sum_o - 1.0F;
    } else if (rtb_Sum_o >= -1.0F) {
      rtb_DeadZone = 0.0F;
    } else {
      rtb_DeadZone = rtb_Sum_o - -1.0F;
    }

    /* End of DeadZone: '<S160>/DeadZone' */

    /* Product: '<S165>/IProd Out' */
    rtb_DTC *= Foc_model_Matlab_ultimo_ConstB.CastToSingle;

    /* Gain: '<S16>/PositionToCount' incorporates:
     *  Constant: '<S1>/Constant2'
     *  Gain: '<S1>/Get_PU1'
     *  Math: '<S1>/Mod'
     */
    CurrentControl_ELAPS_T = (uint32_T)(0.0795774683F * rt_modf_snf(rtb_Merge1,
      12.566371F) * 4.2949673E+9F);

    /* Gain: '<S16>/SpeedGain' incorporates:
     *  DataTypeConversion: '<S188>/DTC'
     *  Delay: '<S16>/Delay'
     *  Sum: '<S16>/SpeedCount'
     */
    Foc_model_Matlab_ultimo_B.SpeedGain = (real32_T)((int32_T)
      CurrentControl_ELAPS_T - (int32_T)
      Foc_model_Matlab_ultimo_DW.Delay_DSTATE[Foc_model_Matlab_ultimo_DW.CircBufIdx])
      * 6.98491931E-9F;

    /* Update for Delay: '<S79>/Delay One Step1' */
    Foc_model_Matlab_ultimo_DW.DelayOneStep1_DSTATE = rtb_Compare;

    /* Update for Delay: '<S79>/Delay One Step' incorporates:
     *  Constant: '<S79>/Constant2'
     *  Sum: '<S79>/Sum'
     */
    if (OR) {
      Foc_model_Matlab_ultimo_DW.DelayOneStep_DSTATE = (uint16_T)
        (rtb_DataStoreRead5 - 1);
    }

    /* End of Update for Delay: '<S79>/Delay One Step' */

    /* Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator4' */
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator4_SYSTEM_ = 0U;
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator4_DSTATE = rtb_Merge1_p;

    /* Saturate: '<S175>/Saturation' */
    if (rtb_Sum_o > 1.0F) {
      /* Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator4' */
      Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator4_PREV_U = 1.0F;
    } else if (rtb_Sum_o < -1.0F) {
      /* Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator4' */
      Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator4_PREV_U = -1.0F;
    } else {
      /* Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator4' */
      Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator4_PREV_U = rtb_Sum_o;
    }

    /* End of Saturate: '<S175>/Saturation' */

    /* Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator2' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S48>/a16'
     *  MATLAB Function: '<S14>/Back-EMF Estimation'
     *  Product: '<S1>/Product'
     */
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator2_SYSTEM_ = 0U;
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator2_DSTATE = rtb_Merge_ik;

    /* Outputs for Atomic SubSystem: '<S19>/Two inputs CRL' */
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator2_PREV_U = ((real32_T)(P *
      (real_T)rtb_Step * 2666.6666666666665) + -1697.8667F * rtb_Merge_ik) -
      2666.66675F * z_alpha;

    /* End of Outputs for SubSystem: '<S19>/Two inputs CRL' */

    /* Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator3' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S48>/a16'
     *  MATLAB Function: '<S14>/Back-EMF Estimation'
     *  Product: '<S1>/Product'
     */
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator3_SYSTEM_ = 0U;
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator3_DSTATE = rtb_Integrator;

    /* Outputs for Atomic SubSystem: '<S19>/Two inputs CRL' */
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator3_PREV_U = ((real32_T)(wnq *
      (real_T)rtb_Step * 2666.6666666666665) + -1697.8667F * rtb_Integrator) -
      2666.66675F * z_beta;

    /* End of Outputs for SubSystem: '<S19>/Two inputs CRL' */

    /* Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator1' */
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator1_SYSTEM_ = 0U;
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator1_DSTATE = int_sat_ia;
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator1_PREV_U = rtb_DTC_p;

    /* Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator' */
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator_SYSTEM_E = 0U;
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator_DSTATE_l = int_sat_ib;
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator_PREV_U = rtb_algDD_o1_f;

    /* Switch: '<S158>/Switch1' incorporates:
     *  Constant: '<S158>/Clamping_zero'
     *  Constant: '<S158>/Constant'
     *  Constant: '<S158>/Constant2'
     *  RelationalOperator: '<S158>/fix for DT propagation issue'
     */
    if (rtb_DeadZone > 0.0F) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    /* Switch: '<S158>/Switch2' incorporates:
     *  Constant: '<S158>/Clamping_zero'
     *  Constant: '<S158>/Constant3'
     *  Constant: '<S158>/Constant4'
     *  RelationalOperator: '<S158>/fix for DT propagation issue1'
     */
    if (rtb_DTC > 0.0F) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    /* Switch: '<S158>/Switch' incorporates:
     *  Constant: '<S158>/Clamping_zero'
     *  Constant: '<S158>/Constant1'
     *  Logic: '<S158>/AND3'
     *  RelationalOperator: '<S158>/Equal1'
     *  RelationalOperator: '<S158>/Relational Operator'
     *  Switch: '<S158>/Switch1'
     *  Switch: '<S158>/Switch2'
     */
    if ((rtb_DeadZone != 0.0F) && (tmp == tmp_0)) {
      rtb_DTC = 0.0F;
    }

    /* Update for DiscreteIntegrator: '<S168>/Integrator' incorporates:
     *  Logic: '<S14>/NOT'
     *  Switch: '<S158>/Switch'
     */
    Foc_model_Matlab_ultimo_DW.Integrator_DSTATE += rtb_DTC;
    Foc_model_Matlab_ultimo_DW.Integrator_PrevResetState = (int8_T)rtb_NOT;

    /* Update for Delay: '<S16>/Delay' */
    Foc_model_Matlab_ultimo_DW.Delay_DSTATE[Foc_model_Matlab_ultimo_DW.CircBufIdx]
      = CurrentControl_ELAPS_T;
    if (Foc_model_Matlab_ultimo_DW.CircBufIdx < 9U) {
      Foc_model_Matlab_ultimo_DW.CircBufIdx++;
    } else {
      Foc_model_Matlab_ultimo_DW.CircBufIdx = 0U;
    }

    /* End of Update for Delay: '<S16>/Delay' */

    /* user code (Output function Trailer) */

    /* Profile Code : Compute function execution time in us. */
    tmp2 = profiler_get_cnt();
    profile_buffer[0] = gt_pf(tmp1, tmp2);

    /* End of Profile Code */
  }
}

/* Output and update for atomic system: '<Root>/SpeedControl' */
void Foc_model_Matl_SpeedControl(void)
{
  real32_T rtb_SpeedError;
  real32_T rtb_TSamp;
  real32_T s;

  /* user code (Output function Body) */
  {
    /* Start of Profile Code */
    uint32_t tmp1;
    uint32_t tmp2;
    tmp1 = profiler_get_cnt();

    /* Start Profiling This Function.*/

    /* Outputs for Atomic SubSystem: '<S8>/Speed_Ref_Selector' */
    /* Switch: '<S222>/Switch' incorporates:
     *  DataStoreRead: '<S222>/Data Store Read1'
     *  DataTypeConversion: '<S222>/Data Type Conversion'
     */
    if ((real32_T)Enable > 0.5F) {
      /* Switch: '<S222>/Switch' */
      Speed_Ref = Speed_Ref_PU;
    } else {
      /* Switch: '<S222>/Switch' incorporates:
       *  Switch: '<S8>/Switch'
       */
      Speed_Ref = Speed_fb;
    }

    /* End of Switch: '<S222>/Switch' */
    /* End of Outputs for SubSystem: '<S8>/Speed_Ref_Selector' */

    /* Constant: '<S8>/Id_ref' */
    Foc_model_Matlab_ultimo_B.Id_ref = 0.0F;

    /* Outputs for Atomic SubSystem: '<S8>/STSMC' */
    /* SampleTimeMath: '<S223>/TSamp'
     *
     * About '<S223>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     *   */
    rtb_TSamp = Speed_Ref * 1000.0F;

    /* Sum: '<S221>/Sum' incorporates:
     *  Switch: '<S8>/Switch'
     */
    rtb_SpeedError = Speed_fb - Speed_Ref;

    /* MATLAB Function: '<S221>/MATLAB Function' incorporates:
     *  Constant: '<S221>/Constant10'
     *  Constant: '<S221>/Constant8'
     *  Constant: '<S221>/Constant9'
     *  DataStoreRead: '<S221>/Data Store Read2'
     *  DataStoreRead: '<S221>/Data Store Read3'
     *  DataStoreRead: '<S221>/Data Store Read4'
     *  DiscreteIntegrator: '<S221>/Discrete-Time Integrator'
     *  Sum: '<S223>/Diff'
     *  Switch: '<S8>/Switch'
     *  UnitDelay: '<S223>/UD'
     *
     * Block description for '<S223>/Diff':
     *
     *  Add in CPU
     *
     * Block description for '<S223>/UD':
     *
     *  Store in Global RAM
     */
    /* MATLAB Function 'SpeedControl/STSMC/MATLAB Function': '<S224>:1' */
    /* '<S224>:1:5' */
    s = Lambda * Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator_DSTATE +
      rtb_SpeedError;
    if ((real32_T)fabs(s) > Epsilon) {
      /* '<S224>:1:8' */
      /* '<S224>:1:9' */
      if (rtIsNaNF(s)) {
        s = (rtNaNF);
      } else if (s < 0.0F) {
        s = -1.0F;
      } else {
        s = (real32_T)(s > 0.0F);
      }

      s *= -Foc_model_Matlab_ultimo_DW.Rho;
    } else {
      /* '<S224>:1:11' */
      s = s / Epsilon * -Foc_model_Matlab_ultimo_DW.Rho;
    }

    /* '<S224>:1:15' */
    /* '<S224>:1:16' */
    /* '<S224>:1:20' */
    Foc_model_Matlab_ultimo_B.I_ref = ((((rtb_TSamp -
      Foc_model_Matlab_ultimo_DW.UD_DSTATE) + Lambda * rtb_SpeedError) * 1.2E-5F
      + 1.0E-7F * Speed_fb) + s) / 0.097F;
    if (!(Foc_model_Matlab_ultimo_B.I_ref >= -0.5F)) {
      Foc_model_Matlab_ultimo_B.I_ref = -0.5F;
    }

    if (!(Foc_model_Matlab_ultimo_B.I_ref <= 0.5F)) {
      Foc_model_Matlab_ultimo_B.I_ref = 0.5F;
    }

    /* End of MATLAB Function: '<S221>/MATLAB Function' */

    /* Update for UnitDelay: '<S223>/UD'
     *
     * Block description for '<S223>/UD':
     *
     *  Store in Global RAM
     */
    Foc_model_Matlab_ultimo_DW.UD_DSTATE = rtb_TSamp;

    /* Update for DiscreteIntegrator: '<S221>/Discrete-Time Integrator' */
    Foc_model_Matlab_ultimo_DW.DiscreteTimeIntegrator_DSTATE += 0.001F *
      rtb_SpeedError;

    /* End of Outputs for SubSystem: '<S8>/STSMC' */

    /* user code (Output function Trailer) */

    /* Profile Code : Compute function execution time in us. */
    tmp2 = profiler_get_cnt();
    profile_buffer[1] = gt_pf(tmp1, tmp2);

    /* End of Profile Code */
  }
}

/* Model step function for TID0 */
void Foc_model_Matlab_ultimo_step0(void) /* Sample time: [5.0E-5s, 0.0s] */
{
  {                                    /* Sample time: [5.0E-5s, 0.0s] */
    rate_monotonic_scheduler();
  }
}

/* Model step function for TID1 */
void Foc_model_Matlab_ultimo_step1(void) /* Sample time: [0.0001s, 0.0s] */
{
  int32_T tmp;

  /* End of Outputs for S-Function (ftm_s32k_hall_sensor): '<S3>/FTM_Hall_Sensor' */

  /* DataStoreWrite: '<S3>/Data Store Write1' */
  HallCntActual = CntHall;

  /* RateTransition: '<Root>/RT11' */
  tmp = Foc_model_Matlab_ultimo_DW.RT11_ActiveBufIdx << 1;
  Idq_ref_PU[0] = Foc_model_Matlab_ultimo_DW.RT11_Buffer[tmp];
  Idq_ref_PU[1] = Foc_model_Matlab_ultimo_DW.RT11_Buffer[tmp + 1];

  /* End of Outputs for S-Function (pdb_s32k_isr): '<S208>/PDB1_ISR' */
  /* End of Outputs for S-Function (adc_s32k_isr): '<S208>/ADC1_ISR' */

  /* DataStoreWrite: '<S208>/Data Store Write' incorporates:
   *  DataTypeConversion: '<S208>/Data Type Conversion'
   */
  ADC_A = (real32_T)ADC_IA;

  /* DataStoreWrite: '<S208>/Data Store Write1' incorporates:
   *  DataTypeConversion: '<S208>/Data Type Conversion'
   */
  ADC_B = (real32_T)ADC_IB;

  /* End of Outputs for SubSystem: '<S3>/Subsystem2' */
  /* S-Function (adc_s32k_start): '<S4>/ADC1_IRQ' */
  {
  }

  /* If: '<S4>/If' incorporates:
   *  DataStoreRead: '<S4>/Data Store Read'
   */
  if (!FAULT) {
    /* Outputs for IfAction SubSystem: '<S4>/If Action Subsystem' incorporates:
     *  ActionPort: '<S217>/Action Port'
     */
    /* S-Function (gpio_s32k_output): '<S217>/LED_GREEN_ON' incorporates:
     *  Constant: '<S217>/LED_GREEN'
     */

    /* GPOPORTD16 Data Signal Update */
    if (false) {
      PINS_DRV_SetPins(PTD, 1UL<<16);
    } else {
      PINS_DRV_ClearPins(PTD, 1UL<<16);
    }

    /* S-Function (gpio_s32k_output): '<S217>/LED_RED_OFF' incorporates:
     *  Constant: '<S217>/LED_RED'
     */

    /* GPOPORTD15 Data Signal Update */
    if (true) {
      PINS_DRV_SetPins(PTD, 1UL<<15);
    } else {
      PINS_DRV_ClearPins(PTD, 1UL<<15);
    }

    /* End of Outputs for SubSystem: '<S4>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S4>/FAULT' incorporates:
     *  ActionPort: '<S215>/Action Port'
     */
    /* S-Function (gpio_s32k_output): '<S215>/LED_GREEN' incorporates:
     *  Constant: '<S215>/OFF'
     */

    /* GPOPORTD16 Data Signal Update */
    if (true) {
      PINS_DRV_SetPins(PTD, 1UL<<16);
    } else {
      PINS_DRV_ClearPins(PTD, 1UL<<16);
    }

    /* S-Function (gpio_s32k_output): '<S215>/LED_RED' incorporates:
     *  Constant: '<S215>/ON'
     */

    /* GPOPORTD15 Data Signal Update */
    if (false) {
      PINS_DRV_SetPins(PTD, 1UL<<15);
    } else {
      PINS_DRV_ClearPins(PTD, 1UL<<15);
    }

    /* End of Outputs for SubSystem: '<S4>/FAULT' */
  }

  /* End of If: '<S4>/If' */

  /* Chart: '<S4>/Enable PDB and start FTM' */
  /* Gateway: Hardware Initialization/Enable PDB and start FTM */
  /* During: Hardware Initialization/Enable PDB and start FTM */
  if (Foc_model_Matlab_ultimo_DW.is_active_c1_Foc_model_Matlab_u == 0) {
    /* Entry: Hardware Initialization/Enable PDB and start FTM */
    Foc_model_Matlab_ultimo_DW.is_active_c1_Foc_model_Matlab_u = 1U;

    /* Entry Internal: Hardware Initialization/Enable PDB and start FTM */
    /* Transition: '<S214>:10' */
    Foc_model_Matlab_ultimo_DW.is_c1_Foc_model_Matlab_ultimo =
      Foc_model_Matlab_ultimo_IN_A;
  } else if (Foc_model_Matlab_ultimo_DW.is_c1_Foc_model_Matlab_ultimo ==
             Foc_model_Matlab_ultimo_IN_A) {
    /* Outputs for Function Call SubSystem: '<S4>/enable_FTM_PDB_ADC_triggering' */
    /* During 'A': '<S214>:5' */
    /* Transition: '<S214>:103' */
    /* Event: '<S214>:104' */

    /* S-Function (ftm_s32k_pwm_disen): '<S218>/FTM_PWM_Disable_Enable' */
    FTM_DRV_InitPwm(FTM_PWM3, &flexTimer_pwm3_PwmConfig);

    /* S-Function (ftm_s32k_init_disen): '<S218>/FTM_Init_Trigger_Disable_Enable' */

    /* FTM PWM Initialization Trigger Enable Disable*/
    FTM_DRV_SetInitTriggerCmd(FTM3, true);

    /* S-Function (pdb_s32k_enable): '<S218>/PDB0_Enable' */

    /* Enable PDB Module0 */
    PDB_DRV_Enable(0);

    /* S-Function (pdb_s32k_enable): '<S218>/PDB1_Enable' */

    /* Enable PDB Module1 */
    PDB_DRV_Enable(1);

    /* S-Function (tpp_s32k_isr_enable): '<S218>/TPP_ISR_Enable_Disable' */
    tpp_interrupt_enable(15);

    /* user code (Output function Trailer) */

    /* System '<S4>/enable_FTM_PDB_ADC_triggering' */
    PDB_DRV_LoadValuesCmd(0);
    PDB_DRV_LoadValuesCmd(1);

    /* End of Outputs for SubSystem: '<S4>/enable_FTM_PDB_ADC_triggering' */
    Foc_model_Matlab_ultimo_DW.is_c1_Foc_model_Matlab_ultimo =
      Foc_model_Matlab_ultimo_IN_END;
  } else {
    /* During 'END': '<S214>:39' */
    /* Transition: '<S214>:41' */
    /* Event: '<S214>:36' */
    Foc_model_Matlab_ultimo_DW.is_c1_Foc_model_Matlab_ultimo =
      Foc_model_Matlab_ultimo_IN_END;
  }

  /* End of Chart: '<S4>/Enable PDB and start FTM' */
  /* End of Outputs for S-Function (tpp_s32k_isr): '<S4>/GD300_ISR_Callback ' */

  /* Update absolute time */
  /* The "clockTick1" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.0001, which is the step size
   * of the task. Size of "clockTick1" ensures timer will not overflow during the
   * application lifespan selected.
   */
  Foc_model_Matlab_ultimo_M->Timing.clockTick1++;
}

/* Model step function for TID2 */
void Foc_model_Matlab_ultimo_step2(void) /* Sample time: [0.001s, 0.0s] */
{
  /* RateTransition: '<Root>/RT2' */
  Speed_Ref_PU = Foc_model_Matlab_ultimo_DW.RT2_Buffer0;

  /* RateTransition: '<Root>/RT1' */
  Speed_fb = SpeedMeasured;

  /* RateTransition: '<Root>/RT3' */
  Foc_model_Matlab_ultimo_B.RT3 = Foc_model_Matlab_ultimo_B.SpeedGain;

  /* Outputs for Atomic SubSystem: '<Root>/SpeedControl' */
  Foc_model_Matl_SpeedControl();

  /* End of Outputs for SubSystem: '<Root>/SpeedControl' */

  /* RateTransition: '<Root>/RT11' */
  Foc_model_Matlab_ultimo_DW.RT11_Buffer
    [(Foc_model_Matlab_ultimo_DW.RT11_ActiveBufIdx == 0) << 1] =
    Foc_model_Matlab_ultimo_B.Id_ref;
  Foc_model_Matlab_ultimo_DW.RT11_Buffer[1 +
    ((Foc_model_Matlab_ultimo_DW.RT11_ActiveBufIdx == 0) << 1)] =
    Foc_model_Matlab_ultimo_B.I_ref;
  Foc_model_Matlab_ultimo_DW.RT11_ActiveBufIdx = (int8_T)
    (Foc_model_Matlab_ultimo_DW.RT11_ActiveBufIdx == 0);

  /* Update absolute time */
  /* The "clockTick2" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.001, which is the step size
   * of the task. Size of "clockTick2" ensures timer will not overflow during the
   * application lifespan selected.
   */
  Foc_model_Matlab_ultimo_M->Timing.clockTick2++;
}

/* Model step function for TID3 */
void Foc_model_Matlab_ultimo_step3(void) /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcgen): '<S3>/SCI_Rx_INT' incorporates:
   *  SubSystem: '<Root>/Serial Receive'
   */
  /* RateTransition: '<Root>/RT2' incorporates:
   *  DataStoreRead: '<S220>/Data Store Read2'
   *  Gain: '<S220>/rpm2PU'
   */
  Foc_model_Matlab_ultimo_DW.RT2_Buffer0 = 0.0005F * DesiredSpeed;

  /* End of Outputs for S-Function (fcgen): '<S3>/SCI_Rx_INT' */
}

/* Model initialize function */
void Foc_model_Matlab_ultimo_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* Start for S-Function (pdb_s32k_config): '<S4>/PDB0_Init' */
  trgmuxAllMappingConfig[0] = pdb0MappingConfig;
  trgmuxAllMappingConfig[1] = pdb1MappingConfig;

  /* Initializes TRGMUX instance for operation. */
  TRGMUX_DRV_Init(0, &pdbTrgmuxUserConfig);

  /* Set PDB0 clock source */
  PCC_SetPeripheralClockControl(PCC, PDB0_CLK, true, CLK_SRC_SPLL, 0, 0);

  /* Enable clock for PDB0 */
  PCC_SetClockMode(PCC, PDB0_CLK, true);

  /* Initialize PDB0 driver. */
  PDB_DRV_Init(0, &pdb0TimerConfig);

  /* Set the value to PDB modulus register */
  PDB_DRV_SetTimerModulusValue(0, 5000U);

  /* Configure the ADC pre_trigger 0U in the PDB0 module */
  PDB_DRV_ConfigAdcPreTrigger(0, 0U, &pdb0Ch0UPreTrigConfig0U);

  /* Set the ADC pre_trigger 0U delay value in the PDB0 module */
  PDB_DRV_SetAdcPreTriggerDelayValue(0, 0U, 0U, 2000U);

  /* Command the PDB instance to load the fresh values */
  PDB_DRV_LoadValuesCmd(0);

  /* Start for S-Function (pdb_s32k_config): '<S4>/PDB1_Init' */
  trgmuxAllMappingConfig[0] = pdb0MappingConfig;
  trgmuxAllMappingConfig[1] = pdb1MappingConfig;

  /* Initializes TRGMUX instance for operation. */
  TRGMUX_DRV_Init(0, &pdbTrgmuxUserConfig);

  /* Set PDB1 clock source */
  PCC_SetPeripheralClockControl(PCC, PDB1_CLK, true, CLK_SRC_SPLL, 0, 0);

  /* Enable clock for PDB1 */
  PCC_SetClockMode(PCC, PDB1_CLK, true);

  /* Initialize PDB1 driver. */
  PDB_DRV_Init(1, &pdb1TimerConfig);

  /* Set the value to PDB modulus register */
  PDB_DRV_SetTimerModulusValue(1, 5000U);

  /* Configure the ADC pre_trigger 0U in the PDB1 module */
  PDB_DRV_ConfigAdcPreTrigger(1, 0U, &pdb1Ch0UPreTrigConfig0U);

  /* Set the ADC pre_trigger 0U delay value in the PDB1 module */
  PDB_DRV_SetAdcPreTriggerDelayValue(1, 0U, 0U, 0U);

  /* Configure the ADC pre_trigger 1U in the PDB1 module */
  PDB_DRV_ConfigAdcPreTrigger(1, 0U, &pdb1Ch0UPreTrigConfig1U);

  /* Set the ADC pre_trigger 1U delay value in the PDB1 module */
  PDB_DRV_SetAdcPreTriggerDelayValue(1, 0U, 1U, 1000U);

  /* Configure the ADC pre_trigger 2U in the PDB1 module */
  PDB_DRV_ConfigAdcPreTrigger(1, 0U, &pdb1Ch0UPreTrigConfig2U);

  /* Set the ADC pre_trigger 2U delay value in the PDB1 module */
  PDB_DRV_SetAdcPreTriggerDelayValue(1, 0U, 2U, 2000U);

  /* Command the PDB instance to load the fresh values */
  PDB_DRV_LoadValuesCmd(1);

  /* Start for S-Function (adc_s32k_config): '<S4>/ADC0_Init' */
  {
    const adc_converter_config_t adc0_cfg = {
      .clockDivide = ADC_CLK_DIVIDE_1,
      .sampleTime = 1.0,
      .resolution = ADC_RESOLUTION_12BIT,
      .inputClock = ADC_CLK_ALT_1,
      .trigger = ADC_TRIGGER_HARDWARE,
      .pretriggerSel = ADC_PRETRIGGER_SEL_PDB,
      .triggerSel = ADC_TRIGGER_SEL_PDB,
      .dmaEnable = false,
      .voltageRef = ADC_VOLTAGEREF_VREF,
      .continuousConvEnable = false,
      .supplyMonitoringEnable = false
    };

    const adc_compare_config_t adc0_cmp_cfg = {
      .compareEnable = false,
      .compareGreaterThanEnable = false,
      .compareRangeFuncEnable = false,
      .compVal1 = 0,
      .compVal2 = 0
    };

    const adc_average_config_t adc0_avrg_cfg = {
      .hwAvgEnable = false,
      .hwAverage = ADC_AVERAGE_4
    };

    /* Enable ADC0 clock */
    PCC_SetClockMode(PCC, PCC_ADC0_CLOCK, false);

    /* Set ADC0 clock source */
    PCC_SetPeripheralClockControl(PCC, PCC_ADC0_CLOCK, true, CLK_SRC_SPLL, 0, 0);

    /* Enable ADC0 clock */
    PCC_SetClockMode(PCC, PCC_ADC0_CLOCK, true);
    ADC_DRV_Reset(0);

    /* Configure ADC0 */
    ADC_DRV_ConfigConverter(0, &adc0_cfg);
    ADC_DRV_SetSwPretrigger(0,ADC_SW_PRETRIGGER_DISABLED);
    ADC_DRV_ConfigHwCompare(0, &adc0_cmp_cfg);
    ADC_DRV_ConfigHwAverage(0, &adc0_avrg_cfg);

    /* Do calibration before initialize the ADC0. */
    ADC_DRV_AutoCalibration(0);
  }

  /* Start for S-Function (adc_s32k_config): '<S4>/ADC1_Init' */
  {
    const adc_converter_config_t adc1_cfg = {
      .clockDivide = ADC_CLK_DIVIDE_1,
      .sampleTime = 1.0,
      .resolution = ADC_RESOLUTION_12BIT,
      .inputClock = ADC_CLK_ALT_1,
      .trigger = ADC_TRIGGER_HARDWARE,
      .pretriggerSel = ADC_PRETRIGGER_SEL_PDB,
      .triggerSel = ADC_TRIGGER_SEL_PDB,
      .dmaEnable = false,
      .voltageRef = ADC_VOLTAGEREF_VREF,
      .continuousConvEnable = false,
      .supplyMonitoringEnable = false
    };

    const adc_compare_config_t adc1_cmp_cfg = {
      .compareEnable = false,
      .compareGreaterThanEnable = false,
      .compareRangeFuncEnable = false,
      .compVal1 = 0,
      .compVal2 = 0
    };

    const adc_average_config_t adc1_avrg_cfg = {
      .hwAvgEnable = false,
      .hwAverage = ADC_AVERAGE_4
    };

    /* Enable ADC1 clock */
    PCC_SetClockMode(PCC, PCC_ADC1_CLOCK, false);

    /* Set ADC1 clock source */
    PCC_SetPeripheralClockControl(PCC, PCC_ADC1_CLOCK, true, CLK_SRC_SPLL, 0, 0);

    /* Enable ADC1 clock */
    PCC_SetClockMode(PCC, PCC_ADC1_CLOCK, true);
    ADC_DRV_Reset(1);

    /* Configure ADC1 */
    ADC_DRV_ConfigConverter(1, &adc1_cfg);
    ADC_DRV_SetSwPretrigger(1,ADC_SW_PRETRIGGER_DISABLED);
    ADC_DRV_ConfigHwCompare(1, &adc1_cmp_cfg);
    ADC_DRV_ConfigHwAverage(1, &adc1_avrg_cfg);

    /* Do calibration before initialize the ADC1. */
    ADC_DRV_AutoCalibration(1);
  }

  /* Start for S-Function (adc_s32k_start): '<S4>/ADC1_IRQ' */
  {
    adc_chan_config_t adc1_chan_cfg = {
      .interruptEnable = true,
      .channel = ADC_INPUTCHAN_EXT15
    };

    /* Initialize channel configuration of ADC1. */
    ADC_DRV_ConfigChan(1, 2, &adc1_chan_cfg);
  }

  /* Start for S-Function (adc_s32k_interleave): '<S4>/ADC_Interleave' */
  SIM_HAL_SetAdcInterleaveSel(SIM, 2U);

  /* Start for S-Function (fm_s32k_config): '<S4>/FreeMaster_Config' */

  /* Initialize FreeMaster. */
  freemaster_interface_init();
  freemaster_interface_isr_init();
  FMSTR_Init();

  /* Start for S-Function (lpspi_s32k_config): '<S4>/LPSPI_Config ' */
  {
    /* Enable LPSPI clock */
    PCC_SetPeripheralClockControl(PCC, LPSPI0_CLK, true, CLK_SRC_FIRC_DIV2,
      DIVIDE_BY_ONE, MULTIPLY_BY_ONE);

    /* Enable clock for PORTB */
    PCC_SetPeripheralClockControl(PCC, PORTB_CLK, true, CLK_SRC_OFF,
      DIVIDE_BY_ONE, MULTIPLY_BY_ONE);

    /* Enable clock for PORTB */
    PCC_SetPeripheralClockControl(PCC, PORTB_CLK, true, CLK_SRC_OFF,
      DIVIDE_BY_ONE, MULTIPLY_BY_ONE);

    /* Enable clock for PORTB */
    PCC_SetPeripheralClockControl(PCC, PORTB_CLK, true, CLK_SRC_OFF,
      DIVIDE_BY_ONE, MULTIPLY_BY_ONE);

    /* PCS1! Enable clock for PORTB */
    PCC_SetPeripheralClockControl(PCC, PORTB_CLK, true, CLK_SRC_OFF,
      DIVIDE_BY_ONE, MULTIPLY_BY_ONE);

    /* Setup SPI pins */
    pin_settings_config_t spi_pin_mux[4U]= {
      {
        /* SIN pin */
        .base = PORTB,
        .pinPortIdx = 3,
        .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PORT_LOW_DRIVE_STRENGTH,
        .mux = PORT_MUX_ALT3,
        .pinLock = false,
        .intConfig = PORT_DMA_INT_DISABLED,
        .clearIntFlag = false,
      },

      {
        /* SOUT pin */
        .base = PORTB,
        .pinPortIdx = 4,
        .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PORT_LOW_DRIVE_STRENGTH,
        .mux = PORT_MUX_ALT3,
        .pinLock = false,
        .intConfig = PORT_DMA_INT_DISABLED,
        .clearIntFlag = false,
      },

      {
        /* SCK pin */
        .base = PORTB,
        .pinPortIdx = 2,
        .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PORT_LOW_DRIVE_STRENGTH,
        .mux = PORT_MUX_ALT3,
        .pinLock = false,
        .intConfig = PORT_DMA_INT_DISABLED,
        .clearIntFlag = false,
      },

      {
        /* PCS1 pin */
        .base = PORTB,
        .pinPortIdx = 5,
        .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
        .passiveFilter = false,
        .driveSelect = PORT_LOW_DRIVE_STRENGTH,
        .mux = PORT_MUX_ALT3,
        .pinLock = false,
        .intConfig = PORT_DMA_INT_DISABLED,
        .clearIntFlag = false,
      },
    };

    PINS_DRV_Init(4U, spi_pin_mux);
  }

  {
    /* Configure the SPI init structure. */
    lpspi_master_config_t spiConfig0 = {
      .bitsPerSec = 1000000U,
      .whichPcs = LPSPI_PCS0,
      .pcsPolarity = LPSPI_ACTIVE_LOW,
      .isPcsContinuous = false,
      .bitcount = 8U,
      .clkPhase = LPSPI_CLOCK_PHASE_2ND_EDGE,
      .clkPolarity = LPSPI_SCK_ACTIVE_HIGH,
      .lsbFirst = false,
      .transferType = LPSPI_USING_INTERRUPTS,
      .callback = (spi_callback_t)lpspi_master_transfer_callback0,
    };

    /* Module source clock */
    uint32_t frequency;
    CLOCK_SYS_GetFreq(LPSPI0_CLK, &frequency);
    spiConfig0.lpspiSrcClk = frequency;

    /* Initializes a LPSPI instance for interrupt driven master mode operation */
    LPSPI_DRV_MasterInit(0, &lpspiMasterState0, &spiConfig0);
  }

  /* Start for DataStoreMemory: '<Root>/Data Store Memory5' */
  IaOffset = 2040.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory6' */
  IbOffset = 2040.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory8' */
  Gamma = 0.6F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory10' */
  Foc_model_Matlab_ultimo_DW.Rho = 0.05F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory13' */
  Epsilon = 20.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory9' */
  Lambda = 20.0F;

  /* SystemInitialize for S-Function (ftm_s32k_hall_sensor): '<S3>/FTM_Hall_Sensor' incorporates:
   *  SubSystem: '<Root>/Hall Sensor'
   */
  /* System initialize for function-call system: '<Root>/Hall Sensor' */

  /* Start for S-Function (gpio_s32k_input): '<S189>/Digital_Input_HALL_A' */
  {
    /* Enable clock for PORTA */
    PCC_SetClockMode(PCC, PCC_PORTA_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTAPin1 = {
      .base = PORTA,
      .pinPortIdx = 1,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTA,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTA1. */
    PINS_DRV_Init(1, &gpioPORTAPin1);
  }

  /* Start for S-Function (gpio_s32k_input): '<S189>/Digital_Input_HALL_B' */
  {
    /* Enable clock for PORTD */
    PCC_SetClockMode(PCC, PCC_PORTD_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTDPin10 = {
      .base = PORTD,
      .pinPortIdx = 10,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTD,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTD10. */
    PINS_DRV_Init(1, &gpioPORTDPin10);
  }

  /* Start for S-Function (gpio_s32k_input): '<S189>/Digital_Input_HALL_C' */
  {
    /* Enable clock for PORTD */
    PCC_SetClockMode(PCC, PCC_PORTD_CLOCK, true);

    /* Configure the input port init structure. */
    const pin_settings_config_t gpioPORTDPin11 = {
      .base = PORTD,
      .pinPortIdx = 11,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTD,
      .direction = GPIO_INPUT_DIRECTION,
    };

    /* Initialize GPIPORTD11. */
    PINS_DRV_Init(1, &gpioPORTDPin11);
  }

  /* Set FTM_2 clock source */
  PCC_SetPeripheralClockControl (PCC, FTM2_CLK, true, CLK_SRC_SPLL, 0, 0);

  /* Enable clock for FTM_2 */
  PCC_SetClockMode (PCC, FTM2_CLK, true);

  /* Initialize FTM instances, PWM and Input capture */
  static ftm_state_t ftm2StateStruct;
  FTM_DRV_Init(2, &flexTimer_ic2_InitConfig, &ftm2StateStruct);

  /* Setup input capture for FMT2*/
  FTM_DRV_InitInputCapture(2, &flexTimer_ic2_InputCaptureConfig);

  /* Enable clock for PORTA */
  PCC_SetClockMode (PCC, PCC_PORTA_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel(PORTA, 1, PORT_MUX_ALT2);

  /* Enable clock for PORTD */
  PCC_SetClockMode (PCC, PCC_PORTD_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel(PORTD, 10, PORT_MUX_ALT2);

  /* Enable clock for PORTD */
  PCC_SetClockMode (PCC, PCC_PORTD_CLOCK, true);

  /* Pin is configured for FTM function */
  PINS_SetMuxModeSel(PORTD, 11, PORT_MUX_ALT2);

  /* FTM2: channel 1 counter reset */
  FTM_RMW_CnSCV_REG(FTM2, 1, FTM_CnSC_ICRST_MASK , FTM_CnSC_ICRST(true));

  /* FTM2: channel 1 ISR enable */
  FTM_DRV_EnableChnInt(FTM2, 1);

  /* Select as the input of FTM2: XOR of FTM2_CH0, FTM2_CH1 and FTM1_CH1 */
  SIM->FTMOPT1 |= SIM_FTMOPT1_FTM2CH1SEL(1);

  /* FTM2: ISR level */
  INT_SYS_SetPriority(FTM2_Ch0_Ch1_IRQn, 5.0);

  /* FTM2: enable ISR */
  INT_SYS_EnableIRQ(FTM2_Ch0_Ch1_IRQn);

  /* FTM2:installing ISR Handler */
  INT_SYS_InstallHandler(FTM2_Ch0_Ch1_IRQn, FTM2_Ch0_1_IRQHandler, (isr_t *)0);

  /* Adding interrupt handler in the vector from ftm_chn_irq.c file */
  FTM_CHR_DRV_InstallCallback(2, 1, FTM2_Ch1_Hall_Sensor_isr);

  /* SystemInitialize for Atomic SubSystem: '<S3>/Subsystem2' */
  /* SystemInitialize for S-Function (pdb_s32k_isr): '<S208>/PDB1_ISR' */

  /* Table of base addresses for PDB instances. */
  static PDB_Type * const s_pdbBase[PDB_INSTANCE_COUNT] = PDB_BASE_PTRS;

  /* Set value for PDB0_IDLY register (interrupt delay) */
  PDB_DRV_SetValueForTimerInterrupt(0, 4999U);

  /* Load and lock interrupt delay value */
  PDB_DRV_LoadValuesCmd(0);

  {
    uint32_t sc = 0;
    PDB_Type * base = s_pdbBase[0];
    sc = base->SC;
    sc &= ~((uint32_t) PDB_SC_PDBEIE_MASK |
            (uint32_t)PDB_SC_PDBIE_MASK);

    /* Enable PDB interrupt */
    sc |= PDB_SC_PDBIE_MASK;
    base->SC = sc;

    /* Enable PDB0 interrupt and set priority for it */
    INT_SYS_InstallHandler(PDB0_IRQn, pdb0_isr, (isr_t *)0);
    INT_SYS_SetPriority(PDB0_IRQn, 6);
    INT_SYS_EnableIRQ(PDB0_IRQn);
  }

  /* SystemInitialize for S-Function (adc_s32k_isr): '<S208>/ADC1_ISR' incorporates:
   *  SubSystem: '<S208>/ADC1_IRQHandler'
   */
  /* System initialize for function-call system: '<S208>/ADC1_IRQHandler' */

  /* Start for S-Function (adc_s32k_start): '<S209>/ADC_AD4_IA' */
  {
    adc_chan_config_t adc0_chan_cfg = {
      .interruptEnable = false,
      .channel = ADC_INPUTCHAN_EXT4
    };

    /* Initialize channel configuration of ADC0. */
    ADC_DRV_ConfigChan(0, 0, &adc0_chan_cfg);
  }

  /* Start for S-Function (adc_s32k_start): '<S209>/ADC_AD7_VDC' */
  {
    adc_chan_config_t adc1_chan_cfg = {
      .interruptEnable = false,
      .channel = ADC_INPUTCHAN_EXT7
    };

    /* Initialize channel configuration of ADC1. */
    ADC_DRV_ConfigChan(1, 0, &adc1_chan_cfg);
  }

  /* Start for S-Function (adc_s32k_start): '<S209>/ADC_AD6_IDC' */
  {
    adc_chan_config_t adc1_chan_cfg = {
      .interruptEnable = false,
      .channel = ADC_INPUTCHAN_EXT6
    };

    /* Initialize channel configuration of ADC1. */
    ADC_DRV_ConfigChan(1, 1, &adc1_chan_cfg);
  }

  /* SystemInitialize for S-Function (fcgen): '<S209>/Function-Call Generator' incorporates:
   *  SubSystem: '<Root>/CurrentControl'
   */
  Foc_mod_CurrentControl_Init();

  /* End of SystemInitialize for S-Function (fcgen): '<S209>/Function-Call Generator' */
  ADC_InstallCallback(1, 2U, ADC1_SC1reg2U_callback);

  /* Set ADC1 interrupt priority */
  INT_SYS_SetPriority(ADC1_IRQn, 5);

  /* Enable ADC1 interrupt */
  INT_SYS_EnableIRQ(ADC1_IRQn);

  /* SystemInitialize for IfAction SubSystem: '<S4>/FAULT' */
  /* Start for S-Function (gpio_s32k_output): '<S215>/LED_GREEN' incorporates:
   *  Constant: '<S215>/OFF'
   */
  {
    /* Enable clock for PORTD */
    PCC_SetClockMode(PCC, PCC_PORTD_CLOCK, true);

    /* Configure the output port init structure. */
    const pin_settings_config_t gpioPORTDPin16 = {
      .base = PORTD,
      .pinPortIdx = 16,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTD,
      .direction = GPIO_OUTPUT_DIRECTION,
      .initValue = 0U
    };

    /* Initialize GPIPORTD16. */
    PINS_DRV_Init(1, &gpioPORTDPin16);
  }

  /* Start for S-Function (gpio_s32k_output): '<S215>/LED_RED' incorporates:
   *  Constant: '<S215>/ON'
   */
  {
    /* Enable clock for PORTD */
    PCC_SetClockMode(PCC, PCC_PORTD_CLOCK, true);

    /* Configure the output port init structure. */
    const pin_settings_config_t gpioPORTDPin15 = {
      .base = PORTD,
      .pinPortIdx = 15,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTD,
      .direction = GPIO_OUTPUT_DIRECTION,
      .initValue = 0U
    };

    /* Initialize GPIPORTD15. */
    PINS_DRV_Init(1, &gpioPORTDPin15);
  }

  /* End of SystemInitialize for SubSystem: '<S4>/FAULT' */
  /* SystemInitialize for Chart: '<S4>/Enable PDB and start FTM' incorporates:
   *  SubSystem: '<S4>/enable_FTM_PDB_ADC_triggering'
   */

  /* SystemInitialize for S-Function (tpp_s32k_isr): '<S4>/GD300_ISR_Callback ' */
  {
  }

  /* End of Enable for S-Function (adc_s32k_isr): '<S208>/ADC1_ISR' */

  /* Enable for S-Function (adc_s32k_isr): '<S208>/ADC1_ISR' incorporates:
   *  SubSystem: '<S208>/ADC1_IRQHandler'
   */
  /* Enable for function-call system: '<S208>/ADC1_IRQHandler' */

  /* Enable for S-Function (fcgen): '<S209>/Function-Call Generator' incorporates:
   *  SubSystem: '<Root>/CurrentControl'
   */
  Foc_m_CurrentControl_Enable();

  /* End of Enable for S-Function (fcgen): '<S209>/Function-Call Generator' */

  /* End of Enable for S-Function (adc_s32k_isr): '<S208>/ADC1_ISR' */
  /* End of Enable for SubSystem: '<S3>/Subsystem2' */
}

/* Model terminate function */
void Foc_model_Matlab_ultimo_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
