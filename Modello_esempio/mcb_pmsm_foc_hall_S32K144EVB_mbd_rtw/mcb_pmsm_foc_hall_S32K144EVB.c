/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: mcb_pmsm_foc_hall_S32K144EVB.c
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

#include "mcb_pmsm_foc_hall_S32K144EVB.h"
#include "rtwtypes.h"
#include "mcb_pmsm_foc_hall_S32K144EVB_private.h"
#include <math.h>
#include "rt_nonfinite.h"

/* Named constants for Chart: '<S4>/Enable PDB and start FTM' */
#define mcb_pmsm_foc_hall_S32K144E_IN_A ((uint8_T)1U)
#define mcb_pmsm_foc_hall_S32K14_IN_END ((uint8_T)2U)

lpspi_state_t lpspiMasterState0;
void lpspi_master_transfer_callback0(void *driverState, spi_event_t event, void *
  userData) __attribute__((weak));

/* Exported data definition */

/* Volatile memory section */
/* Definition for custom storage class: Volatile */
volatile real32_T ADC_A;               /* '<Root>/Data Store Memory11' */
volatile real32_T ADC_B;               /* '<Root>/Data Store Memory12' */
volatile uint32_T ADC_IA;              /* '<S243>/ADC_AD4_IA' */
volatile uint32_T ADC_IB;              /* '<S243>/ADC_IB' */
volatile uint32_T ADC_IDC;             /* '<S243>/ADC_AD6_IDC' */
volatile uint32_T ADC_VDC;             /* '<S243>/ADC_AD7_VDC' */
volatile uint32_T CH0S_ERR;            /* '<S242>/PDB1_ISR' */
volatile uint32_T CH1S_ERR;            /* '<S242>/PDB1_ISR' */
volatile real32_T COS;                 /* '<S154>/Sum6' */
volatile uint16_T CntHall;             /* '<S3>/FTM_Hall_Sensor' */
volatile uint32_T CntHallDecoder;      /* '<S171>/Read_Register' */
volatile uint16_T CntHallValidityIn;
                                /* '<S2>/SigConvForSigProp_Variant_Source2_0' */
volatile real32_T DesiredSpeed;        /* '<Root>/Data Store Memory7' */
volatile boolean_T Enable;             /* '<Root>/Data Store Memory29' */
volatile boolean_T FAULT;              /* '<Root>/I_MAX Scalling3' */
volatile int16_T GlobalDirection;      /* '<Root>/Data Store Memory3' */
volatile uint32_T GlobalHallState;     /* '<Root>/Data Store Memory4' */
volatile uint16_T GlobalSpeedCount;    /* '<Root>/Data Store Memory1' */
volatile uint16_T GlobalSpeedValidity; /* '<Root>/Data Store Memory2' */
volatile uint32_T HALL_A;              /* '<S225>/bit_shift' */
volatile uint32_T HALL_A_controller;   /* '<S174>/Data Type Conversion6' */
volatile uint32_T HALL_B;              /* '<S226>/bit_shift' */
volatile uint32_T HALL_B_controller;   /* '<S176>/bit_shift' */
volatile uint32_T HALL_C;              /* '<S223>/Data Type Conversion6' */
volatile uint32_T HALL_C_controller;   /* '<S175>/bit_shift' */
volatile uint16_T HallCntActual;       /* '<Root>/Data Store Memory25' */
volatile uint16_T HallCntPrev;         /* '<Root>/Data Store Memory24' */
volatile uint16_T HallStateChangeFlag; /* '<Root>/Data Store Memory' */
volatile uint32_T HallVal;             /* '<S174>/Add1' */
volatile uint16_T HallValididyInvalid; /* '<S229>/Merge' */
volatile real32_T I_ab_afterOffset[2]; /* '<S170>/Add' */
volatile real32_T IaOffset;            /* '<Root>/Data Store Memory5' */
volatile real32_T Iab_fb[2];           /* '<S170>/Multiply' */
volatile real32_T IbOffset;            /* '<Root>/Data Store Memory6' */
volatile real32_T Id_err;              /* '<S23>/Sum' */
volatile real32_T Idc_afterOffset;     /* '<S245>/Sum' */
volatile real32_T Iq_err;              /* '<S24>/Sum' */
volatile real32_T PWM[3];              /* '<S11>/Switch1' */
volatile real32_T PWM_Duty_Cycles[3];  /* '<S12>/Gain' */
volatile real32_T PWM_Enable;          /* '<S12>/Data Type Conversion' */
volatile real32_T Pos_PU;              /* '<S217>/Add' */
volatile uint32_T SC_PDBIF;            /* '<S242>/PDB1_ISR' */
volatile real32_T SIN;                 /* '<S154>/Sum4' */
volatile real32_T SpeedError;          /* '<S255>/Sum' */
volatile real32_T SpeedMeasured;       /* '<S1>/Input Scaling' */
volatile real32_T Speed_Ref;           /* '<S315>/Add1' */
volatile real32_T Speed_Ref_PU;        /* '<Root>/RT2' */
volatile real32_T Speed_fb;            /* '<Root>/RT1' */
volatile real32_T ThetaHalls;          /* '<S172>/Merge1' */
volatile real32_T Vd_ref_beforeLimiter;/* '<S83>/Saturation' */
volatile real32_T Vq_ref_beforeLimiter;/* '<S138>/Saturation' */

/* Block signals (default storage) */
B_mcb_pmsm_foc_hall_S32K144EV_T mcb_pmsm_foc_hall_S32K144EVB_B;

/* Block states (default storage) */
DW_mcb_pmsm_foc_hall_S32K144E_T mcb_pmsm_foc_hall_S32K144EVB_DW;

/* Real-time model */
static RT_MODEL_mcb_pmsm_foc_hall_S3_T mcb_pmsm_foc_hall_S32K144EVB_M_;
RT_MODEL_mcb_pmsm_foc_hall_S3_T *const mcb_pmsm_foc_hall_S32K144EVB_M =
  &mcb_pmsm_foc_hall_S32K144EVB_M_;
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

  /* Output and update for function-call system: '<S242>/PDB1_IRQHandler' */

  /* S-Function (ftm_s32k_init_disen): '<S244>/FTM_Init_Trigger_Enable' */

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
  mcb_pmsm_foc_hall_S32K144EVB_B.ADC1_ISR_o2 = result;
  ADC_DRV_GetChanConfig(1, 2U, &config);
  mcb_pmsm_foc_hall_S32K144EVB_B.ADC1_ISR_o3 = config.channel;

  /* Output and update for function-call system: '<S242>/ADC1_IRQHandler' */

  /* user code (Output function Body) */
  {
    /* Start of Profile Code */
    uint32_t tmp1;
    uint32_t tmp2;
    tmp1 = profiler_get_cnt();

    /* Start Profiling This Function.*/

    /* S-Function (ftm_s32k_init_disen): '<S243>/FTM_Init_Trigger_Disable' */

    /* FTM PWM Initialization Trigger Enable Disable*/
    FTM_DRV_SetInitTriggerCmd(FTM3, false);

    /* S-Function (adc_s32k_start): '<S243>/ADC_AD4_IA' */
    {
      uint16_t result;

      /* Get conversion result of ADC0 */
      ADC_DRV_WaitConvDone(0);
      ADC_DRV_GetChanResult(0, 0, &result);
      ADC_IA = result;
    }

    /* SignalConversion generated from: '<S243>/ADC_IB' */
    ADC_IB = mcb_pmsm_foc_hall_S32K144EVB_B.ADC1_ISR_o2;

    /* S-Function (adc_s32k_start): '<S243>/ADC_AD7_VDC' */
    {
      uint16_t result;

      /* Get conversion result of ADC1 */
      ADC_DRV_WaitConvDone(1);
      ADC_DRV_GetChanResult(1, 0, &result);
      ADC_VDC = result;
    }

    /* S-Function (adc_s32k_start): '<S243>/ADC_AD6_IDC' */
    {
      uint16_t result;

      /* Get conversion result of ADC1 */
      ADC_DRV_WaitConvDone(1);
      ADC_DRV_GetChanResult(1, 1, &result);
      ADC_IDC = result;
    }

    /* Outputs for Atomic SubSystem: '<S243>/FaultDetection' */
    mcb_pmsm_foc_FaultDetection(ADC_VDC, ADC_IDC);

    /* End of Outputs for SubSystem: '<S243>/FaultDetection' */

    /* S-Function (fcgen): '<S243>/Function-Call Generator' incorporates:
     *  SubSystem: '<Root>/CurrentControl'
     */
    mcb_pmsm_foc_CurrentControl();

    /* End of Outputs for S-Function (fcgen): '<S243>/Function-Call Generator' */

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

  /* DataStoreWrite: '<S250>/FAULT_write' incorporates:
   *  Constant: '<S250>/NOK'
   */
  FAULT = true;

  /* S-Function (ftm_s32k_pwm_disen): '<S250>/FTM_PWM_Disable_Enable' */
  FTM_DRV_DeinitPwm(FTM_PWM3);

  /* S-Function (tpp_s32k_func_mode): '<S250>/TPP_Functional_Mode' */
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
void mcb_pmsm_foc_hall_S32K144EVB_SetEventsForThisBaseStep(boolean_T *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(mcb_pmsm_foc_hall_S32K144EVB_M, 1));
  eventFlags[2] = ((boolean_T)rtmStepTask(mcb_pmsm_foc_hall_S32K144EVB_M, 2));
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
  (mcb_pmsm_foc_hall_S32K144EVB_M->Timing.TaskCounters.TID[1])++;
  if ((mcb_pmsm_foc_hall_S32K144EVB_M->Timing.TaskCounters.TID[1]) > 9) {/* Sample time: [0.001s, 0.0s] */
    mcb_pmsm_foc_hall_S32K144EVB_M->Timing.TaskCounters.TID[1] = 0;
  }

  (mcb_pmsm_foc_hall_S32K144EVB_M->Timing.TaskCounters.TID[2])++;
  if ((mcb_pmsm_foc_hall_S32K144EVB_M->Timing.TaskCounters.TID[2]) > 999) {/* Sample time: [0.1s, 0.0s] */
    mcb_pmsm_foc_hall_S32K144EVB_M->Timing.TaskCounters.TID[2] = 0;
  }
}

/* Output and update for atomic system: '<S243>/FaultDetection' */
void mcb_pmsm_foc_FaultDetection(uint32_T rtu_Vdc, uint32_T rtu_Idc)
{
  real32_T rtb_Product1_f;

  /* Product: '<S245>/Product1' incorporates:
   *  Constant: '<S245>/ADC1_AD7_Offset'
   *  Constant: '<S245>/bits2volts'
   *  DataTypeConversion: '<S245>/Data Type Conversion1'
   *  Sum: '<S245>/Add'
   */
  rtb_Product1_f = ((real32_T)rtu_Vdc - 17.0F) * 0.0109890113F;

  /* Sum: '<S245>/Sum' incorporates:
   *  Constant: '<S245>/ADC_AD6 offset  and  Logic power supply compensation'
   *  DataTypeConversion: '<S245>/Data Type Conversion'
   */
  Idc_afterOffset = (real32_T)rtu_Idc - 2090.0F;

  /* If: '<S245>/Check_Voltage_Current_Limits' incorporates:
   *  Constant: '<S245>/bits2amps'
   *  Product: '<S245>/Product'
   */
  if ((rtb_Product1_f < 8.0F) || (rtb_Product1_f > 16.0F) || (Idc_afterOffset *
       0.00805664062F > 2.3F)) {
    /* Outputs for IfAction SubSystem: '<S245>/FAILURE' incorporates:
     *  ActionPort: '<S246>/Action Port'
     */
    /* If: '<S246>/If' incorporates:
     *  DataStoreRead: '<S246>/FAULT_read'
     */
    if (!FAULT) {
      /* Outputs for IfAction SubSystem: '<S246>/Failed Subsystem' incorporates:
       *  ActionPort: '<S247>/Action Port'
       */
      /* S-Function (ftm_s32k_pwm_disen): '<S247>/FTM_PWM_Disable_Enable' */
      FTM_DRV_DeinitPwm(FTM_PWM3);

      /* DataStoreWrite: '<S247>/FAULT_write' incorporates:
       *  Constant: '<S247>/NOK'
       */
      FAULT = true;

      /* S-Function (tpp_s32k_func_mode): '<S247>/TPP_Functional_Mode' */
      TPP_SetOperationalMode(&tppDrvConfig, tppModeSleep);

      /* End of Outputs for SubSystem: '<S246>/Failed Subsystem' */
    }

    /* End of If: '<S246>/If' */
    /* End of Outputs for SubSystem: '<S245>/FAILURE' */
  }

  /* End of If: '<S245>/Check_Voltage_Current_Limits' */
}

/*
 * Output and update for action system:
 *    '<S197>/Hall Value of 1'
 *    '<S196>/Hall Value of 2'
 */
void mcb_pmsm_foc_h_HallValueof1(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S207>/position' incorporates:
   *  Constant: '<S207>/Constant'
   */
  *rty_position = 0.16667F;
}

/*
 * Output and update for action system:
 *    '<S197>/Hall Value of 2'
 *    '<S196>/Hall Value of 3'
 */
void mcb_pmsm_foc_h_HallValueof2(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S208>/position' incorporates:
   *  Constant: '<S208>/Constant'
   */
  *rty_position = 0.33333F;
}

/*
 * Output and update for action system:
 *    '<S197>/Hall Value of 3'
 *    '<S196>/Hall Value of 4'
 */
void mcb_pmsm_foc_h_HallValueof3(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S209>/position' incorporates:
   *  Constant: '<S209>/Constant'
   */
  *rty_position = 0.5F;
}

/*
 * Output and update for action system:
 *    '<S197>/Hall Value of 4'
 *    '<S196>/Hall Value of 5'
 */
void mcb_pmsm_foc_h_HallValueof4(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S210>/position' incorporates:
   *  Constant: '<S210>/Constant'
   */
  *rty_position = 0.66667F;
}

/*
 * Output and update for action system:
 *    '<S197>/Hall Value of 5'
 *    '<S196>/Hall Value of 6'
 */
void mcb_pmsm_foc_h_HallValueof5(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S211>/position' incorporates:
   *  Constant: '<S211>/Constant'
   */
  *rty_position = 0.83333F;
}

/*
 * Output and update for action system:
 *    '<S197>/Hall Value of 7'
 *    '<S196>/Hall Value of 1'
 *    '<S196>/Hall Value of 7'
 *    '<S185>/Hall Value of 7'
 */
void mcb_pmsm_foc_h_HallValueof7(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S213>/position' incorporates:
   *  Constant: '<S213>/Constant'
   */
  *rty_position = 0.0F;
}

/* System initialize for function-call system: '<Root>/CurrentControl' */
void mcb_pms_CurrentControl_Init(void)
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

  /* InitializeConditions for Delay: '<S180>/Delay One Step1' */
  mcb_pmsm_foc_hall_S32K144EVB_DW.DelayOneStep1_DSTATE = true;

  /* InitializeConditions for Delay: '<S180>/Delay One Step' */
  mcb_pmsm_foc_hall_S32K144EVB_DW.DelayOneStep_DSTATE = 500U;

  /* InitializeConditions for DiscreteIntegrator: '<S131>/Integrator' */
  mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE_e =
    mcb_pmsm_foc_hall_S32K144EVB_B.Kp1;

  /* InitializeConditions for DiscreteIntegrator: '<S76>/Integrator' */
  mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE_l =
    mcb_pmsm_foc_hall_S32K144EVB_B.Ki1;

  /* SystemInitialize for Atomic SubSystem: '<S169>/Atomic Hall Reading' */

  /* Start for S-Function (register_s32k_read): '<S171>/Read_Register' */
  PCC_SetClockMode(PCC, FTM2_CLK, true);

  /* End of SystemInitialize for SubSystem: '<S169>/Atomic Hall Reading' */
}

/* System reset for function-call system: '<Root>/CurrentControl' */
void mcb_pm_CurrentControl_Reset(void)
{
  /* InitializeConditions for Delay: '<S180>/Delay One Step1' */
  mcb_pmsm_foc_hall_S32K144EVB_DW.DelayOneStep1_DSTATE = true;

  /* InitializeConditions for Delay: '<S180>/Delay One Step' */
  mcb_pmsm_foc_hall_S32K144EVB_DW.DelayOneStep_DSTATE = 500U;

  /* InitializeConditions for DiscreteIntegrator: '<S131>/Integrator' */
  mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE_e =
    mcb_pmsm_foc_hall_S32K144EVB_B.Kp1;
  mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_PrevResetState_j = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S76>/Integrator' */
  mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE_l =
    mcb_pmsm_foc_hall_S32K144EVB_B.Ki1;
  mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_PrevResetState_n = 0;
}

/* Output and update for function-call system: '<Root>/CurrentControl' */
void mcb_pmsm_foc_CurrentControl(void)
{
  real32_T rtb_Add3;
  real32_T rtb_Max;
  real32_T rtb_Merge1;
  real32_T rtb_Merge1_p;
  real32_T rtb_Merge_a;
  real32_T rtb_Merge_idx_1;
  real32_T rtb_Switch_c_idx_0;
  real32_T rtb_Switch_n;
  uint16_T rtb_DataStoreRead5;
  int8_T tmp;
  int8_T tmp_0;
  boolean_T OR;
  boolean_T rtb_Compare;
  boolean_T rtb_LogicalOperator_tmp;
  boolean_T rtb_RelationalOperator_d;
  boolean_T rtb_fixforDTpropagationissue;

  /* user code (Output function Body) */
  {
    /* Start of Profile Code */
    uint32_t tmp1;
    uint32_t tmp2;
    tmp1 = profiler_get_cnt();

    /* Start Profiling This Function.*/

    /* Outputs for Atomic SubSystem: '<S169>/Atomic Hall Reading' */
    /* S-Function (register_s32k_read): '<S171>/Read_Register' */

    /* read from <empty>_SC1A register */
    CntHallDecoder = *((uint32_t *) 0x4003A004);

    /* DataStoreRead: '<S171>/Data Store Read5' */
    rtb_DataStoreRead5 = HallStateChangeFlag;

    /* S-Function (gpio_s32k_input): '<S174>/Digital_Input_HALL_C' */

    /* GPIPORTA1 signal update */
    mcb_pmsm_foc_hall_S32K144EVB_B.Digital_Input_HALL_C = (PINS_DRV_ReadPins(PTA)
      >> 1) & 0x01;

    /* Outputs for Atomic SubSystem: '<S174>/Bit Shift' */
    /* MATLAB Function: '<S175>/bit_shift' incorporates:
     *  DataTypeConversion: '<S174>/Data Type Conversion3'
     */
    /* MATLAB Function 'Logic and Bit Operations/Bit Shift/bit_shift': '<S177>:1' */
    /* '<S177>:1:6' */
    HALL_C_controller = (uint32_T)
      mcb_pmsm_foc_hall_S32K144EVB_B.Digital_Input_HALL_C << 2;

    /* End of Outputs for SubSystem: '<S174>/Bit Shift' */

    /* S-Function (gpio_s32k_input): '<S174>/Digital_Input_HALL_B' */

    /* GPIPORTD10 signal update */
    mcb_pmsm_foc_hall_S32K144EVB_B.Digital_Input_HALL_B = (PINS_DRV_ReadPins(PTD)
      >> 10) & 0x01;

    /* Outputs for Atomic SubSystem: '<S174>/Bit Shift1' */
    /* MATLAB Function: '<S176>/bit_shift' incorporates:
     *  DataTypeConversion: '<S174>/Data Type Conversion2'
     */
    /* MATLAB Function 'Logic and Bit Operations/Bit Shift/bit_shift': '<S178>:1' */
    /* '<S178>:1:6' */
    HALL_B_controller = (uint32_T)
      mcb_pmsm_foc_hall_S32K144EVB_B.Digital_Input_HALL_B << 1;

    /* End of Outputs for SubSystem: '<S174>/Bit Shift1' */

    /* S-Function (gpio_s32k_input): '<S174>/Digital_Input_HALL_A' */

    /* GPIPORTD11 signal update */
    mcb_pmsm_foc_hall_S32K144EVB_B.Digital_Input_HALL_A = (PINS_DRV_ReadPins(PTD)
      >> 11) & 0x01;

    /* DataTypeConversion: '<S174>/Data Type Conversion6' */
    HALL_A_controller = (uint32_T)
      mcb_pmsm_foc_hall_S32K144EVB_B.Digital_Input_HALL_A;

    /* Sum: '<S174>/Add1' */
    HallVal = (HALL_C_controller + HALL_B_controller) + HALL_A_controller;

    /* Switch: '<S172>/Switch' incorporates:
     *  Constant: '<S172>/WatchDog'
     *  DataStoreRead: '<S171>/Data Store Read5'
     *  DataStoreWrite: '<S169>/Data Store Write2'
     */
    if (HallStateChangeFlag != 0) {
      HallStateChangeFlag = 0U;
    }

    /* End of Switch: '<S172>/Switch' */
    /* End of Outputs for SubSystem: '<S169>/Atomic Hall Reading' */

    /* Logic: '<S180>/OR' incorporates:
     *  DataTypeConversion: '<S172>/Data Type Conversion4'
     *  Delay: '<S180>/Delay One Step1'
     */
    OR = (mcb_pmsm_foc_hall_S32K144EVB_DW.DelayOneStep1_DSTATE ||
          (rtb_DataStoreRead5 != 0));

    /* Delay: '<S180>/Delay One Step' incorporates:
     *  DataTypeConversion: '<S172>/Data Type Conversion4'
     */
    if (OR) {
      if (rtb_DataStoreRead5 != 0) {
        mcb_pmsm_foc_hall_S32K144EVB_DW.DelayOneStep_DSTATE = 500U;
      }

      /* Delay: '<S180>/Delay One Step' incorporates:
       *  DataTypeConversion: '<S172>/Data Type Conversion4'
       */
      mcb_pmsm_foc_hall_S32K144EVB_B.DelayOneStep =
        mcb_pmsm_foc_hall_S32K144EVB_DW.DelayOneStep_DSTATE;
    }

    /* End of Delay: '<S180>/Delay One Step' */

    /* RelationalOperator: '<S184>/Compare' incorporates:
     *  Constant: '<S184>/Constant'
     */
    rtb_Compare = (mcb_pmsm_foc_hall_S32K144EVB_B.DelayOneStep > 0);

    /* Switch: '<S183>/watchdog check' incorporates:
     *  Constant: '<S183>/Constant'
     */
    if (rtb_Compare) {
      /* MinMax: '<S183>/Max' incorporates:
       *  DataStoreRead: '<S171>/Data Store Read2'
       *  DataTypeConversion: '<S172>/counterSize1'
       */
      if (GlobalSpeedCount >= (uint16_T)CntHallDecoder) {
        rtb_DataStoreRead5 = GlobalSpeedCount;
      } else {
        rtb_DataStoreRead5 = (uint16_T)CntHallDecoder;
      }

      /* Switch: '<S183>/speed check' incorporates:
       *  Constant: '<S183>/Constant'
       *  DataStoreRead: '<S171>/Data Store Read4'
       *  DataTypeConversion: '<S179>/Data Type Conversion'
       *  Logic: '<S179>/Logical Operator'
       *  MinMax: '<S183>/Max'
       */
      if (rtb_DataStoreRead5 >= 31250) {
        rtb_DataStoreRead5 = 0U;
      } else {
        rtb_DataStoreRead5 = (uint16_T)((GlobalSpeedValidity != 0) ||
          mcb_pmsm_foc_hall_S32K144EVB_B.validityDelay);
      }

      /* End of Switch: '<S183>/speed check' */
    } else {
      rtb_DataStoreRead5 = 0U;
    }

    /* End of Switch: '<S183>/watchdog check' */

    /* If: '<S172>/If' */
    if (rtb_DataStoreRead5 != 0) {
      /* Outputs for IfAction SubSystem: '<S172>/Speed and direction are valid Use speed to extrapolate position' incorporates:
       *  ActionPort: '<S182>/Action Port'
       */
      /* If: '<S182>/If' incorporates:
       *  DataStoreRead: '<S171>/Data Store Read3'
       */
      if (GlobalDirection > 0) {
        /* Outputs for IfAction SubSystem: '<S182>/If Action Subsystem' incorporates:
         *  ActionPort: '<S193>/Action Port'
         */
        /* SignalConversion generated from: '<S193>/In1' incorporates:
         *  DataStoreRead: '<S171>/Data Store Read2'
         *  DataTypeConversion: '<S182>/currentSpeedData'
         *  Gain: '<S182>/SpeedGain'
         *  Product: '<S182>/Divide'
         */
        rtb_Merge_a = mcb_pmsm_foc_hall_S32K14_ConstB.SpeedConstData / (real32_T)
          GlobalSpeedCount * 0.05F;

        /* End of Outputs for SubSystem: '<S182>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S182>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S194>/Action Port'
         */
        /* UnaryMinus: '<S194>/Unary Minus' incorporates:
         *  DataStoreRead: '<S171>/Data Store Read2'
         *  DataTypeConversion: '<S182>/currentSpeedData'
         *  Gain: '<S182>/SpeedGain'
         *  Product: '<S182>/Divide'
         */
        rtb_Merge_a = -(mcb_pmsm_foc_hall_S32K14_ConstB.SpeedConstData /
                        (real32_T)GlobalSpeedCount * 0.05F);

        /* End of Outputs for SubSystem: '<S182>/If Action Subsystem1' */
      }

      /* End of If: '<S182>/If' */

      /* Outputs for Enabled SubSystem: '<S182>/Subsystem1' incorporates:
       *  EnablePort: '<S195>/Enable'
       */
      /* Outputs for IfAction SubSystem: '<S195>/first_order' incorporates:
       *  ActionPort: '<S198>/Action Port'
       */
      /* If: '<S195>/If1' incorporates:
       *  DataStoreRead: '<S171>/Data Store Read2'
       *  DataTypeConversion: '<S172>/counterSize1'
       *  DataTypeConversion: '<S198>/countData'
       *  DataTypeConversion: '<S198>/currentSpeedData'
       *  Gain: '<S198>/Gain'
       *  Product: '<S198>/Divide'
       */
      rtb_Merge1 = (real32_T)(uint16_T)CntHallDecoder / (real32_T)
        GlobalSpeedCount * 0.166666672F;

      /* End of Outputs for SubSystem: '<S195>/first_order' */

      /* Saturate: '<S195>/Saturation' */
      if (rtb_Merge1 > 0.16667F) {
        rtb_Merge1 = 0.16667F;
      }

      /* End of Saturate: '<S195>/Saturation' */

      /* If: '<S195>/If' incorporates:
       *  DataStoreRead: '<S171>/Data Store Read3'
       */
      if (GlobalDirection != 1) {
        /* Outputs for IfAction SubSystem: '<S195>/-ve Direction' incorporates:
         *  ActionPort: '<S197>/Action Port'
         */
        /* SwitchCase: '<S197>/Switch Case' */
        switch ((int32_T)HallVal) {
         case 6:
          /* Outputs for IfAction SubSystem: '<S197>/Hall Value of 1' incorporates:
           *  ActionPort: '<S207>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof1(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S197>/Hall Value of 1' */
          break;

         case 4:
          /* Outputs for IfAction SubSystem: '<S197>/Hall Value of 2' incorporates:
           *  ActionPort: '<S208>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof2(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S197>/Hall Value of 2' */
          break;

         case 5:
          /* Outputs for IfAction SubSystem: '<S197>/Hall Value of 3' incorporates:
           *  ActionPort: '<S209>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof3(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S197>/Hall Value of 3' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S197>/Hall Value of 4' incorporates:
           *  ActionPort: '<S210>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof4(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S197>/Hall Value of 4' */
          break;

         case 3:
          /* Outputs for IfAction SubSystem: '<S197>/Hall Value of 5' incorporates:
           *  ActionPort: '<S211>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof5(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S197>/Hall Value of 5' */
          break;

         case 2:
          /* Outputs for IfAction SubSystem: '<S197>/Hall Value of 6' incorporates:
           *  ActionPort: '<S212>/Action Port'
           */
          /* SignalConversion generated from: '<S212>/position' incorporates:
           *  Constant: '<S212>/Constant'
           */
          rtb_Merge1_p = 1.0F;

          /* End of Outputs for SubSystem: '<S197>/Hall Value of 6' */
          break;

         default:
          /* Outputs for IfAction SubSystem: '<S197>/Hall Value of 7' incorporates:
           *  ActionPort: '<S213>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof7(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S197>/Hall Value of 7' */
          break;
        }

        /* End of SwitchCase: '<S197>/Switch Case' */

        /* Merge: '<S195>/Merge' incorporates:
         *  Sum: '<S197>/Sum'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge = rtb_Merge1_p - rtb_Merge1;

        /* End of Outputs for SubSystem: '<S195>/-ve Direction' */
      } else {
        /* Outputs for IfAction SubSystem: '<S195>/+ve Direction' incorporates:
         *  ActionPort: '<S196>/Action Port'
         */
        /* SwitchCase: '<S196>/Switch Case' */
        switch ((int32_T)HallVal) {
         case 6:
          /* Outputs for IfAction SubSystem: '<S196>/Hall Value of 1' incorporates:
           *  ActionPort: '<S200>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof7(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S196>/Hall Value of 1' */
          break;

         case 4:
          /* Outputs for IfAction SubSystem: '<S196>/Hall Value of 2' incorporates:
           *  ActionPort: '<S201>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof1(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S196>/Hall Value of 2' */
          break;

         case 5:
          /* Outputs for IfAction SubSystem: '<S196>/Hall Value of 3' incorporates:
           *  ActionPort: '<S202>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof2(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S196>/Hall Value of 3' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S196>/Hall Value of 4' incorporates:
           *  ActionPort: '<S203>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof3(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S196>/Hall Value of 4' */
          break;

         case 3:
          /* Outputs for IfAction SubSystem: '<S196>/Hall Value of 5' incorporates:
           *  ActionPort: '<S204>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof4(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S196>/Hall Value of 5' */
          break;

         case 2:
          /* Outputs for IfAction SubSystem: '<S196>/Hall Value of 6' incorporates:
           *  ActionPort: '<S205>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof5(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S196>/Hall Value of 6' */
          break;

         default:
          /* Outputs for IfAction SubSystem: '<S196>/Hall Value of 7' incorporates:
           *  ActionPort: '<S206>/Action Port'
           */
          mcb_pmsm_foc_h_HallValueof7(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S196>/Hall Value of 7' */
          break;
        }

        /* End of SwitchCase: '<S196>/Switch Case' */

        /* Merge: '<S195>/Merge' incorporates:
         *  Sum: '<S196>/Sum'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge = rtb_Merge1_p + rtb_Merge1;

        /* End of Outputs for SubSystem: '<S195>/+ve Direction' */
      }

      /* End of If: '<S195>/If' */
      /* End of Outputs for SubSystem: '<S182>/Subsystem1' */

      /* Merge: '<S172>/Merge1' incorporates:
       *  SignalConversion generated from: '<S182>/rawPosition'
       */
      ThetaHalls = mcb_pmsm_foc_hall_S32K144EVB_B.Merge;

      /* End of Outputs for SubSystem: '<S172>/Speed and direction are valid Use speed to extrapolate position' */
    } else {
      /* Outputs for IfAction SubSystem: '<S172>/Speed and direction are not valid Position will be set to the middle of the Hall quadrant' incorporates:
       *  ActionPort: '<S181>/Action Port'
       */
      /* SwitchCase: '<S185>/Switch Case' */
      switch ((int32_T)HallVal) {
       case 6:
        /* Outputs for IfAction SubSystem: '<S185>/Hall Value of 1' incorporates:
         *  ActionPort: '<S186>/Action Port'
         */
        /* Merge: '<S172>/Merge1' incorporates:
         *  Constant: '<S186>/Constant'
         *  SignalConversion generated from: '<S186>/position'
         */
        ThetaHalls = 0.083333F;

        /* End of Outputs for SubSystem: '<S185>/Hall Value of 1' */
        break;

       case 4:
        /* Outputs for IfAction SubSystem: '<S185>/Hall Value of 2' incorporates:
         *  ActionPort: '<S187>/Action Port'
         */
        /* Merge: '<S172>/Merge1' incorporates:
         *  Constant: '<S187>/Constant'
         *  SignalConversion generated from: '<S187>/position'
         */
        ThetaHalls = 0.25F;

        /* End of Outputs for SubSystem: '<S185>/Hall Value of 2' */
        break;

       case 5:
        /* Outputs for IfAction SubSystem: '<S185>/Hall Value of 3' incorporates:
         *  ActionPort: '<S188>/Action Port'
         */
        /* Merge: '<S172>/Merge1' incorporates:
         *  Constant: '<S188>/Constant'
         *  SignalConversion generated from: '<S188>/position'
         */
        ThetaHalls = 0.41667F;

        /* End of Outputs for SubSystem: '<S185>/Hall Value of 3' */
        break;

       case 1:
        /* Outputs for IfAction SubSystem: '<S185>/Hall Value of 4' incorporates:
         *  ActionPort: '<S189>/Action Port'
         */
        /* Merge: '<S172>/Merge1' incorporates:
         *  Constant: '<S189>/Constant'
         *  SignalConversion generated from: '<S189>/position'
         */
        ThetaHalls = 0.58333F;

        /* End of Outputs for SubSystem: '<S185>/Hall Value of 4' */
        break;

       case 3:
        /* Outputs for IfAction SubSystem: '<S185>/Hall Value of 5' incorporates:
         *  ActionPort: '<S190>/Action Port'
         */
        /* Merge: '<S172>/Merge1' incorporates:
         *  Constant: '<S190>/Constant'
         *  SignalConversion generated from: '<S190>/position'
         */
        ThetaHalls = 0.75F;

        /* End of Outputs for SubSystem: '<S185>/Hall Value of 5' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S185>/Hall Value of 6' incorporates:
         *  ActionPort: '<S191>/Action Port'
         */
        /* Merge: '<S172>/Merge1' incorporates:
         *  Constant: '<S191>/Constant'
         *  SignalConversion generated from: '<S191>/position'
         */
        ThetaHalls = 0.91667F;

        /* End of Outputs for SubSystem: '<S185>/Hall Value of 6' */
        break;

       default:
        /* Outputs for IfAction SubSystem: '<S185>/Hall Value of 7' incorporates:
         *  ActionPort: '<S192>/Action Port'
         */
        mcb_pmsm_foc_h_HallValueof7((real32_T *)&ThetaHalls);

        /* End of Outputs for SubSystem: '<S185>/Hall Value of 7' */
        break;
      }

      /* End of SwitchCase: '<S185>/Switch Case' */

      /* SignalConversion generated from: '<S181>/Speed(r.p.m)' incorporates:
       *  Constant: '<S181>/Constant'
       */
      rtb_Merge_a = 0.0F;

      /* End of Outputs for SubSystem: '<S172>/Speed and direction are not valid Position will be set to the middle of the Hall quadrant' */
    }

    /* End of If: '<S172>/If' */

    /* Sum: '<S180>/Sum' */
    rtb_DataStoreRead5 = mcb_pmsm_foc_hall_S32K144EVB_B.DelayOneStep;

    /* If: '<S216>/If' incorporates:
     *  Constant: '<S215>/Constant1'
     *  Switch: '<S215>/Switch'
     */
    if (ThetaHalls <= 0.7021F) {
      /* Outputs for IfAction SubSystem: '<S216>/If Action Subsystem' incorporates:
       *  ActionPort: '<S218>/Action Port'
       */
      /* Sum: '<S218>/Add' incorporates:
       *  Constant: '<S218>/Constant'
       */
      rtb_Merge1 = (ThetaHalls + 1.0F) - 0.7021F;

      /* End of Outputs for SubSystem: '<S216>/If Action Subsystem' */
    } else {
      /* Outputs for IfAction SubSystem: '<S216>/If Action Subsystem1' incorporates:
       *  ActionPort: '<S219>/Action Port'
       */
      /* Sum: '<S219>/Add' */
      rtb_Merge1 = ThetaHalls - 0.7021F;

      /* End of Outputs for SubSystem: '<S216>/If Action Subsystem1' */
    }

    /* End of If: '<S216>/If' */

    /* Sum: '<S217>/Add' incorporates:
     *  Rounding: '<S217>/Floor'
     */
    Pos_PU = rtb_Merge1 - (real32_T)floor(rtb_Merge1);

    /* Sum: '<S170>/Add' incorporates:
     *  DataStoreRead: '<S170>/Data Store Read'
     *  DataStoreRead: '<S170>/Data Store Read1'
     *  DataStoreRead: '<S222>/Data Store Read'
     *  DataStoreRead: '<S222>/Data Store Read1'
     */
    I_ab_afterOffset[0] = IaOffset - ADC_A;
    I_ab_afterOffset[1] = IbOffset - ADC_B;

    /* Gain: '<S170>/Multiply' */
    Iab_fb[0] = 0.00048828125F * I_ab_afterOffset[0];
    Iab_fb[1] = 0.00048828125F * I_ab_afterOffset[1];

    /* Outputs for Atomic SubSystem: '<S20>/Two phase CRL wrap' */
    /* Gain: '<S21>/one_by_sqrt3' incorporates:
     *  Sum: '<S21>/a_plus_2b'
     */
    rtb_Add3 = ((Iab_fb[0] + Iab_fb[1]) + Iab_fb[1]) * 0.577350259F;

    /* End of Outputs for SubSystem: '<S20>/Two phase CRL wrap' */

    /* If: '<S155>/If' incorporates:
     *  Constant: '<S157>/Constant'
     *  DataTypeConversion: '<S158>/Convert_back'
     *  DataTypeConversion: '<S158>/Convert_uint16'
     *  DataTypeConversion: '<S159>/Convert_back'
     *  DataTypeConversion: '<S159>/Convert_uint16'
     *  Gain: '<S153>/indexing'
     *  RelationalOperator: '<S157>/Compare'
     *  Sum: '<S158>/Sum'
     *  Sum: '<S159>/Sum'
     */
    if (Pos_PU < 0.0F) {
      /* Outputs for IfAction SubSystem: '<S155>/If Action Subsystem' incorporates:
       *  ActionPort: '<S158>/Action Port'
       */
      rtb_Merge1 = Pos_PU - (real32_T)(int16_T)(real32_T)floor(Pos_PU);

      /* End of Outputs for SubSystem: '<S155>/If Action Subsystem' */
    } else {
      /* Outputs for IfAction SubSystem: '<S155>/If Action Subsystem1' incorporates:
       *  ActionPort: '<S159>/Action Port'
       */
      rtb_Merge1 = Pos_PU - (real32_T)(int16_T)Pos_PU;

      /* End of Outputs for SubSystem: '<S155>/If Action Subsystem1' */
    }

    rtb_Merge1 *= 800.0F;

    /* End of If: '<S155>/If' */

    /* Sum: '<S153>/Sum2' incorporates:
     *  DataTypeConversion: '<S153>/Data Type Conversion1'
     *  DataTypeConversion: '<S153>/Get_Integer'
     */
    rtb_Merge1_p = rtb_Merge1 - (real32_T)(uint16_T)rtb_Merge1;

    /* Selector: '<S153>/Lookup' incorporates:
     *  Constant: '<S153>/sine_table_values'
     *  DataTypeConversion: '<S153>/Get_Integer'
     */
    rtb_Switch_n = mcb_pmsm_foc_hall_S32K14_ConstP.sine_table_values_Value
      [(uint16_T)rtb_Merge1];

    /* Sum: '<S154>/Sum4' incorporates:
     *  Constant: '<S153>/offset'
     *  Constant: '<S153>/sine_table_values'
     *  DataTypeConversion: '<S153>/Get_Integer'
     *  Product: '<S154>/Product'
     *  Selector: '<S153>/Lookup'
     *  Sum: '<S153>/Sum'
     *  Sum: '<S154>/Sum3'
     */
    SIN = (mcb_pmsm_foc_hall_S32K14_ConstP.sine_table_values_Value[(int32_T)
           ((uint16_T)rtb_Merge1 + 1U)] - rtb_Switch_n) * rtb_Merge1_p +
      rtb_Switch_n;

    /* Selector: '<S153>/Lookup' incorporates:
     *  Constant: '<S153>/offset'
     *  Constant: '<S153>/sine_table_values'
     *  DataTypeConversion: '<S153>/Get_Integer'
     *  Sum: '<S153>/Sum'
     *  Sum: '<S154>/Sum5'
     */
    rtb_Switch_n = mcb_pmsm_foc_hall_S32K14_ConstP.sine_table_values_Value
      [(int32_T)((uint16_T)rtb_Merge1 + 200U)];

    /* Sum: '<S154>/Sum6' incorporates:
     *  Constant: '<S153>/offset'
     *  Constant: '<S153>/sine_table_values'
     *  DataTypeConversion: '<S153>/Get_Integer'
     *  Product: '<S154>/Product1'
     *  Selector: '<S153>/Lookup'
     *  Sum: '<S153>/Sum'
     *  Sum: '<S154>/Sum5'
     */
    COS = (mcb_pmsm_foc_hall_S32K14_ConstP.sine_table_values_Value[(int32_T)
           ((uint16_T)rtb_Merge1 + 201U)] - rtb_Switch_n) * rtb_Merge1_p +
      rtb_Switch_n;

    /* Outputs for Atomic SubSystem: '<S17>/Two inputs CRL' */
    /* Outputs for Atomic SubSystem: '<S20>/Two phase CRL wrap' */
    /* Sum: '<S24>/Sum' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S21>/a16'
     *  Product: '<S151>/asin'
     *  Product: '<S151>/bcos'
     *  Sum: '<S151>/sum_Qs'
     */
    Iq_err = mcb_pmsm_foc_hall_S32K144EVB_B.RT11[1] - (rtb_Add3 * COS - Iab_fb[0]
      * SIN);

    /* End of Outputs for SubSystem: '<S20>/Two phase CRL wrap' */
    /* End of Outputs for SubSystem: '<S17>/Two inputs CRL' */

    /* Logic: '<S24>/Logical Operator' incorporates:
     *  DataStoreRead: '<S24>/Data Store Read1'
     *  Logic: '<S23>/Logical Operator'
     */
    rtb_LogicalOperator_tmp = !Enable;

    /* Constant: '<S24>/Kp1' */
    mcb_pmsm_foc_hall_S32K144EVB_B.Kp1 = 0.0F;

    /* DiscreteIntegrator: '<S131>/Integrator' incorporates:
     *  Logic: '<S24>/Logical Operator'
     */
    if (rtb_LogicalOperator_tmp ||
        (mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_PrevResetState_j != 0)) {
      mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE_e =
        mcb_pmsm_foc_hall_S32K144EVB_B.Kp1;
    }

    /* Sum: '<S140>/Sum' incorporates:
     *  Constant: '<S24>/Kp'
     *  DiscreteIntegrator: '<S131>/Integrator'
     *  Product: '<S136>/PProd Out'
     */
    rtb_Max = Iq_err * 0.5F +
      mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE_e;

    /* Saturate: '<S138>/Saturation' */
    if (rtb_Max > 1.0F) {
      /* Saturate: '<S138>/Saturation' */
      Vq_ref_beforeLimiter = 1.0F;
    } else if (rtb_Max < -1.0F) {
      /* Saturate: '<S138>/Saturation' */
      Vq_ref_beforeLimiter = -1.0F;
    } else {
      /* Saturate: '<S138>/Saturation' */
      Vq_ref_beforeLimiter = rtb_Max;
    }

    /* End of Saturate: '<S138>/Saturation' */

    /* Outputs for Atomic SubSystem: '<S17>/Two inputs CRL' */
    /* Outputs for Atomic SubSystem: '<S20>/Two phase CRL wrap' */
    /* Sum: '<S23>/Sum' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S21>/a16'
     *  Product: '<S151>/acos'
     *  Product: '<S151>/bsin'
     *  Sum: '<S151>/sum_Ds'
     */
    Id_err = mcb_pmsm_foc_hall_S32K144EVB_B.RT11[0] - (Iab_fb[0] * COS +
      rtb_Add3 * SIN);

    /* End of Outputs for SubSystem: '<S20>/Two phase CRL wrap' */
    /* End of Outputs for SubSystem: '<S17>/Two inputs CRL' */

    /* Constant: '<S23>/Ki1' */
    mcb_pmsm_foc_hall_S32K144EVB_B.Ki1 = 0.0F;

    /* DiscreteIntegrator: '<S76>/Integrator' */
    if (rtb_LogicalOperator_tmp ||
        (mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_PrevResetState_n != 0)) {
      mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE_l =
        mcb_pmsm_foc_hall_S32K144EVB_B.Ki1;
    }

    /* Sum: '<S85>/Sum' incorporates:
     *  Constant: '<S23>/Kp'
     *  DiscreteIntegrator: '<S76>/Integrator'
     *  Product: '<S81>/PProd Out'
     */
    rtb_Merge1 = Id_err * 0.5F +
      mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE_l;

    /* Saturate: '<S83>/Saturation' */
    if (rtb_Merge1 > 1.0F) {
      /* Saturate: '<S83>/Saturation' */
      Vd_ref_beforeLimiter = 1.0F;
    } else if (rtb_Merge1 < -1.0F) {
      /* Saturate: '<S83>/Saturation' */
      Vd_ref_beforeLimiter = -1.0F;
    } else {
      /* Saturate: '<S83>/Saturation' */
      Vd_ref_beforeLimiter = rtb_Merge1;
    }

    /* End of Saturate: '<S83>/Saturation' */

    /* Sum: '<S28>/Sum1' incorporates:
     *  Product: '<S28>/Product'
     *  Product: '<S28>/Product1'
     */
    rtb_Merge1_p = Vd_ref_beforeLimiter * Vd_ref_beforeLimiter +
      Vq_ref_beforeLimiter * Vq_ref_beforeLimiter;

    /* Outputs for IfAction SubSystem: '<S22>/D-Q Equivalence' incorporates:
     *  ActionPort: '<S25>/Action Port'
     */
    /* If: '<S25>/If' incorporates:
     *  If: '<S22>/If'
     *  RelationalOperator: '<S25>/Relational Operator'
     */
    if (rtb_Merge1_p > 0.9025F) {
      /* Outputs for IfAction SubSystem: '<S25>/Limiter' incorporates:
       *  ActionPort: '<S29>/Action Port'
       */
      /* Sqrt: '<S29>/Square Root' */
      rtb_Merge1_p = (real32_T)sqrt(rtb_Merge1_p);

      /* Product: '<S29>/Divide' incorporates:
       *  Constant: '<S27>/Constant3'
       *  Product: '<S29>/Product'
       *  Switch: '<S27>/Switch'
       *  Switch: '<S29>/Switch'
       */
      rtb_Add3 = Vd_ref_beforeLimiter * 0.95F / rtb_Merge1_p;
      rtb_Merge_idx_1 = Vq_ref_beforeLimiter * 0.95F / rtb_Merge1_p;

      /* End of Outputs for SubSystem: '<S25>/Limiter' */
    } else {
      /* Outputs for IfAction SubSystem: '<S25>/Passthrough' incorporates:
       *  ActionPort: '<S30>/Action Port'
       */
      /* SignalConversion generated from: '<S30>/dqRef' */
      rtb_Add3 = Vd_ref_beforeLimiter;
      rtb_Merge_idx_1 = Vq_ref_beforeLimiter;

      /* End of Outputs for SubSystem: '<S25>/Passthrough' */
    }

    /* End of If: '<S25>/If' */
    /* End of Outputs for SubSystem: '<S22>/D-Q Equivalence' */

    /* DeadZone: '<S68>/DeadZone' */
    if (rtb_Merge1 > 1.0F) {
      rtb_Merge1--;
    } else if (rtb_Merge1 >= -1.0F) {
      rtb_Merge1 = 0.0F;
    } else {
      rtb_Merge1++;
    }

    /* End of DeadZone: '<S68>/DeadZone' */

    /* RelationalOperator: '<S66>/Relational Operator' incorporates:
     *  Constant: '<S66>/Clamping_zero'
     */
    rtb_RelationalOperator_d = (rtb_Merge1 != 0.0F);

    /* RelationalOperator: '<S66>/fix for DT propagation issue' incorporates:
     *  Constant: '<S66>/Clamping_zero'
     */
    rtb_fixforDTpropagationissue = (rtb_Merge1 > 0.0F);

    /* Product: '<S73>/IProd Out' incorporates:
     *  Constant: '<S23>/Ki'
     */
    rtb_Merge1 = Id_err * 0.0005F;

    /* Switch: '<S66>/Switch1' incorporates:
     *  Constant: '<S66>/Constant'
     *  Constant: '<S66>/Constant2'
     */
    if (rtb_fixforDTpropagationissue) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    /* Switch: '<S66>/Switch2' incorporates:
     *  Constant: '<S66>/Clamping_zero'
     *  Constant: '<S66>/Constant3'
     *  Constant: '<S66>/Constant4'
     *  RelationalOperator: '<S66>/fix for DT propagation issue1'
     */
    if (rtb_Merge1 > 0.0F) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    /* Switch: '<S66>/Switch' incorporates:
     *  Constant: '<S66>/Constant1'
     *  Logic: '<S66>/AND3'
     *  RelationalOperator: '<S66>/Equal1'
     *  Switch: '<S66>/Switch1'
     *  Switch: '<S66>/Switch2'
     */
    if (rtb_RelationalOperator_d && (tmp == tmp_0)) {
      rtb_Merge1_p = 0.0F;
    } else {
      rtb_Merge1_p = rtb_Merge1;
    }

    /* End of Switch: '<S66>/Switch' */

    /* DeadZone: '<S123>/DeadZone' */
    if (rtb_Max > 1.0F) {
      rtb_Max--;
    } else if (rtb_Max >= -1.0F) {
      rtb_Max = 0.0F;
    } else {
      rtb_Max++;
    }

    /* End of DeadZone: '<S123>/DeadZone' */

    /* Product: '<S128>/IProd Out' incorporates:
     *  Constant: '<S24>/Ki'
     */
    rtb_Merge1 = Iq_err * 0.0005F;

    /* Switch: '<S121>/Switch1' incorporates:
     *  Constant: '<S121>/Clamping_zero'
     *  Constant: '<S121>/Constant'
     *  Constant: '<S121>/Constant2'
     *  RelationalOperator: '<S121>/fix for DT propagation issue'
     */
    if (rtb_Max > 0.0F) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    /* Switch: '<S121>/Switch2' incorporates:
     *  Constant: '<S121>/Clamping_zero'
     *  Constant: '<S121>/Constant3'
     *  Constant: '<S121>/Constant4'
     *  RelationalOperator: '<S121>/fix for DT propagation issue1'
     */
    if (rtb_Merge1 > 0.0F) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    /* Switch: '<S121>/Switch' incorporates:
     *  Constant: '<S121>/Clamping_zero'
     *  Constant: '<S121>/Constant1'
     *  Logic: '<S121>/AND3'
     *  RelationalOperator: '<S121>/Equal1'
     *  RelationalOperator: '<S121>/Relational Operator'
     *  Switch: '<S121>/Switch1'
     *  Switch: '<S121>/Switch2'
     */
    if ((rtb_Max != 0.0F) && (tmp == tmp_0)) {
      rtb_Switch_n = 0.0F;
    } else {
      rtb_Switch_n = rtb_Merge1;
    }

    /* End of Switch: '<S121>/Switch' */

    /* Outputs for Atomic SubSystem: '<S16>/Two inputs CRL' */
    /* Switch: '<S150>/Switch' incorporates:
     *  Product: '<S149>/dcos'
     *  Product: '<S149>/qsin'
     *  Sum: '<S149>/sum_alpha'
     */
    rtb_Switch_c_idx_0 = rtb_Add3 * COS - rtb_Merge_idx_1 * SIN;

    /* Gain: '<S167>/one_by_two' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S149>/a16'
     */
    rtb_Merge1 = 0.5F * rtb_Switch_c_idx_0;

    /* Gain: '<S167>/sqrt3_by_two' incorporates:
     *  Product: '<S149>/dsin'
     *  Product: '<S149>/qcos'
     *  Sum: '<S149>/sum_beta'
     */
    rtb_Max = (rtb_Merge_idx_1 * COS + rtb_Add3 * SIN) * 0.866025388F;

    /* End of Outputs for SubSystem: '<S16>/Two inputs CRL' */

    /* Sum: '<S167>/add_b' */
    rtb_Merge_idx_1 = rtb_Max - rtb_Merge1;

    /* Sum: '<S167>/add_c' */
    rtb_Merge1 = (0.0F - rtb_Merge1) - rtb_Max;

    /* MinMax: '<S164>/Max' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S149>/a16'
     *  MinMax: '<S164>/Min'
     */
    rtb_RelationalOperator_d = rtIsNaNF(rtb_Merge_idx_1);

    /* Outputs for Atomic SubSystem: '<S16>/Two inputs CRL' */
    if ((rtb_Switch_c_idx_0 >= rtb_Merge_idx_1) || rtb_RelationalOperator_d) {
      rtb_Max = rtb_Switch_c_idx_0;
    } else {
      rtb_Max = rtb_Merge_idx_1;
    }

    /* MinMax: '<S164>/Min' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S149>/a16'
     */
    if ((rtb_Switch_c_idx_0 <= rtb_Merge_idx_1) || rtb_RelationalOperator_d) {
      rtb_Add3 = rtb_Switch_c_idx_0;
    } else {
      rtb_Add3 = rtb_Merge_idx_1;
    }

    /* End of Outputs for SubSystem: '<S16>/Two inputs CRL' */

    /* MinMax: '<S164>/Max' incorporates:
     *  MinMax: '<S164>/Min'
     */
    rtb_RelationalOperator_d = !rtIsNaNF(rtb_Merge1);
    if ((!(rtb_Max >= rtb_Merge1)) && rtb_RelationalOperator_d) {
      rtb_Max = rtb_Merge1;
    }

    /* MinMax: '<S164>/Min' */
    if ((!(rtb_Add3 <= rtb_Merge1)) && rtb_RelationalOperator_d) {
      rtb_Add3 = rtb_Merge1;
    }

    /* Gain: '<S164>/one_by_two' incorporates:
     *  MinMax: '<S164>/Max'
     *  MinMax: '<S164>/Min'
     *  Sum: '<S164>/Add'
     */
    rtb_Add3 = (rtb_Max + rtb_Add3) * -0.5F;

    /* Outputs for Atomic SubSystem: '<S16>/Two inputs CRL' */
    /* Gain: '<S12>/Gain' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S149>/a16'
     *  Constant: '<S12>/Constant1'
     *  Gain: '<S163>/Gain'
     *  Sum: '<S12>/Sum1'
     *  Sum: '<S163>/Add1'
     *  Sum: '<S163>/Add2'
     *  Sum: '<S163>/Add3'
     */
    PWM_Duty_Cycles[0] = ((rtb_Switch_c_idx_0 + rtb_Add3) * 1.15470052F + 1.0F) *
      0.5F;

    /* End of Outputs for SubSystem: '<S16>/Two inputs CRL' */
    PWM_Duty_Cycles[1] = ((rtb_Merge_idx_1 + rtb_Add3) * 1.15470052F + 1.0F) *
      0.5F;
    PWM_Duty_Cycles[2] = ((rtb_Add3 + rtb_Merge1) * 1.15470052F + 1.0F) * 0.5F;

    /* DataTypeConversion: '<S12>/Data Type Conversion' incorporates:
     *  DataStoreRead: '<S12>/Enable'
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
    SpeedMeasured = rtb_Merge_a;

    /* Update for Delay: '<S180>/Delay One Step1' */
    mcb_pmsm_foc_hall_S32K144EVB_DW.DelayOneStep1_DSTATE = rtb_Compare;

    /* Update for Delay: '<S180>/Delay One Step' incorporates:
     *  Constant: '<S180>/Constant2'
     *  Sum: '<S180>/Sum'
     */
    if (OR) {
      mcb_pmsm_foc_hall_S32K144EVB_DW.DelayOneStep_DSTATE = (uint16_T)
        (rtb_DataStoreRead5 - 1);
    }

    /* End of Update for Delay: '<S180>/Delay One Step' */

    /* Update for DiscreteIntegrator: '<S131>/Integrator' incorporates:
     *  Logic: '<S24>/Logical Operator'
     */
    mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE_e += rtb_Switch_n;
    mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_PrevResetState_j = (int8_T)
      rtb_LogicalOperator_tmp;

    /* Update for DiscreteIntegrator: '<S76>/Integrator' */
    mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE_l += rtb_Merge1_p;
    mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_PrevResetState_n = (int8_T)
      rtb_LogicalOperator_tmp;

    /* user code (Output function Trailer) */

    /* Profile Code : Compute function execution time in us. */
    tmp2 = profiler_get_cnt();
    profile_buffer[0] = gt_pf(tmp1, tmp2);

    /* End of Profile Code */
  }
}

/* Output and update for atomic system: '<Root>/SpeedControl' */
void mcb_pmsm_foc_h_SpeedControl(void)
{
  real32_T rtb_IProdOut;
  int8_T rtb_Switch1_l;
  int8_T tmp;
  boolean_T rtb_LogicalOperator_k;
  boolean_T rtb_RelationalOperator_h;

  /* user code (Output function Body) */
  {
    /* Start of Profile Code */
    uint32_t tmp1;
    uint32_t tmp2;
    tmp1 = profiler_get_cnt();

    /* Start Profiling This Function.*/

    /* Outputs for Atomic SubSystem: '<S8>/Speed_Ref_Selector' */
    /* Switch: '<S256>/Switch' incorporates:
     *  DataStoreRead: '<S256>/Data Store Read1'
     *  DataTypeConversion: '<S256>/Data Type Conversion'
     */
    if ((real32_T)Enable > 0.5F) {
      rtb_IProdOut = Speed_Ref_PU;
    } else {
      rtb_IProdOut = Speed_fb;
    }

    /* Sum: '<S315>/Add1' incorporates:
     *  Product: '<S315>/Product'
     *  Product: '<S315>/Product1'
     *  Switch: '<S256>/Switch'
     *  UnitDelay: '<S315>/Unit Delay'
     */
    Speed_Ref = rtb_IProdOut * 0.01F + 0.99F * Speed_Ref;

    /* End of Outputs for SubSystem: '<S8>/Speed_Ref_Selector' */

    /* Constant: '<S8>/Id_ref' */
    mcb_pmsm_foc_hall_S32K144EVB_B.Id_ref = 0.0F;

    /* Outputs for Atomic SubSystem: '<S8>/PI_Controller_Speed' */
    /* Sum: '<S255>/Sum' */
    SpeedError = Speed_Ref - Speed_fb;

    /* Logic: '<S255>/Logical Operator' incorporates:
     *  DataStoreRead: '<S255>/Data Store Read1'
     */
    rtb_LogicalOperator_k = !Enable;

    /* DiscreteIntegrator: '<S294>/Integrator' incorporates:
     *  Constant: '<S255>/Ki2'
     */
    if (rtb_LogicalOperator_k ||
        (mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_PrevResetState != 0)) {
      mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE = 0.0F;
    }

    /* Sum: '<S303>/Sum' incorporates:
     *  Constant: '<S255>/Kp1'
     *  DiscreteIntegrator: '<S294>/Integrator'
     *  Product: '<S299>/PProd Out'
     */
    mcb_pmsm_foc_hall_S32K144EVB_B.Saturation = SpeedError * 0.451369852F +
      mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE;

    /* DeadZone: '<S286>/DeadZone' */
    if (mcb_pmsm_foc_hall_S32K144EVB_B.Saturation > 1.0F) {
      rtb_IProdOut = mcb_pmsm_foc_hall_S32K144EVB_B.Saturation - 1.0F;
    } else if (mcb_pmsm_foc_hall_S32K144EVB_B.Saturation >= -1.0F) {
      rtb_IProdOut = 0.0F;
    } else {
      rtb_IProdOut = mcb_pmsm_foc_hall_S32K144EVB_B.Saturation - -1.0F;
    }

    /* End of DeadZone: '<S286>/DeadZone' */

    /* RelationalOperator: '<S284>/Relational Operator' incorporates:
     *  Constant: '<S284>/Clamping_zero'
     */
    rtb_RelationalOperator_h = (rtb_IProdOut != 0.0F);

    /* Switch: '<S284>/Switch1' incorporates:
     *  Constant: '<S284>/Clamping_zero'
     *  Constant: '<S284>/Constant'
     *  Constant: '<S284>/Constant2'
     *  RelationalOperator: '<S284>/fix for DT propagation issue'
     */
    if (rtb_IProdOut > 0.0F) {
      rtb_Switch1_l = 1;
    } else {
      rtb_Switch1_l = -1;
    }

    /* End of Switch: '<S284>/Switch1' */

    /* Product: '<S291>/IProd Out' incorporates:
     *  Constant: '<S255>/Ki1'
     */
    rtb_IProdOut = SpeedError * 0.00429385342F;

    /* Saturate: '<S301>/Saturation' */
    if (mcb_pmsm_foc_hall_S32K144EVB_B.Saturation > 1.0F) {
      /* Sum: '<S303>/Sum' incorporates:
       *  Saturate: '<S301>/Saturation'
       */
      mcb_pmsm_foc_hall_S32K144EVB_B.Saturation = 1.0F;
    } else if (mcb_pmsm_foc_hall_S32K144EVB_B.Saturation < -1.0F) {
      /* Sum: '<S303>/Sum' incorporates:
       *  Saturate: '<S301>/Saturation'
       */
      mcb_pmsm_foc_hall_S32K144EVB_B.Saturation = -1.0F;
    }

    /* End of Saturate: '<S301>/Saturation' */

    /* Switch: '<S284>/Switch2' incorporates:
     *  Constant: '<S284>/Clamping_zero'
     *  Constant: '<S284>/Constant3'
     *  Constant: '<S284>/Constant4'
     *  RelationalOperator: '<S284>/fix for DT propagation issue1'
     */
    if (rtb_IProdOut > 0.0F) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    /* Switch: '<S284>/Switch' incorporates:
     *  Constant: '<S284>/Constant1'
     *  Logic: '<S284>/AND3'
     *  RelationalOperator: '<S284>/Equal1'
     *  Switch: '<S284>/Switch2'
     */
    if (rtb_RelationalOperator_h && (rtb_Switch1_l == tmp)) {
      rtb_IProdOut = 0.0F;
    }

    /* Update for DiscreteIntegrator: '<S294>/Integrator' incorporates:
     *  Switch: '<S284>/Switch'
     */
    mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_DSTATE += rtb_IProdOut;
    mcb_pmsm_foc_hall_S32K144EVB_DW.Integrator_PrevResetState = (int8_T)
      rtb_LogicalOperator_k;

    /* End of Outputs for SubSystem: '<S8>/PI_Controller_Speed' */

    /* user code (Output function Trailer) */

    /* Profile Code : Compute function execution time in us. */
    tmp2 = profiler_get_cnt();
    profile_buffer[1] = gt_pf(tmp1, tmp2);

    /* End of Profile Code */
  }
}

/* Model step function for TID0 */
void mcb_pmsm_foc_hall_S32K144EVB_step0(void) /* Sample time: [0.0001s, 0.0s] */
{
  int32_T tmp;

  {                                    /* Sample time: [0.0001s, 0.0s] */
    rate_monotonic_scheduler();
  }

  /* End of Outputs for S-Function (ftm_s32k_hall_sensor): '<S3>/FTM_Hall_Sensor' */

  /* DataStoreWrite: '<S3>/Data Store Write1' */
  HallCntActual = CntHall;

  /* RateTransition: '<Root>/RT11' */
  tmp = mcb_pmsm_foc_hall_S32K144EVB_DW.RT11_ActiveBufIdx << 1;
  mcb_pmsm_foc_hall_S32K144EVB_B.RT11[0] =
    mcb_pmsm_foc_hall_S32K144EVB_DW.RT11_Buffer[tmp];
  mcb_pmsm_foc_hall_S32K144EVB_B.RT11[1] =
    mcb_pmsm_foc_hall_S32K144EVB_DW.RT11_Buffer[tmp + 1];

  /* End of Outputs for S-Function (pdb_s32k_isr): '<S242>/PDB1_ISR' */
  /* End of Outputs for S-Function (adc_s32k_isr): '<S242>/ADC1_ISR' */

  /* DataStoreWrite: '<S242>/Data Store Write' incorporates:
   *  DataTypeConversion: '<S242>/Data Type Conversion'
   */
  ADC_A = (real32_T)ADC_IA;

  /* DataStoreWrite: '<S242>/Data Store Write1' incorporates:
   *  DataTypeConversion: '<S242>/Data Type Conversion'
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
     *  ActionPort: '<S251>/Action Port'
     */
    /* S-Function (gpio_s32k_output): '<S251>/LED_GREEN_ON' incorporates:
     *  Constant: '<S251>/LED_GREEN'
     */

    /* GPOPORTD16 Data Signal Update */
    if (false) {
      PINS_DRV_SetPins(PTD, 1UL<<16);
    } else {
      PINS_DRV_ClearPins(PTD, 1UL<<16);
    }

    /* S-Function (gpio_s32k_output): '<S251>/LED_RED_OFF' incorporates:
     *  Constant: '<S251>/LED_RED'
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
     *  ActionPort: '<S249>/Action Port'
     */
    /* S-Function (gpio_s32k_output): '<S249>/LED_GREEN' incorporates:
     *  Constant: '<S249>/OFF'
     */

    /* GPOPORTD16 Data Signal Update */
    if (true) {
      PINS_DRV_SetPins(PTD, 1UL<<16);
    } else {
      PINS_DRV_ClearPins(PTD, 1UL<<16);
    }

    /* S-Function (gpio_s32k_output): '<S249>/LED_RED' incorporates:
     *  Constant: '<S249>/ON'
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
  if (mcb_pmsm_foc_hall_S32K144EVB_DW.is_active_c1_mcb_pmsm_foc_hall_ == 0) {
    /* Entry: Hardware Initialization/Enable PDB and start FTM */
    mcb_pmsm_foc_hall_S32K144EVB_DW.is_active_c1_mcb_pmsm_foc_hall_ = 1U;

    /* Entry Internal: Hardware Initialization/Enable PDB and start FTM */
    /* Transition: '<S248>:10' */
    mcb_pmsm_foc_hall_S32K144EVB_DW.is_c1_mcb_pmsm_foc_hall_S32K144 =
      mcb_pmsm_foc_hall_S32K144E_IN_A;
  } else if (mcb_pmsm_foc_hall_S32K144EVB_DW.is_c1_mcb_pmsm_foc_hall_S32K144 ==
             mcb_pmsm_foc_hall_S32K144E_IN_A) {
    /* Outputs for Function Call SubSystem: '<S4>/enable_FTM_PDB_ADC_triggering' */
    /* During 'A': '<S248>:5' */
    /* Transition: '<S248>:103' */
    /* Event: '<S248>:104' */

    /* S-Function (ftm_s32k_pwm_disen): '<S252>/FTM_PWM_Disable_Enable' */
    FTM_DRV_InitPwm(FTM_PWM3, &flexTimer_pwm3_PwmConfig);

    /* S-Function (ftm_s32k_init_disen): '<S252>/FTM_Init_Trigger_Disable_Enable' */

    /* FTM PWM Initialization Trigger Enable Disable*/
    FTM_DRV_SetInitTriggerCmd(FTM3, true);

    /* S-Function (pdb_s32k_enable): '<S252>/PDB0_Enable' */

    /* Enable PDB Module0 */
    PDB_DRV_Enable(0);

    /* S-Function (pdb_s32k_enable): '<S252>/PDB1_Enable' */

    /* Enable PDB Module1 */
    PDB_DRV_Enable(1);

    /* S-Function (tpp_s32k_isr_enable): '<S252>/TPP_ISR_Enable_Disable' */
    tpp_interrupt_enable(15);

    /* user code (Output function Trailer) */

    /* System '<S4>/enable_FTM_PDB_ADC_triggering' */
    PDB_DRV_LoadValuesCmd(0);
    PDB_DRV_LoadValuesCmd(1);

    /* End of Outputs for SubSystem: '<S4>/enable_FTM_PDB_ADC_triggering' */
    mcb_pmsm_foc_hall_S32K144EVB_DW.is_c1_mcb_pmsm_foc_hall_S32K144 =
      mcb_pmsm_foc_hall_S32K14_IN_END;
  } else {
    /* During 'END': '<S248>:39' */
    /* Transition: '<S248>:41' */
    /* Event: '<S248>:36' */
    mcb_pmsm_foc_hall_S32K144EVB_DW.is_c1_mcb_pmsm_foc_hall_S32K144 =
      mcb_pmsm_foc_hall_S32K14_IN_END;
  }

  /* End of Chart: '<S4>/Enable PDB and start FTM' */
  /* End of Outputs for S-Function (tpp_s32k_isr): '<S4>/GD300_ISR_Callback ' */
}

/* Model step function for TID1 */
void mcb_pmsm_foc_hall_S32K144EVB_step1(void) /* Sample time: [0.001s, 0.0s] */
{
  /* RateTransition: '<Root>/RT2' */
  Speed_Ref_PU = mcb_pmsm_foc_hall_S32K144EVB_DW.RT2_Buffer0;

  /* RateTransition: '<Root>/RT1' */
  Speed_fb = SpeedMeasured;

  /* Outputs for Atomic SubSystem: '<Root>/SpeedControl' */
  mcb_pmsm_foc_h_SpeedControl();

  /* End of Outputs for SubSystem: '<Root>/SpeedControl' */

  /* RateTransition: '<Root>/RT11' */
  mcb_pmsm_foc_hall_S32K144EVB_DW.RT11_Buffer
    [(mcb_pmsm_foc_hall_S32K144EVB_DW.RT11_ActiveBufIdx == 0) << 1] =
    mcb_pmsm_foc_hall_S32K144EVB_B.Id_ref;
  mcb_pmsm_foc_hall_S32K144EVB_DW.RT11_Buffer[1 +
    ((mcb_pmsm_foc_hall_S32K144EVB_DW.RT11_ActiveBufIdx == 0) << 1)] =
    mcb_pmsm_foc_hall_S32K144EVB_B.Saturation;
  mcb_pmsm_foc_hall_S32K144EVB_DW.RT11_ActiveBufIdx = (int8_T)
    (mcb_pmsm_foc_hall_S32K144EVB_DW.RT11_ActiveBufIdx == 0);
}

/* Model step function for TID2 */
void mcb_pmsm_foc_hall_S32K144EVB_step2(void) /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcgen): '<S3>/SCI_Rx_INT' incorporates:
   *  SubSystem: '<Root>/Serial Receive'
   */
  /* RateTransition: '<Root>/RT2' incorporates:
   *  DataStoreRead: '<S254>/Data Store Read2'
   *  Gain: '<S254>/rpm2PU'
   */
  mcb_pmsm_foc_hall_S32K144EVB_DW.RT2_Buffer0 = 0.0005F * DesiredSpeed;

  /* End of Outputs for S-Function (fcgen): '<S3>/SCI_Rx_INT' */
}

/* Model initialize function */
void mcb_pmsm_foc_hall_S32K144EVB_initialize(void)
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

  /* Start for DataStoreMemory: '<Root>/Data Store Memory7' */
  DesiredSpeed = 450.0F;

  /* SystemInitialize for S-Function (ftm_s32k_hall_sensor): '<S3>/FTM_Hall_Sensor' incorporates:
   *  SubSystem: '<Root>/Hall Sensor'
   */
  /* System initialize for function-call system: '<Root>/Hall Sensor' */

  /* Start for S-Function (gpio_s32k_input): '<S223>/Digital_Input_HALL_A' */
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

  /* Start for S-Function (gpio_s32k_input): '<S223>/Digital_Input_HALL_B' */
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

  /* Start for S-Function (gpio_s32k_input): '<S223>/Digital_Input_HALL_C' */
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
  /* SystemInitialize for S-Function (pdb_s32k_isr): '<S242>/PDB1_ISR' */

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

  /* SystemInitialize for S-Function (adc_s32k_isr): '<S242>/ADC1_ISR' incorporates:
   *  SubSystem: '<S242>/ADC1_IRQHandler'
   */
  /* System initialize for function-call system: '<S242>/ADC1_IRQHandler' */

  /* Start for S-Function (adc_s32k_start): '<S243>/ADC_AD4_IA' */
  {
    adc_chan_config_t adc0_chan_cfg = {
      .interruptEnable = false,
      .channel = ADC_INPUTCHAN_EXT4
    };

    /* Initialize channel configuration of ADC0. */
    ADC_DRV_ConfigChan(0, 0, &adc0_chan_cfg);
  }

  /* Start for S-Function (adc_s32k_start): '<S243>/ADC_AD7_VDC' */
  {
    adc_chan_config_t adc1_chan_cfg = {
      .interruptEnable = false,
      .channel = ADC_INPUTCHAN_EXT7
    };

    /* Initialize channel configuration of ADC1. */
    ADC_DRV_ConfigChan(1, 0, &adc1_chan_cfg);
  }

  /* Start for S-Function (adc_s32k_start): '<S243>/ADC_AD6_IDC' */
  {
    adc_chan_config_t adc1_chan_cfg = {
      .interruptEnable = false,
      .channel = ADC_INPUTCHAN_EXT6
    };

    /* Initialize channel configuration of ADC1. */
    ADC_DRV_ConfigChan(1, 1, &adc1_chan_cfg);
  }

  /* SystemInitialize for S-Function (fcgen): '<S243>/Function-Call Generator' incorporates:
   *  SubSystem: '<Root>/CurrentControl'
   */
  mcb_pms_CurrentControl_Init();

  /* End of SystemInitialize for S-Function (fcgen): '<S243>/Function-Call Generator' */
  ADC_InstallCallback(1, 2U, ADC1_SC1reg2U_callback);

  /* Set ADC1 interrupt priority */
  INT_SYS_SetPriority(ADC1_IRQn, 5);

  /* Enable ADC1 interrupt */
  INT_SYS_EnableIRQ(ADC1_IRQn);

  /* SystemInitialize for IfAction SubSystem: '<S4>/FAULT' */
  /* Start for S-Function (gpio_s32k_output): '<S249>/LED_GREEN' incorporates:
   *  Constant: '<S249>/OFF'
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

  /* Start for S-Function (gpio_s32k_output): '<S249>/LED_RED' incorporates:
   *  Constant: '<S249>/ON'
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
}

/* Model terminate function */
void mcb_pmsm_foc_hall_S32K144EVB_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
