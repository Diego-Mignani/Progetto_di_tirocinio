/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Foc_model_Matlab.c
 *
 * Code generated for Simulink model 'Foc_model_Matlab'.
 *
 * Model version                   : 10.87
 * Simulink Coder version          : 24.2 (R2024b) 21-Jun-2024
 * MBDT for S32K1xx Series Version : 4.2.0 (R2016a-R2020a) 20-Jul-2020
 * C/C++ source code generated on  : Wed May 21 11:31:21 2025
 *
 * Target selection: mbd_s32k.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Foc_model_Matlab.h"
#include "rtwtypes.h"
#include "Foc_model_Matlab_private.h"
#include <math.h>
#include "rt_nonfinite.h"

/* Named constants for Chart: '<S4>/Enable PDB and start FTM' */
#define Foc_model_Matlab_IN_A          ((uint8_T)1U)
#define Foc_model_Matlab_IN_END        ((uint8_T)2U)

lpspi_state_t lpspiMasterState0;
void lpspi_master_transfer_callback0(void *driverState, spi_event_t event, void *
  userData) __attribute__((weak));

/* Exported data definition */

/* Volatile memory section */
/* Definition for custom storage class: Volatile */
volatile real32_T ADC_A;               /* '<Root>/Data Store Memory11' */
volatile real32_T ADC_B;               /* '<Root>/Data Store Memory12' */
volatile uint32_T ADC_IA;              /* '<S141>/ADC_AD4_IA' */
volatile uint32_T ADC_IB;              /* '<S141>/ADC_IB' */
volatile uint32_T ADC_IDC;             /* '<S141>/ADC_AD6_IDC' */
volatile uint32_T ADC_VDC;             /* '<S141>/ADC_AD7_VDC' */
volatile uint32_T CH0S_ERR;            /* '<S140>/PDB1_ISR' */
volatile uint32_T CH1S_ERR;            /* '<S140>/PDB1_ISR' */
volatile uint16_T CntHall;             /* '<S3>/FTM_Hall_Sensor' */
volatile uint16_T CntHallValidityIn;
                                /* '<S2>/SigConvForSigProp_Variant_Source2_0' */
volatile real32_T DesiredSpeed;        /* '<Root>/Data Store Memory7' */
volatile boolean_T Enable;             /* '<Root>/Data Store Memory29' */
volatile real32_T Epsilon_d;           /* '<Root>/Data Store Memory16' */
volatile real32_T Epsilon_q;           /* '<Root>/Data Store Memory14' */
volatile real32_T Epsilon_w;           /* '<Root>/Data Store Memory13' */
volatile boolean_T FAULT;              /* '<Root>/I_MAX Scalling3' */
volatile int16_T GlobalDirection;      /* '<Root>/Data Store Memory3' */
volatile uint32_T GlobalHallState;     /* '<Root>/Data Store Memory4' */
volatile uint16_T GlobalSpeedCount;    /* '<Root>/Data Store Memory1' */
volatile uint16_T GlobalSpeedValidity; /* '<Root>/Data Store Memory2' */
volatile uint32_T HALL_A;              /* '<S123>/bit_shift' */
volatile uint32_T HALL_A_controller;   /* '<S72>/Data Type Conversion6' */
volatile uint32_T HALL_B;              /* '<S124>/bit_shift' */
volatile uint32_T HALL_B_controller;   /* '<S74>/bit_shift' */
volatile uint32_T HALL_C;              /* '<S121>/Data Type Conversion6' */
volatile uint32_T HALL_C_controller;   /* '<S73>/bit_shift' */
volatile uint16_T HallCntActual;       /* '<Root>/Data Store Memory25' */
volatile uint16_T HallCntPrev;         /* '<Root>/Data Store Memory24' */
volatile uint16_T HallStateChangeFlag; /* '<Root>/Data Store Memory' */
volatile uint16_T HallValididyInvalid; /* '<S127>/Merge' */
volatile real32_T I_ab_afterOffset[2]; /* '<S68>/Add' */
volatile real32_T IaOffset;            /* '<Root>/Data Store Memory5' */
volatile real32_T IbOffset;            /* '<Root>/Data Store Memory6' */
volatile real32_T Id_fb;               /* '<S15>/Signal Copy1' */
volatile real32_T Idc_afterOffset;     /* '<S143>/Sum' */
volatile real32_T Iq_err;              /* '<S23>/Sum1' */
volatile real32_T Iq_fb;               /* '<S15>/Signal Copy' */
volatile real32_T Iq_ref_PU;           /* '<Root>/RT11' */
volatile real32_T Lambda_d;            /* '<Root>/Data Store Memory17' */
volatile real32_T Lambda_q;            /* '<Root>/Data Store Memory15' */
volatile real32_T Lambda_w;            /* '<Root>/Data Store Memory9' */
volatile real32_T PWM[3];              /* '<S11>/Switch1' */
volatile real32_T PWM_Duty_Cycles[3];  /* '<S12>/Gain' */
volatile real32_T PWM_Enable;          /* '<S12>/Data Type Conversion' */
volatile real32_T Rho_d;               /* '<Root>/Data Store Memory18' */
volatile real32_T Rho_q;               /* '<Root>/Data Store Memory8' */
volatile real32_T Rho_w;               /* '<Root>/Data Store Memory10' */
volatile uint32_T SC_PDBIF;            /* '<S140>/PDB1_ISR' */
volatile real32_T SpeedError;          /* '<S156>/Sum' */
volatile real32_T SpeedMeasured;       /* '<S1>/Input Scaling' */
volatile real32_T Speed_Ref;           /* '<S154>/Switch' */
volatile real32_T Speed_Ref_PU;        /* '<Root>/RT2' */
volatile real32_T Speed_fb;            /* '<Root>/RT1' */
volatile real32_T ThetaHalls;          /* '<S70>/Merge1' */
volatile real32_T limit_d;             /* '<Root>/Data Store Memory19' */
volatile real32_T limit_q;             /* '<Root>/Data Store Memory21' */
volatile real32_T limit_w;             /* '<Root>/Data Store Memory20' */

/* Block signals (default storage) */
B_Foc_model_Matlab_T Foc_model_Matlab_B;

/* Block states (default storage) */
DW_Foc_model_Matlab_T Foc_model_Matlab_DW;

/* Real-time model */
static RT_MODEL_Foc_model_Matlab_T Foc_model_Matlab_M_;
RT_MODEL_Foc_model_Matlab_T *const Foc_model_Matlab_M = &Foc_model_Matlab_M_;
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

  /* Output and update for function-call system: '<S140>/PDB1_IRQHandler' */

  /* S-Function (ftm_s32k_init_disen): '<S142>/FTM_Init_Trigger_Enable' */

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
  Foc_model_Matlab_B.ADC1_ISR_o2 = result;
  ADC_DRV_GetChanConfig(1, 2U, &config);
  Foc_model_Matlab_B.ADC1_ISR_o3 = config.channel;

  /* Output and update for function-call system: '<S140>/ADC1_IRQHandler' */

  /* user code (Output function Body) */
  {
    /* Start of Profile Code */
    uint32_t tmp1;
    uint32_t tmp2;
    tmp1 = profiler_get_cnt();

    /* Start Profiling This Function.*/

    /* S-Function (ftm_s32k_init_disen): '<S141>/FTM_Init_Trigger_Disable' */

    /* FTM PWM Initialization Trigger Enable Disable*/
    FTM_DRV_SetInitTriggerCmd(FTM3, false);

    /* S-Function (adc_s32k_start): '<S141>/ADC_AD4_IA' */
    {
      uint16_t result;

      /* Get conversion result of ADC0 */
      ADC_DRV_WaitConvDone(0);
      ADC_DRV_GetChanResult(0, 0, &result);
      ADC_IA = result;
    }

    /* SignalConversion generated from: '<S141>/ADC_IB' */
    ADC_IB = Foc_model_Matlab_B.ADC1_ISR_o2;

    /* S-Function (adc_s32k_start): '<S141>/ADC_AD7_VDC' */
    {
      uint16_t result;

      /* Get conversion result of ADC1 */
      ADC_DRV_WaitConvDone(1);
      ADC_DRV_GetChanResult(1, 0, &result);
      ADC_VDC = result;
    }

    /* S-Function (adc_s32k_start): '<S141>/ADC_AD6_IDC' */
    {
      uint16_t result;

      /* Get conversion result of ADC1 */
      ADC_DRV_WaitConvDone(1);
      ADC_DRV_GetChanResult(1, 1, &result);
      ADC_IDC = result;
    }

    /* Outputs for Atomic SubSystem: '<S141>/FaultDetection' */
    Foc_model_Ma_FaultDetection(ADC_VDC, ADC_IDC);

    /* End of Outputs for SubSystem: '<S141>/FaultDetection' */

    /* S-Function (fcgen): '<S141>/Function-Call Generator' incorporates:
     *  SubSystem: '<Root>/CurrentControl'
     */
    Foc_model_Ma_CurrentControl();

    /* End of Outputs for S-Function (fcgen): '<S141>/Function-Call Generator' */

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

  /* DataStoreWrite: '<S148>/FAULT_write' incorporates:
   *  Constant: '<S148>/NOK'
   */
  FAULT = true;

  /* S-Function (ftm_s32k_pwm_disen): '<S148>/FTM_PWM_Disable_Enable' */
  FTM_DRV_DeinitPwm(FTM_PWM3);

  /* S-Function (tpp_s32k_func_mode): '<S148>/TPP_Functional_Mode' */
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
void Foc_model_Matlab_SetEventsForThisBaseStep(boolean_T *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(Foc_model_Matlab_M, 1));
  eventFlags[2] = ((boolean_T)rtmStepTask(Foc_model_Matlab_M, 2));
  eventFlags[3] = ((boolean_T)rtmStepTask(Foc_model_Matlab_M, 3));
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
  (Foc_model_Matlab_M->Timing.TaskCounters.TID[1])++;
  if ((Foc_model_Matlab_M->Timing.TaskCounters.TID[1]) > 1) {/* Sample time: [0.0001s, 0.0s] */
    Foc_model_Matlab_M->Timing.TaskCounters.TID[1] = 0;
  }

  (Foc_model_Matlab_M->Timing.TaskCounters.TID[2])++;
  if ((Foc_model_Matlab_M->Timing.TaskCounters.TID[2]) > 19) {/* Sample time: [0.001s, 0.0s] */
    Foc_model_Matlab_M->Timing.TaskCounters.TID[2] = 0;
  }

  (Foc_model_Matlab_M->Timing.TaskCounters.TID[3])++;
  if ((Foc_model_Matlab_M->Timing.TaskCounters.TID[3]) > 1999) {/* Sample time: [0.1s, 0.0s] */
    Foc_model_Matlab_M->Timing.TaskCounters.TID[3] = 0;
  }
}

/* Output and update for atomic system: '<S141>/FaultDetection' */
void Foc_model_Ma_FaultDetection(uint32_T rtu_Vdc, uint32_T rtu_Idc)
{
  real32_T rtb_Product1_f;

  /* Product: '<S143>/Product1' incorporates:
   *  Constant: '<S143>/ADC1_AD7_Offset'
   *  Constant: '<S143>/bits2volts'
   *  DataTypeConversion: '<S143>/Data Type Conversion1'
   *  Sum: '<S143>/Add'
   */
  rtb_Product1_f = ((real32_T)rtu_Vdc - 17.0F) * 0.0109890113F;

  /* Sum: '<S143>/Sum' incorporates:
   *  Constant: '<S143>/ADC_AD6 offset  and  Logic power supply compensation'
   *  DataTypeConversion: '<S143>/Data Type Conversion'
   */
  Idc_afterOffset = (real32_T)rtu_Idc - 2090.0F;

  /* If: '<S143>/Check_Voltage_Current_Limits' incorporates:
   *  Constant: '<S143>/bits2amps'
   *  Product: '<S143>/Product'
   */
  if ((rtb_Product1_f < 8.0F) || (rtb_Product1_f > 16.0F) || (Idc_afterOffset *
       0.00805664062F > 2.3F)) {
    /* Outputs for IfAction SubSystem: '<S143>/FAILURE' incorporates:
     *  ActionPort: '<S144>/Action Port'
     */
    /* If: '<S144>/If' incorporates:
     *  DataStoreRead: '<S144>/FAULT_read'
     */
    if (!FAULT) {
      /* Outputs for IfAction SubSystem: '<S144>/Failed Subsystem' incorporates:
       *  ActionPort: '<S145>/Action Port'
       */
      /* S-Function (ftm_s32k_pwm_disen): '<S145>/FTM_PWM_Disable_Enable' */
      FTM_DRV_DeinitPwm(FTM_PWM3);

      /* DataStoreWrite: '<S145>/FAULT_write' incorporates:
       *  Constant: '<S145>/NOK'
       */
      FAULT = true;

      /* S-Function (tpp_s32k_func_mode): '<S145>/TPP_Functional_Mode' */
      TPP_SetOperationalMode(&tppDrvConfig, tppModeSleep);

      /* End of Outputs for SubSystem: '<S144>/Failed Subsystem' */
    }

    /* End of If: '<S144>/If' */
    /* End of Outputs for SubSystem: '<S143>/FAILURE' */
  }

  /* End of If: '<S143>/Check_Voltage_Current_Limits' */
}

/*
 * Output and update for action system:
 *    '<S95>/Hall Value of 1'
 *    '<S94>/Hall Value of 2'
 */
void Foc_model_Matl_HallValueof1(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S105>/position' incorporates:
   *  Constant: '<S105>/Constant'
   */
  *rty_position = 0.16667F;
}

/*
 * Output and update for action system:
 *    '<S95>/Hall Value of 2'
 *    '<S94>/Hall Value of 3'
 */
void Foc_model_Matl_HallValueof2(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S106>/position' incorporates:
   *  Constant: '<S106>/Constant'
   */
  *rty_position = 0.33333F;
}

/*
 * Output and update for action system:
 *    '<S95>/Hall Value of 3'
 *    '<S94>/Hall Value of 4'
 */
void Foc_model_Matl_HallValueof3(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S107>/position' incorporates:
   *  Constant: '<S107>/Constant'
   */
  *rty_position = 0.5F;
}

/*
 * Output and update for action system:
 *    '<S95>/Hall Value of 4'
 *    '<S94>/Hall Value of 5'
 */
void Foc_model_Matl_HallValueof4(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S108>/position' incorporates:
   *  Constant: '<S108>/Constant'
   */
  *rty_position = 0.66667F;
}

/*
 * Output and update for action system:
 *    '<S95>/Hall Value of 5'
 *    '<S94>/Hall Value of 6'
 */
void Foc_model_Matl_HallValueof5(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S109>/position' incorporates:
   *  Constant: '<S109>/Constant'
   */
  *rty_position = 0.83333F;
}

/*
 * Output and update for action system:
 *    '<S95>/Hall Value of 7'
 *    '<S94>/Hall Value of 1'
 *    '<S94>/Hall Value of 7'
 *    '<S83>/Hall Value of 7'
 */
void Foc_model_Matl_HallValueof7(real32_T *rty_position)
{
  /* SignalConversion generated from: '<S111>/position' incorporates:
   *  Constant: '<S111>/Constant'
   */
  *rty_position = 0.0F;
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

  /* InitializeConditions for Delay: '<S78>/Delay One Step1' */
  Foc_model_Matlab_DW.DelayOneStep1_DSTATE = true;

  /* InitializeConditions for Delay: '<S78>/Delay One Step' */
  Foc_model_Matlab_DW.DelayOneStep_DSTATE = 500U;

  /* SystemInitialize for Atomic SubSystem: '<S67>/Atomic Hall Reading' */

  /* Start for S-Function (register_s32k_read): '<S69>/Read_Register' */
  PCC_SetClockMode(PCC, ADC0_CLK, true);

  /* End of SystemInitialize for SubSystem: '<S67>/Atomic Hall Reading' */
}

/* System reset for function-call system: '<Root>/CurrentControl' */
void Foc_mo_CurrentControl_Reset(void)
{
  /* InitializeConditions for Delay: '<S78>/Delay One Step1' */
  Foc_model_Matlab_DW.DelayOneStep1_DSTATE = true;

  /* InitializeConditions for Delay: '<S78>/Delay One Step' */
  Foc_model_Matlab_DW.DelayOneStep_DSTATE = 500U;
}

/* Disable for function-call system: '<Root>/CurrentControl' */
void Foc__CurrentControl_Disable(void)
{
  /* Disable for Enabled SubSystem: '<S22>/Subsystem' */
  Foc_model_Matlab_DW.Subsystem_MODE_n = false;

  /* End of Disable for SubSystem: '<S22>/Subsystem' */

  /* Disable for Enabled SubSystem: '<S23>/Subsystem' */
  Foc_model_Matlab_DW.Subsystem_MODE = false;

  /* End of Disable for SubSystem: '<S23>/Subsystem' */
}

/* Output and update for function-call system: '<Root>/CurrentControl' */
void Foc_model_Ma_CurrentControl(void)
{
  int32_T rtb_reset;
  int32_T rtb_reset_bp;
  real32_T V_ref_unsat;
  real32_T rtb_Add2;
  real32_T rtb_Add3;
  real32_T rtb_Gain2;
  real32_T rtb_Merge1;
  real32_T rtb_Merge1_p;
  real32_T rtb_Merge_a;
  uint32_T rtb_Add1;
  uint16_T rtb_DataStoreRead5;
  boolean_T OR;
  boolean_T rtb_Compare;
  boolean_T tmp;

  /* user code (Output function Body) */
  {
    /* Start of Profile Code */
    uint32_t tmp1;
    uint32_t tmp2;
    tmp1 = profiler_get_cnt();

    /* Start Profiling This Function.*/

    /* Outputs for Atomic SubSystem: '<S67>/Atomic Hall Reading' */
    /* S-Function (register_s32k_read): '<S69>/Read_Register' */

    /* read from ADC0_SC1A register */
    Foc_model_Matlab_B.Read_Register = *((uint32_t *) 0x4003B000);

    /* DataStoreRead: '<S69>/Data Store Read5' */
    rtb_DataStoreRead5 = HallStateChangeFlag;

    /* S-Function (gpio_s32k_input): '<S72>/Digital_Input_HALL_C' */

    /* GPIPORTA1 signal update */
    Foc_model_Matlab_B.Digital_Input_HALL_C = (PINS_DRV_ReadPins(PTA) >> 1) &
      0x01;

    /* Outputs for Atomic SubSystem: '<S72>/Bit Shift' */
    /* MATLAB Function: '<S73>/bit_shift' incorporates:
     *  DataTypeConversion: '<S72>/Data Type Conversion3'
     */
    /* MATLAB Function 'Logic and Bit Operations/Bit Shift/bit_shift': '<S75>:1' */
    /* '<S75>:1:6' */
    HALL_C_controller = (uint32_T)Foc_model_Matlab_B.Digital_Input_HALL_C << 2;

    /* End of Outputs for SubSystem: '<S72>/Bit Shift' */

    /* S-Function (gpio_s32k_input): '<S72>/Digital_Input_HALL_B' */

    /* GPIPORTD10 signal update */
    Foc_model_Matlab_B.Digital_Input_HALL_B = (PINS_DRV_ReadPins(PTD) >> 10) &
      0x01;

    /* Outputs for Atomic SubSystem: '<S72>/Bit Shift1' */
    /* MATLAB Function: '<S74>/bit_shift' incorporates:
     *  DataTypeConversion: '<S72>/Data Type Conversion2'
     */
    /* MATLAB Function 'Logic and Bit Operations/Bit Shift/bit_shift': '<S76>:1' */
    /* '<S76>:1:6' */
    HALL_B_controller = (uint32_T)Foc_model_Matlab_B.Digital_Input_HALL_B << 1;

    /* End of Outputs for SubSystem: '<S72>/Bit Shift1' */

    /* S-Function (gpio_s32k_input): '<S72>/Digital_Input_HALL_A' */

    /* GPIPORTD11 signal update */
    Foc_model_Matlab_B.Digital_Input_HALL_A = (PINS_DRV_ReadPins(PTD) >> 11) &
      0x01;

    /* DataTypeConversion: '<S72>/Data Type Conversion6' */
    HALL_A_controller = (uint32_T)Foc_model_Matlab_B.Digital_Input_HALL_A;

    /* Sum: '<S72>/Add1' */
    rtb_Add1 = (HALL_C_controller + HALL_B_controller) + HALL_A_controller;

    /* Switch: '<S70>/Switch' incorporates:
     *  Constant: '<S70>/WatchDog'
     *  DataStoreRead: '<S69>/Data Store Read5'
     *  DataStoreWrite: '<S67>/Data Store Write2'
     */
    if (HallStateChangeFlag != 0) {
      HallStateChangeFlag = 0U;
    }

    /* End of Switch: '<S70>/Switch' */
    /* End of Outputs for SubSystem: '<S67>/Atomic Hall Reading' */

    /* Logic: '<S78>/OR' incorporates:
     *  DataTypeConversion: '<S70>/Data Type Conversion4'
     *  Delay: '<S78>/Delay One Step1'
     */
    OR = (Foc_model_Matlab_DW.DelayOneStep1_DSTATE || (rtb_DataStoreRead5 != 0));

    /* Delay: '<S78>/Delay One Step' incorporates:
     *  DataTypeConversion: '<S70>/Data Type Conversion4'
     */
    if (OR) {
      if (rtb_DataStoreRead5 != 0) {
        Foc_model_Matlab_DW.DelayOneStep_DSTATE = 500U;
      }

      /* Delay: '<S78>/Delay One Step' incorporates:
       *  DataTypeConversion: '<S70>/Data Type Conversion4'
       */
      Foc_model_Matlab_B.DelayOneStep = Foc_model_Matlab_DW.DelayOneStep_DSTATE;
    }

    /* End of Delay: '<S78>/Delay One Step' */

    /* RelationalOperator: '<S82>/Compare' incorporates:
     *  Constant: '<S82>/Constant'
     */
    rtb_Compare = (Foc_model_Matlab_B.DelayOneStep > 0);

    /* Switch: '<S81>/watchdog check' incorporates:
     *  Constant: '<S81>/Constant'
     */
    if (rtb_Compare) {
      /* MinMax: '<S81>/Max' incorporates:
       *  DataStoreRead: '<S69>/Data Store Read2'
       *  DataTypeConversion: '<S70>/counterSize1'
       */
      if (GlobalSpeedCount >= (uint16_T)Foc_model_Matlab_B.Read_Register) {
        rtb_DataStoreRead5 = GlobalSpeedCount;
      } else {
        rtb_DataStoreRead5 = (uint16_T)Foc_model_Matlab_B.Read_Register;
      }

      /* Switch: '<S81>/speed check' incorporates:
       *  Constant: '<S81>/Constant'
       *  DataStoreRead: '<S69>/Data Store Read4'
       *  DataTypeConversion: '<S77>/Data Type Conversion'
       *  Logic: '<S77>/Logical Operator'
       *  MinMax: '<S81>/Max'
       */
      if (rtb_DataStoreRead5 >= 31250) {
        rtb_DataStoreRead5 = 0U;
      } else {
        rtb_DataStoreRead5 = (uint16_T)((GlobalSpeedValidity != 0) ||
          Foc_model_Matlab_B.validityDelay);
      }

      /* End of Switch: '<S81>/speed check' */
    } else {
      rtb_DataStoreRead5 = 0U;
    }

    /* End of Switch: '<S81>/watchdog check' */

    /* If: '<S70>/If' */
    if (rtb_DataStoreRead5 != 0) {
      /* Outputs for IfAction SubSystem: '<S70>/Speed and direction are valid Use speed to extrapolate position' incorporates:
       *  ActionPort: '<S80>/Action Port'
       */
      /* If: '<S80>/If' incorporates:
       *  DataStoreRead: '<S69>/Data Store Read3'
       */
      if (GlobalDirection > 0) {
        /* Outputs for IfAction SubSystem: '<S80>/If Action Subsystem' incorporates:
         *  ActionPort: '<S91>/Action Port'
         */
        /* SignalConversion generated from: '<S91>/In1' incorporates:
         *  DataStoreRead: '<S69>/Data Store Read2'
         *  DataTypeConversion: '<S80>/currentSpeedData'
         *  Gain: '<S80>/SpeedGain'
         *  Product: '<S80>/Divide'
         */
        rtb_Merge_a = Foc_model_Matlab_ConstB.SpeedConstData / (real32_T)
          GlobalSpeedCount * 0.05F;

        /* End of Outputs for SubSystem: '<S80>/If Action Subsystem' */
      } else {
        /* Outputs for IfAction SubSystem: '<S80>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S92>/Action Port'
         */
        /* UnaryMinus: '<S92>/Unary Minus' incorporates:
         *  DataStoreRead: '<S69>/Data Store Read2'
         *  DataTypeConversion: '<S80>/currentSpeedData'
         *  Gain: '<S80>/SpeedGain'
         *  Product: '<S80>/Divide'
         */
        rtb_Merge_a = -(Foc_model_Matlab_ConstB.SpeedConstData / (real32_T)
                        GlobalSpeedCount * 0.05F);

        /* End of Outputs for SubSystem: '<S80>/If Action Subsystem1' */
      }

      /* End of If: '<S80>/If' */

      /* Outputs for Enabled SubSystem: '<S80>/Subsystem1' incorporates:
       *  EnablePort: '<S93>/Enable'
       */
      /* Outputs for IfAction SubSystem: '<S93>/first_order' incorporates:
       *  ActionPort: '<S96>/Action Port'
       */
      /* If: '<S93>/If1' incorporates:
       *  DataStoreRead: '<S69>/Data Store Read2'
       *  DataTypeConversion: '<S70>/counterSize1'
       *  DataTypeConversion: '<S96>/countData'
       *  DataTypeConversion: '<S96>/currentSpeedData'
       *  Gain: '<S96>/Gain'
       *  Product: '<S96>/Divide'
       */
      rtb_Merge1 = (real32_T)(uint16_T)Foc_model_Matlab_B.Read_Register /
        (real32_T)GlobalSpeedCount * 0.166666672F;

      /* End of Outputs for SubSystem: '<S93>/first_order' */

      /* Saturate: '<S93>/Saturation' */
      if (rtb_Merge1 > 0.16667F) {
        rtb_Merge1 = 0.16667F;
      }

      /* End of Saturate: '<S93>/Saturation' */

      /* If: '<S93>/If' incorporates:
       *  DataStoreRead: '<S69>/Data Store Read3'
       */
      if (GlobalDirection != 1) {
        /* Outputs for IfAction SubSystem: '<S93>/-ve Direction' incorporates:
         *  ActionPort: '<S95>/Action Port'
         */
        /* SwitchCase: '<S95>/Switch Case' */
        switch ((int32_T)rtb_Add1) {
         case 6:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 1' incorporates:
           *  ActionPort: '<S105>/Action Port'
           */
          Foc_model_Matl_HallValueof1(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 1' */
          break;

         case 4:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 2' incorporates:
           *  ActionPort: '<S106>/Action Port'
           */
          Foc_model_Matl_HallValueof2(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 2' */
          break;

         case 5:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 3' incorporates:
           *  ActionPort: '<S107>/Action Port'
           */
          Foc_model_Matl_HallValueof3(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 3' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 4' incorporates:
           *  ActionPort: '<S108>/Action Port'
           */
          Foc_model_Matl_HallValueof4(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 4' */
          break;

         case 3:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 5' incorporates:
           *  ActionPort: '<S109>/Action Port'
           */
          Foc_model_Matl_HallValueof5(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 5' */
          break;

         case 2:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 6' incorporates:
           *  ActionPort: '<S110>/Action Port'
           */
          /* SignalConversion generated from: '<S110>/position' incorporates:
           *  Constant: '<S110>/Constant'
           */
          rtb_Merge1_p = 1.0F;

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 6' */
          break;

         default:
          /* Outputs for IfAction SubSystem: '<S95>/Hall Value of 7' incorporates:
           *  ActionPort: '<S111>/Action Port'
           */
          Foc_model_Matl_HallValueof7(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S95>/Hall Value of 7' */
          break;
        }

        /* End of SwitchCase: '<S95>/Switch Case' */

        /* Merge: '<S93>/Merge' incorporates:
         *  Sum: '<S95>/Sum'
         */
        Foc_model_Matlab_B.Merge = rtb_Merge1_p - rtb_Merge1;

        /* End of Outputs for SubSystem: '<S93>/-ve Direction' */
      } else {
        /* Outputs for IfAction SubSystem: '<S93>/+ve Direction' incorporates:
         *  ActionPort: '<S94>/Action Port'
         */
        /* SwitchCase: '<S94>/Switch Case' */
        switch ((int32_T)rtb_Add1) {
         case 6:
          /* Outputs for IfAction SubSystem: '<S94>/Hall Value of 1' incorporates:
           *  ActionPort: '<S98>/Action Port'
           */
          Foc_model_Matl_HallValueof7(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S94>/Hall Value of 1' */
          break;

         case 4:
          /* Outputs for IfAction SubSystem: '<S94>/Hall Value of 2' incorporates:
           *  ActionPort: '<S99>/Action Port'
           */
          Foc_model_Matl_HallValueof1(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S94>/Hall Value of 2' */
          break;

         case 5:
          /* Outputs for IfAction SubSystem: '<S94>/Hall Value of 3' incorporates:
           *  ActionPort: '<S100>/Action Port'
           */
          Foc_model_Matl_HallValueof2(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S94>/Hall Value of 3' */
          break;

         case 1:
          /* Outputs for IfAction SubSystem: '<S94>/Hall Value of 4' incorporates:
           *  ActionPort: '<S101>/Action Port'
           */
          Foc_model_Matl_HallValueof3(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S94>/Hall Value of 4' */
          break;

         case 3:
          /* Outputs for IfAction SubSystem: '<S94>/Hall Value of 5' incorporates:
           *  ActionPort: '<S102>/Action Port'
           */
          Foc_model_Matl_HallValueof4(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S94>/Hall Value of 5' */
          break;

         case 2:
          /* Outputs for IfAction SubSystem: '<S94>/Hall Value of 6' incorporates:
           *  ActionPort: '<S103>/Action Port'
           */
          Foc_model_Matl_HallValueof5(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S94>/Hall Value of 6' */
          break;

         default:
          /* Outputs for IfAction SubSystem: '<S94>/Hall Value of 7' incorporates:
           *  ActionPort: '<S104>/Action Port'
           */
          Foc_model_Matl_HallValueof7(&rtb_Merge1_p);

          /* End of Outputs for SubSystem: '<S94>/Hall Value of 7' */
          break;
        }

        /* End of SwitchCase: '<S94>/Switch Case' */

        /* Merge: '<S93>/Merge' incorporates:
         *  Sum: '<S94>/Sum'
         */
        Foc_model_Matlab_B.Merge = rtb_Merge1_p + rtb_Merge1;

        /* End of Outputs for SubSystem: '<S93>/+ve Direction' */
      }

      /* End of If: '<S93>/If' */
      /* End of Outputs for SubSystem: '<S80>/Subsystem1' */

      /* Merge: '<S70>/Merge1' incorporates:
       *  SignalConversion generated from: '<S80>/rawPosition'
       */
      ThetaHalls = Foc_model_Matlab_B.Merge;

      /* End of Outputs for SubSystem: '<S70>/Speed and direction are valid Use speed to extrapolate position' */
    } else {
      /* Outputs for IfAction SubSystem: '<S70>/Speed and direction are not valid Position will be set to the middle of the Hall quadrant' incorporates:
       *  ActionPort: '<S79>/Action Port'
       */
      /* SwitchCase: '<S83>/Switch Case' */
      switch ((int32_T)rtb_Add1) {
       case 6:
        /* Outputs for IfAction SubSystem: '<S83>/Hall Value of 1' incorporates:
         *  ActionPort: '<S84>/Action Port'
         */
        /* Merge: '<S70>/Merge1' incorporates:
         *  Constant: '<S84>/Constant'
         *  SignalConversion generated from: '<S84>/position'
         */
        ThetaHalls = 0.083333F;

        /* End of Outputs for SubSystem: '<S83>/Hall Value of 1' */
        break;

       case 4:
        /* Outputs for IfAction SubSystem: '<S83>/Hall Value of 2' incorporates:
         *  ActionPort: '<S85>/Action Port'
         */
        /* Merge: '<S70>/Merge1' incorporates:
         *  Constant: '<S85>/Constant'
         *  SignalConversion generated from: '<S85>/position'
         */
        ThetaHalls = 0.25F;

        /* End of Outputs for SubSystem: '<S83>/Hall Value of 2' */
        break;

       case 5:
        /* Outputs for IfAction SubSystem: '<S83>/Hall Value of 3' incorporates:
         *  ActionPort: '<S86>/Action Port'
         */
        /* Merge: '<S70>/Merge1' incorporates:
         *  Constant: '<S86>/Constant'
         *  SignalConversion generated from: '<S86>/position'
         */
        ThetaHalls = 0.41667F;

        /* End of Outputs for SubSystem: '<S83>/Hall Value of 3' */
        break;

       case 1:
        /* Outputs for IfAction SubSystem: '<S83>/Hall Value of 4' incorporates:
         *  ActionPort: '<S87>/Action Port'
         */
        /* Merge: '<S70>/Merge1' incorporates:
         *  Constant: '<S87>/Constant'
         *  SignalConversion generated from: '<S87>/position'
         */
        ThetaHalls = 0.58333F;

        /* End of Outputs for SubSystem: '<S83>/Hall Value of 4' */
        break;

       case 3:
        /* Outputs for IfAction SubSystem: '<S83>/Hall Value of 5' incorporates:
         *  ActionPort: '<S88>/Action Port'
         */
        /* Merge: '<S70>/Merge1' incorporates:
         *  Constant: '<S88>/Constant'
         *  SignalConversion generated from: '<S88>/position'
         */
        ThetaHalls = 0.75F;

        /* End of Outputs for SubSystem: '<S83>/Hall Value of 5' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S83>/Hall Value of 6' incorporates:
         *  ActionPort: '<S89>/Action Port'
         */
        /* Merge: '<S70>/Merge1' incorporates:
         *  Constant: '<S89>/Constant'
         *  SignalConversion generated from: '<S89>/position'
         */
        ThetaHalls = 0.91667F;

        /* End of Outputs for SubSystem: '<S83>/Hall Value of 6' */
        break;

       default:
        /* Outputs for IfAction SubSystem: '<S83>/Hall Value of 7' incorporates:
         *  ActionPort: '<S90>/Action Port'
         */
        Foc_model_Matl_HallValueof7((real32_T *)&ThetaHalls);

        /* End of Outputs for SubSystem: '<S83>/Hall Value of 7' */
        break;
      }

      /* End of SwitchCase: '<S83>/Switch Case' */

      /* SignalConversion generated from: '<S79>/Speed(r.p.m)' incorporates:
       *  Constant: '<S79>/Constant'
       */
      rtb_Merge_a = 0.0F;

      /* End of Outputs for SubSystem: '<S70>/Speed and direction are not valid Position will be set to the middle of the Hall quadrant' */
    }

    /* End of If: '<S70>/If' */

    /* Sum: '<S78>/Sum' */
    rtb_DataStoreRead5 = Foc_model_Matlab_B.DelayOneStep;

    /* If: '<S114>/If' incorporates:
     *  Constant: '<S113>/Constant1'
     *  Switch: '<S113>/Switch'
     */
    if (ThetaHalls <= 0.7061F) {
      /* Outputs for IfAction SubSystem: '<S114>/If Action Subsystem' incorporates:
       *  ActionPort: '<S116>/Action Port'
       */
      /* Sum: '<S116>/Add' incorporates:
       *  Constant: '<S116>/Constant'
       */
      rtb_Merge1_p = (ThetaHalls + 1.0F) - 0.7061F;

      /* End of Outputs for SubSystem: '<S114>/If Action Subsystem' */
    } else {
      /* Outputs for IfAction SubSystem: '<S114>/If Action Subsystem1' incorporates:
       *  ActionPort: '<S117>/Action Port'
       */
      /* Sum: '<S117>/Add' */
      rtb_Merge1_p = ThetaHalls - 0.7061F;

      /* End of Outputs for SubSystem: '<S114>/If Action Subsystem1' */
    }

    /* End of If: '<S114>/If' */

    /* Sum: '<S115>/Add' incorporates:
     *  Rounding: '<S115>/Floor'
     */
    rtb_Merge1 = rtb_Merge1_p - (real32_T)floor(rtb_Merge1_p);

    /* Sum: '<S68>/Add' incorporates:
     *  DataStoreRead: '<S120>/Data Store Read'
     *  DataStoreRead: '<S120>/Data Store Read1'
     *  DataStoreRead: '<S68>/Data Store Read'
     *  DataStoreRead: '<S68>/Data Store Read1'
     */
    I_ab_afterOffset[0] = IaOffset - ADC_A;
    I_ab_afterOffset[1] = IbOffset - ADC_B;

    /* Gain: '<S68>/Multiply' */
    rtb_Gain2 = 0.00048828125F * I_ab_afterOffset[0];
    rtb_Add3 = 0.00048828125F * I_ab_afterOffset[1];

    /* Outputs for Atomic SubSystem: '<S20>/Two phase CRL wrap' */
    /* Gain: '<S21>/one_by_sqrt3' incorporates:
     *  Sum: '<S21>/a_plus_2b'
     */
    rtb_Add3 = ((rtb_Gain2 + rtb_Add3) + rtb_Add3) * 0.577350259F;

    /* End of Outputs for SubSystem: '<S20>/Two phase CRL wrap' */

    /* If: '<S53>/If' incorporates:
     *  Constant: '<S55>/Constant'
     *  DataTypeConversion: '<S56>/Convert_back'
     *  DataTypeConversion: '<S56>/Convert_uint16'
     *  DataTypeConversion: '<S57>/Convert_back'
     *  DataTypeConversion: '<S57>/Convert_uint16'
     *  Gain: '<S51>/indexing'
     *  RelationalOperator: '<S55>/Compare'
     *  Sum: '<S56>/Sum'
     *  Sum: '<S57>/Sum'
     */
    if (rtb_Merge1 < 0.0F) {
      /* Outputs for IfAction SubSystem: '<S53>/If Action Subsystem' incorporates:
       *  ActionPort: '<S56>/Action Port'
       */
      rtb_Add2 = rtb_Merge1 - (real32_T)(int16_T)(real32_T)floor(rtb_Merge1);

      /* End of Outputs for SubSystem: '<S53>/If Action Subsystem' */
    } else {
      /* Outputs for IfAction SubSystem: '<S53>/If Action Subsystem1' incorporates:
       *  ActionPort: '<S57>/Action Port'
       */
      rtb_Add2 = rtb_Merge1 - (real32_T)(int16_T)rtb_Merge1;

      /* End of Outputs for SubSystem: '<S53>/If Action Subsystem1' */
    }

    rtb_Merge1_p = 800.0F * rtb_Add2;

    /* End of If: '<S53>/If' */

    /* Sum: '<S51>/Sum2' incorporates:
     *  DataTypeConversion: '<S51>/Data Type Conversion1'
     *  DataTypeConversion: '<S51>/Get_Integer'
     */
    rtb_Add2 = rtb_Merge1_p - (real32_T)(uint16_T)rtb_Merge1_p;

    /* Selector: '<S51>/Lookup' incorporates:
     *  Constant: '<S51>/sine_table_values'
     *  DataTypeConversion: '<S51>/Get_Integer'
     */
    rtb_Merge1 = Foc_model_Matlab_ConstP.sine_table_values_Value[(uint16_T)
      rtb_Merge1_p];

    /* Sum: '<S52>/Sum4' incorporates:
     *  Constant: '<S51>/offset'
     *  Constant: '<S51>/sine_table_values'
     *  DataTypeConversion: '<S51>/Get_Integer'
     *  Product: '<S52>/Product'
     *  Selector: '<S51>/Lookup'
     *  Sum: '<S51>/Sum'
     *  Sum: '<S52>/Sum3'
     */
    rtb_Merge1 += (Foc_model_Matlab_ConstP.sine_table_values_Value[(int32_T)
                   ((uint16_T)rtb_Merge1_p + 1U)] - rtb_Merge1) * rtb_Add2;

    /* Selector: '<S51>/Lookup' incorporates:
     *  Constant: '<S51>/offset'
     *  Constant: '<S51>/sine_table_values'
     *  DataTypeConversion: '<S51>/Get_Integer'
     *  Sum: '<S51>/Sum'
     *  Sum: '<S52>/Sum5'
     */
    V_ref_unsat = Foc_model_Matlab_ConstP.sine_table_values_Value[(int32_T)
      ((uint16_T)rtb_Merge1_p + 200U)];

    /* Sum: '<S52>/Sum6' incorporates:
     *  Constant: '<S51>/offset'
     *  Constant: '<S51>/sine_table_values'
     *  DataTypeConversion: '<S51>/Get_Integer'
     *  Product: '<S52>/Product1'
     *  Selector: '<S51>/Lookup'
     *  Sum: '<S51>/Sum'
     *  Sum: '<S52>/Sum5'
     */
    rtb_Merge1_p = (Foc_model_Matlab_ConstP.sine_table_values_Value[(int32_T)
                    ((uint16_T)rtb_Merge1_p + 201U)] - V_ref_unsat) * rtb_Add2 +
      V_ref_unsat;

    /* Outputs for Atomic SubSystem: '<S17>/Two inputs CRL' */
    /* Outputs for Atomic SubSystem: '<S20>/Two phase CRL wrap' */
    /* SignalConversion: '<S15>/Signal Copy' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S21>/a16'
     *  Product: '<S49>/asin'
     *  Product: '<S49>/bcos'
     *  Sum: '<S49>/sum_Qs'
     */
    Iq_fb = rtb_Add3 * rtb_Merge1_p - rtb_Gain2 * rtb_Merge1;

    /* SignalConversion: '<S15>/Signal Copy1' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S21>/a16'
     *  Product: '<S49>/acos'
     *  Product: '<S49>/bsin'
     *  Sum: '<S49>/sum_Ds'
     */
    Id_fb = rtb_Gain2 * rtb_Merge1_p + rtb_Add3 * rtb_Merge1;

    /* End of Outputs for SubSystem: '<S20>/Two phase CRL wrap' */
    /* End of Outputs for SubSystem: '<S17>/Two inputs CRL' */

    /* Gain: '<S22>/Gain2' */
    rtb_Gain2 = 31.25F * Id_fb;

    /* Outputs for Enabled SubSystem: '<S22>/Subsystem' incorporates:
     *  EnablePort: '<S29>/Enable'
     */
    /* UnitDelay: '<S22>/Unit Delay' */
    if (Foc_model_Matlab_DW.UnitDelay_DSTATE_j > 0.0) {
      if (!Foc_model_Matlab_DW.Subsystem_MODE_n) {
        Foc_model_Matlab_DW.Subsystem_RESET_ELAPS_T_k = true;

        /* Enable for DiscreteIntegrator: '<S29>/Discrete-Time Integrator' */
        Foc_model_Matlab_DW.DiscreteTimeIntegrator_SYSTEM_h = 1U;
        Foc_model_Matlab_DW.Subsystem_MODE_n = true;
      }

      if (Foc_model_Matlab_DW.Subsystem_RESET_ELAPS_T_k) {
        rtb_Add1 = 0U;
      } else {
        rtb_Add1 = Foc_model_Matlab_M->Timing.clockTick1 -
          Foc_model_Matlab_DW.Subsystem_PREV_T_f;
      }

      Foc_model_Matlab_DW.Subsystem_PREV_T_f =
        Foc_model_Matlab_M->Timing.clockTick1;
      Foc_model_Matlab_DW.Subsystem_RESET_ELAPS_T_k = false;

      /* DiscreteIntegrator: '<S29>/Discrete-Time Integrator' */
      if (Foc_model_Matlab_DW.DiscreteTimeIntegrator_SYSTEM_h == 0) {
        /* DiscreteIntegrator: '<S29>/Discrete-Time Integrator' */
        Foc_model_Matlab_DW.DiscreteTimeIntegrator_DSTAT_df += 0.0001F *
          (real32_T)rtb_Add1 *
          Foc_model_Matlab_DW.DiscreteTimeIntegrator_PREV_U_p;
      }

      /* End of DiscreteIntegrator: '<S29>/Discrete-Time Integrator' */

      /* SignalConversion generated from: '<S29>/Out1' */
      Foc_model_Matlab_B.OutportBufferForOut1_b =
        Foc_model_Matlab_DW.DiscreteTimeIntegrator_DSTAT_df;

      /* Update for DiscreteIntegrator: '<S29>/Discrete-Time Integrator' */
      Foc_model_Matlab_DW.DiscreteTimeIntegrator_SYSTEM_h = 0U;
      Foc_model_Matlab_DW.DiscreteTimeIntegrator_PREV_U_p = rtb_Gain2;
    } else {
      Foc_model_Matlab_DW.Subsystem_MODE_n = false;
    }

    /* End of UnitDelay: '<S22>/Unit Delay' */
    /* End of Outputs for SubSystem: '<S22>/Subsystem' */

    /* MATLAB Function: '<S22>/MATLAB Function2' incorporates:
     *  DataStoreRead: '<S28>/Data Store Read2'
     *  DataStoreRead: '<S28>/Data Store Read3'
     *  DataStoreRead: '<S28>/Data Store Read4'
     *  SignalConversion generated from: '<S27>/ SFunction '
     */
    /* MATLAB Function 'CurrentControl/Control_System/Current_Controllers/Controller_Id/MATLAB Function2': '<S27>:1' */
    /* '<S27>:1:4' */
    /* '<S27>:1:5' */
    /* '<S27>:1:6' */
    /* '<S27>:1:8' */
    /* '<S27>:1:11' */
    rtb_Add2 = Lambda_d * Foc_model_Matlab_B.OutportBufferForOut1_b + rtb_Gain2;

    /* '<S27>:1:14' */
    if ((real32_T)fabs(rtb_Add2) > Epsilon_d) {
      /* '<S27>:1:16' */
      /* '<S27>:1:17' */
      if (rtIsNaNF(rtb_Add2)) {
        rtb_Add2 = (rtNaNF);
      } else if (rtb_Add2 < 0.0F) {
        rtb_Add2 = -1.0F;
      } else {
        rtb_Add2 = (real32_T)(rtb_Add2 > 0.0F);
      }

      rtb_Add2 *= -Rho_d;
    } else {
      /* '<S27>:1:19' */
      rtb_Add2 = rtb_Add2 / Epsilon_d * -Rho_d;
    }

    /* Gain: '<S22>/Gain' incorporates:
     *  Gain: '<S22>/Gain3'
     *  Gain: '<S22>/Gain6'
     *  Gain: '<S23>/Gain'
     */
    /* '<S27>:1:22' */
    V_ref_unsat = 3295.0F * rtb_Merge_a * 0.104719758F * 2.0F;

    /* MATLAB Function: '<S22>/MATLAB Function2' incorporates:
     *  Constant: '<S22>/Constant8'
     *  Constant: '<S22>/Constant9'
     *  DataStoreRead: '<S28>/Data Store Read1'
     *  DataStoreRead: '<S28>/Data Store Read2'
     *  Gain: '<S22>/Gain'
     *  Gain: '<S22>/Gain1'
     *  SignalConversion generated from: '<S27>/ SFunction '
     */
    rtb_Gain2 = (0.598333359F * rtb_Gain2 - (31.25F * Iq_fb * V_ref_unsat +
      Lambda_d * rtb_Gain2) * 0.000435F) + rtb_Add2;
    if (rtb_Gain2 >= limit_d) {
      /* '<S27>:1:27' */
      /* '<S27>:1:28' */
      rtb_Gain2 = limit_d;

      /* '<S27>:1:29' */
      rtb_reset_bp = -1;
    } else if (rtb_Gain2 <= -limit_d) {
      /* '<S27>:1:30' */
      /* '<S27>:1:31' */
      rtb_Gain2 = -limit_d;

      /* '<S27>:1:32' */
      rtb_reset_bp = -1;
    } else {
      /* '<S27>:1:34' */
      rtb_reset_bp = 1;
    }

    /* Gain: '<S23>/Gain3' */
    /* '<S27>:1:37' */
    /* '<S27>:1:39' */
    rtb_Add3 = 31.25F * Iq_fb;

    /* Outputs for Enabled SubSystem: '<S23>/Subsystem' incorporates:
     *  EnablePort: '<S32>/Enable'
     */
    /* UnitDelay: '<S23>/Unit Delay' */
    if (Foc_model_Matlab_DW.UnitDelay_DSTATE_f > 0.0) {
      if (!Foc_model_Matlab_DW.Subsystem_MODE) {
        Foc_model_Matlab_DW.Subsystem_RESET_ELAPS_T = true;

        /* Enable for DiscreteIntegrator: '<S32>/Discrete-Time Integrator' */
        Foc_model_Matlab_DW.DiscreteTimeIntegrator_SYSTEM_E = 1U;
        Foc_model_Matlab_DW.Subsystem_MODE = true;
      }

      if (Foc_model_Matlab_DW.Subsystem_RESET_ELAPS_T) {
        rtb_Add1 = 0U;
      } else {
        rtb_Add1 = Foc_model_Matlab_M->Timing.clockTick1 -
          Foc_model_Matlab_DW.Subsystem_PREV_T;
      }

      /* DiscreteIntegrator: '<S32>/Discrete-Time Integrator' */
      if (Foc_model_Matlab_DW.DiscreteTimeIntegrator_SYSTEM_E != 0) {
        /* DiscreteIntegrator: '<S32>/Discrete-Time Integrator' */
        Foc_model_Matlab_B.DiscreteTimeIntegrator_d =
          Foc_model_Matlab_DW.DiscreteTimeIntegrator_DSTATE_d;
      } else {
        /* DiscreteIntegrator: '<S32>/Discrete-Time Integrator' */
        Foc_model_Matlab_B.DiscreteTimeIntegrator_d = 0.0001F * (real32_T)
          rtb_Add1 * Foc_model_Matlab_DW.DiscreteTimeIntegrator_PREV_U +
          Foc_model_Matlab_DW.DiscreteTimeIntegrator_DSTATE_d;
      }

      /* End of DiscreteIntegrator: '<S32>/Discrete-Time Integrator' */

      /* SignalConversion generated from: '<S32>/Out1' */
      Foc_model_Matlab_B.OutportBufferForOut1 =
        Foc_model_Matlab_B.DiscreteTimeIntegrator_d;
    } else {
      Foc_model_Matlab_DW.Subsystem_MODE = false;
    }

    /* End of UnitDelay: '<S23>/Unit Delay' */
    /* End of Outputs for SubSystem: '<S23>/Subsystem' */

    /* Sum: '<S23>/Sum1' incorporates:
     *  Gain: '<S23>/Gain2'
     */
    Iq_err = rtb_Add3 - 31.25F * Iq_ref_PU;

    /* MATLAB Function: '<S23>/MATLAB Function2' incorporates:
     *  Constant: '<S23>/Constant10'
     *  Constant: '<S23>/Constant8'
     *  Constant: '<S23>/Constant9'
     *  DataStoreRead: '<S31>/Data Store Read1'
     *  DataStoreRead: '<S31>/Data Store Read2'
     *  DataStoreRead: '<S31>/Data Store Read3'
     *  DataStoreRead: '<S31>/Data Store Read4'
     *  Gain: '<S23>/Gain1'
     *  SignalConversion generated from: '<S30>/ SFunction '
     */
    /* MATLAB Function 'CurrentControl/Control_System/Current_Controllers/Controller_Iq/MATLAB Function2': '<S30>:1' */
    /* '<S30>:1:2' */
    /* '<S30>:1:3' */
    /* '<S30>:1:4' */
    /* '<S30>:1:5' */
    /* '<S30>:1:12' */
    rtb_Add2 = Lambda_q * Foc_model_Matlab_B.OutportBufferForOut1 + Iq_err;
    if ((real32_T)fabs(rtb_Add2) > Epsilon_q) {
      /* '<S30>:1:15' */
      /* '<S30>:1:16' */
      if (rtIsNaNF(rtb_Add2)) {
        rtb_Add2 = (rtNaNF);
      } else if (rtb_Add2 < 0.0F) {
        rtb_Add2 = -1.0F;
      } else {
        rtb_Add2 = (real32_T)(rtb_Add2 > 0.0F);
      }

      rtb_Add2 *= -Rho_q;
    } else {
      /* '<S30>:1:18' */
      rtb_Add2 = rtb_Add2 / Epsilon_q * -Rho_q;
    }

    /* '<S30>:1:22' */
    /* '<S30>:1:23' */
    V_ref_unsat = ((31.25F * Id_fb * V_ref_unsat - Lambda_q * Iq_err) *
                   0.000375F + (0.598333359F * rtb_Add3 + 0.00799426902F *
      V_ref_unsat)) + rtb_Add2;
    if (V_ref_unsat >= limit_q) {
      /* '<S30>:1:28' */
      /* '<S30>:1:29' */
      V_ref_unsat = limit_q;

      /* '<S30>:1:30' */
      rtb_reset = -1;
    } else if (V_ref_unsat <= -limit_q) {
      /* '<S30>:1:31' */
      /* '<S30>:1:32' */
      V_ref_unsat = -limit_q;

      /* '<S30>:1:33' */
      rtb_reset = -1;
    } else {
      /* '<S30>:1:35' */
      rtb_reset = 1;
    }

    /* Gain: '<S26>/Gain1' */
    /* '<S30>:1:38' */
    /* '<S30>:1:40' */
    rtb_Add2 = 2.0F * rtb_Merge_a;

    /* Sum: '<S15>/Sum1' incorporates:
     *  Constant: '<S26>/Constant4'
     *  DataTypeConversion: '<S26>/Cast To Single'
     *  Gain: '<S23>/Gain5'
     *  Gain: '<S26>/Gain'
     *  MATLAB Function: '<S23>/MATLAB Function2'
     *  Product: '<S26>/Product'
     *  Product: '<S26>/Product1'
     */
    rtb_Add3 = (0.144337565F * V_ref_unsat + (real32_T)(0.0079942689836159844 *
      rtb_Add2)) + rtb_Add2 * Id_fb * 0.000375F;

    /* Sum: '<S15>/Sum' incorporates:
     *  Gain: '<S22>/Gain4'
     *  Gain: '<S25>/Gain'
     *  Gain: '<S25>/Gain1'
     *  MATLAB Function: '<S22>/MATLAB Function2'
     *  Product: '<S25>/Product'
     */
    rtb_Gain2 = 0.144337565F * rtb_Gain2 - 2.0F * rtb_Merge_a * Iq_fb *
      0.000435F;

    /* Sum: '<S36>/Sum1' incorporates:
     *  Product: '<S36>/Product'
     *  Product: '<S36>/Product1'
     */
    rtb_Add2 = rtb_Gain2 * rtb_Gain2 + rtb_Add3 * rtb_Add3;

    /* Outputs for IfAction SubSystem: '<S24>/D-Q Equivalence' incorporates:
     *  ActionPort: '<S33>/Action Port'
     */
    /* If: '<S33>/If' incorporates:
     *  If: '<S24>/If'
     *  RelationalOperator: '<S33>/Relational Operator'
     */
    if (rtb_Add2 > 0.9025F) {
      /* Outputs for IfAction SubSystem: '<S33>/Limiter' incorporates:
       *  ActionPort: '<S37>/Action Port'
       */
      /* Sqrt: '<S37>/Square Root' */
      rtb_Add2 = (real32_T)sqrt(rtb_Add2);

      /* Product: '<S37>/Divide' incorporates:
       *  Constant: '<S35>/Constant3'
       *  Product: '<S37>/Product'
       *  Switch: '<S35>/Switch'
       *  Switch: '<S37>/Switch'
       */
      rtb_Gain2 = rtb_Gain2 * 0.95F / rtb_Add2;
      rtb_Add3 = rtb_Add3 * 0.95F / rtb_Add2;

      /* End of Outputs for SubSystem: '<S33>/Limiter' */
    }

    /* End of If: '<S33>/If' */
    /* End of Outputs for SubSystem: '<S24>/D-Q Equivalence' */

    /* Outputs for Atomic SubSystem: '<S16>/Two inputs CRL' */
    /* Switch: '<S48>/Switch' incorporates:
     *  Product: '<S47>/dcos'
     *  Product: '<S47>/qsin'
     *  Sum: '<S47>/sum_alpha'
     */
    V_ref_unsat = rtb_Gain2 * rtb_Merge1_p - rtb_Add3 * rtb_Merge1;

    /* Gain: '<S65>/one_by_two' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S47>/a16'
     */
    rtb_Add2 = 0.5F * V_ref_unsat;

    /* Gain: '<S65>/sqrt3_by_two' incorporates:
     *  Product: '<S47>/dsin'
     *  Product: '<S47>/qcos'
     *  Sum: '<S47>/sum_beta'
     */
    rtb_Merge1 = (rtb_Add3 * rtb_Merge1_p + rtb_Gain2 * rtb_Merge1) *
      0.866025388F;

    /* End of Outputs for SubSystem: '<S16>/Two inputs CRL' */

    /* Sum: '<S65>/add_b' */
    rtb_Merge1_p = rtb_Merge1 - rtb_Add2;

    /* Sum: '<S65>/add_c' */
    rtb_Add2 = (0.0F - rtb_Add2) - rtb_Merge1;

    /* MinMax: '<S62>/Max' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S47>/a16'
     *  MinMax: '<S62>/Min'
     */
    tmp = rtIsNaNF(rtb_Merge1_p);

    /* Outputs for Atomic SubSystem: '<S16>/Two inputs CRL' */
    if ((V_ref_unsat >= rtb_Merge1_p) || tmp) {
      rtb_Merge1 = V_ref_unsat;
    } else {
      rtb_Merge1 = rtb_Merge1_p;
    }

    /* MinMax: '<S62>/Min' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S47>/a16'
     */
    if ((V_ref_unsat <= rtb_Merge1_p) || tmp) {
      rtb_Gain2 = V_ref_unsat;
    } else {
      rtb_Gain2 = rtb_Merge1_p;
    }

    /* End of Outputs for SubSystem: '<S16>/Two inputs CRL' */

    /* MinMax: '<S62>/Max' incorporates:
     *  MinMax: '<S62>/Min'
     */
    tmp = !rtIsNaNF(rtb_Add2);
    if ((!(rtb_Merge1 >= rtb_Add2)) && tmp) {
      rtb_Merge1 = rtb_Add2;
    }

    /* MinMax: '<S62>/Min' */
    if ((!(rtb_Gain2 <= rtb_Add2)) && tmp) {
      rtb_Gain2 = rtb_Add2;
    }

    /* Gain: '<S62>/one_by_two' incorporates:
     *  MinMax: '<S62>/Max'
     *  MinMax: '<S62>/Min'
     *  Sum: '<S62>/Add'
     */
    rtb_Add3 = (rtb_Merge1 + rtb_Gain2) * -0.5F;

    /* Outputs for Atomic SubSystem: '<S16>/Two inputs CRL' */
    /* Gain: '<S12>/Gain' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S47>/a16'
     *  Constant: '<S12>/Constant1'
     *  Gain: '<S61>/Gain'
     *  Sum: '<S12>/Sum1'
     *  Sum: '<S61>/Add1'
     *  Sum: '<S61>/Add2'
     *  Sum: '<S61>/Add3'
     */
    PWM_Duty_Cycles[0] = ((V_ref_unsat + rtb_Add3) * 1.15470052F + 1.0F) * 0.5F;

    /* End of Outputs for SubSystem: '<S16>/Two inputs CRL' */
    PWM_Duty_Cycles[1] = ((rtb_Merge1_p + rtb_Add3) * 1.15470052F + 1.0F) * 0.5F;
    PWM_Duty_Cycles[2] = ((rtb_Add3 + rtb_Add2) * 1.15470052F + 1.0F) * 0.5F;

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

    /* Update for Delay: '<S78>/Delay One Step1' */
    Foc_model_Matlab_DW.DelayOneStep1_DSTATE = rtb_Compare;

    /* Update for Delay: '<S78>/Delay One Step' incorporates:
     *  Constant: '<S78>/Constant2'
     *  Sum: '<S78>/Sum'
     */
    if (OR) {
      Foc_model_Matlab_DW.DelayOneStep_DSTATE = (uint16_T)(rtb_DataStoreRead5 -
        1);
    }

    /* End of Update for Delay: '<S78>/Delay One Step' */

    /* Update for UnitDelay: '<S22>/Unit Delay' */
    Foc_model_Matlab_DW.UnitDelay_DSTATE_j = rtb_reset_bp;

    /* Update for UnitDelay: '<S23>/Unit Delay' */
    Foc_model_Matlab_DW.UnitDelay_DSTATE_f = rtb_reset;

    /* Update for Enabled SubSystem: '<S23>/Subsystem' incorporates:
     *  EnablePort: '<S32>/Enable'
     */
    Foc_model_Matlab_DW.Subsystem_PREV_T = Foc_model_Matlab_M->Timing.clockTick1;
    Foc_model_Matlab_DW.Subsystem_RESET_ELAPS_T = false;
    if (Foc_model_Matlab_DW.Subsystem_MODE) {
      /* Update for DiscreteIntegrator: '<S32>/Discrete-Time Integrator' */
      Foc_model_Matlab_DW.DiscreteTimeIntegrator_SYSTEM_E = 0U;
      Foc_model_Matlab_DW.DiscreteTimeIntegrator_DSTATE_d =
        Foc_model_Matlab_B.DiscreteTimeIntegrator_d;
      Foc_model_Matlab_DW.DiscreteTimeIntegrator_PREV_U = Iq_err;
    }

    /* End of Update for SubSystem: '<S23>/Subsystem' */

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
  real32_T rtb_Gain1_n;
  real32_T rtb_Uk1;
  real32_T s;

  /* user code (Output function Body) */
  {
    /* Start of Profile Code */
    uint32_t tmp1;
    uint32_t tmp2;
    tmp1 = profiler_get_cnt();

    /* Start Profiling This Function.*/

    /* Outputs for Atomic SubSystem: '<S8>/Speed_Ref_Selector' */
    /* Switch: '<S154>/Switch' incorporates:
     *  DataStoreRead: '<S154>/Data Store Read1'
     *  DataTypeConversion: '<S154>/Data Type Conversion'
     */
    if ((real32_T)Enable > 0.5F) {
      /* Switch: '<S154>/Switch' */
      Speed_Ref = Speed_Ref_PU;
    } else {
      /* Switch: '<S154>/Switch' */
      Speed_Ref = Speed_fb;
    }

    /* End of Switch: '<S154>/Switch' */
    /* End of Outputs for SubSystem: '<S8>/Speed_Ref_Selector' */

    /* Outputs for Atomic SubSystem: '<S8>/Contrloller_wr' */
    /* Gain: '<S156>/Gain' incorporates:
     *  Gain: '<S153>/Gain'
     *  Gain: '<S153>/Gain6'
     */
    rtb_Uk1 = 0.104719758F * Speed_Ref * 3295.0F * 2.0F;

    /* Gain: '<S156>/Gain1' incorporates:
     *  Gain: '<S153>/Gain1'
     *  Gain: '<S153>/Gain3'
     */
    rtb_Gain1_n = 0.104719758F * Speed_fb * 3295.0F * 2.0F;

    /* Sum: '<S156>/Sum' */
    SpeedError = rtb_Gain1_n - rtb_Uk1;

    /* Outputs for Enabled SubSystem: '<S156>/Subsystem' incorporates:
     *  EnablePort: '<S159>/reset'
     */
    /* UnitDelay: '<S153>/Unit Delay' */
    if (Foc_model_Matlab_DW.UnitDelay_DSTATE > 0.0) {
      /* DiscreteIntegrator: '<S159>/Discrete-Time Integrator' */
      Foc_model_Matlab_B.DiscreteTimeIntegrator =
        Foc_model_Matlab_DW.DiscreteTimeIntegrator_DSTATE;

      /* Update for DiscreteIntegrator: '<S159>/Discrete-Time Integrator' */
      Foc_model_Matlab_DW.DiscreteTimeIntegrator_DSTATE += 0.001F * SpeedError;
    }

    /* End of UnitDelay: '<S153>/Unit Delay' */
    /* End of Outputs for SubSystem: '<S156>/Subsystem' */

    /* SampleTimeMath: '<S158>/TSamp'
     *
     * About '<S158>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     *   */
    rtb_Uk1 *= 1000.0F;

    /* MATLAB Function: '<S153>/MATLAB Function' incorporates:
     *  DataStoreRead: '<S157>/Data Store Read1'
     *  DataStoreRead: '<S157>/Data Store Read2'
     *  DataStoreRead: '<S157>/Data Store Read3'
     *  DataStoreRead: '<S157>/Data Store Read4'
     *  SignalConversion generated from: '<S155>/ SFunction '
     *  Sum: '<S158>/Diff'
     *  UnitDelay: '<S158>/UD'
     *
     * Block description for '<S158>/Diff':
     *
     *  Add in CPU
     *
     * Block description for '<S158>/UD':
     *
     *  Store in Global RAM
     */
    /* MATLAB Function 'SpeedControl/Contrloller_wr/MATLAB Function': '<S155>:1' */
    /* '<S155>:1:6' */
    /* '<S155>:1:7' */
    /* '<S155>:1:8' */
    /* '<S155>:1:9' */
    /* '<S155>:1:12' */
    s = Lambda_w * Foc_model_Matlab_B.DiscreteTimeIntegrator + SpeedError;
    if ((real32_T)fabs(s) > Epsilon_w) {
      /* '<S155>:1:15' */
      /* '<S155>:1:16' */
      if (rtIsNaNF(s)) {
        s = (rtNaNF);
      } else if (s < 0.0F) {
        s = -1.0F;
      } else {
        s = (real32_T)(s > 0.0F);
      }

      s *= -Rho_w;
    } else {
      /* '<S155>:1:18' */
      s = s / Epsilon_w * -Rho_w;
    }

    /* '<S155>:1:22' */
    /* '<S155>:1:25' */
    /* '<S155>:1:27' */
    rtb_Gain1_n = ((((rtb_Uk1 - Foc_model_Matlab_DW.UD_DSTATE) - Lambda_w *
                     SpeedError) * Foc_model_Matlab_ConstB.CastToSingle +
                    Foc_model_Matlab_ConstB.CastToSingle1 * rtb_Gain1_n) + s) /
      Foc_model_Matlab_ConstB.CastToSingle2;
    if (rtb_Gain1_n >= limit_w) {
      /* '<S155>:1:30' */
      /* '<S155>:1:31' */
      rtb_Gain1_n = limit_w;

      /* Update for UnitDelay: '<S153>/Unit Delay' */
      /* '<S155>:1:32' */
      Foc_model_Matlab_DW.UnitDelay_DSTATE = -1.0;
    } else if (rtb_Gain1_n <= -limit_w) {
      /* '<S155>:1:33' */
      /* '<S155>:1:34' */
      rtb_Gain1_n = -limit_w;

      /* Update for UnitDelay: '<S153>/Unit Delay' */
      /* '<S155>:1:35' */
      Foc_model_Matlab_DW.UnitDelay_DSTATE = -1.0;
    } else {
      /* Update for UnitDelay: '<S153>/Unit Delay' */
      /* '<S155>:1:37' */
      Foc_model_Matlab_DW.UnitDelay_DSTATE = 1.0;
    }

    /* Gain: '<S153>/Gain2' incorporates:
     *  MATLAB Function: '<S153>/MATLAB Function'
     */
    /* '<S155>:1:40' */
    /* '<S155>:1:42' */
    Foc_model_Matlab_B.Gain2 = 0.032F * rtb_Gain1_n;

    /* Update for UnitDelay: '<S158>/UD'
     *
     * Block description for '<S158>/UD':
     *
     *  Store in Global RAM
     */
    Foc_model_Matlab_DW.UD_DSTATE = rtb_Uk1;

    /* End of Outputs for SubSystem: '<S8>/Contrloller_wr' */

    /* user code (Output function Trailer) */

    /* Profile Code : Compute function execution time in us. */
    tmp2 = profiler_get_cnt();
    profile_buffer[1] = gt_pf(tmp1, tmp2);

    /* End of Profile Code */
  }
}

/* Model step function for TID0 */
void Foc_model_Matlab_step0(void)      /* Sample time: [5.0E-5s, 0.0s] */
{
  {                                    /* Sample time: [5.0E-5s, 0.0s] */
    rate_monotonic_scheduler();
  }
}

/* Model step function for TID1 */
void Foc_model_Matlab_step1(void)      /* Sample time: [0.0001s, 0.0s] */
{
  /* End of Outputs for S-Function (ftm_s32k_hall_sensor): '<S3>/FTM_Hall_Sensor' */

  /* DataStoreWrite: '<S3>/Data Store Write1' */
  HallCntActual = CntHall;

  /* RateTransition: '<Root>/RT11' */
  Iq_ref_PU = Foc_model_Matlab_DW.RT11_Buffer0;

  /* End of Outputs for S-Function (pdb_s32k_isr): '<S140>/PDB1_ISR' */
  /* End of Outputs for S-Function (adc_s32k_isr): '<S140>/ADC1_ISR' */

  /* DataStoreWrite: '<S140>/Data Store Write' incorporates:
   *  DataTypeConversion: '<S140>/Data Type Conversion'
   */
  ADC_A = (real32_T)ADC_IA;

  /* DataStoreWrite: '<S140>/Data Store Write1' incorporates:
   *  DataTypeConversion: '<S140>/Data Type Conversion'
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
     *  ActionPort: '<S149>/Action Port'
     */
    /* S-Function (gpio_s32k_output): '<S149>/LED_GREEN_ON' incorporates:
     *  Constant: '<S149>/LED_GREEN'
     */

    /* GPOPORTD16 Data Signal Update */
    if (false) {
      PINS_DRV_SetPins(PTD, 1UL<<16);
    } else {
      PINS_DRV_ClearPins(PTD, 1UL<<16);
    }

    /* S-Function (gpio_s32k_output): '<S149>/LED_RED_OFF' incorporates:
     *  Constant: '<S149>/LED_RED'
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
     *  ActionPort: '<S147>/Action Port'
     */
    /* S-Function (gpio_s32k_output): '<S147>/LED_GREEN' incorporates:
     *  Constant: '<S147>/OFF'
     */

    /* GPOPORTD16 Data Signal Update */
    if (true) {
      PINS_DRV_SetPins(PTD, 1UL<<16);
    } else {
      PINS_DRV_ClearPins(PTD, 1UL<<16);
    }

    /* S-Function (gpio_s32k_output): '<S147>/LED_RED' incorporates:
     *  Constant: '<S147>/ON'
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
  if (Foc_model_Matlab_DW.is_active_c1_Foc_model_Matlab == 0) {
    /* Entry: Hardware Initialization/Enable PDB and start FTM */
    Foc_model_Matlab_DW.is_active_c1_Foc_model_Matlab = 1U;

    /* Entry Internal: Hardware Initialization/Enable PDB and start FTM */
    /* Transition: '<S146>:10' */
    Foc_model_Matlab_DW.is_c1_Foc_model_Matlab = Foc_model_Matlab_IN_A;
  } else if (Foc_model_Matlab_DW.is_c1_Foc_model_Matlab == Foc_model_Matlab_IN_A)
  {
    /* Outputs for Function Call SubSystem: '<S4>/enable_FTM_PDB_ADC_triggering' */
    /* During 'A': '<S146>:5' */
    /* Transition: '<S146>:103' */
    /* Event: '<S146>:104' */

    /* S-Function (ftm_s32k_pwm_disen): '<S150>/FTM_PWM_Disable_Enable' */
    FTM_DRV_InitPwm(FTM_PWM3, &flexTimer_pwm3_PwmConfig);

    /* S-Function (ftm_s32k_init_disen): '<S150>/FTM_Init_Trigger_Disable_Enable' */

    /* FTM PWM Initialization Trigger Enable Disable*/
    FTM_DRV_SetInitTriggerCmd(FTM3, true);

    /* S-Function (pdb_s32k_enable): '<S150>/PDB0_Enable' */

    /* Enable PDB Module0 */
    PDB_DRV_Enable(0);

    /* S-Function (pdb_s32k_enable): '<S150>/PDB1_Enable' */

    /* Enable PDB Module1 */
    PDB_DRV_Enable(1);

    /* S-Function (tpp_s32k_isr_enable): '<S150>/TPP_ISR_Enable_Disable' */
    tpp_interrupt_enable(15);

    /* user code (Output function Trailer) */

    /* System '<S4>/enable_FTM_PDB_ADC_triggering' */
    PDB_DRV_LoadValuesCmd(0);
    PDB_DRV_LoadValuesCmd(1);

    /* End of Outputs for SubSystem: '<S4>/enable_FTM_PDB_ADC_triggering' */
    Foc_model_Matlab_DW.is_c1_Foc_model_Matlab = Foc_model_Matlab_IN_END;
  } else {
    /* During 'END': '<S146>:39' */
    /* Transition: '<S146>:41' */
    /* Event: '<S146>:36' */
    Foc_model_Matlab_DW.is_c1_Foc_model_Matlab = Foc_model_Matlab_IN_END;
  }

  /* End of Chart: '<S4>/Enable PDB and start FTM' */
  /* End of Outputs for S-Function (tpp_s32k_isr): '<S4>/GD300_ISR_Callback ' */

  /* Update absolute time */
  /* The "clockTick1" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.0001, which is the step size
   * of the task. Size of "clockTick1" ensures timer will not overflow during the
   * application lifespan selected.
   */
  Foc_model_Matlab_M->Timing.clockTick1++;
}

/* Model step function for TID2 */
void Foc_model_Matlab_step2(void)      /* Sample time: [0.001s, 0.0s] */
{
  /* RateTransition: '<Root>/RT2' */
  Speed_Ref_PU = Foc_model_Matlab_DW.RT2_Buffer0;

  /* RateTransition: '<Root>/RT1' */
  Speed_fb = SpeedMeasured;

  /* Outputs for Atomic SubSystem: '<Root>/SpeedControl' */
  Foc_model_Matl_SpeedControl();

  /* End of Outputs for SubSystem: '<Root>/SpeedControl' */

  /* RateTransition: '<Root>/RT11' */
  Foc_model_Matlab_DW.RT11_Buffer0 = Foc_model_Matlab_B.Gain2;
}

/* Model step function for TID3 */
void Foc_model_Matlab_step3(void)      /* Sample time: [0.1s, 0.0s] */
{
  /* S-Function (fcgen): '<S3>/SCI_Rx_INT' incorporates:
   *  SubSystem: '<Root>/Serial Receive'
   */
  /* RateTransition: '<Root>/RT2' incorporates:
   *  DataStoreRead: '<S152>/Data Store Read2'
   *  Gain: '<S152>/rpm2PU'
   */
  Foc_model_Matlab_DW.RT2_Buffer0 = 0.0005F * DesiredSpeed;

  /* End of Outputs for S-Function (fcgen): '<S3>/SCI_Rx_INT' */
}

/* Model initialize function */
void Foc_model_Matlab_initialize(void)
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

  /* Start for DataStoreMemory: '<Root>/Data Store Memory14' */
  Epsilon_q = 90.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory15' */
  Lambda_q = 95.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory16' */
  Epsilon_d = 2.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory17' */
  Lambda_d = 80.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory18' */
  Rho_d = 5.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory19' */
  limit_d = 1.38564062F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory21' */
  limit_q = 18.7061481F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory5' */
  IaOffset = 2040.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory6' */
  IbOffset = 2040.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory8' */
  Rho_q = 55.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory10' */
  Rho_w = 0.25F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory13' */
  Epsilon_w = 60.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory20' */
  limit_w = 5.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory9' */
  Lambda_w = 5.0F;

  /* SystemInitialize for S-Function (ftm_s32k_hall_sensor): '<S3>/FTM_Hall_Sensor' incorporates:
   *  SubSystem: '<Root>/Hall Sensor'
   */
  /* System initialize for function-call system: '<Root>/Hall Sensor' */

  /* Start for S-Function (gpio_s32k_input): '<S121>/Digital_Input_HALL_A' */
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

  /* Start for S-Function (gpio_s32k_input): '<S121>/Digital_Input_HALL_B' */
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

  /* Start for S-Function (gpio_s32k_input): '<S121>/Digital_Input_HALL_C' */
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
  /* SystemInitialize for S-Function (pdb_s32k_isr): '<S140>/PDB1_ISR' */

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

  /* SystemInitialize for S-Function (adc_s32k_isr): '<S140>/ADC1_ISR' incorporates:
   *  SubSystem: '<S140>/ADC1_IRQHandler'
   */
  /* System initialize for function-call system: '<S140>/ADC1_IRQHandler' */

  /* Start for S-Function (adc_s32k_start): '<S141>/ADC_AD4_IA' */
  {
    adc_chan_config_t adc0_chan_cfg = {
      .interruptEnable = false,
      .channel = ADC_INPUTCHAN_EXT4
    };

    /* Initialize channel configuration of ADC0. */
    ADC_DRV_ConfigChan(0, 0, &adc0_chan_cfg);
  }

  /* Start for S-Function (adc_s32k_start): '<S141>/ADC_AD7_VDC' */
  {
    adc_chan_config_t adc1_chan_cfg = {
      .interruptEnable = false,
      .channel = ADC_INPUTCHAN_EXT7
    };

    /* Initialize channel configuration of ADC1. */
    ADC_DRV_ConfigChan(1, 0, &adc1_chan_cfg);
  }

  /* Start for S-Function (adc_s32k_start): '<S141>/ADC_AD6_IDC' */
  {
    adc_chan_config_t adc1_chan_cfg = {
      .interruptEnable = false,
      .channel = ADC_INPUTCHAN_EXT6
    };

    /* Initialize channel configuration of ADC1. */
    ADC_DRV_ConfigChan(1, 1, &adc1_chan_cfg);
  }

  /* SystemInitialize for S-Function (fcgen): '<S141>/Function-Call Generator' incorporates:
   *  SubSystem: '<Root>/CurrentControl'
   */
  Foc_mod_CurrentControl_Init();

  /* End of SystemInitialize for S-Function (fcgen): '<S141>/Function-Call Generator' */
  ADC_InstallCallback(1, 2U, ADC1_SC1reg2U_callback);

  /* Set ADC1 interrupt priority */
  INT_SYS_SetPriority(ADC1_IRQn, 5);

  /* Enable ADC1 interrupt */
  INT_SYS_EnableIRQ(ADC1_IRQn);

  /* SystemInitialize for IfAction SubSystem: '<S4>/FAULT' */
  /* Start for S-Function (gpio_s32k_output): '<S147>/LED_GREEN' incorporates:
   *  Constant: '<S147>/OFF'
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

  /* Start for S-Function (gpio_s32k_output): '<S147>/LED_RED' incorporates:
   *  Constant: '<S147>/ON'
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
void Foc_model_Matlab_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
