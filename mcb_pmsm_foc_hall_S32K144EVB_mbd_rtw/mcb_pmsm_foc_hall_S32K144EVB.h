/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: mcb_pmsm_foc_hall_S32K144EVB.h
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

#ifndef mcb_pmsm_foc_hall_S32K144EVB_h_
#define mcb_pmsm_foc_hall_S32K144EVB_h_
#ifndef mcb_pmsm_foc_hall_S32K144EVB_COMMON_INCLUDES_
#define mcb_pmsm_foc_hall_S32K144EVB_COMMON_INCLUDES_
#include <string.h>
#include "rtwtypes.h"
#include "pcc_hw_access.h"
#include "pins_driver.h"
#include "ftm_hw_access.h"
#include "ftm_pwm_driver.h"
#include "stdint.h"
#include "clock_manager.h"
#include "pins_port_hw_access.h"
#include "ftm3_pwm_params.h"
#include "freemaster.h"
#include "adc_driver.h"
#include "interrupt_manager.h"
#include "pdb_driver.h"
#include "mbd_adc_irq.h"
#include "trgmux_driver.h"
#include "ftm_chn_irq.h"
#include "ftm_ic_driver.h"
#include "device_registers.h"
#include "tpp_ic_init.h"
#include "common_aml.h"
#include "gpio_irq.h"
#include "adc_interleave.h"
#include "freemaster_interface_init.h"
#include "lpspi_master_driver.h"
#include "lpspi_slave_driver.h"
#include "pdb0_params.h"
#include "pdb1_params.h"
#include "ftm2_input_params_config.h"
#include "ftm2_ch1_hall_sensor_isr.h"
#endif                       /* mcb_pmsm_foc_hall_S32K144EVB_COMMON_INCLUDES_ */

#include "mcb_pmsm_foc_hall_S32K144EVB_types.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmStepTask
#define rtmStepTask(rtm, idx)          ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmTaskCounter
#define rtmTaskCounter(rtm, idx)       ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

/* Block signals (default storage) */
typedef struct {
  uint32_T ADC1_ISR_o2;                /* '<S242>/ADC1_ISR' */
  real32_T RT11[2];                    /* '<Root>/RT11' */
  real32_T Id_ref;                     /* '<S8>/Id_ref' */
  real32_T Saturation;                 /* '<S301>/Saturation' */
  real32_T Kp1;                        /* '<S24>/Kp1' */
  real32_T Ki1;                        /* '<S23>/Ki1' */
  real32_T Merge;                      /* '<S195>/Merge' */
  uint16_T speedCountDelay;            /* '<S179>/speedCountDelay' */
  uint16_T DelayOneStep;               /* '<S180>/Delay One Step' */
  uint16_T Merge_m;                    /* '<S231>/Merge' */
  uint16_T Merge1;                     /* '<S231>/Merge1' */
  int16_T Merge2;                      /* '<S231>/Merge2' */
  uint8_T ADC1_ISR_o3;                 /* '<S242>/ADC1_ISR' */
  boolean_T validityDelay;             /* '<S179>/validityDelay' */
  boolean_T Digital_Input_HALL_C;      /* '<S174>/Digital_Input_HALL_C' */
  boolean_T Digital_Input_HALL_B;      /* '<S174>/Digital_Input_HALL_B' */
  boolean_T Digital_Input_HALL_A;      /* '<S174>/Digital_Input_HALL_A' */
  boolean_T Digital_Input_HALL_A_b;    /* '<S223>/Digital_Input_HALL_A' */
  boolean_T Digital_Input_HALL_B_l;    /* '<S223>/Digital_Input_HALL_B' */
  boolean_T Digital_Input_HALL_C_h;    /* '<S223>/Digital_Input_HALL_C' */
} B_mcb_pmsm_foc_hall_S32K144EV_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T Integrator_DSTATE;          /* '<S294>/Integrator' */
  real32_T Integrator_DSTATE_e;        /* '<S131>/Integrator' */
  real32_T Integrator_DSTATE_l;        /* '<S76>/Integrator' */
  volatile real32_T RT11_Buffer[4];    /* '<Root>/RT11' */
  volatile real32_T RT2_Buffer0;       /* '<Root>/RT2' */
  uint16_T DelayOneStep_DSTATE;        /* '<S180>/Delay One Step' */
  boolean_T DelayOneStep1_DSTATE;      /* '<S180>/Delay One Step1' */
  volatile int8_T RT11_ActiveBufIdx;   /* '<Root>/RT11' */
  int8_T Integrator_PrevResetState;    /* '<S294>/Integrator' */
  int8_T Integrator_PrevResetState_j;  /* '<S131>/Integrator' */
  int8_T Integrator_PrevResetState_n;  /* '<S76>/Integrator' */
  uint8_T is_active_c1_mcb_pmsm_foc_hall_;/* '<S4>/Enable PDB and start FTM' */
  uint8_T is_c1_mcb_pmsm_foc_hall_S32K144;/* '<S4>/Enable PDB and start FTM' */
} DW_mcb_pmsm_foc_hall_S32K144E_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real32_T SpeedConstData;       /* '<S182>/SpeedConstData' */
} ConstB_mcb_pmsm_foc_hall_S32K_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: sine_table_values_Value
   * Referenced by: '<S153>/sine_table_values'
   */
  real32_T sine_table_values_Value[1002];
} ConstP_mcb_pmsm_foc_hall_S32K_T;

/* Real-time Model Data Structure */
struct tag_RTM_mcb_pmsm_foc_hall_S32_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint16_T TID[3];
    } TaskCounters;
  } Timing;
};

/* Block signals (default storage) */
extern B_mcb_pmsm_foc_hall_S32K144EV_T mcb_pmsm_foc_hall_S32K144EVB_B;

/* Block states (default storage) */
extern DW_mcb_pmsm_foc_hall_S32K144E_T mcb_pmsm_foc_hall_S32K144EVB_DW;
extern const ConstB_mcb_pmsm_foc_hall_S32K_T mcb_pmsm_foc_hall_S32K14_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_mcb_pmsm_foc_hall_S32K_T mcb_pmsm_foc_hall_S32K14_ConstP;

/* External function called from main */
extern void mcb_pmsm_foc_hall_S32K144EVB_SetEventsForThisBaseStep(boolean_T
  *eventFlags);

/* Model entry point functions */
extern void mcb_pmsm_foc_hall_S32K144EVB_initialize(void);
extern void mcb_pmsm_foc_hall_S32K144EVB_step0(void);/* Sample time: [0.0001s, 0.0s] */
extern void mcb_pmsm_foc_hall_S32K144EVB_step1(void);/* Sample time: [0.001s, 0.0s] */
extern void mcb_pmsm_foc_hall_S32K144EVB_step2(void);/* Sample time: [0.1s, 0.0s] */
extern void mcb_pmsm_foc_hall_S32K144EVB_terminate(void);

/* Exported data declaration */

/* Volatile memory section */
/* Declaration for custom storage class: Volatile */
extern volatile real32_T ADC_A;        /* '<Root>/Data Store Memory11' */
extern volatile real32_T ADC_B;        /* '<Root>/Data Store Memory12' */
extern volatile uint32_T ADC_IA;       /* '<S243>/ADC_AD4_IA' */
extern volatile uint32_T ADC_IB;       /* '<S243>/ADC_IB' */
extern volatile uint32_T ADC_IDC;      /* '<S243>/ADC_AD6_IDC' */
extern volatile uint32_T ADC_VDC;      /* '<S243>/ADC_AD7_VDC' */
extern volatile uint32_T CH0S_ERR;     /* '<S242>/PDB1_ISR' */
extern volatile uint32_T CH1S_ERR;     /* '<S242>/PDB1_ISR' */
extern volatile real32_T COS;          /* '<S154>/Sum6' */
extern volatile uint16_T CntHall;      /* '<S3>/FTM_Hall_Sensor' */
extern volatile uint32_T CntHallDecoder;/* '<S171>/Read_Register' */
extern volatile uint16_T CntHallValidityIn;
                                /* '<S2>/SigConvForSigProp_Variant_Source2_0' */
extern volatile real32_T DesiredSpeed; /* '<Root>/Data Store Memory7' */
extern volatile boolean_T Enable;      /* '<Root>/Data Store Memory29' */
extern volatile boolean_T FAULT;       /* '<Root>/I_MAX Scalling3' */
extern volatile int16_T GlobalDirection;/* '<Root>/Data Store Memory3' */
extern volatile uint32_T GlobalHallState;/* '<Root>/Data Store Memory4' */
extern volatile uint16_T GlobalSpeedCount;/* '<Root>/Data Store Memory1' */
extern volatile uint16_T GlobalSpeedValidity;/* '<Root>/Data Store Memory2' */
extern volatile uint32_T HALL_A;       /* '<S225>/bit_shift' */
extern volatile uint32_T HALL_A_controller;/* '<S174>/Data Type Conversion6' */
extern volatile uint32_T HALL_B;       /* '<S226>/bit_shift' */
extern volatile uint32_T HALL_B_controller;/* '<S176>/bit_shift' */
extern volatile uint32_T HALL_C;       /* '<S223>/Data Type Conversion6' */
extern volatile uint32_T HALL_C_controller;/* '<S175>/bit_shift' */
extern volatile uint16_T HallCntActual;/* '<Root>/Data Store Memory25' */
extern volatile uint16_T HallCntPrev;  /* '<Root>/Data Store Memory24' */
extern volatile uint16_T HallStateChangeFlag;/* '<Root>/Data Store Memory' */
extern volatile uint32_T HallVal;      /* '<S174>/Add1' */
extern volatile uint16_T HallValididyInvalid;/* '<S229>/Merge' */
extern volatile real32_T I_ab_afterOffset[2];/* '<S170>/Add' */
extern volatile real32_T IaOffset;     /* '<Root>/Data Store Memory5' */
extern volatile real32_T Iab_fb[2];    /* '<S170>/Multiply' */
extern volatile real32_T IbOffset;     /* '<Root>/Data Store Memory6' */
extern volatile real32_T Id_err;       /* '<S23>/Sum' */
extern volatile real32_T Idc_afterOffset;/* '<S245>/Sum' */
extern volatile real32_T Iq_err;       /* '<S24>/Sum' */
extern volatile real32_T PWM[3];       /* '<S11>/Switch1' */
extern volatile real32_T PWM_Duty_Cycles[3];/* '<S12>/Gain' */
extern volatile real32_T PWM_Enable;   /* '<S12>/Data Type Conversion' */
extern volatile real32_T Pos_PU;       /* '<S217>/Add' */
extern volatile uint32_T SC_PDBIF;     /* '<S242>/PDB1_ISR' */
extern volatile real32_T SIN;          /* '<S154>/Sum4' */
extern volatile real32_T SpeedError;   /* '<S255>/Sum' */
extern volatile real32_T SpeedMeasured;/* '<S1>/Input Scaling' */
extern volatile real32_T Speed_Ref;    /* '<S315>/Add1' */
extern volatile real32_T Speed_Ref_PU; /* '<Root>/RT2' */
extern volatile real32_T Speed_fb;     /* '<Root>/RT1' */
extern volatile real32_T ThetaHalls;   /* '<S172>/Merge1' */
extern volatile real32_T Vd_ref_beforeLimiter;/* '<S83>/Saturation' */
extern volatile real32_T Vq_ref_beforeLimiter;/* '<S138>/Saturation' */

/* Real-time Model object */
extern RT_MODEL_mcb_pmsm_foc_hall_S3_T *const mcb_pmsm_foc_hall_S32K144EVB_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S21>/Data Type Duplicate' : Unused code path elimination
 * Block '<S29>/Data Type Duplicate' : Unused code path elimination
 * Block '<S36>/Data Type Duplicate' : Unused code path elimination
 * Block '<S36>/Data Type Propagation' : Unused code path elimination
 * Block '<S37>/Data Type Duplicate' : Unused code path elimination
 * Block '<S22>/Data Type Duplicate' : Unused code path elimination
 * Block '<S27>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S27>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S28>/Sqrt' : Unused code path elimination
 * Block '<S149>/Data Type Duplicate' : Unused code path elimination
 * Block '<S149>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S151>/Data Type Duplicate' : Unused code path elimination
 * Block '<S151>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S153>/Data Type Duplicate' : Unused code path elimination
 * Block '<S153>/Data Type Propagation' : Unused code path elimination
 * Block '<S158>/Data Type Duplicate' : Unused code path elimination
 * Block '<S159>/Data Type Duplicate' : Unused code path elimination
 * Block '<S19>/Data Type Duplicate' : Unused code path elimination
 * Block '<S19>/Vc' : Unused code path elimination
 * Block '<S167>/Data Type Duplicate' : Unused code path elimination
 * Block '<S215>/Data Type Duplicate' : Unused code path elimination
 * Block '<S173>/ReplaceInport_Npp' : Unused code path elimination
 * Block '<S1>/Scope' : Unused code path elimination
 * Block '<S1>/Scope1' : Unused code path elimination
 * Block '<S1>/Scope2' : Unused code path elimination
 * Block '<Root>/RT10' : Unused code path elimination
 * Block '<Root>/RT8' : Unused code path elimination
 * Block '<Root>/RT9' : Unused code path elimination
 * Block '<Root>/Scope3' : Unused code path elimination
 * Block '<S312>/Data Type Duplicate' : Unused code path elimination
 * Block '<S20>/Kalpha' : Eliminated nontunable gain of 1
 * Block '<S20>/Kbeta' : Eliminated nontunable gain of 1
 * Block '<S153>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S155>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S167>/Ka' : Eliminated nontunable gain of 1
 * Block '<S167>/Kb' : Eliminated nontunable gain of 1
 * Block '<S167>/Kc' : Eliminated nontunable gain of 1
 * Block '<S172>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S172>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S172>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S172>/PositionUnit' : Eliminated nontunable gain of 1
 * Block '<S172>/counterSize2' : Eliminate redundant data type conversion
 * Block '<S217>/Multiply' : Eliminated nontunable gain of 1
 * Block '<S217>/Multiply1' : Eliminated nontunable gain of 1
 * Block '<S221>/Number of pole pairs' : Eliminated nontunable gain of 1
 * Block '<S170>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S224>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S224>/counterSize' : Eliminate redundant data type conversion
 * Block '<S27>/enableInportSatLim' : Unused code path elimination
 * Block '<S27>/enableInportSatMethod' : Unused code path elimination
 * Block '<S22>/ReplaceInport_satLim' : Unused code path elimination
 * Block '<S22>/ReplaceInport_satMethod' : Unused code path elimination
 * Block '<S150>/Offset' : Unused code path elimination
 * Block '<S150>/Unary_Minus' : Unused code path elimination
 * Block '<S152>/Offset' : Unused code path elimination
 * Block '<S152>/Unary_Minus' : Unused code path elimination
 * Block '<S215>/Constant' : Unused code path elimination
 * Block '<S173>/ReplaceInport_Offset' : Unused code path elimination
 * Block '<S312>/One' : Unused code path elimination
 * Block '<S312>/Sum' : Unused code path elimination
 * Block '<S312>/UseInputPort' : Unused code path elimination
 * Block '<S312>/a' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'mcb_pmsm_foc_hall_S32K144EVB'
 * '<S1>'   : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl'
 * '<S2>'   : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor'
 * '<S3>'   : 'mcb_pmsm_foc_hall_S32K144EVB/HallCodeGen'
 * '<S4>'   : 'mcb_pmsm_foc_hall_S32K144EVB/Hardware Initialization'
 * '<S5>'   : 'mcb_pmsm_foc_hall_S32K144EVB/Inverter and Motor - Plant Model'
 * '<S6>'   : 'mcb_pmsm_foc_hall_S32K144EVB/Model Info'
 * '<S7>'   : 'mcb_pmsm_foc_hall_S32K144EVB/Serial Receive'
 * '<S8>'   : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl'
 * '<S9>'   : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System'
 * '<S10>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling'
 * '<S11>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Inverter (Code Generation)'
 * '<S12>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Output Scaling'
 * '<S13>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Sensor Driver Blocks'
 * '<S14>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Clarke Transform'
 * '<S15>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers'
 * '<S16>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Inverse Park Transform'
 * '<S17>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Park Transform'
 * '<S18>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Sine-Cosine Lookup'
 * '<S19>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Space Vector Generator'
 * '<S20>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Clarke Transform/Two phase input'
 * '<S21>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Clarke Transform/Two phase input/Two phase CRL wrap'
 * '<S22>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter'
 * '<S23>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id'
 * '<S24>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq'
 * '<S25>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D-Q Equivalence'
 * '<S26>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority'
 * '<S27>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/Inport//Dialog Selection'
 * '<S28>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/Magnitude_calc'
 * '<S29>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D-Q Equivalence/Limiter'
 * '<S30>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D-Q Equivalence/Passthrough'
 * '<S31>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/Compare To Constant'
 * '<S32>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/Compare To Constant1'
 * '<S33>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/flipInputs'
 * '<S34>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/flipInputs1'
 * '<S35>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter'
 * '<S36>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/limitRef1'
 * '<S37>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/limitRef2'
 * '<S38>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/passThrough'
 * '<S39>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset'
 * '<S40>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Anti-windup'
 * '<S41>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/D Gain'
 * '<S42>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/External Derivative'
 * '<S43>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Filter'
 * '<S44>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Filter ICs'
 * '<S45>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/I Gain'
 * '<S46>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Ideal P Gain'
 * '<S47>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Ideal P Gain Fdbk'
 * '<S48>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Integrator'
 * '<S49>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Integrator ICs'
 * '<S50>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/N Copy'
 * '<S51>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/N Gain'
 * '<S52>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/P Copy'
 * '<S53>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Parallel P Gain'
 * '<S54>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Reset Signal'
 * '<S55>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Saturation'
 * '<S56>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Saturation Fdbk'
 * '<S57>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Sum'
 * '<S58>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Sum Fdbk'
 * '<S59>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tracking Mode'
 * '<S60>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tracking Mode Sum'
 * '<S61>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tsamp - Integral'
 * '<S62>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tsamp - Ngain'
 * '<S63>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/postSat Signal'
 * '<S64>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/preInt Signal'
 * '<S65>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/preSat Signal'
 * '<S66>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel'
 * '<S67>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S68>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S69>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/D Gain/Disabled'
 * '<S70>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/External Derivative/Disabled'
 * '<S71>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Filter/Disabled'
 * '<S72>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Filter ICs/Disabled'
 * '<S73>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/I Gain/External Parameters'
 * '<S74>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Ideal P Gain/Passthrough'
 * '<S75>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Ideal P Gain Fdbk/Disabled'
 * '<S76>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Integrator/Discrete'
 * '<S77>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Integrator ICs/External IC'
 * '<S78>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/N Copy/Disabled wSignal Specification'
 * '<S79>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/N Gain/Disabled'
 * '<S80>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/P Copy/Disabled'
 * '<S81>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Parallel P Gain/External Parameters'
 * '<S82>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Reset Signal/External Reset'
 * '<S83>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Saturation/Enabled'
 * '<S84>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Saturation Fdbk/Disabled'
 * '<S85>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Sum/Sum_PI'
 * '<S86>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Sum Fdbk/Disabled'
 * '<S87>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tracking Mode/Disabled'
 * '<S88>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tracking Mode Sum/Passthrough'
 * '<S89>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tsamp - Integral/TsSignalSpecification'
 * '<S90>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tsamp - Ngain/Passthrough'
 * '<S91>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/postSat Signal/Forward_Path'
 * '<S92>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/preInt Signal/Internal PreInt'
 * '<S93>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/preSat Signal/Forward_Path'
 * '<S94>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset'
 * '<S95>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Anti-windup'
 * '<S96>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/D Gain'
 * '<S97>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/External Derivative'
 * '<S98>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Filter'
 * '<S99>'  : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Filter ICs'
 * '<S100>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/I Gain'
 * '<S101>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Ideal P Gain'
 * '<S102>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Ideal P Gain Fdbk'
 * '<S103>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Integrator'
 * '<S104>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Integrator ICs'
 * '<S105>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/N Copy'
 * '<S106>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/N Gain'
 * '<S107>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/P Copy'
 * '<S108>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Parallel P Gain'
 * '<S109>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Reset Signal'
 * '<S110>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Saturation'
 * '<S111>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Saturation Fdbk'
 * '<S112>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Sum'
 * '<S113>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Sum Fdbk'
 * '<S114>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tracking Mode'
 * '<S115>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tracking Mode Sum'
 * '<S116>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tsamp - Integral'
 * '<S117>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tsamp - Ngain'
 * '<S118>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/postSat Signal'
 * '<S119>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/preInt Signal'
 * '<S120>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/preSat Signal'
 * '<S121>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel'
 * '<S122>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S123>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S124>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/D Gain/Disabled'
 * '<S125>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/External Derivative/Disabled'
 * '<S126>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Filter/Disabled'
 * '<S127>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Filter ICs/Disabled'
 * '<S128>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/I Gain/External Parameters'
 * '<S129>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Ideal P Gain/Passthrough'
 * '<S130>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Ideal P Gain Fdbk/Disabled'
 * '<S131>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Integrator/Discrete'
 * '<S132>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Integrator ICs/External IC'
 * '<S133>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/N Copy/Disabled wSignal Specification'
 * '<S134>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/N Gain/Disabled'
 * '<S135>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/P Copy/Disabled'
 * '<S136>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Parallel P Gain/External Parameters'
 * '<S137>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Reset Signal/External Reset'
 * '<S138>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Saturation/Enabled'
 * '<S139>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Saturation Fdbk/Disabled'
 * '<S140>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Sum/Sum_PI'
 * '<S141>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Sum Fdbk/Disabled'
 * '<S142>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tracking Mode/Disabled'
 * '<S143>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tracking Mode Sum/Passthrough'
 * '<S144>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tsamp - Integral/TsSignalSpecification'
 * '<S145>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tsamp - Ngain/Passthrough'
 * '<S146>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/postSat Signal/Forward_Path'
 * '<S147>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/preInt Signal/Internal PreInt'
 * '<S148>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/preSat Signal/Forward_Path'
 * '<S149>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Inverse Park Transform/Two inputs CRL'
 * '<S150>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Inverse Park Transform/Two inputs CRL/Switch_Axis'
 * '<S151>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Park Transform/Two inputs CRL'
 * '<S152>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Park Transform/Two inputs CRL/Switch_Axis'
 * '<S153>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup'
 * '<S154>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/Interpolation'
 * '<S155>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp'
 * '<S156>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype'
 * '<S157>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S158>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S159>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S160>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype/datatype no change'
 * '<S161>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Space Vector Generator/Modulation method'
 * '<S162>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Space Vector Generator/Voltage Input'
 * '<S163>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Space Vector Generator/Modulation method/SVPWM'
 * '<S164>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Space Vector Generator/Modulation method/SVPWM/Half(Vmin+Vmax)'
 * '<S165>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Space Vector Generator/Voltage Input/Valphabeta'
 * '<S166>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Space Vector Generator/Voltage Input/Valphabeta/Inverse Clarke Transform'
 * '<S167>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Control_System/Space Vector Generator/Voltage Input/Valphabeta/Inverse Clarke Transform/Two phase input'
 * '<S168>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)'
 * '<S169>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall'
 * '<S170>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Convert ADC value to PU'
 * '<S171>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading'
 * '<S172>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position'
 * '<S173>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position'
 * '<S174>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read'
 * '<S175>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift'
 * '<S176>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift1'
 * '<S177>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift/bit_shift'
 * '<S178>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift1/bit_shift'
 * '<S179>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/ExtrapolationOrder'
 * '<S180>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Software Watchdog Timer'
 * '<S181>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant'
 * '<S182>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position'
 * '<S183>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/VaidityCheck'
 * '<S184>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Software Watchdog Timer/Compare To Zero'
 * '<S185>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction'
 * '<S186>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 1'
 * '<S187>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 2'
 * '<S188>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 3'
 * '<S189>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 4'
 * '<S190>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 5'
 * '<S191>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 6'
 * '<S192>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 7'
 * '<S193>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/If Action Subsystem'
 * '<S194>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/If Action Subsystem1'
 * '<S195>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1'
 * '<S196>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction'
 * '<S197>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction'
 * '<S198>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/first_order'
 * '<S199>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/second_order'
 * '<S200>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 1'
 * '<S201>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 2'
 * '<S202>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 3'
 * '<S203>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 4'
 * '<S204>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 5'
 * '<S205>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 6'
 * '<S206>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 7'
 * '<S207>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 1'
 * '<S208>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 2'
 * '<S209>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 3'
 * '<S210>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 4'
 * '<S211>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 5'
 * '<S212>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 6'
 * '<S213>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 7'
 * '<S214>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec'
 * '<S215>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point'
 * '<S216>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset'
 * '<S217>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec'
 * '<S218>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset/If Action Subsystem'
 * '<S219>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset/If Action Subsystem1'
 * '<S220>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec/Variant Subsystem'
 * '<S221>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec/Variant Subsystem/Dialog'
 * '<S222>' : 'mcb_pmsm_foc_hall_S32K144EVB/CurrentControl/Sensor Driver Blocks/Sensor Driver Blocks (codegen)'
 * '<S223>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/CodeGen'
 * '<S224>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity'
 * '<S225>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/CodeGen/Bit Shift'
 * '<S226>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/CodeGen/Bit Shift1'
 * '<S227>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/CodeGen/Bit Shift/bit_shift'
 * '<S228>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/CodeGen/Bit Shift1/bit_shift'
 * '<S229>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem'
 * '<S230>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem/Bad hall (glitch or wrong connection)'
 * '<S231>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem/Valid Halls'
 * '<S232>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem'
 * '<S233>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem1'
 * '<S234>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem2'
 * '<S235>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem3'
 * '<S236>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem4'
 * '<S237>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem5'
 * '<S238>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem6'
 * '<S239>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem7'
 * '<S240>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem8'
 * '<S241>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hall Sensor/Hall Validity/Subsystem/Valid Halls/Switch Case Action Subsystem'
 * '<S242>' : 'mcb_pmsm_foc_hall_S32K144EVB/HallCodeGen/Subsystem2'
 * '<S243>' : 'mcb_pmsm_foc_hall_S32K144EVB/HallCodeGen/Subsystem2/ADC1_IRQHandler'
 * '<S244>' : 'mcb_pmsm_foc_hall_S32K144EVB/HallCodeGen/Subsystem2/PDB1_IRQHandler'
 * '<S245>' : 'mcb_pmsm_foc_hall_S32K144EVB/HallCodeGen/Subsystem2/ADC1_IRQHandler/FaultDetection'
 * '<S246>' : 'mcb_pmsm_foc_hall_S32K144EVB/HallCodeGen/Subsystem2/ADC1_IRQHandler/FaultDetection/FAILURE'
 * '<S247>' : 'mcb_pmsm_foc_hall_S32K144EVB/HallCodeGen/Subsystem2/ADC1_IRQHandler/FaultDetection/FAILURE/Failed Subsystem'
 * '<S248>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hardware Initialization/Enable PDB and start FTM'
 * '<S249>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hardware Initialization/FAULT'
 * '<S250>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hardware Initialization/GD3000_interrupt'
 * '<S251>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hardware Initialization/If Action Subsystem'
 * '<S252>' : 'mcb_pmsm_foc_hall_S32K144EVB/Hardware Initialization/enable_FTM_PDB_ADC_triggering'
 * '<S253>' : 'mcb_pmsm_foc_hall_S32K144EVB/Inverter and Motor - Plant Model/Codegeneration'
 * '<S254>' : 'mcb_pmsm_foc_hall_S32K144EVB/Serial Receive/Code Generation'
 * '<S255>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed'
 * '<S256>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/Speed_Ref_Selector'
 * '<S257>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset'
 * '<S258>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Anti-windup'
 * '<S259>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/D Gain'
 * '<S260>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/External Derivative'
 * '<S261>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Filter'
 * '<S262>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Filter ICs'
 * '<S263>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/I Gain'
 * '<S264>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Ideal P Gain'
 * '<S265>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Ideal P Gain Fdbk'
 * '<S266>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Integrator'
 * '<S267>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Integrator ICs'
 * '<S268>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/N Copy'
 * '<S269>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/N Gain'
 * '<S270>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/P Copy'
 * '<S271>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Parallel P Gain'
 * '<S272>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Reset Signal'
 * '<S273>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Saturation'
 * '<S274>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Saturation Fdbk'
 * '<S275>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Sum'
 * '<S276>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Sum Fdbk'
 * '<S277>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Tracking Mode'
 * '<S278>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Tracking Mode Sum'
 * '<S279>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Tsamp - Integral'
 * '<S280>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Tsamp - Ngain'
 * '<S281>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/postSat Signal'
 * '<S282>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/preInt Signal'
 * '<S283>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/preSat Signal'
 * '<S284>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel'
 * '<S285>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S286>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S287>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/D Gain/Disabled'
 * '<S288>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/External Derivative/Disabled'
 * '<S289>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Filter/Disabled'
 * '<S290>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Filter ICs/Disabled'
 * '<S291>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/I Gain/External Parameters'
 * '<S292>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Ideal P Gain/Passthrough'
 * '<S293>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Ideal P Gain Fdbk/Disabled'
 * '<S294>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Integrator/Discrete'
 * '<S295>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Integrator ICs/External IC'
 * '<S296>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/N Copy/Disabled wSignal Specification'
 * '<S297>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/N Gain/Disabled'
 * '<S298>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/P Copy/Disabled'
 * '<S299>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Parallel P Gain/External Parameters'
 * '<S300>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Reset Signal/External Reset'
 * '<S301>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Saturation/Enabled'
 * '<S302>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Saturation Fdbk/Disabled'
 * '<S303>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Sum/Sum_PI'
 * '<S304>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Sum Fdbk/Disabled'
 * '<S305>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Tracking Mode/Disabled'
 * '<S306>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Tracking Mode Sum/Passthrough'
 * '<S307>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Tsamp - Integral/TsSignalSpecification'
 * '<S308>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/Tsamp - Ngain/Passthrough'
 * '<S309>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/postSat Signal/Forward_Path'
 * '<S310>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/preInt Signal/Internal PreInt'
 * '<S311>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/PI_Controller_Speed/Discrete PI Controller  with anti-windup & reset/preSat Signal/Forward_Path'
 * '<S312>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/Speed_Ref_Selector/IIR Filter'
 * '<S313>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/Speed_Ref_Selector/IIR Filter/IIR Filter'
 * '<S314>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/Speed_Ref_Selector/IIR Filter/IIR Filter/Low-pass'
 * '<S315>' : 'mcb_pmsm_foc_hall_S32K144EVB/SpeedControl/Speed_Ref_Selector/IIR Filter/IIR Filter/Low-pass/IIR Low Pass Filter'
 */
#endif                                 /* mcb_pmsm_foc_hall_S32K144EVB_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
