/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Foc_model_Matlab_1.h
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

#ifndef Foc_model_Matlab_1_h_
#define Foc_model_Matlab_1_h_
#ifndef Foc_model_Matlab_1_COMMON_INCLUDES_
#define Foc_model_Matlab_1_COMMON_INCLUDES_
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
#endif                                 /* Foc_model_Matlab_1_COMMON_INCLUDES_ */

#include "Foc_model_Matlab_1_types.h"
#include "rt_nonfinite.h"
#include "rtGetNaN.h"

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
  uint32_T ADC1_ISR_o2;                /* '<S206>/ADC1_ISR' */
  real32_T RT3;                        /* '<Root>/RT3' */
  real32_T Id_ref;                     /* '<S8>/Id_ref' */
  real32_T I_ref;                      /* '<S219>/MATLAB Function' */
  real32_T SpeedGain;                  /* '<S16>/SpeedGain' */
  real32_T Merge;                      /* '<S94>/Merge' */
  uint16_T speedCountDelay;            /* '<S78>/speedCountDelay' */
  uint16_T DelayOneStep;               /* '<S79>/Delay One Step' */
  uint16_T Merge_m;                    /* '<S195>/Merge' */
  uint16_T Merge1;                     /* '<S195>/Merge1' */
  int16_T Merge2;                      /* '<S195>/Merge2' */
  uint8_T ADC1_ISR_o3;                 /* '<S206>/ADC1_ISR' */
  boolean_T validityDelay;             /* '<S78>/validityDelay' */
  boolean_T Digital_Input_HALL_C;      /* '<S73>/Digital_Input_HALL_C' */
  boolean_T Digital_Input_HALL_B;      /* '<S73>/Digital_Input_HALL_B' */
  boolean_T Digital_Input_HALL_A;      /* '<S73>/Digital_Input_HALL_A' */
  boolean_T Digital_Input_HALL_A_b;    /* '<S187>/Digital_Input_HALL_A' */
  boolean_T Digital_Input_HALL_B_l;    /* '<S187>/Digital_Input_HALL_B' */
  boolean_T Digital_Input_HALL_C_h;    /* '<S187>/Digital_Input_HALL_C' */
} B_Foc_model_Matlab_1_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T DiscreteTimeIntegrator4_DSTATE;/* '<S14>/Discrete-Time Integrator4' */
  real32_T DiscreteTimeIntegrator2_DSTATE;/* '<S14>/Discrete-Time Integrator2' */
  real32_T DiscreteTimeIntegrator3_DSTATE;/* '<S14>/Discrete-Time Integrator3' */
  real32_T DiscreteTimeIntegrator1_DSTATE;/* '<S14>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE;/* '<S14>/Discrete-Time Integrator' */
  real32_T Integrator_DSTATE;          /* '<S166>/Integrator' */
  uint32_T Delay_DSTATE[10];           /* '<S16>/Delay' */
  volatile real32_T RT11_Buffer[4];    /* '<Root>/RT11' */
  volatile real32_T RT2_Buffer0;       /* '<Root>/RT2' */
  real32_T N_ref_pre;                  /* '<S219>/MATLAB Function' */
  real32_T speed_error_pre;            /* '<S219>/MATLAB Function' */
  real32_T DiscreteTimeIntegrator4_PREV_U;/* '<S14>/Discrete-Time Integrator4' */
  real32_T DiscreteTimeIntegrator2_PREV_U;/* '<S14>/Discrete-Time Integrator2' */
  real32_T DiscreteTimeIntegrator3_PREV_U;/* '<S14>/Discrete-Time Integrator3' */
  real32_T DiscreteTimeIntegrator1_PREV_U;/* '<S14>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_PREV_U;/* '<S14>/Discrete-Time Integrator' */
  real32_T Integrator_PREV_U;          /* '<S166>/Integrator' */
  real32_T integral;                   /* '<S29>/MATLAB Function1' */
  real32_T integral_j;                 /* '<S28>/MATLAB Function1' */
  uint32_T CurrentControl_PREV_T;      /* '<Root>/CurrentControl' */
  uint32_T CircBufIdx;                 /* '<S16>/Delay' */
  uint16_T DelayOneStep_DSTATE;        /* '<S79>/Delay One Step' */
  boolean_T DelayOneStep1_DSTATE;      /* '<S79>/Delay One Step1' */
  volatile int8_T RT11_ActiveBufIdx;   /* '<Root>/RT11' */
  uint8_T is_active_c1_Foc_model_Matlab_1;/* '<S4>/Enable PDB and start FTM' */
  uint8_T is_c1_Foc_model_Matlab_1;    /* '<S4>/Enable PDB and start FTM' */
  uint8_T DiscreteTimeIntegrator4_SYSTEM_;/* '<S14>/Discrete-Time Integrator4' */
  uint8_T DiscreteTimeIntegrator2_SYSTEM_;/* '<S14>/Discrete-Time Integrator2' */
  uint8_T DiscreteTimeIntegrator3_SYSTEM_;/* '<S14>/Discrete-Time Integrator3' */
  uint8_T DiscreteTimeIntegrator1_SYSTEM_;/* '<S14>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator_SYSTEM_E;/* '<S14>/Discrete-Time Integrator' */
  uint8_T Integrator_SYSTEM_ENABLE;    /* '<S166>/Integrator' */
  boolean_T N_ref_pre_not_empty;       /* '<S219>/MATLAB Function' */
  boolean_T speed_error_pre_not_empty; /* '<S219>/MATLAB Function' */
  boolean_T T_n_pre_not_empty;         /* '<S219>/MATLAB Function' */
  boolean_T CurrentControl_RESET_ELAPS_T;/* '<Root>/CurrentControl' */
  boolean_T integral_not_empty;        /* '<S29>/MATLAB Function1' */
  boolean_T integral_not_empty_p;      /* '<S28>/MATLAB Function1' */
} DW_Foc_model_Matlab_1_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real32_T CastToSingle6;        /* '<S14>/Cast To Single6' */
  const real32_T SpeedConstData;       /* '<S81>/SpeedConstData' */
} ConstB_Foc_model_Matlab_1_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: sine_table_values_Value
   * Referenced by: '<S52>/sine_table_values'
   */
  real32_T sine_table_values_Value[1002];
} ConstP_Foc_model_Matlab_1_T;

/* Real-time Model Data Structure */
struct tag_RTM_Foc_model_Matlab_1_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick1;
    uint32_T clockTick2;
    struct {
      uint16_T TID[4];
    } TaskCounters;
  } Timing;
};

/* Block signals (default storage) */
extern B_Foc_model_Matlab_1_T Foc_model_Matlab_1_B;

/* Block states (default storage) */
extern DW_Foc_model_Matlab_1_T Foc_model_Matlab_1_DW;
extern const ConstB_Foc_model_Matlab_1_T Foc_model_Matlab_1_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_Foc_model_Matlab_1_T Foc_model_Matlab_1_ConstP;

/* External function called from main */
extern void Foc_model_Matlab_1_SetEventsForThisBaseStep(boolean_T *eventFlags);

/* Model entry point functions */
extern void Foc_model_Matlab_1_initialize(void);
extern void Foc_model_Matlab_1_step0(void);/* Sample time: [5.0E-5s, 0.0s] */
extern void Foc_model_Matlab_1_step1(void);/* Sample time: [0.0001s, 0.0s] */
extern void Foc_model_Matlab_1_step2(void);/* Sample time: [0.001s, 0.0s] */
extern void Foc_model_Matlab_1_step3(void);/* Sample time: [0.1s, 0.0s] */
extern void Foc_model_Matlab_1_terminate(void);

/* Exported data declaration */

/* Volatile memory section */
/* Declaration for custom storage class: Volatile */
extern volatile real32_T ADC_A;        /* '<Root>/Data Store Memory11' */
extern volatile real32_T ADC_B;        /* '<Root>/Data Store Memory12' */
extern volatile uint32_T ADC_IA;       /* '<S207>/ADC_AD4_IA' */
extern volatile uint32_T ADC_IB;       /* '<S207>/ADC_IB' */
extern volatile uint32_T ADC_IDC;      /* '<S207>/ADC_AD6_IDC' */
extern volatile uint32_T ADC_VDC;      /* '<S207>/ADC_AD7_VDC' */
extern volatile uint32_T CH0S_ERR;     /* '<S206>/PDB1_ISR' */
extern volatile uint32_T CH1S_ERR;     /* '<S206>/PDB1_ISR' */
extern volatile real32_T COS;          /* '<S53>/Sum6' */
extern volatile uint16_T CntHall;      /* '<S3>/FTM_Hall_Sensor' */
extern volatile uint32_T CntHallDecoder;/* '<S70>/Read_Register' */
extern volatile uint16_T CntHallValidityIn;
                                /* '<S2>/SigConvForSigProp_Variant_Source2_0' */
extern volatile real32_T DesiredSpeed; /* '<Root>/Data Store Memory7' */
extern volatile boolean_T Enable;      /* '<Root>/Data Store Memory29' */
extern volatile real32_T Epsilon;      /* '<Root>/Data Store Memory13' */
extern volatile boolean_T FAULT;       /* '<Root>/I_MAX Scalling3' */
extern volatile real32_T Gamma;        /* '<Root>/Data Store Memory8' */
extern volatile int16_T GlobalDirection;/* '<Root>/Data Store Memory3' */
extern volatile uint32_T GlobalHallState;/* '<Root>/Data Store Memory4' */
extern volatile uint16_T GlobalSpeedCount;/* '<Root>/Data Store Memory1' */
extern volatile uint16_T GlobalSpeedValidity;/* '<Root>/Data Store Memory2' */
extern volatile uint32_T HALL_A;       /* '<S189>/bit_shift' */
extern volatile uint32_T HALL_A_controller;/* '<S73>/Data Type Conversion6' */
extern volatile uint32_T HALL_B;       /* '<S190>/bit_shift' */
extern volatile uint32_T HALL_B_controller;/* '<S75>/bit_shift' */
extern volatile uint32_T HALL_C;       /* '<S187>/Data Type Conversion6' */
extern volatile uint32_T HALL_C_controller;/* '<S74>/bit_shift' */
extern volatile uint16_T HallCntActual;/* '<Root>/Data Store Memory25' */
extern volatile uint16_T HallCntPrev;  /* '<Root>/Data Store Memory24' */
extern volatile uint16_T HallStateChangeFlag;/* '<Root>/Data Store Memory' */
extern volatile uint32_T HallVal;      /* '<S73>/Add1' */
extern volatile uint16_T HallValididyInvalid;/* '<S193>/Merge' */
extern volatile real32_T I_ab_afterOffset[2];/* '<S69>/Add' */
extern volatile real32_T IaOffset;     /* '<Root>/Data Store Memory5' */
extern volatile real32_T Iab_fb[2];    /* '<S69>/Multiply' */
extern volatile real32_T IbOffset;     /* '<Root>/Data Store Memory6' */
extern volatile real32_T Id_err;       /* '<S28>/Sum' */
extern volatile real32_T Id_fb;        /* '<S18>/Signal Copy1' */
extern volatile real32_T Idc_afterOffset;/* '<S209>/Sum' */
extern volatile real32_T Idq_ref_PU[2];/* '<Root>/RT11' */
extern volatile real32_T Iq_err;       /* '<S29>/Sum' */
extern volatile real32_T Iq_fb;        /* '<S18>/Signal Copy' */
extern volatile real32_T K2;           /* '<Root>/Data Store Memory16' */
extern volatile real32_T Ki;           /* '<Root>/Data Store Memory15' */
extern volatile real32_T Kp;           /* '<Root>/Data Store Memory14' */
extern volatile real32_T Lambda;       /* '<Root>/Data Store Memory9' */
extern volatile real32_T PWM[3];       /* '<S11>/Switch1' */
extern volatile real32_T PWM_Duty_Cycles[3];/* '<S13>/Gain' */
extern volatile real32_T PWM_Enable;   /* '<S13>/Data Type Conversion' */
extern volatile real32_T Pos_PU;       /* '<S116>/Add' */
extern volatile uint32_T SC_PDBIF;     /* '<S206>/PDB1_ISR' */
extern volatile real32_T SIN;          /* '<S53>/Sum4' */
extern volatile real32_T SpeedError;   /* '<S219>/Sum' */
extern volatile real32_T SpeedMeasured;/* '<S1>/Input Scaling' */
extern volatile real32_T Speed_Ref;    /* '<S220>/Switch' */
extern volatile real32_T Speed_Ref_PU; /* '<Root>/RT2' */
extern volatile real32_T Speed_fb;     /* '<Root>/RT1' */
extern volatile real32_T Theta;        /* '<Root>/Data Store Memory10' */
extern volatile real32_T ThetaHalls;   /* '<S71>/Merge1' */
extern volatile real32_T Vd_ref_beforeLimiter;/* '<S28>/MATLAB Function1' */
extern volatile real32_T Vq_ref_beforeLimiter;/* '<S29>/MATLAB Function1' */

/* Real-time Model object */
extern RT_MODEL_Foc_model_Matlab_1_T *const Foc_model_Matlab_1_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S24>/Data Type Duplicate' : Unused code path elimination
 * Block '<S34>/Data Type Duplicate' : Unused code path elimination
 * Block '<S41>/Data Type Duplicate' : Unused code path elimination
 * Block '<S41>/Data Type Propagation' : Unused code path elimination
 * Block '<S42>/Data Type Duplicate' : Unused code path elimination
 * Block '<S25>/Data Type Duplicate' : Unused code path elimination
 * Block '<S32>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S32>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S33>/Sqrt' : Unused code path elimination
 * Block '<S48>/Data Type Duplicate' : Unused code path elimination
 * Block '<S48>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S50>/Data Type Duplicate' : Unused code path elimination
 * Block '<S50>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S52>/Data Type Duplicate' : Unused code path elimination
 * Block '<S52>/Data Type Propagation' : Unused code path elimination
 * Block '<S57>/Data Type Duplicate' : Unused code path elimination
 * Block '<S58>/Data Type Duplicate' : Unused code path elimination
 * Block '<S22>/Data Type Duplicate' : Unused code path elimination
 * Block '<S22>/Vc' : Unused code path elimination
 * Block '<S66>/Data Type Duplicate' : Unused code path elimination
 * Block '<S1>/Gain1' : Unused code path elimination
 * Block '<S114>/Data Type Duplicate' : Unused code path elimination
 * Block '<S72>/ReplaceInport_Npp' : Unused code path elimination
 * Block '<S122>/Data Type Duplicate' : Unused code path elimination
 * Block '<S12>/ReplaceInport_Npp' : Unused code path elimination
 * Block '<S14>/in PID' : Unused code path elimination
 * Block '<Root>/RT10' : Unused code path elimination
 * Block '<Root>/RT8' : Unused code path elimination
 * Block '<Root>/RT9' : Unused code path elimination
 * Block '<Root>/Scope3' : Unused code path elimination
 * Block '<S8>/Gain' : Unused code path elimination
 * Block '<S219>/Scope' : Unused code path elimination
 * Block '<S219>/To Workspace' : Unused code path elimination
 * Block '<S8>/Scope' : Unused code path elimination
 * Block '<S23>/Kalpha' : Eliminated nontunable gain of 1
 * Block '<S23>/Kbeta' : Eliminated nontunable gain of 1
 * Block '<S52>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S54>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S66>/Ka' : Eliminated nontunable gain of 1
 * Block '<S66>/Kb' : Eliminated nontunable gain of 1
 * Block '<S66>/Kc' : Eliminated nontunable gain of 1
 * Block '<S71>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S71>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S71>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S71>/PositionUnit' : Eliminated nontunable gain of 1
 * Block '<S71>/counterSize2' : Eliminate redundant data type conversion
 * Block '<S116>/Multiply' : Eliminated nontunable gain of 1
 * Block '<S116>/Multiply1' : Eliminated nontunable gain of 1
 * Block '<S120>/Number of pole pairs' : Eliminated nontunable gain of 1
 * Block '<S69>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S124>/Multiply' : Eliminated nontunable gain of 1
 * Block '<S124>/Multiply1' : Eliminated nontunable gain of 1
 * Block '<S14>/Cast To Single' : Eliminate redundant data type conversion
 * Block '<S14>/Cast To Single1' : Eliminate redundant data type conversion
 * Block '<S14>/Cast To Single7' : Eliminate redundant data type conversion
 * Block '<S188>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S188>/counterSize' : Eliminate redundant data type conversion
 * Block '<S32>/enableInportSatLim' : Unused code path elimination
 * Block '<S32>/enableInportSatMethod' : Unused code path elimination
 * Block '<S25>/ReplaceInport_satLim' : Unused code path elimination
 * Block '<S25>/ReplaceInport_satMethod' : Unused code path elimination
 * Block '<S49>/Offset' : Unused code path elimination
 * Block '<S49>/Unary_Minus' : Unused code path elimination
 * Block '<S51>/Offset' : Unused code path elimination
 * Block '<S51>/Unary_Minus' : Unused code path elimination
 * Block '<S114>/Constant' : Unused code path elimination
 * Block '<S72>/ReplaceInport_Offset' : Unused code path elimination
 * Block '<S122>/Constant' : Unused code path elimination
 * Block '<S12>/ReplaceInport_Offset' : Unused code path elimination
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
 * '<Root>' : 'Foc_model_Matlab_1'
 * '<S1>'   : 'Foc_model_Matlab_1/CurrentControl'
 * '<S2>'   : 'Foc_model_Matlab_1/Hall Sensor'
 * '<S3>'   : 'Foc_model_Matlab_1/HallCodeGen'
 * '<S4>'   : 'Foc_model_Matlab_1/Hardware Initialization'
 * '<S5>'   : 'Foc_model_Matlab_1/Inverter and Motor - Plant Model'
 * '<S6>'   : 'Foc_model_Matlab_1/Model Info'
 * '<S7>'   : 'Foc_model_Matlab_1/Serial Receive'
 * '<S8>'   : 'Foc_model_Matlab_1/SpeedControl'
 * '<S9>'   : 'Foc_model_Matlab_1/CurrentControl/Control_System'
 * '<S10>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling'
 * '<S11>'  : 'Foc_model_Matlab_1/CurrentControl/Inverter (Code Generation)'
 * '<S12>'  : 'Foc_model_Matlab_1/CurrentControl/Mechanical to Electrical Position'
 * '<S13>'  : 'Foc_model_Matlab_1/CurrentControl/Output Scaling'
 * '<S14>'  : 'Foc_model_Matlab_1/CurrentControl/STSMO'
 * '<S15>'  : 'Foc_model_Matlab_1/CurrentControl/Sensor Driver Blocks'
 * '<S16>'  : 'Foc_model_Matlab_1/CurrentControl/Speed Measurement'
 * '<S17>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Clarke Transform'
 * '<S18>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers'
 * '<S19>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Inverse Park Transform'
 * '<S20>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Park Transform'
 * '<S21>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Sine-Cosine Lookup'
 * '<S22>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Space Vector Generator'
 * '<S23>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Clarke Transform/Two phase input'
 * '<S24>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Clarke Transform/Two phase input/Two phase CRL wrap'
 * '<S25>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter'
 * '<S26>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/Nonlinear feedforwar compensation'
 * '<S27>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/Nonlinear feedforwar compensation1'
 * '<S28>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id'
 * '<S29>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq'
 * '<S30>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D-Q Equivalence'
 * '<S31>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority'
 * '<S32>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/Inport//Dialog Selection'
 * '<S33>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/Magnitude_calc'
 * '<S34>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D-Q Equivalence/Limiter'
 * '<S35>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D-Q Equivalence/Passthrough'
 * '<S36>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/Compare To Constant'
 * '<S37>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/Compare To Constant1'
 * '<S38>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/flipInputs'
 * '<S39>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/flipInputs1'
 * '<S40>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter'
 * '<S41>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/limitRef1'
 * '<S42>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/limitRef2'
 * '<S43>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/passThrough'
 * '<S44>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/MATLAB Function'
 * '<S45>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/MATLAB Function1'
 * '<S46>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/MATLAB Function'
 * '<S47>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/MATLAB Function1'
 * '<S48>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Inverse Park Transform/Two inputs CRL'
 * '<S49>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Inverse Park Transform/Two inputs CRL/Switch_Axis'
 * '<S50>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Park Transform/Two inputs CRL'
 * '<S51>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Park Transform/Two inputs CRL/Switch_Axis'
 * '<S52>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup'
 * '<S53>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/Interpolation'
 * '<S54>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp'
 * '<S55>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype'
 * '<S56>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S57>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S58>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S59>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype/datatype no change'
 * '<S60>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Space Vector Generator/Modulation method'
 * '<S61>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Space Vector Generator/Voltage Input'
 * '<S62>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Space Vector Generator/Modulation method/SVPWM'
 * '<S63>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Space Vector Generator/Modulation method/SVPWM/Half(Vmin+Vmax)'
 * '<S64>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Space Vector Generator/Voltage Input/Valphabeta'
 * '<S65>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Space Vector Generator/Voltage Input/Valphabeta/Inverse Clarke Transform'
 * '<S66>'  : 'Foc_model_Matlab_1/CurrentControl/Control_System/Space Vector Generator/Voltage Input/Valphabeta/Inverse Clarke Transform/Two phase input'
 * '<S67>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)'
 * '<S68>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall'
 * '<S69>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Convert ADC value to PU'
 * '<S70>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading'
 * '<S71>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position'
 * '<S72>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position'
 * '<S73>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read'
 * '<S74>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift'
 * '<S75>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift1'
 * '<S76>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift/bit_shift'
 * '<S77>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift1/bit_shift'
 * '<S78>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/ExtrapolationOrder'
 * '<S79>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Software Watchdog Timer'
 * '<S80>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant'
 * '<S81>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position'
 * '<S82>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/VaidityCheck'
 * '<S83>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Software Watchdog Timer/Compare To Zero'
 * '<S84>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction'
 * '<S85>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 1'
 * '<S86>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 2'
 * '<S87>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 3'
 * '<S88>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 4'
 * '<S89>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 5'
 * '<S90>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 6'
 * '<S91>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 7'
 * '<S92>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/If Action Subsystem'
 * '<S93>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/If Action Subsystem1'
 * '<S94>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1'
 * '<S95>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction'
 * '<S96>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction'
 * '<S97>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/first_order'
 * '<S98>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/second_order'
 * '<S99>'  : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 1'
 * '<S100>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 2'
 * '<S101>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 3'
 * '<S102>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 4'
 * '<S103>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 5'
 * '<S104>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 6'
 * '<S105>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 7'
 * '<S106>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 1'
 * '<S107>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 2'
 * '<S108>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 3'
 * '<S109>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 4'
 * '<S110>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 5'
 * '<S111>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 6'
 * '<S112>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 7'
 * '<S113>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec'
 * '<S114>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point'
 * '<S115>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset'
 * '<S116>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec'
 * '<S117>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset/If Action Subsystem'
 * '<S118>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset/If Action Subsystem1'
 * '<S119>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec/Variant Subsystem'
 * '<S120>' : 'Foc_model_Matlab_1/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec/Variant Subsystem/Dialog'
 * '<S121>' : 'Foc_model_Matlab_1/CurrentControl/Mechanical to Electrical Position/MechToElec'
 * '<S122>' : 'Foc_model_Matlab_1/CurrentControl/Mechanical to Electrical Position/MechToElec/floating-point'
 * '<S123>' : 'Foc_model_Matlab_1/CurrentControl/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset'
 * '<S124>' : 'Foc_model_Matlab_1/CurrentControl/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec'
 * '<S125>' : 'Foc_model_Matlab_1/CurrentControl/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset/If Action Subsystem'
 * '<S126>' : 'Foc_model_Matlab_1/CurrentControl/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset/If Action Subsystem1'
 * '<S127>' : 'Foc_model_Matlab_1/CurrentControl/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec/Variant Subsystem'
 * '<S128>' : 'Foc_model_Matlab_1/CurrentControl/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec/Variant Subsystem/Dialog'
 * '<S129>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/Back-EMF Estimation'
 * '<S130>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1'
 * '<S131>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/Saturation '
 * '<S132>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Anti-windup'
 * '<S133>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/D Gain'
 * '<S134>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/External Derivative'
 * '<S135>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Filter'
 * '<S136>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Filter ICs'
 * '<S137>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/I Gain'
 * '<S138>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Ideal P Gain'
 * '<S139>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Ideal P Gain Fdbk'
 * '<S140>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Integrator'
 * '<S141>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Integrator ICs'
 * '<S142>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/N Copy'
 * '<S143>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/N Gain'
 * '<S144>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/P Copy'
 * '<S145>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Parallel P Gain'
 * '<S146>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Reset Signal'
 * '<S147>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Saturation'
 * '<S148>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Saturation Fdbk'
 * '<S149>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Sum'
 * '<S150>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Sum Fdbk'
 * '<S151>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Tracking Mode'
 * '<S152>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Tracking Mode Sum'
 * '<S153>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Tsamp - Integral'
 * '<S154>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Tsamp - Ngain'
 * '<S155>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/postSat Signal'
 * '<S156>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/preInt Signal'
 * '<S157>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/preSat Signal'
 * '<S158>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Anti-windup/Passthrough'
 * '<S159>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/D Gain/Disabled'
 * '<S160>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/External Derivative/Disabled'
 * '<S161>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Filter/Disabled'
 * '<S162>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Filter ICs/Disabled'
 * '<S163>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/I Gain/External Parameters'
 * '<S164>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Ideal P Gain/Passthrough'
 * '<S165>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S166>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Integrator/Discrete'
 * '<S167>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Integrator ICs/Internal IC'
 * '<S168>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/N Copy/Disabled wSignal Specification'
 * '<S169>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/N Gain/Disabled'
 * '<S170>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/P Copy/Disabled'
 * '<S171>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Parallel P Gain/External Parameters'
 * '<S172>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Reset Signal/Disabled'
 * '<S173>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Saturation/Passthrough'
 * '<S174>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Saturation Fdbk/Disabled'
 * '<S175>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Sum/Sum_PI'
 * '<S176>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Sum Fdbk/Disabled'
 * '<S177>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Tracking Mode/Disabled'
 * '<S178>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S179>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S180>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S181>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/postSat Signal/Forward_Path'
 * '<S182>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/preInt Signal/Internal PreInt'
 * '<S183>' : 'Foc_model_Matlab_1/CurrentControl/STSMO/PID Controller1/preSat Signal/Forward_Path'
 * '<S184>' : 'Foc_model_Matlab_1/CurrentControl/Sensor Driver Blocks/Sensor Driver Blocks (codegen)'
 * '<S185>' : 'Foc_model_Matlab_1/CurrentControl/Speed Measurement/DT_Handle'
 * '<S186>' : 'Foc_model_Matlab_1/CurrentControl/Speed Measurement/DT_Handle/floating-point'
 * '<S187>' : 'Foc_model_Matlab_1/Hall Sensor/CodeGen'
 * '<S188>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity'
 * '<S189>' : 'Foc_model_Matlab_1/Hall Sensor/CodeGen/Bit Shift'
 * '<S190>' : 'Foc_model_Matlab_1/Hall Sensor/CodeGen/Bit Shift1'
 * '<S191>' : 'Foc_model_Matlab_1/Hall Sensor/CodeGen/Bit Shift/bit_shift'
 * '<S192>' : 'Foc_model_Matlab_1/Hall Sensor/CodeGen/Bit Shift1/bit_shift'
 * '<S193>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem'
 * '<S194>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem/Bad hall (glitch or wrong connection)'
 * '<S195>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem/Valid Halls'
 * '<S196>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem'
 * '<S197>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem1'
 * '<S198>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem2'
 * '<S199>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem3'
 * '<S200>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem4'
 * '<S201>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem5'
 * '<S202>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem6'
 * '<S203>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem7'
 * '<S204>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem8'
 * '<S205>' : 'Foc_model_Matlab_1/Hall Sensor/Hall Validity/Subsystem/Valid Halls/Switch Case Action Subsystem'
 * '<S206>' : 'Foc_model_Matlab_1/HallCodeGen/Subsystem2'
 * '<S207>' : 'Foc_model_Matlab_1/HallCodeGen/Subsystem2/ADC1_IRQHandler'
 * '<S208>' : 'Foc_model_Matlab_1/HallCodeGen/Subsystem2/PDB1_IRQHandler'
 * '<S209>' : 'Foc_model_Matlab_1/HallCodeGen/Subsystem2/ADC1_IRQHandler/FaultDetection'
 * '<S210>' : 'Foc_model_Matlab_1/HallCodeGen/Subsystem2/ADC1_IRQHandler/FaultDetection/FAILURE'
 * '<S211>' : 'Foc_model_Matlab_1/HallCodeGen/Subsystem2/ADC1_IRQHandler/FaultDetection/FAILURE/Failed Subsystem'
 * '<S212>' : 'Foc_model_Matlab_1/Hardware Initialization/Enable PDB and start FTM'
 * '<S213>' : 'Foc_model_Matlab_1/Hardware Initialization/FAULT'
 * '<S214>' : 'Foc_model_Matlab_1/Hardware Initialization/GD3000_interrupt'
 * '<S215>' : 'Foc_model_Matlab_1/Hardware Initialization/If Action Subsystem'
 * '<S216>' : 'Foc_model_Matlab_1/Hardware Initialization/enable_FTM_PDB_ADC_triggering'
 * '<S217>' : 'Foc_model_Matlab_1/Inverter and Motor - Plant Model/Codegeneration'
 * '<S218>' : 'Foc_model_Matlab_1/Serial Receive/Code Generation'
 * '<S219>' : 'Foc_model_Matlab_1/SpeedControl/PI_Controller_Speed'
 * '<S220>' : 'Foc_model_Matlab_1/SpeedControl/Speed_Ref_Selector'
 * '<S221>' : 'Foc_model_Matlab_1/SpeedControl/PI_Controller_Speed/MATLAB Function'
 */
#endif                                 /* Foc_model_Matlab_1_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */