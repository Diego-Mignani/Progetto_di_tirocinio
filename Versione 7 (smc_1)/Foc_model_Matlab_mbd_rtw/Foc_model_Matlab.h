/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Foc_model_Matlab.h
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

#ifndef Foc_model_Matlab_h_
#define Foc_model_Matlab_h_
#ifndef Foc_model_Matlab_COMMON_INCLUDES_
#define Foc_model_Matlab_COMMON_INCLUDES_
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
#endif                                 /* Foc_model_Matlab_COMMON_INCLUDES_ */

#include "Foc_model_Matlab_types.h"
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
  uint32_T ADC1_ISR_o2;                /* '<S140>/ADC1_ISR' */
  uint32_T Read_Register;              /* '<S69>/Read_Register' */
  real32_T Gain2;                      /* '<S153>/Gain2' */
  real32_T DiscreteTimeIntegrator;     /* '<S159>/Discrete-Time Integrator' */
  real32_T Merge;                      /* '<S93>/Merge' */
  real32_T DiscreteTimeIntegrator_d;   /* '<S32>/Discrete-Time Integrator' */
  real32_T OutportBufferForOut1;       /* '<S32>/Discrete-Time Integrator' */
  real32_T OutportBufferForOut1_b;     /* '<S29>/Discrete-Time Integrator' */
  uint16_T speedCountDelay;            /* '<S77>/speedCountDelay' */
  uint16_T DelayOneStep;               /* '<S78>/Delay One Step' */
  uint16_T Merge_m;                    /* '<S129>/Merge' */
  uint16_T Merge1;                     /* '<S129>/Merge1' */
  int16_T Merge2;                      /* '<S129>/Merge2' */
  uint8_T ADC1_ISR_o3;                 /* '<S140>/ADC1_ISR' */
  boolean_T validityDelay;             /* '<S77>/validityDelay' */
  boolean_T Digital_Input_HALL_C;      /* '<S72>/Digital_Input_HALL_C' */
  boolean_T Digital_Input_HALL_B;      /* '<S72>/Digital_Input_HALL_B' */
  boolean_T Digital_Input_HALL_A;      /* '<S72>/Digital_Input_HALL_A' */
  boolean_T Digital_Input_HALL_A_b;    /* '<S121>/Digital_Input_HALL_A' */
  boolean_T Digital_Input_HALL_B_l;    /* '<S121>/Digital_Input_HALL_B' */
  boolean_T Digital_Input_HALL_C_h;    /* '<S121>/Digital_Input_HALL_C' */
} B_Foc_model_Matlab_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay_DSTATE;             /* '<S153>/Unit Delay' */
  real_T UnitDelay_DSTATE_j;           /* '<S22>/Unit Delay' */
  real_T UnitDelay_DSTATE_f;           /* '<S23>/Unit Delay' */
  real32_T UD_DSTATE;                  /* '<S158>/UD' */
  real32_T DiscreteTimeIntegrator_DSTATE;/* '<S159>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_d;/* '<S32>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_df;/* '<S29>/Discrete-Time Integrator' */
  volatile real32_T RT11_Buffer0;      /* '<Root>/RT11' */
  volatile real32_T RT2_Buffer0;       /* '<Root>/RT2' */
  real32_T DiscreteTimeIntegrator_PREV_U;/* '<S32>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_PREV_U_p;/* '<S29>/Discrete-Time Integrator' */
  uint32_T Subsystem_PREV_T;           /* '<S23>/Subsystem' */
  uint32_T Subsystem_PREV_T_f;         /* '<S22>/Subsystem' */
  uint16_T DelayOneStep_DSTATE;        /* '<S78>/Delay One Step' */
  boolean_T DelayOneStep1_DSTATE;      /* '<S78>/Delay One Step1' */
  uint8_T is_active_c1_Foc_model_Matlab;/* '<S4>/Enable PDB and start FTM' */
  uint8_T is_c1_Foc_model_Matlab;      /* '<S4>/Enable PDB and start FTM' */
  uint8_T DiscreteTimeIntegrator_SYSTEM_E;/* '<S32>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator_SYSTEM_h;/* '<S29>/Discrete-Time Integrator' */
  boolean_T Subsystem_RESET_ELAPS_T;   /* '<S23>/Subsystem' */
  boolean_T Subsystem_RESET_ELAPS_T_k; /* '<S22>/Subsystem' */
  boolean_T Subsystem_MODE;            /* '<S23>/Subsystem' */
  boolean_T Subsystem_MODE_n;          /* '<S22>/Subsystem' */
} DW_Foc_model_Matlab_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real32_T CastToSingle;         /* '<S153>/Cast To Single' */
  const real32_T CastToSingle1;        /* '<S153>/Cast To Single1' */
  const real32_T CastToSingle2;        /* '<S153>/Cast To Single2' */
  const real32_T SpeedConstData;       /* '<S80>/SpeedConstData' */
} ConstB_Foc_model_Matlab_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: sine_table_values_Value
   * Referenced by: '<S51>/sine_table_values'
   */
  real32_T sine_table_values_Value[1002];
} ConstP_Foc_model_Matlab_T;

/* Real-time Model Data Structure */
struct tag_RTM_Foc_model_Matlab_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick1;
    struct {
      uint16_T TID[4];
    } TaskCounters;
  } Timing;
};

/* Block signals (default storage) */
extern B_Foc_model_Matlab_T Foc_model_Matlab_B;

/* Block states (default storage) */
extern DW_Foc_model_Matlab_T Foc_model_Matlab_DW;
extern const ConstB_Foc_model_Matlab_T Foc_model_Matlab_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_Foc_model_Matlab_T Foc_model_Matlab_ConstP;

/* External function called from main */
extern void Foc_model_Matlab_SetEventsForThisBaseStep(boolean_T *eventFlags);

/* Model entry point functions */
extern void Foc_model_Matlab_initialize(void);
extern void Foc_model_Matlab_step0(void);/* Sample time: [5.0E-5s, 0.0s] */
extern void Foc_model_Matlab_step1(void);/* Sample time: [0.0001s, 0.0s] */
extern void Foc_model_Matlab_step2(void);/* Sample time: [0.001s, 0.0s] */
extern void Foc_model_Matlab_step3(void);/* Sample time: [0.1s, 0.0s] */
extern void Foc_model_Matlab_terminate(void);

/* Exported data declaration */

/* Volatile memory section */
/* Declaration for custom storage class: Volatile */
extern volatile real32_T ADC_A;        /* '<Root>/Data Store Memory11' */
extern volatile real32_T ADC_B;        /* '<Root>/Data Store Memory12' */
extern volatile uint32_T ADC_IA;       /* '<S141>/ADC_AD4_IA' */
extern volatile uint32_T ADC_IB;       /* '<S141>/ADC_IB' */
extern volatile uint32_T ADC_IDC;      /* '<S141>/ADC_AD6_IDC' */
extern volatile uint32_T ADC_VDC;      /* '<S141>/ADC_AD7_VDC' */
extern volatile uint32_T CH0S_ERR;     /* '<S140>/PDB1_ISR' */
extern volatile uint32_T CH1S_ERR;     /* '<S140>/PDB1_ISR' */
extern volatile uint16_T CntHall;      /* '<S3>/FTM_Hall_Sensor' */
extern volatile uint16_T CntHallValidityIn;
                                /* '<S2>/SigConvForSigProp_Variant_Source2_0' */
extern volatile real32_T DesiredSpeed; /* '<Root>/Data Store Memory7' */
extern volatile boolean_T Enable;      /* '<Root>/Data Store Memory29' */
extern volatile real32_T Epsilon_d;    /* '<Root>/Data Store Memory16' */
extern volatile real32_T Epsilon_q;    /* '<Root>/Data Store Memory14' */
extern volatile real32_T Epsilon_w;    /* '<Root>/Data Store Memory13' */
extern volatile boolean_T FAULT;       /* '<Root>/I_MAX Scalling3' */
extern volatile int16_T GlobalDirection;/* '<Root>/Data Store Memory3' */
extern volatile uint32_T GlobalHallState;/* '<Root>/Data Store Memory4' */
extern volatile uint16_T GlobalSpeedCount;/* '<Root>/Data Store Memory1' */
extern volatile uint16_T GlobalSpeedValidity;/* '<Root>/Data Store Memory2' */
extern volatile uint32_T HALL_A;       /* '<S123>/bit_shift' */
extern volatile uint32_T HALL_A_controller;/* '<S72>/Data Type Conversion6' */
extern volatile uint32_T HALL_B;       /* '<S124>/bit_shift' */
extern volatile uint32_T HALL_B_controller;/* '<S74>/bit_shift' */
extern volatile uint32_T HALL_C;       /* '<S121>/Data Type Conversion6' */
extern volatile uint32_T HALL_C_controller;/* '<S73>/bit_shift' */
extern volatile uint16_T HallCntActual;/* '<Root>/Data Store Memory25' */
extern volatile uint16_T HallCntPrev;  /* '<Root>/Data Store Memory24' */
extern volatile uint16_T HallStateChangeFlag;/* '<Root>/Data Store Memory' */
extern volatile uint16_T HallValididyInvalid;/* '<S127>/Merge' */
extern volatile real32_T I_ab_afterOffset[2];/* '<S68>/Add' */
extern volatile real32_T IaOffset;     /* '<Root>/Data Store Memory5' */
extern volatile real32_T IbOffset;     /* '<Root>/Data Store Memory6' */
extern volatile real32_T Id_fb;        /* '<S15>/Signal Copy1' */
extern volatile real32_T Idc_afterOffset;/* '<S143>/Sum' */
extern volatile real32_T Iq_err;       /* '<S23>/Sum1' */
extern volatile real32_T Iq_fb;        /* '<S15>/Signal Copy' */
extern volatile real32_T Iq_ref_PU;    /* '<Root>/RT11' */
extern volatile real32_T Lambda_d;     /* '<Root>/Data Store Memory17' */
extern volatile real32_T Lambda_q;     /* '<Root>/Data Store Memory15' */
extern volatile real32_T Lambda_w;     /* '<Root>/Data Store Memory9' */
extern volatile real32_T PWM[3];       /* '<S11>/Switch1' */
extern volatile real32_T PWM_Duty_Cycles[3];/* '<S12>/Gain' */
extern volatile real32_T PWM_Enable;   /* '<S12>/Data Type Conversion' */
extern volatile real32_T Rho_d;        /* '<Root>/Data Store Memory18' */
extern volatile real32_T Rho_q;        /* '<Root>/Data Store Memory8' */
extern volatile real32_T Rho_w;        /* '<Root>/Data Store Memory10' */
extern volatile uint32_T SC_PDBIF;     /* '<S140>/PDB1_ISR' */
extern volatile real32_T SpeedError;   /* '<S156>/Sum' */
extern volatile real32_T SpeedMeasured;/* '<S1>/Input Scaling' */
extern volatile real32_T Speed_Ref;    /* '<S154>/Switch' */
extern volatile real32_T Speed_Ref_PU; /* '<Root>/RT2' */
extern volatile real32_T Speed_fb;     /* '<Root>/RT1' */
extern volatile real32_T ThetaHalls;   /* '<S70>/Merge1' */
extern volatile real32_T limit_d;      /* '<Root>/Data Store Memory19' */
extern volatile real32_T limit_q;      /* '<Root>/Data Store Memory21' */
extern volatile real32_T limit_w;      /* '<Root>/Data Store Memory20' */

/* Real-time Model object */
extern RT_MODEL_Foc_model_Matlab_T *const Foc_model_Matlab_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S21>/Data Type Duplicate' : Unused code path elimination
 * Block '<S22>/Scope' : Unused code path elimination
 * Block '<S23>/Scope' : Unused code path elimination
 * Block '<S37>/Data Type Duplicate' : Unused code path elimination
 * Block '<S44>/Data Type Duplicate' : Unused code path elimination
 * Block '<S44>/Data Type Propagation' : Unused code path elimination
 * Block '<S45>/Data Type Duplicate' : Unused code path elimination
 * Block '<S24>/Data Type Duplicate' : Unused code path elimination
 * Block '<S35>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S35>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S36>/Sqrt' : Unused code path elimination
 * Block '<S15>/Id_ref_PU' : Unused code path elimination
 * Block '<S47>/Data Type Duplicate' : Unused code path elimination
 * Block '<S47>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S49>/Data Type Duplicate' : Unused code path elimination
 * Block '<S49>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S51>/Data Type Duplicate' : Unused code path elimination
 * Block '<S51>/Data Type Propagation' : Unused code path elimination
 * Block '<S56>/Data Type Duplicate' : Unused code path elimination
 * Block '<S57>/Data Type Duplicate' : Unused code path elimination
 * Block '<S19>/Data Type Duplicate' : Unused code path elimination
 * Block '<S19>/Vc' : Unused code path elimination
 * Block '<S65>/Data Type Duplicate' : Unused code path elimination
 * Block '<S113>/Data Type Duplicate' : Unused code path elimination
 * Block '<S71>/ReplaceInport_Npp' : Unused code path elimination
 * Block '<Root>/RT10' : Unused code path elimination
 * Block '<Root>/RT8' : Unused code path elimination
 * Block '<Root>/RT9' : Unused code path elimination
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<Root>/Scope1' : Unused code path elimination
 * Block '<Root>/Scope3' : Unused code path elimination
 * Block '<S158>/Data Type Duplicate' : Unused code path elimination
 * Block '<S156>/To Workspace' : Unused code path elimination
 * Block '<S153>/Scope' : Unused code path elimination
 * Block '<S20>/Kalpha' : Eliminated nontunable gain of 1
 * Block '<S20>/Kbeta' : Eliminated nontunable gain of 1
 * Block '<S51>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S53>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S65>/Ka' : Eliminated nontunable gain of 1
 * Block '<S65>/Kb' : Eliminated nontunable gain of 1
 * Block '<S65>/Kc' : Eliminated nontunable gain of 1
 * Block '<S70>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S70>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S70>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S70>/PositionUnit' : Eliminated nontunable gain of 1
 * Block '<S70>/counterSize2' : Eliminate redundant data type conversion
 * Block '<S115>/Multiply' : Eliminated nontunable gain of 1
 * Block '<S115>/Multiply1' : Eliminated nontunable gain of 1
 * Block '<S119>/Number of pole pairs' : Eliminated nontunable gain of 1
 * Block '<S68>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S122>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S122>/counterSize' : Eliminate redundant data type conversion
 * Block '<S35>/enableInportSatLim' : Unused code path elimination
 * Block '<S35>/enableInportSatMethod' : Unused code path elimination
 * Block '<S24>/ReplaceInport_satLim' : Unused code path elimination
 * Block '<S24>/ReplaceInport_satMethod' : Unused code path elimination
 * Block '<S48>/Offset' : Unused code path elimination
 * Block '<S48>/Unary_Minus' : Unused code path elimination
 * Block '<S50>/Offset' : Unused code path elimination
 * Block '<S50>/Unary_Minus' : Unused code path elimination
 * Block '<S113>/Constant' : Unused code path elimination
 * Block '<S71>/ReplaceInport_Offset' : Unused code path elimination
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
 * '<Root>' : 'Foc_model_Matlab'
 * '<S1>'   : 'Foc_model_Matlab/CurrentControl'
 * '<S2>'   : 'Foc_model_Matlab/Hall Sensor'
 * '<S3>'   : 'Foc_model_Matlab/HallCodeGen'
 * '<S4>'   : 'Foc_model_Matlab/Hardware Initialization'
 * '<S5>'   : 'Foc_model_Matlab/Inverter and Motor - Plant Model'
 * '<S6>'   : 'Foc_model_Matlab/Model Info'
 * '<S7>'   : 'Foc_model_Matlab/Serial Receive'
 * '<S8>'   : 'Foc_model_Matlab/SpeedControl'
 * '<S9>'   : 'Foc_model_Matlab/CurrentControl/Control_System'
 * '<S10>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling'
 * '<S11>'  : 'Foc_model_Matlab/CurrentControl/Inverter (Code Generation)'
 * '<S12>'  : 'Foc_model_Matlab/CurrentControl/Output Scaling'
 * '<S13>'  : 'Foc_model_Matlab/CurrentControl/Sensor Driver Blocks'
 * '<S14>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Clarke Transform'
 * '<S15>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers'
 * '<S16>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Inverse Park Transform'
 * '<S17>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Park Transform'
 * '<S18>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup'
 * '<S19>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator'
 * '<S20>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Clarke Transform/Two phase input'
 * '<S21>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Clarke Transform/Two phase input/Two phase CRL wrap'
 * '<S22>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/Controller_Id'
 * '<S23>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/Controller_Iq'
 * '<S24>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter'
 * '<S25>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/Nonlinear feedforwar compensation'
 * '<S26>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/Nonlinear feedforwar compensation1'
 * '<S27>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/Controller_Id/MATLAB Function2'
 * '<S28>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/Controller_Id/Parametri_design'
 * '<S29>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/Controller_Id/Subsystem'
 * '<S30>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/Controller_Iq/MATLAB Function2'
 * '<S31>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/Controller_Iq/Parametri_design'
 * '<S32>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/Controller_Iq/Subsystem'
 * '<S33>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D-Q Equivalence'
 * '<S34>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority'
 * '<S35>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/Inport//Dialog Selection'
 * '<S36>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/Magnitude_calc'
 * '<S37>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D-Q Equivalence/Limiter'
 * '<S38>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D-Q Equivalence/Passthrough'
 * '<S39>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/Compare To Constant'
 * '<S40>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/Compare To Constant1'
 * '<S41>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/flipInputs'
 * '<S42>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/flipInputs1'
 * '<S43>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter'
 * '<S44>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/limitRef1'
 * '<S45>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/limitRef2'
 * '<S46>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/passThrough'
 * '<S47>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Inverse Park Transform/Two inputs CRL'
 * '<S48>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Inverse Park Transform/Two inputs CRL/Switch_Axis'
 * '<S49>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Park Transform/Two inputs CRL'
 * '<S50>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Park Transform/Two inputs CRL/Switch_Axis'
 * '<S51>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup'
 * '<S52>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/Interpolation'
 * '<S53>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp'
 * '<S54>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype'
 * '<S55>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S56>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S57>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S58>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype/datatype no change'
 * '<S59>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Modulation method'
 * '<S60>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Voltage Input'
 * '<S61>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Modulation method/SVPWM'
 * '<S62>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Modulation method/SVPWM/Half(Vmin+Vmax)'
 * '<S63>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Voltage Input/Valphabeta'
 * '<S64>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Voltage Input/Valphabeta/Inverse Clarke Transform'
 * '<S65>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Voltage Input/Valphabeta/Inverse Clarke Transform/Two phase input'
 * '<S66>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)'
 * '<S67>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall'
 * '<S68>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Convert ADC value to PU'
 * '<S69>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading'
 * '<S70>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position'
 * '<S71>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position'
 * '<S72>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read'
 * '<S73>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift'
 * '<S74>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift1'
 * '<S75>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift/bit_shift'
 * '<S76>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift1/bit_shift'
 * '<S77>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/ExtrapolationOrder'
 * '<S78>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Software Watchdog Timer'
 * '<S79>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant'
 * '<S80>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position'
 * '<S81>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/VaidityCheck'
 * '<S82>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Software Watchdog Timer/Compare To Zero'
 * '<S83>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction'
 * '<S84>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 1'
 * '<S85>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 2'
 * '<S86>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 3'
 * '<S87>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 4'
 * '<S88>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 5'
 * '<S89>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 6'
 * '<S90>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 7'
 * '<S91>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/If Action Subsystem'
 * '<S92>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/If Action Subsystem1'
 * '<S93>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1'
 * '<S94>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction'
 * '<S95>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction'
 * '<S96>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/first_order'
 * '<S97>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/second_order'
 * '<S98>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 1'
 * '<S99>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 2'
 * '<S100>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 3'
 * '<S101>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 4'
 * '<S102>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 5'
 * '<S103>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 6'
 * '<S104>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 7'
 * '<S105>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 1'
 * '<S106>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 2'
 * '<S107>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 3'
 * '<S108>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 4'
 * '<S109>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 5'
 * '<S110>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 6'
 * '<S111>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 7'
 * '<S112>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec'
 * '<S113>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point'
 * '<S114>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset'
 * '<S115>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec'
 * '<S116>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset/If Action Subsystem'
 * '<S117>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset/If Action Subsystem1'
 * '<S118>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec/Variant Subsystem'
 * '<S119>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec/Variant Subsystem/Dialog'
 * '<S120>' : 'Foc_model_Matlab/CurrentControl/Sensor Driver Blocks/Sensor Driver Blocks (codegen)'
 * '<S121>' : 'Foc_model_Matlab/Hall Sensor/CodeGen'
 * '<S122>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity'
 * '<S123>' : 'Foc_model_Matlab/Hall Sensor/CodeGen/Bit Shift'
 * '<S124>' : 'Foc_model_Matlab/Hall Sensor/CodeGen/Bit Shift1'
 * '<S125>' : 'Foc_model_Matlab/Hall Sensor/CodeGen/Bit Shift/bit_shift'
 * '<S126>' : 'Foc_model_Matlab/Hall Sensor/CodeGen/Bit Shift1/bit_shift'
 * '<S127>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem'
 * '<S128>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Bad hall (glitch or wrong connection)'
 * '<S129>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls'
 * '<S130>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem'
 * '<S131>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem1'
 * '<S132>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem2'
 * '<S133>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem3'
 * '<S134>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem4'
 * '<S135>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem5'
 * '<S136>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem6'
 * '<S137>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem7'
 * '<S138>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem8'
 * '<S139>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/Switch Case Action Subsystem'
 * '<S140>' : 'Foc_model_Matlab/HallCodeGen/Subsystem2'
 * '<S141>' : 'Foc_model_Matlab/HallCodeGen/Subsystem2/ADC1_IRQHandler'
 * '<S142>' : 'Foc_model_Matlab/HallCodeGen/Subsystem2/PDB1_IRQHandler'
 * '<S143>' : 'Foc_model_Matlab/HallCodeGen/Subsystem2/ADC1_IRQHandler/FaultDetection'
 * '<S144>' : 'Foc_model_Matlab/HallCodeGen/Subsystem2/ADC1_IRQHandler/FaultDetection/FAILURE'
 * '<S145>' : 'Foc_model_Matlab/HallCodeGen/Subsystem2/ADC1_IRQHandler/FaultDetection/FAILURE/Failed Subsystem'
 * '<S146>' : 'Foc_model_Matlab/Hardware Initialization/Enable PDB and start FTM'
 * '<S147>' : 'Foc_model_Matlab/Hardware Initialization/FAULT'
 * '<S148>' : 'Foc_model_Matlab/Hardware Initialization/GD3000_interrupt'
 * '<S149>' : 'Foc_model_Matlab/Hardware Initialization/If Action Subsystem'
 * '<S150>' : 'Foc_model_Matlab/Hardware Initialization/enable_FTM_PDB_ADC_triggering'
 * '<S151>' : 'Foc_model_Matlab/Inverter and Motor - Plant Model/Codegeneration'
 * '<S152>' : 'Foc_model_Matlab/Serial Receive/Code Generation'
 * '<S153>' : 'Foc_model_Matlab/SpeedControl/Contrloller_wr'
 * '<S154>' : 'Foc_model_Matlab/SpeedControl/Speed_Ref_Selector'
 * '<S155>' : 'Foc_model_Matlab/SpeedControl/Contrloller_wr/MATLAB Function'
 * '<S156>' : 'Foc_model_Matlab/SpeedControl/Contrloller_wr/Operazioni matematiche'
 * '<S157>' : 'Foc_model_Matlab/SpeedControl/Contrloller_wr/Parametri_design'
 * '<S158>' : 'Foc_model_Matlab/SpeedControl/Contrloller_wr/Operazioni matematiche/Discrete Derivative'
 * '<S159>' : 'Foc_model_Matlab/SpeedControl/Contrloller_wr/Operazioni matematiche/Subsystem'
 */
#endif                                 /* Foc_model_Matlab_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
