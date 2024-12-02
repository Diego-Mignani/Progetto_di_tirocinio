/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Foc_model_Matlab.h
 *
 * Code generated for Simulink model 'Foc_model_Matlab'.
 *
 * Model version                   : 10.28
 * Simulink Coder version          : 24.2 (R2024b) 21-Jun-2024
 * MBDT for S32K1xx Series Version : 4.2.0 (R2016a-R2020a) 20-Jul-2020
 * C/C++ source code generated on  : Fri Nov 29 11:21:38 2024
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
  uint32_T ADC1_ISR_o2;                /* '<S138>/ADC1_ISR' */
  uint32_T Read_Register;              /* '<S67>/Read_Register' */
  real32_T Id_ref;                     /* '<S8>/Id_ref' */
  real32_T I_ref;                      /* '<S151>/MATLAB Function' */
  real32_T Merge;                      /* '<S91>/Merge' */
  uint16_T speedCountDelay;            /* '<S75>/speedCountDelay' */
  uint16_T DelayOneStep;               /* '<S76>/Delay One Step' */
  uint16_T Merge_m;                    /* '<S127>/Merge' */
  uint16_T Merge1;                     /* '<S127>/Merge1' */
  int16_T Merge2;                      /* '<S127>/Merge2' */
  uint8_T ADC1_ISR_o3;                 /* '<S138>/ADC1_ISR' */
  boolean_T validityDelay;             /* '<S75>/validityDelay' */
  boolean_T Digital_Input_HALL_C;      /* '<S70>/Digital_Input_HALL_C' */
  boolean_T Digital_Input_HALL_B;      /* '<S70>/Digital_Input_HALL_B' */
  boolean_T Digital_Input_HALL_A;      /* '<S70>/Digital_Input_HALL_A' */
  boolean_T Digital_Input_HALL_A_b;    /* '<S119>/Digital_Input_HALL_A' */
  boolean_T Digital_Input_HALL_B_l;    /* '<S119>/Digital_Input_HALL_B' */
  boolean_T Digital_Input_HALL_C_h;    /* '<S119>/Digital_Input_HALL_C' */
} B_Foc_model_Matlab_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  volatile real32_T RT11_Buffer[4];    /* '<Root>/RT11' */
  volatile real32_T RT2_Buffer0;       /* '<Root>/RT2' */
  real32_T N_ref_pre;                  /* '<S151>/MATLAB Function' */
  real32_T speed_error_pre;            /* '<S151>/MATLAB Function' */
  real32_T integral;                   /* '<S26>/MATLAB Function1' */
  real32_T integral_j;                 /* '<S25>/MATLAB Function1' */
  uint16_T DelayOneStep_DSTATE;        /* '<S76>/Delay One Step' */
  boolean_T DelayOneStep1_DSTATE;      /* '<S76>/Delay One Step1' */
  volatile int8_T RT11_ActiveBufIdx;   /* '<Root>/RT11' */
  uint8_T is_active_c1_Foc_model_Matlab;/* '<S4>/Enable PDB and start FTM' */
  uint8_T is_c1_Foc_model_Matlab;      /* '<S4>/Enable PDB and start FTM' */
  boolean_T N_ref_pre_not_empty;       /* '<S151>/MATLAB Function' */
  boolean_T speed_error_pre_not_empty; /* '<S151>/MATLAB Function' */
  boolean_T T_n_pre_not_empty;         /* '<S151>/MATLAB Function' */
  boolean_T integral_not_empty;        /* '<S26>/MATLAB Function1' */
  boolean_T integral_not_empty_p;      /* '<S25>/MATLAB Function1' */
} DW_Foc_model_Matlab_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real32_T SpeedConstData;       /* '<S78>/SpeedConstData' */
} ConstB_Foc_model_Matlab_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: sine_table_values_Value
   * Referenced by: '<S49>/sine_table_values'
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
extern volatile uint32_T ADC_IA;       /* '<S139>/ADC_AD4_IA' */
extern volatile uint32_T ADC_IB;       /* '<S139>/ADC_IB' */
extern volatile uint32_T ADC_IDC;      /* '<S139>/ADC_AD6_IDC' */
extern volatile uint32_T ADC_VDC;      /* '<S139>/ADC_AD7_VDC' */
extern volatile uint32_T CH0S_ERR;     /* '<S138>/PDB1_ISR' */
extern volatile uint32_T CH1S_ERR;     /* '<S138>/PDB1_ISR' */
extern volatile uint16_T CntHall;      /* '<S3>/FTM_Hall_Sensor' */
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
extern volatile uint32_T HALL_A;       /* '<S121>/bit_shift' */
extern volatile uint32_T HALL_A_controller;/* '<S70>/Data Type Conversion6' */
extern volatile uint32_T HALL_B;       /* '<S122>/bit_shift' */
extern volatile uint32_T HALL_B_controller;/* '<S72>/bit_shift' */
extern volatile uint32_T HALL_C;       /* '<S119>/Data Type Conversion6' */
extern volatile uint32_T HALL_C_controller;/* '<S71>/bit_shift' */
extern volatile uint16_T HallCntActual;/* '<Root>/Data Store Memory25' */
extern volatile uint16_T HallCntPrev;  /* '<Root>/Data Store Memory24' */
extern volatile uint16_T HallStateChangeFlag;/* '<Root>/Data Store Memory' */
extern volatile uint16_T HallValididyInvalid;/* '<S125>/Merge' */
extern volatile real32_T I_ab_afterOffset[2];/* '<S66>/Add' */
extern volatile real32_T IaOffset;     /* '<Root>/Data Store Memory5' */
extern volatile real32_T IbOffset;     /* '<Root>/Data Store Memory6' */
extern volatile real32_T Id_err;       /* '<S25>/Sum' */
extern volatile real32_T Id_fb;        /* '<S15>/Signal Copy1' */
extern volatile real32_T Idc_afterOffset;/* '<S141>/Sum' */
extern volatile real32_T Idq_ref_PU[2];/* '<Root>/RT11' */
extern volatile real32_T Iq_err;       /* '<S26>/Sum' */
extern volatile real32_T Iq_fb;        /* '<S15>/Signal Copy' */
extern volatile real32_T Lambda;       /* '<Root>/Data Store Memory9' */
extern volatile real32_T PWM[3];       /* '<S11>/Switch1' */
extern volatile real32_T PWM_Duty_Cycles[3];/* '<S12>/Gain' */
extern volatile real32_T PWM_Enable;   /* '<S12>/Data Type Conversion' */
extern volatile uint32_T SC_PDBIF;     /* '<S138>/PDB1_ISR' */
extern volatile real32_T SpeedError;   /* '<S151>/Sum' */
extern volatile real32_T SpeedMeasured;/* '<S1>/Input Scaling' */
extern volatile real32_T Speed_Ref;    /* '<S152>/Switch' */
extern volatile real32_T Speed_Ref_PU; /* '<Root>/RT2' */
extern volatile real32_T Speed_fb;     /* '<Root>/RT1' */
extern volatile real32_T Theta;        /* '<Root>/Data Store Memory10' */
extern volatile real32_T ThetaHalls;   /* '<S68>/Merge1' */

/* Real-time Model object */
extern RT_MODEL_Foc_model_Matlab_T *const Foc_model_Matlab_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S21>/Data Type Duplicate' : Unused code path elimination
 * Block '<S31>/Data Type Duplicate' : Unused code path elimination
 * Block '<S38>/Data Type Duplicate' : Unused code path elimination
 * Block '<S38>/Data Type Propagation' : Unused code path elimination
 * Block '<S39>/Data Type Duplicate' : Unused code path elimination
 * Block '<S22>/Data Type Duplicate' : Unused code path elimination
 * Block '<S29>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S29>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S30>/Sqrt' : Unused code path elimination
 * Block '<S45>/Data Type Duplicate' : Unused code path elimination
 * Block '<S45>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S47>/Data Type Duplicate' : Unused code path elimination
 * Block '<S47>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S49>/Data Type Duplicate' : Unused code path elimination
 * Block '<S49>/Data Type Propagation' : Unused code path elimination
 * Block '<S54>/Data Type Duplicate' : Unused code path elimination
 * Block '<S55>/Data Type Duplicate' : Unused code path elimination
 * Block '<S19>/Data Type Duplicate' : Unused code path elimination
 * Block '<S19>/Vc' : Unused code path elimination
 * Block '<S63>/Data Type Duplicate' : Unused code path elimination
 * Block '<S111>/Data Type Duplicate' : Unused code path elimination
 * Block '<S69>/ReplaceInport_Npp' : Unused code path elimination
 * Block '<Root>/RT10' : Unused code path elimination
 * Block '<Root>/RT8' : Unused code path elimination
 * Block '<Root>/RT9' : Unused code path elimination
 * Block '<Root>/Scope3' : Unused code path elimination
 * Block '<S151>/Scope' : Unused code path elimination
 * Block '<S151>/To Workspace' : Unused code path elimination
 * Block '<S20>/Kalpha' : Eliminated nontunable gain of 1
 * Block '<S20>/Kbeta' : Eliminated nontunable gain of 1
 * Block '<S49>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S51>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S63>/Ka' : Eliminated nontunable gain of 1
 * Block '<S63>/Kb' : Eliminated nontunable gain of 1
 * Block '<S63>/Kc' : Eliminated nontunable gain of 1
 * Block '<S68>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S68>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S68>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S68>/PositionUnit' : Eliminated nontunable gain of 1
 * Block '<S68>/counterSize2' : Eliminate redundant data type conversion
 * Block '<S113>/Multiply' : Eliminated nontunable gain of 1
 * Block '<S113>/Multiply1' : Eliminated nontunable gain of 1
 * Block '<S117>/Number of pole pairs' : Eliminated nontunable gain of 1
 * Block '<S66>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S120>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S120>/counterSize' : Eliminate redundant data type conversion
 * Block '<S29>/enableInportSatLim' : Unused code path elimination
 * Block '<S29>/enableInportSatMethod' : Unused code path elimination
 * Block '<S22>/ReplaceInport_satLim' : Unused code path elimination
 * Block '<S22>/ReplaceInport_satMethod' : Unused code path elimination
 * Block '<S46>/Offset' : Unused code path elimination
 * Block '<S46>/Unary_Minus' : Unused code path elimination
 * Block '<S48>/Offset' : Unused code path elimination
 * Block '<S48>/Unary_Minus' : Unused code path elimination
 * Block '<S111>/Constant' : Unused code path elimination
 * Block '<S69>/ReplaceInport_Offset' : Unused code path elimination
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
 * '<S22>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter'
 * '<S23>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/Nonlinear feedforwar compensation'
 * '<S24>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/Nonlinear feedforwar compensation1'
 * '<S25>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id'
 * '<S26>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq'
 * '<S27>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D-Q Equivalence'
 * '<S28>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority'
 * '<S29>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/Inport//Dialog Selection'
 * '<S30>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/Magnitude_calc'
 * '<S31>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D-Q Equivalence/Limiter'
 * '<S32>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D-Q Equivalence/Passthrough'
 * '<S33>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/Compare To Constant'
 * '<S34>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/Compare To Constant1'
 * '<S35>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/flipInputs'
 * '<S36>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/flipInputs1'
 * '<S37>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter'
 * '<S38>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/limitRef1'
 * '<S39>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/limitRef2'
 * '<S40>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/passThrough'
 * '<S41>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/MATLAB Function'
 * '<S42>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/PI_Controller_Id/MATLAB Function1'
 * '<S43>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/MATLAB Function'
 * '<S44>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Current_Controllers/PI_Controller_Iq/MATLAB Function1'
 * '<S45>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Inverse Park Transform/Two inputs CRL'
 * '<S46>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Inverse Park Transform/Two inputs CRL/Switch_Axis'
 * '<S47>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Park Transform/Two inputs CRL'
 * '<S48>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Park Transform/Two inputs CRL/Switch_Axis'
 * '<S49>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup'
 * '<S50>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/Interpolation'
 * '<S51>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp'
 * '<S52>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype'
 * '<S53>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S54>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S55>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S56>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype/datatype no change'
 * '<S57>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Modulation method'
 * '<S58>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Voltage Input'
 * '<S59>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Modulation method/SVPWM'
 * '<S60>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Modulation method/SVPWM/Half(Vmin+Vmax)'
 * '<S61>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Voltage Input/Valphabeta'
 * '<S62>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Voltage Input/Valphabeta/Inverse Clarke Transform'
 * '<S63>'  : 'Foc_model_Matlab/CurrentControl/Control_System/Space Vector Generator/Voltage Input/Valphabeta/Inverse Clarke Transform/Two phase input'
 * '<S64>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)'
 * '<S65>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall'
 * '<S66>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Convert ADC value to PU'
 * '<S67>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading'
 * '<S68>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position'
 * '<S69>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position'
 * '<S70>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read'
 * '<S71>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift'
 * '<S72>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift1'
 * '<S73>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift/bit_shift'
 * '<S74>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Atomic Hall Reading/Hall Read/Bit Shift1/bit_shift'
 * '<S75>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/ExtrapolationOrder'
 * '<S76>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Software Watchdog Timer'
 * '<S77>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant'
 * '<S78>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position'
 * '<S79>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/VaidityCheck'
 * '<S80>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Software Watchdog Timer/Compare To Zero'
 * '<S81>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction'
 * '<S82>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 1'
 * '<S83>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 2'
 * '<S84>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 3'
 * '<S85>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 4'
 * '<S86>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 5'
 * '<S87>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 6'
 * '<S88>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are not valid Position will be set to the middle of the Hall quadrant/independent Direction/Hall Value of 7'
 * '<S89>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/If Action Subsystem'
 * '<S90>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/If Action Subsystem1'
 * '<S91>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1'
 * '<S92>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction'
 * '<S93>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction'
 * '<S94>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/first_order'
 * '<S95>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/second_order'
 * '<S96>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 1'
 * '<S97>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 2'
 * '<S98>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 3'
 * '<S99>'  : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 4'
 * '<S100>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 5'
 * '<S101>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 6'
 * '<S102>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/+ve Direction/Hall Value of 7'
 * '<S103>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 1'
 * '<S104>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 2'
 * '<S105>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 3'
 * '<S106>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 4'
 * '<S107>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 5'
 * '<S108>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 6'
 * '<S109>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Hall Speed and Position/Speed and direction are valid Use speed to extrapolate position/Subsystem1/-ve Direction/Hall Value of 7'
 * '<S110>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec'
 * '<S111>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point'
 * '<S112>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset'
 * '<S113>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec'
 * '<S114>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset/If Action Subsystem'
 * '<S115>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Compensate Offset/If Action Subsystem1'
 * '<S116>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec/Variant Subsystem'
 * '<S117>' : 'Foc_model_Matlab/CurrentControl/Input Scaling/Read_Sensor (codegen)/Calculate Position from Hall/Mechanical to Electrical Position/MechToElec/floating-point/Mech To Elec/Variant Subsystem/Dialog'
 * '<S118>' : 'Foc_model_Matlab/CurrentControl/Sensor Driver Blocks/Sensor Driver Blocks (codegen)'
 * '<S119>' : 'Foc_model_Matlab/Hall Sensor/CodeGen'
 * '<S120>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity'
 * '<S121>' : 'Foc_model_Matlab/Hall Sensor/CodeGen/Bit Shift'
 * '<S122>' : 'Foc_model_Matlab/Hall Sensor/CodeGen/Bit Shift1'
 * '<S123>' : 'Foc_model_Matlab/Hall Sensor/CodeGen/Bit Shift/bit_shift'
 * '<S124>' : 'Foc_model_Matlab/Hall Sensor/CodeGen/Bit Shift1/bit_shift'
 * '<S125>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem'
 * '<S126>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Bad hall (glitch or wrong connection)'
 * '<S127>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls'
 * '<S128>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem'
 * '<S129>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem1'
 * '<S130>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem2'
 * '<S131>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem3'
 * '<S132>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem4'
 * '<S133>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem5'
 * '<S134>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem6'
 * '<S135>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem7'
 * '<S136>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/If Action Subsystem8'
 * '<S137>' : 'Foc_model_Matlab/Hall Sensor/Hall Validity/Subsystem/Valid Halls/Switch Case Action Subsystem'
 * '<S138>' : 'Foc_model_Matlab/HallCodeGen/Subsystem2'
 * '<S139>' : 'Foc_model_Matlab/HallCodeGen/Subsystem2/ADC1_IRQHandler'
 * '<S140>' : 'Foc_model_Matlab/HallCodeGen/Subsystem2/PDB1_IRQHandler'
 * '<S141>' : 'Foc_model_Matlab/HallCodeGen/Subsystem2/ADC1_IRQHandler/FaultDetection'
 * '<S142>' : 'Foc_model_Matlab/HallCodeGen/Subsystem2/ADC1_IRQHandler/FaultDetection/FAILURE'
 * '<S143>' : 'Foc_model_Matlab/HallCodeGen/Subsystem2/ADC1_IRQHandler/FaultDetection/FAILURE/Failed Subsystem'
 * '<S144>' : 'Foc_model_Matlab/Hardware Initialization/Enable PDB and start FTM'
 * '<S145>' : 'Foc_model_Matlab/Hardware Initialization/FAULT'
 * '<S146>' : 'Foc_model_Matlab/Hardware Initialization/GD3000_interrupt'
 * '<S147>' : 'Foc_model_Matlab/Hardware Initialization/If Action Subsystem'
 * '<S148>' : 'Foc_model_Matlab/Hardware Initialization/enable_FTM_PDB_ADC_triggering'
 * '<S149>' : 'Foc_model_Matlab/Inverter and Motor - Plant Model/Codegeneration'
 * '<S150>' : 'Foc_model_Matlab/Serial Receive/Code Generation'
 * '<S151>' : 'Foc_model_Matlab/SpeedControl/PI_Controller_Speed'
 * '<S152>' : 'Foc_model_Matlab/SpeedControl/Speed_Ref_Selector'
 * '<S153>' : 'Foc_model_Matlab/SpeedControl/PI_Controller_Speed/MATLAB Function'
 */
#endif                                 /* Foc_model_Matlab_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
