#include "ftm2_ch1_hall_sensor_isr.h"
#include "ftm_ic_driver.h"
#include "ftm_hw_access.h"
#include "ftm_common.h"

void FTM2_Ch1_Hall_Sensor_isr(void)
{
  CntHall = FTM_DRV_GetChnCountVal(FTM2, 1);

  /* Output and update for function-call system: '<Root>/Hall Sensor' */
  {
    int32_T rtb_Add1;
    int16_T rtb_Merge1;
    boolean_T rtb_Merge3;

    /* S-Function (gpio_s32k_input): '<S121>/Digital_Input_HALL_A' */

    /* GPIPORTA1 signal update */
    Foc_model_Matlab_B.Digital_Input_HALL_A_b = (PINS_DRV_ReadPins(PTA) >> 1) &
      0x01;

    /* Outputs for Atomic SubSystem: '<S121>/Bit Shift' */
    /* MATLAB Function: '<S123>/bit_shift' incorporates:
     *  DataTypeConversion: '<S121>/Data Type Conversion3'
     */
    /* MATLAB Function 'Logic and Bit Operations/Bit Shift/bit_shift': '<S125>:1' */
    /* '<S125>:1:6' */
    HALL_A = (uint32_T)Foc_model_Matlab_B.Digital_Input_HALL_A_b << 2;

    /* End of Outputs for SubSystem: '<S121>/Bit Shift' */

    /* S-Function (gpio_s32k_input): '<S121>/Digital_Input_HALL_B' */

    /* GPIPORTD10 signal update */
    Foc_model_Matlab_B.Digital_Input_HALL_B_l = (PINS_DRV_ReadPins(PTD) >> 10) &
      0x01;

    /* Outputs for Atomic SubSystem: '<S121>/Bit Shift1' */
    /* MATLAB Function: '<S124>/bit_shift' incorporates:
     *  DataTypeConversion: '<S121>/Data Type Conversion2'
     */
    /* MATLAB Function 'Logic and Bit Operations/Bit Shift/bit_shift': '<S126>:1' */
    /* '<S126>:1:6' */
    HALL_B = (uint32_T)Foc_model_Matlab_B.Digital_Input_HALL_B_l << 1;

    /* End of Outputs for SubSystem: '<S121>/Bit Shift1' */

    /* S-Function (gpio_s32k_input): '<S121>/Digital_Input_HALL_C' */

    /* GPIPORTD11 signal update */
    Foc_model_Matlab_B.Digital_Input_HALL_C_h = (PINS_DRV_ReadPins(PTD) >> 11) &
      0x01;

    /* DataTypeConversion: '<S121>/Data Type Conversion6' */
    HALL_C = (uint32_T)Foc_model_Matlab_B.Digital_Input_HALL_C_h;

    /* Sum: '<S121>/Add1' */
    rtb_Add1 = (int32_T)((HALL_A + HALL_B) + HALL_C);

    /* SwitchCase: '<S122>/Detects if the halls reading is valid' incorporates:
     *  DataTypeConversion: '<S122>/Data Type Conversion1'
     */
    switch (rtb_Add1) {
     case 6:
     case 4:
     case 5:
     case 1:
     case 3:
     case 2:
      /* Outputs for IfAction SubSystem: '<S127>/Valid Halls' incorporates:
       *  ActionPort: '<S129>/Action Port'
       */
      /* SwitchCase: '<S129>/Switch Case' */
      switch (rtb_Add1) {
       case 6:
        /* Outputs for IfAction SubSystem: '<S129>/If Action Subsystem' incorporates:
         *  ActionPort: '<S130>/Action Port'
         */
        /* Merge: '<S129>/Merge' incorporates:
         *  Constant: '<S130>/previous'
         *  SignalConversion generated from: '<S130>/Out1'
         */
        Foc_model_Matlab_B.Merge_m = 2U;

        /* Merge: '<S129>/Merge1' incorporates:
         *  Constant: '<S130>/next'
         *  SignalConversion generated from: '<S130>/Out2'
         */
        Foc_model_Matlab_B.Merge1 = 4U;

        /* End of Outputs for SubSystem: '<S129>/If Action Subsystem' */
        break;

       case 4:
        /* Outputs for IfAction SubSystem: '<S129>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S131>/Action Port'
         */
        /* Merge: '<S129>/Merge' incorporates:
         *  Constant: '<S131>/previous'
         *  SignalConversion generated from: '<S131>/Out1'
         */
        Foc_model_Matlab_B.Merge_m = 6U;

        /* Merge: '<S129>/Merge1' incorporates:
         *  Constant: '<S131>/next'
         *  SignalConversion generated from: '<S131>/Out2'
         */
        Foc_model_Matlab_B.Merge1 = 5U;

        /* End of Outputs for SubSystem: '<S129>/If Action Subsystem1' */
        break;

       case 5:
        /* Outputs for IfAction SubSystem: '<S129>/If Action Subsystem2' incorporates:
         *  ActionPort: '<S132>/Action Port'
         */
        /* Merge: '<S129>/Merge' incorporates:
         *  Constant: '<S132>/previous'
         *  SignalConversion generated from: '<S132>/Out1'
         */
        Foc_model_Matlab_B.Merge_m = 4U;

        /* Merge: '<S129>/Merge1' incorporates:
         *  Constant: '<S132>/next'
         *  SignalConversion generated from: '<S132>/Out2'
         */
        Foc_model_Matlab_B.Merge1 = 1U;

        /* End of Outputs for SubSystem: '<S129>/If Action Subsystem2' */
        break;

       case 1:
        /* Outputs for IfAction SubSystem: '<S129>/If Action Subsystem3' incorporates:
         *  ActionPort: '<S133>/Action Port'
         */
        /* Merge: '<S129>/Merge' incorporates:
         *  Constant: '<S133>/previous'
         *  SignalConversion generated from: '<S133>/Out1'
         */
        Foc_model_Matlab_B.Merge_m = 5U;

        /* Merge: '<S129>/Merge1' incorporates:
         *  Constant: '<S133>/next'
         *  SignalConversion generated from: '<S133>/Out2'
         */
        Foc_model_Matlab_B.Merge1 = 3U;

        /* End of Outputs for SubSystem: '<S129>/If Action Subsystem3' */
        break;

       case 3:
        /* Outputs for IfAction SubSystem: '<S129>/If Action Subsystem4' incorporates:
         *  ActionPort: '<S134>/Action Port'
         */
        /* Merge: '<S129>/Merge' incorporates:
         *  Constant: '<S134>/previous'
         *  SignalConversion generated from: '<S134>/Out1'
         */
        Foc_model_Matlab_B.Merge_m = 1U;

        /* Merge: '<S129>/Merge1' incorporates:
         *  Constant: '<S134>/next'
         *  SignalConversion generated from: '<S134>/Out2'
         */
        Foc_model_Matlab_B.Merge1 = 2U;

        /* End of Outputs for SubSystem: '<S129>/If Action Subsystem4' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S129>/If Action Subsystem5' incorporates:
         *  ActionPort: '<S135>/Action Port'
         */
        /* Merge: '<S129>/Merge' incorporates:
         *  Constant: '<S135>/previous'
         *  SignalConversion generated from: '<S135>/Out1'
         */
        Foc_model_Matlab_B.Merge_m = 3U;

        /* Merge: '<S129>/Merge1' incorporates:
         *  Constant: '<S135>/next'
         *  SignalConversion generated from: '<S135>/Out2'
         */
        Foc_model_Matlab_B.Merge1 = 6U;

        /* End of Outputs for SubSystem: '<S129>/If Action Subsystem5' */
        break;

       default:
        /* no actions */
        break;
      }

      /* End of SwitchCase: '<S129>/Switch Case' */

      /* If: '<S129>/If' incorporates:
       *  DataStoreRead: '<S2>/Data Store Read'
       *  DataTypeConversion: '<S122>/Data Type Conversion2'
       */
      if ((uint16_T)GlobalHallState == Foc_model_Matlab_B.Merge_m) {
        /* Outputs for IfAction SubSystem: '<S129>/If Action Subsystem6' incorporates:
         *  ActionPort: '<S136>/Action Port'
         */
        /* Merge: '<S129>/Merge2' incorporates:
         *  Constant: '<S136>/Constant'
         *  SignalConversion generated from: '<S136>/direction'
         */
        Foc_model_Matlab_B.Merge2 = 1;

        /* Merge: '<S129>/Merge3' incorporates:
         *  Constant: '<S136>/Constant1'
         *  SignalConversion generated from: '<S136>/sequence_check'
         */
        HallValididyInvalid = 0U;

        /* End of Outputs for SubSystem: '<S129>/If Action Subsystem6' */
      } else if ((uint16_T)GlobalHallState == Foc_model_Matlab_B.Merge1) {
        /* Outputs for IfAction SubSystem: '<S129>/If Action Subsystem7' incorporates:
         *  ActionPort: '<S137>/Action Port'
         */
        /* Merge: '<S129>/Merge2' incorporates:
         *  Constant: '<S137>/Constant'
         *  SignalConversion generated from: '<S137>/direction'
         */
        Foc_model_Matlab_B.Merge2 = -1;

        /* Merge: '<S129>/Merge3' incorporates:
         *  Constant: '<S137>/Constant1'
         *  SignalConversion generated from: '<S137>/sequence_check'
         */
        HallValididyInvalid = 0U;

        /* End of Outputs for SubSystem: '<S129>/If Action Subsystem7' */
      } else {
        /* Outputs for IfAction SubSystem: '<S129>/If Action Subsystem8' incorporates:
         *  ActionPort: '<S138>/Action Port'
         */
        /* Merge: '<S129>/Merge3' incorporates:
         *  Constant: '<S138>/Constant'
         *  SignalConversion generated from: '<S138>/sequence_check'
         */
        HallValididyInvalid = 1U;

        /* End of Outputs for SubSystem: '<S129>/If Action Subsystem8' */
      }

      /* End of If: '<S129>/If' */

      /* SignalConversion: '<S129>/Signal Conversion1' */
      rtb_Merge1 = Foc_model_Matlab_B.Merge2;

      /* Switch: '<S129>/Switch' incorporates:
       *  Constant: '<S129>/Constant'
       *  DataStoreRead: '<S2>/Data Store Read1'
       *  RelationalOperator: '<S129>/Relational Operator'
       */
      if (HallValididyInvalid != 0) {
        rtb_Merge3 = false;
      } else {
        rtb_Merge3 = (Foc_model_Matlab_B.Merge2 == GlobalDirection);
      }

      /* End of Switch: '<S129>/Switch' */
      /* End of Outputs for SubSystem: '<S127>/Valid Halls' */
      break;

     default:
      /* Outputs for IfAction SubSystem: '<S127>/Bad hall (glitch or wrong connection)' incorporates:
       *  ActionPort: '<S128>/Action Port'
       */
      /* Merge: '<S127>/Merge' incorporates:
       *  Constant: '<S128>/Constant'
       *  SignalConversion generated from: '<S128>/inValidHall'
       */
      HallValididyInvalid = 1U;

      /* SignalConversion: '<S128>/Signal Conversion' incorporates:
       *  DataStoreRead: '<S2>/Data Store Read1'
       */
      rtb_Merge1 = GlobalDirection;

      /* SignalConversion generated from: '<S128>/directional_speed_valid_flag' incorporates:
       *  Constant: '<S128>/Constant1'
       */
      rtb_Merge3 = false;

      /* End of Outputs for SubSystem: '<S127>/Bad hall (glitch or wrong connection)' */
      break;
    }

    /* End of SwitchCase: '<S122>/Detects if the halls reading is valid' */

    /* DataTypeConversion: '<S127>/Data Type Conversion' incorporates:
     *  DataStoreWrite: '<S2>/Data Store Write'
     */
    GlobalSpeedValidity = (uint16_T)rtb_Merge3;

    /* DataStoreWrite: '<S2>/Data Store Write1' */
    GlobalDirection = rtb_Merge1;

    /* DataStoreWrite: '<S2>/Data Store Write2' incorporates:
     *  Constant: '<S122>/Constant'
     */
    HallStateChangeFlag = 1U;

    /* Switch: '<S2>/Switch' incorporates:
     *  DataStoreRead: '<S2>/Data Store Read3'
     *  DataStoreWrite: '<S2>/Data Store Write4'
     */
    if (HallCntActual > 100) {
      HallCntPrev = HallCntActual;
    }

    /* End of Switch: '<S2>/Switch' */

    /* SignalConversion generated from: '<S2>/Variant Source2' incorporates:
     *  DataStoreWrite: '<S2>/Data Store Write4'
     */
    CntHallValidityIn = HallCntPrev;

    /* DataStoreWrite: '<S2>/Data Store Write3' */
    GlobalSpeedCount = CntHallValidityIn;

    /* DataStoreWrite: '<S2>/Data Store Write7' */
    GlobalHallState = (uint32_T)rtb_Add1;
  }
}
