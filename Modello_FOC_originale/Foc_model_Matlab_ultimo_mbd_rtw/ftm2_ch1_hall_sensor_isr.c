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

    /* S-Function (gpio_s32k_input): '<S189>/Digital_Input_HALL_A' */

    /* GPIPORTA1 signal update */
    Foc_model_Matlab_ultimo_B.Digital_Input_HALL_A_b = (PINS_DRV_ReadPins(PTA) >>
      1) & 0x01;

    /* Outputs for Atomic SubSystem: '<S189>/Bit Shift' */
    /* MATLAB Function: '<S191>/bit_shift' incorporates:
     *  DataTypeConversion: '<S189>/Data Type Conversion3'
     */
    /* MATLAB Function 'Logic and Bit Operations/Bit Shift/bit_shift': '<S193>:1' */
    /* '<S193>:1:6' */
    HALL_A = (uint32_T)Foc_model_Matlab_ultimo_B.Digital_Input_HALL_A_b << 2;

    /* End of Outputs for SubSystem: '<S189>/Bit Shift' */

    /* S-Function (gpio_s32k_input): '<S189>/Digital_Input_HALL_B' */

    /* GPIPORTD10 signal update */
    Foc_model_Matlab_ultimo_B.Digital_Input_HALL_B_l = (PINS_DRV_ReadPins(PTD) >>
      10) & 0x01;

    /* Outputs for Atomic SubSystem: '<S189>/Bit Shift1' */
    /* MATLAB Function: '<S192>/bit_shift' incorporates:
     *  DataTypeConversion: '<S189>/Data Type Conversion2'
     */
    /* MATLAB Function 'Logic and Bit Operations/Bit Shift/bit_shift': '<S194>:1' */
    /* '<S194>:1:6' */
    HALL_B = (uint32_T)Foc_model_Matlab_ultimo_B.Digital_Input_HALL_B_l << 1;

    /* End of Outputs for SubSystem: '<S189>/Bit Shift1' */

    /* S-Function (gpio_s32k_input): '<S189>/Digital_Input_HALL_C' */

    /* GPIPORTD11 signal update */
    Foc_model_Matlab_ultimo_B.Digital_Input_HALL_C_h = (PINS_DRV_ReadPins(PTD) >>
      11) & 0x01;

    /* DataTypeConversion: '<S189>/Data Type Conversion6' */
    HALL_C = (uint32_T)Foc_model_Matlab_ultimo_B.Digital_Input_HALL_C_h;

    /* Sum: '<S189>/Add1' */
    rtb_Add1 = (int32_T)((HALL_A + HALL_B) + HALL_C);

    /* SwitchCase: '<S190>/Detects if the halls reading is valid' incorporates:
     *  DataTypeConversion: '<S190>/Data Type Conversion1'
     */
    switch (rtb_Add1) {
     case 6:
     case 4:
     case 5:
     case 1:
     case 3:
     case 2:
      /* Outputs for IfAction SubSystem: '<S195>/Valid Halls' incorporates:
       *  ActionPort: '<S197>/Action Port'
       */
      /* SwitchCase: '<S197>/Switch Case' */
      switch (rtb_Add1) {
       case 6:
        /* Outputs for IfAction SubSystem: '<S197>/If Action Subsystem' incorporates:
         *  ActionPort: '<S198>/Action Port'
         */
        /* Merge: '<S197>/Merge' incorporates:
         *  Constant: '<S198>/previous'
         *  SignalConversion generated from: '<S198>/Out1'
         */
        Foc_model_Matlab_ultimo_B.Merge_m = 2U;

        /* Merge: '<S197>/Merge1' incorporates:
         *  Constant: '<S198>/next'
         *  SignalConversion generated from: '<S198>/Out2'
         */
        Foc_model_Matlab_ultimo_B.Merge1 = 4U;

        /* End of Outputs for SubSystem: '<S197>/If Action Subsystem' */
        break;

       case 4:
        /* Outputs for IfAction SubSystem: '<S197>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S199>/Action Port'
         */
        /* Merge: '<S197>/Merge' incorporates:
         *  Constant: '<S199>/previous'
         *  SignalConversion generated from: '<S199>/Out1'
         */
        Foc_model_Matlab_ultimo_B.Merge_m = 6U;

        /* Merge: '<S197>/Merge1' incorporates:
         *  Constant: '<S199>/next'
         *  SignalConversion generated from: '<S199>/Out2'
         */
        Foc_model_Matlab_ultimo_B.Merge1 = 5U;

        /* End of Outputs for SubSystem: '<S197>/If Action Subsystem1' */
        break;

       case 5:
        /* Outputs for IfAction SubSystem: '<S197>/If Action Subsystem2' incorporates:
         *  ActionPort: '<S200>/Action Port'
         */
        /* Merge: '<S197>/Merge' incorporates:
         *  Constant: '<S200>/previous'
         *  SignalConversion generated from: '<S200>/Out1'
         */
        Foc_model_Matlab_ultimo_B.Merge_m = 4U;

        /* Merge: '<S197>/Merge1' incorporates:
         *  Constant: '<S200>/next'
         *  SignalConversion generated from: '<S200>/Out2'
         */
        Foc_model_Matlab_ultimo_B.Merge1 = 1U;

        /* End of Outputs for SubSystem: '<S197>/If Action Subsystem2' */
        break;

       case 1:
        /* Outputs for IfAction SubSystem: '<S197>/If Action Subsystem3' incorporates:
         *  ActionPort: '<S201>/Action Port'
         */
        /* Merge: '<S197>/Merge' incorporates:
         *  Constant: '<S201>/previous'
         *  SignalConversion generated from: '<S201>/Out1'
         */
        Foc_model_Matlab_ultimo_B.Merge_m = 5U;

        /* Merge: '<S197>/Merge1' incorporates:
         *  Constant: '<S201>/next'
         *  SignalConversion generated from: '<S201>/Out2'
         */
        Foc_model_Matlab_ultimo_B.Merge1 = 3U;

        /* End of Outputs for SubSystem: '<S197>/If Action Subsystem3' */
        break;

       case 3:
        /* Outputs for IfAction SubSystem: '<S197>/If Action Subsystem4' incorporates:
         *  ActionPort: '<S202>/Action Port'
         */
        /* Merge: '<S197>/Merge' incorporates:
         *  Constant: '<S202>/previous'
         *  SignalConversion generated from: '<S202>/Out1'
         */
        Foc_model_Matlab_ultimo_B.Merge_m = 1U;

        /* Merge: '<S197>/Merge1' incorporates:
         *  Constant: '<S202>/next'
         *  SignalConversion generated from: '<S202>/Out2'
         */
        Foc_model_Matlab_ultimo_B.Merge1 = 2U;

        /* End of Outputs for SubSystem: '<S197>/If Action Subsystem4' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S197>/If Action Subsystem5' incorporates:
         *  ActionPort: '<S203>/Action Port'
         */
        /* Merge: '<S197>/Merge' incorporates:
         *  Constant: '<S203>/previous'
         *  SignalConversion generated from: '<S203>/Out1'
         */
        Foc_model_Matlab_ultimo_B.Merge_m = 3U;

        /* Merge: '<S197>/Merge1' incorporates:
         *  Constant: '<S203>/next'
         *  SignalConversion generated from: '<S203>/Out2'
         */
        Foc_model_Matlab_ultimo_B.Merge1 = 6U;

        /* End of Outputs for SubSystem: '<S197>/If Action Subsystem5' */
        break;

       default:
        /* no actions */
        break;
      }

      /* End of SwitchCase: '<S197>/Switch Case' */

      /* If: '<S197>/If' incorporates:
       *  DataStoreRead: '<S2>/Data Store Read'
       *  DataTypeConversion: '<S190>/Data Type Conversion2'
       */
      if ((uint16_T)GlobalHallState == Foc_model_Matlab_ultimo_B.Merge_m) {
        /* Outputs for IfAction SubSystem: '<S197>/If Action Subsystem6' incorporates:
         *  ActionPort: '<S204>/Action Port'
         */
        /* Merge: '<S197>/Merge2' incorporates:
         *  Constant: '<S204>/Constant'
         *  SignalConversion generated from: '<S204>/direction'
         */
        Foc_model_Matlab_ultimo_B.Merge2 = 1;

        /* Merge: '<S197>/Merge3' incorporates:
         *  Constant: '<S204>/Constant1'
         *  SignalConversion generated from: '<S204>/sequence_check'
         */
        HallValididyInvalid = 0U;

        /* End of Outputs for SubSystem: '<S197>/If Action Subsystem6' */
      } else if ((uint16_T)GlobalHallState == Foc_model_Matlab_ultimo_B.Merge1)
      {
        /* Outputs for IfAction SubSystem: '<S197>/If Action Subsystem7' incorporates:
         *  ActionPort: '<S205>/Action Port'
         */
        /* Merge: '<S197>/Merge2' incorporates:
         *  Constant: '<S205>/Constant'
         *  SignalConversion generated from: '<S205>/direction'
         */
        Foc_model_Matlab_ultimo_B.Merge2 = -1;

        /* Merge: '<S197>/Merge3' incorporates:
         *  Constant: '<S205>/Constant1'
         *  SignalConversion generated from: '<S205>/sequence_check'
         */
        HallValididyInvalid = 0U;

        /* End of Outputs for SubSystem: '<S197>/If Action Subsystem7' */
      } else {
        /* Outputs for IfAction SubSystem: '<S197>/If Action Subsystem8' incorporates:
         *  ActionPort: '<S206>/Action Port'
         */
        /* Merge: '<S197>/Merge3' incorporates:
         *  Constant: '<S206>/Constant'
         *  SignalConversion generated from: '<S206>/sequence_check'
         */
        HallValididyInvalid = 1U;

        /* End of Outputs for SubSystem: '<S197>/If Action Subsystem8' */
      }

      /* End of If: '<S197>/If' */

      /* SignalConversion: '<S197>/Signal Conversion1' */
      rtb_Merge1 = Foc_model_Matlab_ultimo_B.Merge2;

      /* Switch: '<S197>/Switch' incorporates:
       *  Constant: '<S197>/Constant'
       *  DataStoreRead: '<S2>/Data Store Read1'
       *  RelationalOperator: '<S197>/Relational Operator'
       */
      if (HallValididyInvalid != 0) {
        rtb_Merge3 = false;
      } else {
        rtb_Merge3 = (Foc_model_Matlab_ultimo_B.Merge2 == GlobalDirection);
      }

      /* End of Switch: '<S197>/Switch' */
      /* End of Outputs for SubSystem: '<S195>/Valid Halls' */
      break;

     default:
      /* Outputs for IfAction SubSystem: '<S195>/Bad hall (glitch or wrong connection)' incorporates:
       *  ActionPort: '<S196>/Action Port'
       */
      /* Merge: '<S195>/Merge' incorporates:
       *  Constant: '<S196>/Constant'
       *  SignalConversion generated from: '<S196>/inValidHall'
       */
      HallValididyInvalid = 1U;

      /* SignalConversion: '<S196>/Signal Conversion' incorporates:
       *  DataStoreRead: '<S2>/Data Store Read1'
       */
      rtb_Merge1 = GlobalDirection;

      /* SignalConversion generated from: '<S196>/directional_speed_valid_flag' incorporates:
       *  Constant: '<S196>/Constant1'
       */
      rtb_Merge3 = false;

      /* End of Outputs for SubSystem: '<S195>/Bad hall (glitch or wrong connection)' */
      break;
    }

    /* End of SwitchCase: '<S190>/Detects if the halls reading is valid' */

    /* DataTypeConversion: '<S195>/Data Type Conversion' incorporates:
     *  DataStoreWrite: '<S2>/Data Store Write'
     */
    GlobalSpeedValidity = (uint16_T)rtb_Merge3;

    /* DataStoreWrite: '<S2>/Data Store Write1' */
    GlobalDirection = rtb_Merge1;

    /* DataStoreWrite: '<S2>/Data Store Write2' incorporates:
     *  Constant: '<S190>/Constant'
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
