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

    /* S-Function (gpio_s32k_input): '<S223>/Digital_Input_HALL_A' */

    /* GPIPORTA1 signal update */
    mcb_pmsm_foc_hall_S32K144EVB_B.Digital_Input_HALL_A_b = (PINS_DRV_ReadPins
      (PTA) >> 1) & 0x01;

    /* Outputs for Atomic SubSystem: '<S223>/Bit Shift' */
    /* MATLAB Function: '<S225>/bit_shift' incorporates:
     *  DataTypeConversion: '<S223>/Data Type Conversion3'
     */
    /* MATLAB Function 'Logic and Bit Operations/Bit Shift/bit_shift': '<S227>:1' */
    /* '<S227>:1:6' */
    HALL_A = (uint32_T)mcb_pmsm_foc_hall_S32K144EVB_B.Digital_Input_HALL_A_b <<
      2;

    /* End of Outputs for SubSystem: '<S223>/Bit Shift' */

    /* S-Function (gpio_s32k_input): '<S223>/Digital_Input_HALL_B' */

    /* GPIPORTD10 signal update */
    mcb_pmsm_foc_hall_S32K144EVB_B.Digital_Input_HALL_B_l = (PINS_DRV_ReadPins
      (PTD) >> 10) & 0x01;

    /* Outputs for Atomic SubSystem: '<S223>/Bit Shift1' */
    /* MATLAB Function: '<S226>/bit_shift' incorporates:
     *  DataTypeConversion: '<S223>/Data Type Conversion2'
     */
    /* MATLAB Function 'Logic and Bit Operations/Bit Shift/bit_shift': '<S228>:1' */
    /* '<S228>:1:6' */
    HALL_B = (uint32_T)mcb_pmsm_foc_hall_S32K144EVB_B.Digital_Input_HALL_B_l <<
      1;

    /* End of Outputs for SubSystem: '<S223>/Bit Shift1' */

    /* S-Function (gpio_s32k_input): '<S223>/Digital_Input_HALL_C' */

    /* GPIPORTD11 signal update */
    mcb_pmsm_foc_hall_S32K144EVB_B.Digital_Input_HALL_C_h = (PINS_DRV_ReadPins
      (PTD) >> 11) & 0x01;

    /* DataTypeConversion: '<S223>/Data Type Conversion6' */
    HALL_C = (uint32_T)mcb_pmsm_foc_hall_S32K144EVB_B.Digital_Input_HALL_C_h;

    /* Sum: '<S223>/Add1' */
    rtb_Add1 = (int32_T)((HALL_A + HALL_B) + HALL_C);

    /* SwitchCase: '<S224>/Detects if the halls reading is valid' incorporates:
     *  DataTypeConversion: '<S224>/Data Type Conversion1'
     */
    switch (rtb_Add1) {
     case 6:
     case 4:
     case 5:
     case 1:
     case 3:
     case 2:
      /* Outputs for IfAction SubSystem: '<S229>/Valid Halls' incorporates:
       *  ActionPort: '<S231>/Action Port'
       */
      /* SwitchCase: '<S231>/Switch Case' */
      switch (rtb_Add1) {
       case 6:
        /* Outputs for IfAction SubSystem: '<S231>/If Action Subsystem' incorporates:
         *  ActionPort: '<S232>/Action Port'
         */
        /* Merge: '<S231>/Merge' incorporates:
         *  Constant: '<S232>/previous'
         *  SignalConversion generated from: '<S232>/Out1'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge_m = 2U;

        /* Merge: '<S231>/Merge1' incorporates:
         *  Constant: '<S232>/next'
         *  SignalConversion generated from: '<S232>/Out2'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge1 = 4U;

        /* End of Outputs for SubSystem: '<S231>/If Action Subsystem' */
        break;

       case 4:
        /* Outputs for IfAction SubSystem: '<S231>/If Action Subsystem1' incorporates:
         *  ActionPort: '<S233>/Action Port'
         */
        /* Merge: '<S231>/Merge' incorporates:
         *  Constant: '<S233>/previous'
         *  SignalConversion generated from: '<S233>/Out1'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge_m = 6U;

        /* Merge: '<S231>/Merge1' incorporates:
         *  Constant: '<S233>/next'
         *  SignalConversion generated from: '<S233>/Out2'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge1 = 5U;

        /* End of Outputs for SubSystem: '<S231>/If Action Subsystem1' */
        break;

       case 5:
        /* Outputs for IfAction SubSystem: '<S231>/If Action Subsystem2' incorporates:
         *  ActionPort: '<S234>/Action Port'
         */
        /* Merge: '<S231>/Merge' incorporates:
         *  Constant: '<S234>/previous'
         *  SignalConversion generated from: '<S234>/Out1'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge_m = 4U;

        /* Merge: '<S231>/Merge1' incorporates:
         *  Constant: '<S234>/next'
         *  SignalConversion generated from: '<S234>/Out2'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge1 = 1U;

        /* End of Outputs for SubSystem: '<S231>/If Action Subsystem2' */
        break;

       case 1:
        /* Outputs for IfAction SubSystem: '<S231>/If Action Subsystem3' incorporates:
         *  ActionPort: '<S235>/Action Port'
         */
        /* Merge: '<S231>/Merge' incorporates:
         *  Constant: '<S235>/previous'
         *  SignalConversion generated from: '<S235>/Out1'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge_m = 5U;

        /* Merge: '<S231>/Merge1' incorporates:
         *  Constant: '<S235>/next'
         *  SignalConversion generated from: '<S235>/Out2'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge1 = 3U;

        /* End of Outputs for SubSystem: '<S231>/If Action Subsystem3' */
        break;

       case 3:
        /* Outputs for IfAction SubSystem: '<S231>/If Action Subsystem4' incorporates:
         *  ActionPort: '<S236>/Action Port'
         */
        /* Merge: '<S231>/Merge' incorporates:
         *  Constant: '<S236>/previous'
         *  SignalConversion generated from: '<S236>/Out1'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge_m = 1U;

        /* Merge: '<S231>/Merge1' incorporates:
         *  Constant: '<S236>/next'
         *  SignalConversion generated from: '<S236>/Out2'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge1 = 2U;

        /* End of Outputs for SubSystem: '<S231>/If Action Subsystem4' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S231>/If Action Subsystem5' incorporates:
         *  ActionPort: '<S237>/Action Port'
         */
        /* Merge: '<S231>/Merge' incorporates:
         *  Constant: '<S237>/previous'
         *  SignalConversion generated from: '<S237>/Out1'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge_m = 3U;

        /* Merge: '<S231>/Merge1' incorporates:
         *  Constant: '<S237>/next'
         *  SignalConversion generated from: '<S237>/Out2'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge1 = 6U;

        /* End of Outputs for SubSystem: '<S231>/If Action Subsystem5' */
        break;

       default:
        /* no actions */
        break;
      }

      /* End of SwitchCase: '<S231>/Switch Case' */

      /* If: '<S231>/If' incorporates:
       *  DataStoreRead: '<S2>/Data Store Read'
       *  DataTypeConversion: '<S224>/Data Type Conversion2'
       */
      if ((uint16_T)GlobalHallState == mcb_pmsm_foc_hall_S32K144EVB_B.Merge_m) {
        /* Outputs for IfAction SubSystem: '<S231>/If Action Subsystem6' incorporates:
         *  ActionPort: '<S238>/Action Port'
         */
        /* Merge: '<S231>/Merge2' incorporates:
         *  Constant: '<S238>/Constant'
         *  SignalConversion generated from: '<S238>/direction'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge2 = 1;

        /* Merge: '<S231>/Merge3' incorporates:
         *  Constant: '<S238>/Constant1'
         *  SignalConversion generated from: '<S238>/sequence_check'
         */
        HallValididyInvalid = 0U;

        /* End of Outputs for SubSystem: '<S231>/If Action Subsystem6' */
      } else if ((uint16_T)GlobalHallState ==
                 mcb_pmsm_foc_hall_S32K144EVB_B.Merge1) {
        /* Outputs for IfAction SubSystem: '<S231>/If Action Subsystem7' incorporates:
         *  ActionPort: '<S239>/Action Port'
         */
        /* Merge: '<S231>/Merge2' incorporates:
         *  Constant: '<S239>/Constant'
         *  SignalConversion generated from: '<S239>/direction'
         */
        mcb_pmsm_foc_hall_S32K144EVB_B.Merge2 = -1;

        /* Merge: '<S231>/Merge3' incorporates:
         *  Constant: '<S239>/Constant1'
         *  SignalConversion generated from: '<S239>/sequence_check'
         */
        HallValididyInvalid = 0U;

        /* End of Outputs for SubSystem: '<S231>/If Action Subsystem7' */
      } else {
        /* Outputs for IfAction SubSystem: '<S231>/If Action Subsystem8' incorporates:
         *  ActionPort: '<S240>/Action Port'
         */
        /* Merge: '<S231>/Merge3' incorporates:
         *  Constant: '<S240>/Constant'
         *  SignalConversion generated from: '<S240>/sequence_check'
         */
        HallValididyInvalid = 1U;

        /* End of Outputs for SubSystem: '<S231>/If Action Subsystem8' */
      }

      /* End of If: '<S231>/If' */

      /* SignalConversion: '<S231>/Signal Conversion1' */
      rtb_Merge1 = mcb_pmsm_foc_hall_S32K144EVB_B.Merge2;

      /* Switch: '<S231>/Switch' incorporates:
       *  Constant: '<S231>/Constant'
       *  DataStoreRead: '<S2>/Data Store Read1'
       *  RelationalOperator: '<S231>/Relational Operator'
       */
      if (HallValididyInvalid != 0) {
        rtb_Merge3 = false;
      } else {
        rtb_Merge3 = (mcb_pmsm_foc_hall_S32K144EVB_B.Merge2 == GlobalDirection);
      }

      /* End of Switch: '<S231>/Switch' */
      /* End of Outputs for SubSystem: '<S229>/Valid Halls' */
      break;

     default:
      /* Outputs for IfAction SubSystem: '<S229>/Bad hall (glitch or wrong connection)' incorporates:
       *  ActionPort: '<S230>/Action Port'
       */
      /* Merge: '<S229>/Merge' incorporates:
       *  Constant: '<S230>/Constant'
       *  SignalConversion generated from: '<S230>/inValidHall'
       */
      HallValididyInvalid = 1U;

      /* SignalConversion: '<S230>/Signal Conversion' incorporates:
       *  DataStoreRead: '<S2>/Data Store Read1'
       */
      rtb_Merge1 = GlobalDirection;

      /* SignalConversion generated from: '<S230>/directional_speed_valid_flag' incorporates:
       *  Constant: '<S230>/Constant1'
       */
      rtb_Merge3 = false;

      /* End of Outputs for SubSystem: '<S229>/Bad hall (glitch or wrong connection)' */
      break;
    }

    /* End of SwitchCase: '<S224>/Detects if the halls reading is valid' */

    /* DataTypeConversion: '<S229>/Data Type Conversion' incorporates:
     *  DataStoreWrite: '<S2>/Data Store Write'
     */
    GlobalSpeedValidity = (uint16_T)rtb_Merge3;

    /* DataStoreWrite: '<S2>/Data Store Write1' */
    GlobalDirection = rtb_Merge1;

    /* DataStoreWrite: '<S2>/Data Store Write2' incorporates:
     *  Constant: '<S224>/Constant'
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
