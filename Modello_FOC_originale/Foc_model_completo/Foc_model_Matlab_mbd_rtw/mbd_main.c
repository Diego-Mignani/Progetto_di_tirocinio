/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: mbd_main.c
 *
 * Code generated for Simulink model 'Foc_model_Matlab'.
 *
 * Model version                   : 10.26
 * Simulink Coder version          : 24.2 (R2024b) 21-Jun-2024
 * MBDT for S32K1xx Series Version : 4.2.0 (R2016a-R2020a) 20-Jul-2020
 * C/C++ source code generated on  : Wed Nov 27 16:38:24 2024
 *
 * Target selection: mbd_s32k.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

/* Model's headers */
#include "device_registers.h"
#include "Foc_model_Matlab.h"
#include "freemaster.h"
#include "interrupt_manager.h"
#include "clock_manager.h"
#include "lpit_driver.h"
#include "lpit_hw_access.h"
#include "pcc_hw_access.h"
#include "s32k_clock_init.h"

extern void TPP_IC_Init();
void TPP_InitializeOutputs(void)
{
}

void SYSTEM_INIT_TASK(void)
{
  /* Initialize model */
  Foc_model_Matlab_initialize();
}

void SYSTEM_TASK(void)
{
  static bool tppInit = true;
  if (tppInit) {
    TPP_IC_Init();
    tppInit = false;
  }

  boolean_T eventFlags[4];             /* Model has 4 rates */
  int_T i;

  /*
   * For a bare-board target (i.e., no operating system), the rates
   * that execute this base step are buffered locally to allow for
   * overlapping preemption.
   */
  Foc_model_Matlab_SetEventsForThisBaseStep(eventFlags);

  /* Set model inputs associated with base rate here */
  Foc_model_Matlab_step(0);

  /* Get model outputs here */
  for (i = 1; i < 4; i++) {
    if (eventFlags[i]) {
      Foc_model_Matlab_step(i);

      /* Get model outputs here */
    }
  }
}

void LPIT0_Ch0_IRQHandler (void)
{
  SYSTEM_TASK();
  LPIT_DRV_ClearInterruptFlagTimerChannels(0, (1 << 0));
}

void lpit0_init()
{
}

void lpit0_init_step_timer()
{
  static const lpit_user_channel_config_t lpit0InitStruct = {
    .timerMode = LPIT_PERIODIC_COUNTER,
    .periodUnits = LPIT_PERIOD_UNITS_MICROSECONDS,
    .period = 50,
    .triggerSource = LPIT_TRIGGER_SOURCE_INTERNAL,
    .triggerSelect = 1U,
    .enableReloadOnTrigger = false,
    .enableStopOnInterrupt = false,
    .enableStartOnTrigger = false,
    .chainChannel = false,
    .isInterruptEnabled = true
  };

  /* Initialize PIT timer channel. */
  LPIT_DRV_InitChannel(0, 0, &lpit0InitStruct);

  /* Set priority for LPIT ISR */
  INT_SYS_SetPriority(LPIT0_Ch0_IRQn, 15);
  INT_SYS_InstallHandler(LPIT0_Ch0_IRQn, LPIT0_Ch0_IRQHandler, (isr_t *)0);

  /* Start the timer. */
  LPIT_DRV_StartTimerChannels(0, LPIT0->SETTEN | (1 << 0));
}

void main(void)
{

#ifdef __FPU_USED

  /* FPU ENABLE */
  /* Enable CP10 and CP11 coprocessors */
  S32_SCB->CPACR |= (S32_SCB_CPACR_CP10_MASK | S32_SCB_CPACR_CP11_MASK);

#endif

  /* Disable all interrupts.*/
  INT_SYS_DisableIRQGlobal();

  /* Clock initialization */
  Clock_Setup();

  /* Initialize the processor. */
  SYSTEM_INIT_TASK();

  /* Initialize system timer */
  lpit0_init();

  /* Enable all interrupts.*/
  INT_SYS_EnableIRQGlobal();           /* interrupt_manager.c */

  /* Initialize step timer interrupt. */
  lpit0_init_step_timer();
  while (1) {
    /* FreeMaster Polling */
    FMSTR_Poll();
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
