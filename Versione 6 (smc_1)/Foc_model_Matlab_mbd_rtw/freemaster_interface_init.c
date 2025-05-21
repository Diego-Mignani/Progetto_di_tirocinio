/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: freemaster_interface_init.c
 *
 * Code generated for Simulink model 'Foc_model_Matlab'.
 *
 * Model version                   : 10.52
 * Simulink Coder version          : 24.2 (R2024b) 21-Jun-2024
 * MBDT for S32K1xx Series Version : 4.2.0 (R2016a-R2020a) 20-Jul-2020
 * C/C++ source code generated on  : Mon May 12 12:33:29 2025
 *
 * Target selection: mbd_s32k.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "freemaster.h"
#include "freemaster_interface_init.h"
#include "device_registers.h"
#include "interrupt_manager.h"
#include "lpuart_driver.h"
#include "lpuart_hw_access.h"
#include "pcc_hw_access.h"
#include "pins_port_hw_access.h"
#include "clock_manager.h"

/* FreeMaster UART init function */
void freemaster_interface_init(void)
{
  lpuart_state_t lpuartState;
  lpuart_user_config_t lpuartConfig;

  /* RX pin settings */
  PCC_SetClockMode(PCC, PORTC_CLK, true);
  PINS_SetMuxModeSel(PORTC, 6, PORT_MUX_ALT2);

  /* TX pin settings */
  PCC_SetClockMode(PCC, PORTC_CLK, true);
  PINS_SetMuxModeSel(PORTC, 7, PORT_MUX_ALT2);

  /* Set LPUART clock source */
  PCC_SetPeripheralClockControl(PCC, LPUART1_CLK, true, CLK_SRC_FIRC_DIV2, 0, 0);
  lpuartConfig.baudRate = 57600;
  lpuartConfig.bitCountPerChar = LPUART_8_BITS_PER_CHAR;
  lpuartConfig.parityMode = LPUART_PARITY_DISABLED;
  lpuartConfig.stopBitCount = LPUART_ONE_STOP_BIT;
  LPUART_DRV_Init(1, &lpuartState, &lpuartConfig);

  /* Enable the LPUART transmitter and receiver */
  LPUART_SetTransmitterCmd(LPUART1, true);
  LPUART_SetReceiverCmd(LPUART1, true);
}

void freemaster_interface_isr_init(void)
{
  /* Set priority for LPUART1 RxTx interrupt */
  INT_SYS_SetPriority (LPUART1_RxTx_IRQn, 14);

  /* Register interrupt handler*/
  INT_SYS_InstallHandler(LPUART1_RxTx_IRQn, FMSTR_Isr, (isr_t *)0);

  /* Enable LPUART1 RxTx interrupt.*/
  INT_SYS_EnableIRQ(LPUART1_RxTx_IRQn);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
