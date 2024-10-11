#include "tpp_ic_init.h"
#include "pcc_hw_access.h"

extern void GPIPORTE10_callback (void);
void TPP_tppDrvConfig(void)
{
  tppDrvConfig.en1PinIndex = 2;
  tppDrvConfig.en1PinInstance = instanceA;
  tppDrvConfig.en2PinIndex = 2;
  tppDrvConfig.en2PinInstance = instanceA;
  tppDrvConfig.rstPinIndex = 3;
  tppDrvConfig.rstPinInstance = instanceA;
  tppDrvConfig.deviceConfig.deadtime = INIT_DEADTIME;
  tppDrvConfig.deviceConfig.intMask0 = INIT_INTERRUPTS0;
  tppDrvConfig.deviceConfig.intMask1 = INIT_INTERRUPTS1;
  tppDrvConfig.deviceConfig.modeMask = INIT_MODE;
  tppDrvConfig.deviceConfig.statusRegister[0U] = 0U;
  tppDrvConfig.deviceConfig.statusRegister[1U] = 0U;
  tppDrvConfig.deviceConfig.statusRegister[2U] = 0U;
  tppDrvConfig.deviceConfig.statusRegister[3U] = 0U;
  tppDrvConfig.csPinIndex = 5;
  tppDrvConfig.csPinInstance = instanceB;
  tppDrvConfig.spiInstance = 0U;
  tppDrvConfig.spiTppConfig.baudRateHz = LPSPI_FREQ;
}

;
void TPP_init_pins(void)
{
  pin_settings_config_t tpp_pins_InitConfigArr[5] = {
    {
      .base = PORTA,
      .pinPortIdx = 2,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTA,
      .direction = GPIO_OUTPUT_DIRECTION,
      .digitalFilter = false,
      .initValue = 0u,
    },

    {
      .base = PORTA,
      .pinPortIdx = 2,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTA,
      .direction = GPIO_OUTPUT_DIRECTION,
      .digitalFilter = false,
      .initValue = 0u,
    },

    {
      .base = PORTA,
      .pinPortIdx = 3,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTA,
      .direction = GPIO_OUTPUT_DIRECTION,
      .digitalFilter = false,
      .initValue = 0u,
    },

    {
      .base = PORTB,
      .pinPortIdx = 5,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .driveSelect = PORT_LOW_DRIVE_STRENGTH,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTB,
      .direction = GPIO_OUTPUT_DIRECTION,
      .digitalFilter = false,
      .initValue = 1u,
    },

    {
      .base = PORTE,
      .pinPortIdx = 10,
      .pullConfig = PORT_INTERNAL_PULL_NOT_ENABLED,
      .passiveFilter = false,
      .mux = PORT_MUX_AS_GPIO,
      .pinLock = false,
      .intConfig = PORT_DMA_INT_DISABLED,
      .clearIntFlag = true,
      .gpioBase = PTE,
      .direction = GPIO_INPUT_DIRECTION,
    }
  };

  /* Enable clock for PORTA */
  PCC_SetClockMode(PCC, PCC_PORTA_CLOCK, true);
  PCC_SetClockMode(PCC, PCC_PORTA_CLOCK, true);
  PCC_SetClockMode(PCC, PCC_PORTA_CLOCK, true);
  PCC_SetClockMode(PCC, PCC_PORTB_CLOCK, true);
  PCC_SetClockMode(PCC, PCC_PORTE_CLOCK, true);

  /* Initialize TPP pins */
  PINS_DRV_Init(5, tpp_pins_InitConfigArr);
  PINS_SetMuxModeSel (PORTB, 8, PORT_MUX_AS_GPIO);
  PINS_DRV_SetPinDirection(PTB, 8,
    GPIO_OUTPUT_DIRECTION);
  PINS_DRV_WritePin(PTB, 8,
                    1U);
  PINS_SetMuxModeSel (PORTB, 9, PORT_MUX_AS_GPIO);
  PINS_DRV_SetPinDirection(PTB, 9,
    GPIO_OUTPUT_DIRECTION);
  PINS_DRV_WritePin(PTB, 9,
                    0U);
  PINS_SetMuxModeSel (PORTB, 10, PORT_MUX_AS_GPIO);
  PINS_DRV_SetPinDirection(PTB, 10,
    GPIO_OUTPUT_DIRECTION);
  PINS_DRV_WritePin(PTB, 10,
                    1U);
  PINS_SetMuxModeSel (PORTB, 11, PORT_MUX_AS_GPIO);
  PINS_DRV_SetPinDirection(PTB, 11,
    GPIO_OUTPUT_DIRECTION);
  PINS_DRV_WritePin(PTB, 11,
                    0U);
  PINS_SetMuxModeSel (PORTC, 10, PORT_MUX_AS_GPIO);
  PINS_DRV_SetPinDirection(PTC, 10,
    GPIO_OUTPUT_DIRECTION);
  PINS_DRV_WritePin(PTC, 10,
                    1U);
  PINS_SetMuxModeSel (PORTC, 11, PORT_MUX_AS_GPIO);
  PINS_DRV_SetPinDirection(PTC, 11,
    GPIO_OUTPUT_DIRECTION);
  PINS_DRV_WritePin(PTC, 11,
                    0U);
}

;
void TPP_reconfig_pwm_pins(void)
{
  PINS_SetMuxModeSel (PORTB, 8, PORT_MUX_AS_GPIO);
  PINS_DRV_SetPinDirection(PTB, 8,
    GPIO_OUTPUT_DIRECTION);
  PINS_DRV_WritePin(PTB, 8,
                    1U);
  PINS_SetMuxModeSel (PORTB, 9, PORT_MUX_AS_GPIO);
  PINS_DRV_SetPinDirection(PTB, 9,
    GPIO_OUTPUT_DIRECTION);
  PINS_DRV_WritePin(PTB, 9,
                    0U);
  PINS_SetMuxModeSel (PORTB, 10, PORT_MUX_AS_GPIO);
  PINS_DRV_SetPinDirection(PTB, 10,
    GPIO_OUTPUT_DIRECTION);
  PINS_DRV_WritePin(PTB, 10,
                    1U);
  PINS_SetMuxModeSel (PORTB, 11, PORT_MUX_AS_GPIO);
  PINS_DRV_SetPinDirection(PTB, 11,
    GPIO_OUTPUT_DIRECTION);
  PINS_DRV_WritePin(PTB, 11,
                    0U);
  PINS_SetMuxModeSel (PORTC, 10, PORT_MUX_AS_GPIO);
  PINS_DRV_SetPinDirection(PTC, 10,
    GPIO_OUTPUT_DIRECTION);
  PINS_DRV_WritePin(PTC, 10,
                    1U);
  PINS_SetMuxModeSel (PORTC, 11, PORT_MUX_AS_GPIO);
  PINS_DRV_SetPinDirection(PTC, 11,
    GPIO_OUTPUT_DIRECTION);
  PINS_DRV_WritePin(PTC, 11,
                    0U);
}

void TPP_IC_Init(void)
{
  TPP_tppDrvConfig();
  TPP_init_pins();
  TPP_ConfigureGpio(&tppDrvConfig);
  TPP_reconfig_pwm_pins();
  TPP_Init(&tppDrvConfig, tppModeEnable);
  toggle_restoreTppPwmPins();
}

void toggle_restoreTppPwmPins()
{
  /* Turn on LS pwm channels */
  WAIT_AML_WaitUs(TPP_LS_TOGGLE_DELAY);
  PINS_DRV_WritePin(PTB, 8,
                    1u);
  PINS_DRV_WritePin(PTB, 9,
                    1u);
  PINS_DRV_WritePin(PTB, 10,
                    1u);
  PINS_DRV_WritePin(PTB, 11,
                    1u);
  PINS_DRV_WritePin(PTC, 10,
                    1u);
  PINS_DRV_WritePin(PTC, 11,
                    1u);

  /* Turn off LS pwm channels */
  WAIT_AML_WaitUs(TPP_LS_TOGGLE_DELAY);
  PINS_DRV_WritePin(PTB, 8,
                    1U);
  PINS_DRV_WritePin(PTB, 9,
                    0U);
  PINS_DRV_WritePin(PTB, 10,
                    1U);
  PINS_DRV_WritePin(PTB, 11,
                    0U);
  PINS_DRV_WritePin(PTC, 10,
                    1U);
  PINS_DRV_WritePin(PTC, 11,
                    0U);

  /* Turn on HS pwm channels */
  WAIT_AML_WaitUs(TPP_HS_TOGGLE_DELAY);
  PINS_DRV_WritePin(PTB, 8,
                    0u);
  PINS_DRV_WritePin(PTB, 9,
                    0u);
  PINS_DRV_WritePin(PTB, 10,
                    0u);
  PINS_DRV_WritePin(PTB, 11,
                    0u);
  PINS_DRV_WritePin(PTC, 10,
                    0u);
  PINS_DRV_WritePin(PTC, 11,
                    0u);

  /* Turn off HS pwm channels */
  WAIT_AML_WaitUs(TPP_HS_TOGGLE_DELAY);
  PINS_DRV_WritePin(PTB, 8,
                    1U);
  PINS_DRV_WritePin(PTB, 9,
                    0U);
  PINS_DRV_WritePin(PTB, 10,
                    1U);
  PINS_DRV_WritePin(PTB, 11,
                    0U);
  PINS_DRV_WritePin(PTC, 10,
                    1U);
  PINS_DRV_WritePin(PTC, 11,
                    0U);

  /* Turn on HS pwm channels */
  WAIT_AML_WaitUs(TPP_HS_TOGGLE_DELAY);
  PINS_DRV_WritePin(PTB, 8, 0u);
  PINS_DRV_WritePin(PTB, 9, 0u);
  PINS_DRV_WritePin(PTB, 10, 0u);
  PINS_DRV_WritePin(PTB, 11, 0u);
  PINS_DRV_WritePin(PTC, 10, 0u);
  PINS_DRV_WritePin(PTC, 11, 0u);
  PINS_SetMuxModeSel (PORTB, 8, PORT_MUX_ALT2);
  PINS_SetMuxModeSel (PORTB, 9, PORT_MUX_ALT2);
  PINS_SetMuxModeSel (PORTB, 10, PORT_MUX_ALT2);
  PINS_SetMuxModeSel (PORTB, 11, PORT_MUX_ALT2);
  PINS_SetMuxModeSel (PORTC, 10, PORT_MUX_ALT2);
  PINS_SetMuxModeSel (PORTC, 11, PORT_MUX_ALT2);
}

void tpp_interrupt_enable(uint8_t priority)
{
  /* Set GPIPORTE10 interrupt configuration. */
  PINS_DRV_SetPinIntSel(PORTE, 10, PORT_INT_RISING_EDGE);

  /* Set priority for GPIOPORTE ISR. */
  INT_SYS_SetPriority (PORTE_IRQn, priority);

  /* Enable GPIPORTE interrupt. */
  INT_SYS_EnableIRQ(PORTE_IRQn);

  /* Register GPIPORTE10 callback function.*/
  GPI_DRV_InstallCallback(4, 10, &GPIPORTE10_callback);
}

void tpp_interrupt_disable(void)
{
  /* Clear GPIPORTE10 interrupt configuration. */
  PINS_DRV_SetPinIntSel(PORTE, 10, PORT_DMA_INT_DISABLED);
}
