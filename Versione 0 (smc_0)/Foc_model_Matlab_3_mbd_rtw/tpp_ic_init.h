#ifndef TPP_IC_INIT_H_
#define TPP_IC_INIT_H_

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "common_aml.h"
#include "gpio_aml.h"
#include "tpp.h"
#include "ftm_common.h"
#include "ftm_pwm_driver.h"
#include "pins_driver.h"
#include "gpio_irq.h"

/*******************************************************************************
 * Constants and macros
 *******************************************************************************/
/* Frequency of SPI communication with device in Hz. */
#define LPSPI_FREQ                     (1000000)

/* Device interrupt masks set by MASK0 and MASK1 commands. */
#define INIT_INTERRUPTS0               (13U)
#define INIT_INTERRUPTS1               (14U)

/* Device configuration set by Mode command. */
#define INIT_MODE                      (8U)

/* Dead time of device in nanoseconds. */
#define INIT_DEADTIME                  (500U)

typedef union {
  uint16_t R;
  struct {
    uint16_t tppIntFlag : 1;
    uint16_t tppClearErr : 1;
    uint16_t tppError : 1;
    uint16_t tppInitDone : 1;
    uint16_t Reserved : 12;
  } B;
} tppStatus_t;

/* TPP status variables */
tppStatus_t tppStatus;

/* TPP configuration structure */
tpp_drv_config_t tppDrvConfig;

/*******************************************************************************
 * Global function prototypes
 *******************************************************************************/
void TPP_tppDrvConfig(void);
void TPP_IC_Init(void);
void TPP_init_pins(void);
void TPP_reconfig_pwm_pins(void);
void tpp_interrupt_enable(uint8_t priority);
void tpp_interrupt_disable(void);
void toggle_restoreTppPwmPins (void);

#endif
