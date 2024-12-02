/*
 * Copyright (c) 2017-2020, NXP.
 * All rights reserved.
 */

#include "gpio_irq.h"
#include "pins_driver.h"

#define HW_PORTA  0
#define HW_PORTB  1
#define HW_PORTC  2
#define HW_PORTD  3
#define HW_PORTE  4


/******************************************************************************
 * Variables
 *****************************************************************************/

/*
 * @brief Function table to save GPI isr callback function pointers.
 *
 * Call GPI_DRV_InstallCallback to install isr callback functions.
 */
static gpio_isr_callback_t gpiIsrCallbackTable[PORT_INSTANCE_COUNT][32] = { { 0 } };

/*!
 * @addtogroup gpio_irq
 * @{
 */

/******************************************************************************
 * Code
 *****************************************************************************/
/*!
 * @brief gpio IRQ handler with the same name in startup code
 */
void PORTA_IRQHandler(void)
{
	uint8_t i;
    uint32_t porta_isr_flag = PINS_DRV_GetPortIntFlag(PORTA);
    for (i = 0; i < 32; ++i)
    {
        if ((porta_isr_flag & (1 << i)) != 0)
        {
            (*gpiIsrCallbackTable[HW_PORTA][i])();
        }
    }
    /* Clear interrupt flag.*/
    PINS_DRV_ClearPortIntFlagCmd(PORTA);
}

/*!
 * @brief gpio IRQ handler with the same name in startup code
 */
void PORTB_IRQHandler(void)
{
	uint8_t i;
    uint32_t portb_isr_flag = PINS_DRV_GetPortIntFlag(PORTB);
    for (i = 0; i < 32; ++i)
    {
        if ((portb_isr_flag & (1 << i)) != 0)
        {
            (*gpiIsrCallbackTable[HW_PORTB][i])();
        }
    }
    /* Clear interrupt flag.*/
    PINS_DRV_ClearPortIntFlagCmd(PORTB);
}

/*!
 * @brief gpio IRQ handler with the same name in startup code
 */
void PORTC_IRQHandler(void)
{
	uint8_t i;
    uint32_t portc_isr_flag = PINS_DRV_GetPortIntFlag(PORTC);
    for (i = 0; i < 32; ++i)
    {
        if ((portc_isr_flag & (1 << i)) != 0)
        {
            (*gpiIsrCallbackTable[HW_PORTC][i])();
        }
    }
    /* Clear interrupt flag.*/
    PINS_DRV_ClearPortIntFlagCmd(PORTC);
}

/*!
 * @brief gpio IRQ handler with the same name in startup code
 */
void PORTD_IRQHandler(void)
{
	uint8_t i;
    uint32_t portd_isr_flag = PINS_DRV_GetPortIntFlag(PORTD);
    for (i = 0; i < 32; ++i)
    {
        if ((portd_isr_flag & (1 << i)) != 0)
        {
            (*gpiIsrCallbackTable[HW_PORTD][i])();
        }
    }
    /* Clear interrupt flag.*/
    PINS_DRV_ClearPortIntFlagCmd(PORTD);
}

/*!
 * @brief gpio IRQ handler with the same name in startup code
 */
void PORTE_IRQHandler(void)
{
	uint8_t i;
    uint32_t porte_isr_flag = PINS_DRV_GetPortIntFlag(PORTE);
    for (i = 0; i < 32; ++i)
    {
        if ((porte_isr_flag & (1 << i)) != 0)
        {
            (*gpiIsrCallbackTable[HW_PORTE][i])();
        }
    }
    /* Clear interrupt flag.*/
    PINS_DRV_ClearPortIntFlagCmd(PORTE);
}

/*! @} */

/*FUNCTION*********************************************************************
*
* Function Name : GPIO_DRV_InstallCallback
* Description   : Install the user-defined callback in GPI.
* When an GPI interrupt request is served, the callback will be executed
* inside the ISR.
*
*END*************************************************************************/
void
GPI_DRV_InstallCallback(uint16_t port,
                        uint16_t pin,
                        gpio_isr_callback_t userCallback)
{

    gpiIsrCallbackTable[port][pin] = userCallback;
}

/******************************************************************************
 * EOF
 *****************************************************************************/
