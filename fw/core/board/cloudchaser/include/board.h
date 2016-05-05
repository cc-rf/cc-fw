#pragma once

#include "clock_config.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief The board name */
#define BOARD_NAME "CLOUDCHASER"


/*! @brief The rtc instance used for rtc_func */
#define BOARD_RTC_FUNC_BASEADDR RTC

/* Board led color mapping */
#define LOGIC_LED_ON    1U
#define LOGIC_LED_OFF   0U

#define BOARD_LED_ABCD_GPIO         GPIOB
#define BOARD_LED_ABCD_PORT         PORTB
#define BOARD_LED_ABCD_PORT_CLOCK   kCLOCK_PortB
#define BOARD_LED_A_GPIO_PIN        0U
#define BOARD_LED_B_GPIO_PIN        1U
#define BOARD_LED_C_GPIO_PIN        2U
#define BOARD_LED_D_GPIO_PIN        3U

#define LED_ABCD_INIT(ABCD, output) \
    GPIO_PinInit(BOARD_LED_ABCD_GPIO, BOARD_LED_##ABCD##_GPIO_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})

#define LED_ABCD_ON(ABCD)           \
    GPIO_SetPinsOutput(BOARD_LED_ABCD_GPIO, 1U << BOARD_LED_##ABCD##_GPIO_PIN)

#define LED_ABCD_OFF(ABCD)          \
    GPIO_ClearPinsOutput(BOARD_LED_ABCD_GPIO, 1U << BOARD_LED_##ABCD##_GPIO_PIN)

#define LED_ABCD_TOGGLE(ABCD)       \
    GPIO_TogglePinsOutput(BOARD_LED_ABCD_GPIO, 1U << BOARD_LED_##ABCD##_GPIO_PIN)

#define LED_A_ON()                  LED_ABCD_ON(A)
#define LED_B_ON()                  LED_ABCD_ON(B)
#define LED_C_ON()                  LED_ABCD_ON(C)
#define LED_D_ON()                  LED_ABCD_ON(D)
#define LED_ABCD_ALL_ON()           LED_A_ON(),LED_B_ON(),LED_C_ON(),LED_D_ON()
#define LED_A_OFF()                 LED_ABCD_OFF(A)
#define LED_B_OFF()                 LED_ABCD_OFF(B)
#define LED_C_OFF()                 LED_ABCD_OFF(C)
#define LED_D_OFF()                 LED_ABCD_OFF(D)
#define LED_ABCD_ALL_OFF()          LED_A_OFF(),LED_B_OFF(),LED_C_OFF(),LED_D_OFF()
#define LED_A_TOGGLE()              LED_ABCD_TOGGLE(A)
#define LED_B_TOGGLE()              LED_ABCD_TOGGLE(B)
#define LED_C_TOGGLE()              LED_ABCD_TOGGLE(C)
#define LED_D_TOGGLE()              LED_ABCD_TOGGLE(D)
#define LED_ABCD_ALL_TOGGLE()       LED_A_TOGGLE(),LED_B_TOGGLE(),LED_C_TOGGLE(),LED_D_TOGGLE()

/* @brief FreeRTOS tickless timer configuration. */
#define vPortLptmrIsr LPTMR0_IRQHandler /*!< Timer IRQ handler. */
#define TICKLESS_LPTMR_BASE_PTR LPTMR0  /*!< Tickless timer base address. */
#define TICKLESS_LPTMR_IRQn LPTMR0_IRQn /*!< Tickless timer IRQ number. */


/*******************************************************************************
 * API
 ******************************************************************************/

void BOARD_InitDebugConsole(void);
