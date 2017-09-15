#pragma once

#include "clock_config.h"
#include "fsl_gpio.h"

#if BOARD_CLOUDCHASER != 1
#error BOARD_CLOUDCHASER not set
#endif

#define BOARD_LED_PORT      PORTB
#define BOARD_LED_GPIO      GPIOB
#define BOARD_LED_GPIO_0    0u
#define BOARD_LED_GPIO_1    1u
#if BOARD_REVISION == 1
#define BOARD_LED_GPIO_2    2u
#define BOARD_LED_GPIO_3    3u
#endif

/* @brief FreeRTOS tickless timer configuration. */
#define vPortLptmrIsr LPTMR0_IRQHandler /*!< Timer IRQ handler. */
#define TICKLESS_LPTMR_BASE_PTR LPTMR0  /*!< Tickless timer base address. */
#define TICKLESS_LPTMR_IRQn LPTMR0_IRQn /*!< Tickless timer IRQ number. */
