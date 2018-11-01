#pragma once


#define BOARD_XTAL0_CLK_HZ      24000000U
#define BOARD_XTAL32K_CLK_HZ    32768U

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

/* USB PHY condfiguration */
#define BOARD_USB_PHY_D_CAL     (0x0CU)
#define BOARD_USB_PHY_TXCAL45DP (0x06U)
#define BOARD_USB_PHY_TXCAL45DM (0x06U)


void board_boot(void);
void board_rtos_init(void);
