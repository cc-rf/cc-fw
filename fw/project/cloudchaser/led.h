#pragma once

#include <usr/type.h>
#include <board.h>

#include <limits.h>

#if !defined(BOARD_CLOUDCHASER)
#error This code is only for the amazing Cloud Chaser board!
#endif


#define LED_ON      ((u8)UCHAR_MAX)
#define LED_OFF     ((u8)0)


typedef enum __packed {
    //LED_ALL = -1,
    LED_0,
    LED_1,
    LED_2,
    LED_3,

    #if BOARD_REVISION == 1
        LED_GREEN_0 = LED_0,
        LED_GREEN_1 = LED_1,
        LED_GREEN_2 = LED_2,
        LED_GREEN_3 = LED_3,
    #elif BOARD_REVISION == 2
        LED_BLUE_0 = LED_0,
        LED_BLUE_1 = LED_1,
        LED_WHITE_0 = LED_2,
        LED_WHITE_1 = LED_3,
        LED_RGB0_BLUE,
        LED_RGB0_GREEN,
        LED_RGB0_RED,
        LED_RGB1_BLUE,
        LED_RGB1_GREEN,
        LED_RGB1_RED,
    #endif

} led_t;



void led_init(void);

void led_set(led_t led, u8 value);
void led_on(led_t led);
void led_off(led_t led);
void led_toggle(led_t led);
u8 led_get(led_t led);
