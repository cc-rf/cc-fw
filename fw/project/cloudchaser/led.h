#pragma once

#include <usr/type.h>
#include <board.h>

#include <limits.h>
#include <FreeRTOS.h>
#include <task.h>

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

typedef struct __packed {
    u8 r;
    u8 g;
    u8 b;
    u8 w;

} led_rgb_t;


void led_init(void);

void led_set(led_t led, u8 value);
u8 led_get(led_t led);
void led_on(led_t led);
void led_off(led_t led);


static inline void led_toggle(led_t led)
{
    led_get(led) ? led_off(led) : led_on(led);
}


static u8 led_interp(u8 begin, u8 end, u16 resolution, u16 value)
{
    if (begin > end) return led_interp(end, begin, resolution, resolution - value);

    float r = 1.0f / resolution;
    r *= value * (end - begin) + (resolution >> 1);

    return begin + (u8)r;
}


#if BOARD_REVISION == 2

static inline void led_rgb_set(u8 chan, u8 r, u8 g, u8 b, u8 w)
{
    led_set(LED_WHITE_0 + chan, w);

    chan *= 3;

    led_set(LED_RGB0_RED + chan, r);
    led_set(LED_RGB0_GREEN + chan, g);
    led_set(LED_RGB0_BLUE + chan, b);
}

static inline void led_rgbw_set(u8 chan, led_rgb_t rgb)
{
    led_rgb_set(chan, rgb.r, rgb.g, rgb.b, rgb.w);
}


static inline void led_rgb_interp(u8 chan, u16 resolution, u16 value, led_rgb_t rgb0, led_rgb_t rgb1)
{
    led_rgb_set(
            chan,
            led_interp(rgb0.r, rgb1.r, resolution, value),
            led_interp(rgb0.g, rgb1.g, resolution, value),
            led_interp(rgb0.b, rgb1.b, resolution, value),
            led_interp(rgb0.w, rgb1.w, resolution, value)
    );
}

static inline void led_run_program(u16 resolution, TickType_t delay, const led_rgb_t table[][2], u16 size)
{
    for (u16 pc = 0; pc < (size - 1); ++pc) {
        for (u16 i = 0; i <= resolution; ++i) {
            led_rgb_interp(0, resolution, i, table[pc][0], table[pc+1][0]);
            led_rgb_interp(1, resolution, i, table[pc][1], table[pc+1][1]);
            vTaskDelay(delay);
        }
    }
}

#endif
