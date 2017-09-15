#include "led.h"
#include "lp5562.h"

#if BOARD_REVISION == 2

    static lp5562_t lp5562[2] = {NULL};

#endif

void led_init(void)
{
    #if BOARD_REVISION == 2

        lp5562[0] = lp5562_init(LP5562_ADDR);
        lp5562[1] = lp5562_init(LP5562_ADDR + 2);

    #endif
}


void led_set(led_t led, u8 value)
{
    switch (led) {
        default:
            value ? led_on(led) : led_off(led);
            break;

        #if BOARD_REVISION == 2
        case LED_WHITE_0:       return lp5562_set_pwm(lp5562[0], LP5562_PWM_WHITE, value);
        case LED_WHITE_1:       return lp5562_set_pwm(lp5562[1], LP5562_PWM_WHITE, value);
        case LED_RGB0_BLUE:     return lp5562_set_pwm(lp5562[0], LP5562_PWM_BLUE, value);
        case LED_RGB0_GREEN:    return lp5562_set_pwm(lp5562[0], LP5562_PWM_GREEN, value);
        case LED_RGB0_RED:      return lp5562_set_pwm(lp5562[0], LP5562_PWM_RED, value);
        case LED_RGB1_BLUE:     return lp5562_set_pwm(lp5562[1], LP5562_PWM_BLUE, value);
        case LED_RGB1_GREEN:    return lp5562_set_pwm(lp5562[1], LP5562_PWM_GREEN, value);
        case LED_RGB1_RED:      return lp5562_set_pwm(lp5562[1], LP5562_PWM_RED, value);
        #endif
    }
}


void led_on(led_t led)
{
    switch (led) {
        #if BOARD_REVISION == 1
        case LED_GREEN_0: return GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_0);
        case LED_GREEN_1: return GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_1);
        case LED_GREEN_2: return GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_2);
        case LED_GREEN_3: return GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_3);
        #elif BOARD_REVISION == 2
        case LED_BLUE_0: return GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_0);
        case LED_BLUE_1: return GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_1);
        case LED_WHITE_0:
        case LED_WHITE_1:
        case LED_RGB0_BLUE:
        case LED_RGB0_GREEN:
        case LED_RGB0_RED:
        case LED_RGB1_BLUE:
        case LED_RGB1_GREEN:
        case LED_RGB1_RED:
            return led_set(led, LED_ON);
        #endif
    }
}


void led_off(led_t led)
{
    switch (led) {
        #if BOARD_REVISION == 1
        case LED_GREEN_0: return GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_0);
        case LED_GREEN_1: return GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_1);
        case LED_GREEN_2: return GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_2);
        case LED_GREEN_3: return GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_3);
        #elif BOARD_REVISION == 2
        case LED_BLUE_0: return GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_0);
        case LED_BLUE_1: return GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_1);
        case LED_WHITE_0:
        case LED_WHITE_1:
        case LED_RGB0_BLUE:
        case LED_RGB0_GREEN:
        case LED_RGB0_RED:
        case LED_RGB1_BLUE:
        case LED_RGB1_GREEN:
        case LED_RGB1_RED:
            return led_set(led, LED_OFF);
        #endif
    }
}


void led_toggle(led_t led)
{
    switch (led) {
        #if BOARD_REVISION == 1
        case LED_GREEN_0: return GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_0);
        case LED_GREEN_1: return GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_1);
        case LED_GREEN_2: return GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_2);
        case LED_GREEN_3: return GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_3);
        #elif BOARD_REVISION == 2
        case LED_BLUE_0:        return GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_0);
        case LED_BLUE_1:        return GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_1);
        case LED_WHITE_0:       return led_set(led, lp5562_get_current(lp5562[0], LP5562_PWM_WHITE) ? LED_OFF : LED_ON);
        case LED_WHITE_1:       return led_set(led, lp5562_get_current(lp5562[1], LP5562_PWM_WHITE) ? LED_OFF : LED_ON);
        case LED_RGB0_BLUE:     return led_set(led, lp5562_get_current(lp5562[0], LP5562_PWM_BLUE) ? LED_OFF : LED_ON);
        case LED_RGB0_GREEN:    return led_set(led, lp5562_get_current(lp5562[0], LP5562_PWM_GREEN) ? LED_OFF : LED_ON);
        case LED_RGB0_RED:      return led_set(led, lp5562_get_current(lp5562[0], LP5562_PWM_RED) ? LED_OFF : LED_ON);
        case LED_RGB1_BLUE:     return led_set(led, lp5562_get_current(lp5562[1], LP5562_PWM_BLUE) ? LED_OFF : LED_ON);
        case LED_RGB1_GREEN:    return led_set(led, lp5562_get_current(lp5562[1], LP5562_PWM_GREEN) ? LED_OFF : LED_ON);
        case LED_RGB1_RED:      return led_set(led, lp5562_get_current(lp5562[1], LP5562_PWM_RED) ? LED_OFF : LED_ON);
        #endif
    }
}

