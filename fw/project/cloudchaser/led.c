#include "led.h"
#include "lp5562.h"

#if BOARD_REVISION == 2

    static lp5562_t lp5562[2] = {NULL, NULL};

    static struct {
        u8 r, g, b, w;

    } lp5562_state[2] = {{0,0,0},{0,0,0}};

    static struct __packed {
        bool on : 1;

    } gpio_state[2] = {{true}, {true}};

#elif BOARD_REVISION == 1

    static struct __packed {
        bool on : 1;

    } gpio_state[4] = {0,0,0,0};

#endif


void led_init(void)
{
    #if BOARD_REVISION == 2

        lp5562[0] = lp5562_init(LP5562_ADDR);
        lp5562[1] = lp5562_init(LP5562_ADDR + 2);

    #endif
}


u8 led_get(led_t led)
{
    switch (led) {

        #if BOARD_REVISION == 1

        case LED_GREEN_0: return gpio_state[0].on ? LED_ON : LED_OFF;
        case LED_GREEN_1: return gpio_state[1].on ? LED_ON : LED_OFF;
        case LED_GREEN_2: return gpio_state[2].on ? LED_ON : LED_OFF;
        case LED_GREEN_3: return gpio_state[3].on ? LED_ON : LED_OFF;

        #elif BOARD_REVISION == 2

        case LED_BLUE_0:        return gpio_state[0].on ? LED_ON : LED_OFF;
        case LED_BLUE_1:        return gpio_state[1].on ? LED_ON : LED_OFF;
        case LED_WHITE_0:       return lp5562_state[0].w;
        case LED_WHITE_1:       return lp5562_state[1].w;
        case LED_RGB0_BLUE:     return lp5562_state[0].b;
        case LED_RGB0_GREEN:    return lp5562_state[0].g;
        case LED_RGB0_RED:      return lp5562_state[0].r;
        case LED_RGB1_BLUE:     return lp5562_state[1].b;
        case LED_RGB1_GREEN:    return lp5562_state[1].g;
        case LED_RGB1_RED:      return lp5562_state[1].r;

        #endif

        default: return 0;
    }
}


void led_set(led_t led, u8 value)
{
    switch (led) {
        default:
            value ? led_on(led) : led_off(led);
            break;

        #if BOARD_REVISION == 2
        case LED_WHITE_0:
            if (lp5562_state[0].w != value) {
                lp5562_state[0].w = value;
                lp5562_set_pwm(lp5562[0], LP5562_PWM_WHITE, value);
            }
            break;

        case LED_WHITE_1:
            if (lp5562_state[1].w != value) {
                lp5562_state[1].w = value;
                lp5562_set_pwm(lp5562[1], LP5562_PWM_WHITE, value);
            }
            break;

        case LED_RGB0_BLUE:
            if (lp5562_state[0].b != value) {
                lp5562_state[0].b = value;
                lp5562_set_pwm(lp5562[0], LP5562_PWM_BLUE, value);
            }
            break;

        case LED_RGB0_GREEN:
            if (lp5562_state[0].g != value) {
                lp5562_state[0].g = value;
                lp5562_set_pwm(lp5562[0], LP5562_PWM_GREEN, value);
            }
            break;
        
        case LED_RGB0_RED:
            if (lp5562_state[0].r != value) {
                lp5562_state[0].r = value;
                lp5562_set_pwm(lp5562[0], LP5562_PWM_RED, value);
            }
            break;

        case LED_RGB1_BLUE:
            if (lp5562_state[1].b != value) {
                lp5562_state[1].b = value;
                lp5562_set_pwm(lp5562[1], LP5562_PWM_BLUE, value);
            }
            break;

        case LED_RGB1_GREEN:
            if (lp5562_state[1].g != value) {
                lp5562_state[1].g = value;
                lp5562_set_pwm(lp5562[1], LP5562_PWM_GREEN, value);
            }
            break;

        case LED_RGB1_RED:
            if (lp5562_state[1].r != value) {
                lp5562_state[1].r = value;
                lp5562_set_pwm(lp5562[1], LP5562_PWM_RED, value);
            }
            break;
        #endif
    }
}


void led_on(led_t led)
{
    switch (led) {
        
        #if BOARD_REVISION == 1
        
        case LED_GREEN_0:
            if (!gpio_state[0].on) {
                gpio_state[0].on = true;
                GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_0);
            }
            break;
        
        case LED_GREEN_1:
            if (!gpio_state[1].on) {
                gpio_state[1].on = true;
                GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_1);
            }
            break;
            
        case LED_GREEN_2:
            if (!gpio_state[2].on) {
                gpio_state[2].on = true;
                GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_2);
            }
            break;
            
        case LED_GREEN_3:
            if (!gpio_state[3].on) {
                gpio_state[3].on = true;
                GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_3);
            }
            break;
            
        #elif BOARD_REVISION == 2
            
        case LED_BLUE_0:
            if (!gpio_state[0].on) {
                gpio_state[0].on = true;
                GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_0);
            }
            break;
        
        case LED_BLUE_1:
            if (!gpio_state[1].on) {
                gpio_state[1].on = true;
                GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_1);
            }
            break;
        
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
            
        case LED_GREEN_0:
            if (gpio_state[0].on) {
                gpio_state[0].on = false;
                GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_0);
            }
            break;
        
        case LED_GREEN_1:
            if (gpio_state[1].on) {
                gpio_state[1].on = false;
                GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_1);
            }
            break;
            
        case LED_GREEN_2:
            if (gpio_state[2].on) {
                gpio_state[2].on = false;
                GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_2);
            }
            break;
            
        case LED_GREEN_3:
            if (gpio_state[3].on) {
                gpio_state[3].on = false;
                GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_3);
            }
            break;
            
        #elif BOARD_REVISION == 2

        case LED_BLUE_0:
            if (gpio_state[0].on) {
                gpio_state[0].on = false;
                GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_0);
            }
            break;
        
        case LED_BLUE_1:
            if (gpio_state[1].on) {
                gpio_state[1].on = false;
                GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GPIO_1);
            }
            break;
        
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
