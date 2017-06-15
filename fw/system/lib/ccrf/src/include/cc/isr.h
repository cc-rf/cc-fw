#pragma once

#include <cc/common.h>


enum __packed isr_pin {
    ISR_PIN_BASE = 0,
    ISR_PIN_GPIO3 = ISR_PIN_BASE,
    ISR_PIN_GPIO2,
    ISR_PIN_GPIO0,
    ISR_PIN_COUNT,
    ISR_PIN_ANY = 0xFE,
    ISR_PIN_NONE = 0xFF
};

enum __packed isr_edge {
    ISR_EDGE_SETUP,
    ISR_EDGE_FALLING,
    ISR_EDGE_RISING,
    ISR_EDGE_BOTH,
    ISR_EDGE_HIGH,
    ISR_EDGE_LOW
};

typedef void *isr_handle_t;

isr_handle_t cc_isr_setup(cc_dev_t dev, enum isr_pin *pin, enum isr_edge edge);
void cc_isr_wait(isr_handle_t isr_handle);
