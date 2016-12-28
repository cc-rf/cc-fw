#pragma once

#include <cc/common.h>
#include <fsl_port.h>

#define ISRD_PORTC
#define ISRD_PORTC_MAX_PIN      10
//#define ISRD_PORTD
//#define ISRD_PORTD_MAX_PIN      7


typedef void (* isr_t)(void);

void isrd_configure(const u8 port, const u8 pin, const port_interrupt_t type, const isr_t isr);
