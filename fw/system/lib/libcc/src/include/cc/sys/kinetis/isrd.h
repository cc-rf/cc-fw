#pragma once

#include <cc/common.h>
#include <fsl_port.h>

#define ISRD_PORTC
#define ISRD_PORTC_MAX_PIN      12
//#define ISRD_PORTD
//#define ISRD_PORTD_MAX_PIN      7
// For use with SPI1/external radio
//#define ISRD_PORTE
//#define ISRD_PORTE_MAX_PIN      12

typedef void (* isr_t)(void);

void isrd_configure(const u8 port, const u8 pin, const port_interrupt_t type, const isr_t isr, const s8 prio_decrease);
bool isrd_state(const u8 port, const u8 pin);
