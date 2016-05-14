#pragma once

#include <usr/type.h>
#include <itm.h>
#include <semi.h>
#include <uart.h>

#define TRACE_ITM_PORT 31

static inline void trace_init(void)
{
    itm_init();
    semi_init();
    uart_init();
}

static inline void trace_write(const u8 *const buf, const size_t len)
{
    if (itm_available() && itm_port_available(TRACE_ITM_PORT))
        itm_write(TRACE_ITM_PORT, buf, len);
    if (semi_available())
        semi_write(buf, len);
    uart_write(buf, len);
}

static inline void trace_puts(const char *const str)
{
    if (itm_available() && itm_port_available(TRACE_ITM_PORT))
    itm_puts(TRACE_ITM_PORT, str);
    if (semi_available())
        semi_puts(str);
    uart_puts(str);
}
