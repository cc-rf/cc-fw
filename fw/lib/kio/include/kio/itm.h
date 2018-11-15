#pragma once

#include <usr/type.h>

#if !DEBUG && CONFIG_DISABLE_ITM == 1

static inline void itm_init(void) {}
static inline void itm_puts(const uint8_t port, const char *str) {}
static inline void itm_printf(const uint8_t port, const char *format, ...) {}
static inline void itm_write(const uint8_t port, const uint8_t *buf, const size_t len) {}

#else

void itm_init(void);

bool itm_available(void);

bool itm_port_available(uint8_t port);

void itm_putch(uint8_t port, u8 ch);

void itm_puts(uint8_t port, const char *str);

void itm_write(uint8_t port, const uint8_t *buf, size_t len);

void itm_printf(uint8_t port, const char *format, ...);

/*#define itm_writeXX(port, buf, len) _Generic((buf), \
    uint8_t: itm_write, \
    uint16_t: itm_write16, \
    uint32_t: itm_write32, \
    default: itm_write \
    )(port,buf,len)

#define itm_write(port, buf, len) _Generic((buf), \
    uint8_t *: itm_write(port,buf,len), \
    uint16_t *: itm_write16(port,buf,len), \
    uint32_t *: itm_write32(port,buf,len), \
    default: itm_write(port,buf,len) \
    )*/

void itm_write16(uint8_t port, const uint16_t *buf, size_t len);
void itm_write32(uint8_t port, const uint32_t *buf, size_t len);

#endif // !DEBUG && ITM_DISABLE==1
