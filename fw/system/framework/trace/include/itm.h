#pragma once

#include <usr/type.h>
#include <core_cm4.h>
#include <fsl_clock.h>

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

static inline void itm_init(void)
{
    uint32_t SWOSpeed = 1200000; //1000kbps, default for JLinkSWOViewer
    //uint32_t SWOSpeed = 6000000; //6000kbps, default for JLinkSWOViewer
    uint32_t SWOPrescaler = (CLOCK_GetCoreSysClkFreq() / SWOSpeed) - 1; // SWOSpeed in Hz, note that F_CPU is expected to be 96000000 in this case
    CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk;
    //*((volatile unsigned *)(ITM_BASE + 0x400F0)) = 0x00000002; // "Selected PIN Protocol Register": Select which protocol to use for trace output (2: SWO)
    TPI->SPPR = 0x00000002;
    //*((volatile unsigned *)(ITM_BASE + 0x40010)) = SWOPrescaler; // "Async Clock Prescaler Register". Scale the baud rate of the asynchronous output
    TPI->ACPR = SWOPrescaler;
    //*((volatile unsigned *)(ITM_BASE + 0x00FB0)) = 0xC5ACCE55; // ITM Lock Access Register, C5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC
    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk; // ITM Trace Control Register
    ITM->TPR = ITM_TPR_PRIVMASK_Msk; // ITM Trace Privilege Register
    ITM->TER = 0xFFFFFFFF; // ITM Trace Enable Register. Enabled tracing on stimulus ports. One bit per stimulus port.
    //*((volatile unsigned *)(ITM_BASE + 0x01000)) = 0x400003FE; // DWT_CTRL
    DWT->CTRL = 0x400003FE;
    //*((volatile unsigned *)(ITM_BASE + 0x40304)) = 0x00000100; // Formatter and Flush Control Register
    TPI->FFCR = 0x00000100;
}

static inline bool itm_available(void)
{
    return (ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL;
}

static inline bool itm_port_available(const uint8_t port)
{
    return (ITM->TER & (1UL << port)) != 0UL;
}

static inline void itm_putch(const uint8_t port, const char ch)
{
    while (ITM->PORT[port].u32 == 0UL) __NOP();
    ITM->PORT[port].u8 = ch;
}

static inline void itm_puts(const uint8_t port, const char *str)
{
    //if (!itm_available() || !itm_port_available(port)) return;
    char ch;

    while ((ch = *str++)) {
        itm_putch(port, ch);
    }
}

static inline void itm_write(const uint8_t port, const uint8_t *buf, const size_t len)
{
    //if (!itm_available() || !itm_port_available(port)) return;
    const uint8_t *const end = buf + len;

    while (buf < end) {
        while (ITM->PORT[port].u32 == 0UL) __NOP();
        ITM->PORT[port].u8 = *buf++;
    }
}

static inline void itm_printf(const uint8_t port, const char *format, ...)
{
    va_list va;
    u8 *output;
    int result;

    va_start(va, format);
    result = vasprintf((char **)&output, format, va);

    if (result >= 0) {
        itm_write(port, output, (size_t) result);
        free(output);
    } else {
        itm_puts(0, "\r\n\r\n<itm> WRITE FAILED\r\n\r\n");
    }

    va_end(va);
}

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

static inline void itm_write16(const uint8_t port, const uint16_t *buf, const size_t len)
{
    //if (!itm_available() || !itm_port_available(port)) return;
    const uint16_t *const end = buf + len;

    while (buf < end) {
        while (ITM->PORT[port].u32 == 0UL) __NOP();
        ITM->PORT[port].u16 = *buf++;
    }
}

static inline void itm_write32(const uint8_t port, const uint32_t *buf, const size_t len)
{
    //if (!itm_available() || !itm_port_available(port)) return;
    const uint32_t *const end = buf + len;

    while (buf < end) {
        while (ITM->PORT[port].u32 == 0UL) __NOP();
        ITM->PORT[port].u16 = *buf++;
    }
}
