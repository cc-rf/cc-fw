#pragma once

#include <usr/type.h>
#include <core_cm4.h>
#include <stdio.h>

// TODO: don't assume rdimon clib

static inline void semi_init(void)
{

}

static inline bool semi_available(void)
{
    return (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) == CoreDebug_DHCSR_C_DEBUGEN_Msk;
}

static inline void semi_puts(const char *const str)
{
    //fputs(str, stdout);
    printf("%s", str);
}

static inline void semi_write(const u8 *const buf, const size_t len)
{
    fwrite((void *)buf, 1UL, len, stdout);
}