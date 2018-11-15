#pragma once

#include <stddef.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#define __fast_data __section(".fast_data")
#define __fast_code __section(".fast_text")
#define __fast_isr  __section(".fast_text_isr")

// http://stackoverflow.com/questions/4415524/common-array-length-macro-for-c
#define COUNT_OF(x) ((sizeof(x)/sizeof(x[0])) / ((size_t)(!(sizeof(x) % sizeof(x[0])))))


typedef int8_t      s8;
typedef uint8_t     u8;
typedef int16_t     s16;
typedef uint16_t    u16;
typedef int32_t     s32;
typedef uint32_t    u32;
typedef int64_t     s64;
typedef uint64_t    u64;


static inline void wcpy(volatile u32 *restrict src, volatile u32 *restrict end, volatile u32 *restrict dst)
{
    while (src < end) {
        *dst++ = *src++;
    }
}


static inline void wset(u32 val, volatile u32 *restrict dst, volatile u32 *restrict end)
{
    while (dst < end) {
        *dst++ = val;
    }
}
