#pragma once

#include <stddef.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

// http://stackoverflow.com/questions/4415524/common-array-length-macro-for-c
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

typedef int8_t      s8;
typedef uint8_t     u8;
typedef int16_t     s16;
typedef uint16_t    u16;
typedef int32_t     s32;
typedef uint32_t    u32;
typedef int64_t     s64;
typedef uint64_t    u64;

typedef u8          cc_dev_t;
typedef u64         nsec_t;

typedef u16         channel_t; // chn_t? chan_t?


typedef enum {
    MOD_ASK,
    MOD_OOK,
    MOD_FSK,
    MOD_FSK_2,
    MOD_GFSK,
    // ...
} mod_t;

