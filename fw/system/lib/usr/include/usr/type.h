#pragma once

#include <stddef.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

typedef int8_t      s8;
typedef uint8_t     u8;
typedef int16_t     s16;
typedef uint16_t    u16;
typedef int32_t     s32;
typedef uint32_t    u32;
typedef int64_t     s64;
typedef uint64_t    u64;

// TODO: decide whether or not these are a good idea. Same goes for _t suffix...
typedef u8                  byte_t;
typedef unsigned char       uchar_t;
typedef unsigned short      ushort_t;
typedef unsigned int        uint_t;
typedef unsigned long       ulong_t;
typedef unsigned long long  ulonglong_t;

