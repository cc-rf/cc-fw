#pragma once

#include <usr/type.h>


#define CCRF_CLOCK_MSEC(x)      ((u32)((x)/1000u))


typedef u64 ccrf_clock_t;


bool ccrf_clock_init(void);
ccrf_clock_t ccrf_clock(void);
