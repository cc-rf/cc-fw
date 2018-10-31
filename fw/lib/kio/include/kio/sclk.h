#pragma once

#include <usr/type.h>


#define SCLK_MSEC(x)    ((u32)((x)/1000u))

typedef u64 sclk_t;

void sclk_init(void);
sclk_t sclk_time(void) __fast_code;
