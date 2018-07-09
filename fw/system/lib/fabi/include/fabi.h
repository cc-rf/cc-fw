#pragma once

#include <usr/type.h>


#define FABI_LED_MAX        512u
#define FABI_CHAN_COUNT     ((u8) 6u)

typedef struct __packed fabi_rgb {
    u8 g, r, b;

} fabi_rgb_t;


void fabi_init(void);
void fabi_write(u8 mask, fabi_rgb_t *data, size_t size);
