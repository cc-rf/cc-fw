#pragma once

#include <usr/type.h>


#define FABI_LED_MAX        512


typedef struct __packed fabi_rgb {
    u8 g, r, b;

} fabi_rgb_t;


void fabi_init(void);
void fabi_write(fabi_rgb_t *data, size_t size);
