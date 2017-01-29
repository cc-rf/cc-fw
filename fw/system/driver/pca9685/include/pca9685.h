#pragma once
#include <usr/type.h>

#define PCA9685_CHAN_ALL                9
#define PCA9685_ONOFF_MASK             ((u16)0x1FFF)
#define PCA9685_ONOFF_FULL             ((u16)0x1000)

typedef void *pca9685_handle_t;

pca9685_handle_t pca9685_init(u8 bus);
void pca9685_set(pca9685_handle_t handle, u8 chan, u16 on, u16 off);
