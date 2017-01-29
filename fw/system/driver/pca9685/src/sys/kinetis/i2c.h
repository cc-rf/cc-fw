#pragma once
#include <usr/type.h>

typedef void *i2c_handle_t;

typedef enum __packed {
    I2C_WRITE,
    I2C_READ

} i2c_rw_t;

i2c_handle_t i2c_init(u8 bus);
void i2c_io(i2c_handle_t handle, i2c_rw_t rw, u8 addr, u8 cmd, size_t size, void *data);
