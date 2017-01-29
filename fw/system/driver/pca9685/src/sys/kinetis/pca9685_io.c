#include <pca9685_io.h>
#include "i2c.h"

pca9685_io_handle_t pca9685_io_init(u8 bus)
{
    return (pca9685_io_handle_t)i2c_init(bus);
}

void pca9685_io(pca9685_io_handle_t handle, pca9685_rw_t rw, u8 addr, u8 cmd, size_t size, void *data)
{
    return i2c_io((i2c_handle_t)handle, (i2c_rw_t)rw, addr, cmd, size, data);
}
