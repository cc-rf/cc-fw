#include <pca9685.h>
#include <pca9685_io.h>
#include <stdio.h>

#define PCA_ADDR 0x40

static void pca9685_reset(pca9685_handle_t handle);

pca9685_handle_t pca9685_init(u8 bus)
{
    pca9685_handle_t handle = pca9685_io_init(bus);

    if (handle) {
        pca9685_reset(handle);
    }

    return handle;
}

static void pca9685_reset(pca9685_handle_t handle)
{
    // TODO: refine this

    u8 value = 0x21; //  RESTART | AUTOINCR | ALLADRR
    pca9685_io(handle, PCA9685_WRITE, PCA_ADDR, PCA9685_MODE1, 1, &value);

    value = 0;
    pca9685_io(handle, PCA9685_READ, PCA_ADDR, PCA9685_MODE1, 1, &value);
    printf("<pca> mode1=0x%02X\r\n", value);

    value = 0x04;
    pca9685_io(handle, PCA9685_WRITE, PCA_ADDR, PCA9685_MODE2, 1, &value);
}

void pca9685_set(pca9685_handle_t handle, u8 chan, u16 on, u16 off)
{
    u8 on_data[2] = { (u8)(on & 0xFF), (u8)((on >> 8) & 0x1F) };
    u8 off_data[2] = { (u8)(off & 0xFF), (u8)((off >> 8) & 0x1F) };
    pca9685_io(handle, PCA9685_WRITE, PCA_ADDR,  PCA9685_CHAN_ON_L(chan), 1, &on_data[0]);
    pca9685_io(handle, PCA9685_WRITE, PCA_ADDR, PCA9685_CHAN_ON_H(chan), 1, &on_data[1]);
    pca9685_io(handle, PCA9685_WRITE, PCA_ADDR, PCA9685_CHAN_OFF_L(chan), 1, &off_data[0]);
    pca9685_io(handle, PCA9685_WRITE, PCA_ADDR, PCA9685_CHAN_OFF_H(chan), 1, &off_data[1]);
}
