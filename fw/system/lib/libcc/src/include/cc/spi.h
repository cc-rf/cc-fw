#pragma once

#include <cc/common.h>


void cc_spi_init(cc_dev_t dev);
u8 cc_spi_io(cc_dev_t dev, u8 flag, u16 addr, u8 *tx, u8 *rx, u32 len);
