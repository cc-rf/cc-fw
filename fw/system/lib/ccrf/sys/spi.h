#pragma once

#include "rdio/rdio.h"


bool ccrf_spi_init(rdio_t rdio);
rdio_status_t ccrf_spi_io(rdio_t rdio, u8 flag, u16 addr, u8 *tx, u8 *rx, size_t size);
