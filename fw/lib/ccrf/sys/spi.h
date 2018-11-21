#pragma once

#include <ccrf/ccrf.h>


#define CCRF_SPI_MAX_SIZE   130


typedef struct ccrf_spi *ccrf_spi_t;


ccrf_spi_t ccrf_spi_init(u8 rdio_id);
u8 ccrf_spi_io(ccrf_spi_t spi, u8 flag, u16 addr, u8 *tx, u8 *rx, size_t size) __ccrf_code __nonnull((1));
