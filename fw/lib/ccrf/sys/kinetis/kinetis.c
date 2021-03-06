#include "kinetis.h"

const interface_t cc_interface[CCRF_CONFIG_RDIO_COUNT] = {
#if BOARD_CLOUDCHASER
    {
        .spi = {SPI0, 0},
        .amp = {
            .hgm = {SYS_PORT_E,  6},
            .lna = {SYS_PORT_E,  8},
            .pa  = {SYS_PORT_E,  9},
        },
        .isr = {
            {SYS_PORT_C, 10},
            {SYS_PORT_C, 11},
            {SYS_PORT_C, 12},
        }
#if BOARD_CLOUDCHASER && BOARD_REVISION == 1
    },{
        .spi = {SPI2, 0},
        .amp = {
            .hgm = {SYS_PORT_E,  7},
            .lna = {SYS_PORT_E, 10},
            .pa  = {SYS_PORT_E, 11},
        },
        .isr = {
            {SYS_PORT_D,  7},
            {SYS_PORT_D,  8},
            {SYS_PORT_D,  9},
        }
#endif
    }/*,{
        .spi = {SPI1, 1},
        .amp = {
            .hgm = {SYS_PORT_NONE,  0},
            .lna = {SYS_PORT_NONE,  0},
            .pa  = {SYS_PORT_NONE,  0},
        },
        .isr = {
            {SYS_PORT_NONE,  0},
            {SYS_PORT_E,     4},
            {SYS_PORT_NONE,  0},
        }
    }*/
#endif
};
