#pragma once


#if BOARD_CLOUDCHASER

    #define CCRF_CONFIG_RDIO_COUNT      1 //2

    #define ISR_KINETIS_HAVE_PORTC
    #define ISR_KINETIS_HAVE_PORTD

    #define ISRD_PORTC
    #define ISRD_PORTC_MAX_PIN          12
    #define ISRD_PORTD
    #define ISRD_PORTD_MAX_PIN          9

    #define SPI_KINETIS_HAVE_SPI0
    #define SPI_KINETIS_HAVE_SPI2

#else
#error unknown board
#endif
