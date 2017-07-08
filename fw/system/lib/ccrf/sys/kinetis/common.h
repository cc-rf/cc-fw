#pragma once


#if BOARD_CLOUDCHASER

#define CC_NUM_DEVICES              2
#define CC_DEV_MIN                  0
#define CC_DEV_MAX                  1

#define ISR_KINETIS_HAVE_PORTC
#define ISR_KINETIS_HAVE_PORTD
//#define ISR_KINETIS_HAVE_PORTE

#define ISRD_PORTC
#define ISRD_PORTC_MAX_PIN      12
//#define ISRD_PORTD
//#define ISRD_PORTD_MAX_PIN      7
// For use with SPI1/external radio
//#define ISRD_PORTE
//#define ISRD_PORTE_MAX_PIN      12

#define SPI_KINETIS_HAVE_SPI0
//#define SPI_KINETIS_HAVE_SPI1
#define SPI_KINETIS_HAVE_SPI2

#elif BOARD_FRDM_K22F

#define CC_NUM_DEVICES              1
#define ISR_KINETIS_HAVE_PORTC

#elif BOARD_FRDM_K66F

#define CC_NUM_DEVICES              1
#define ISR_KINETIS_HAVE_PORTB

#elif BOARD_TWR_K65F180M

#define CC_NUM_DEVICES              1
#define ISR_KINETIS_HAVE_PORTC

#else
#error unknown board
#endif
