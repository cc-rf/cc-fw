#pragma once

#include <cc/type.h>

#include <cc/interface/kinetis/tmr.h>

#include <fsl_dspi.h>
#include <fsl_gpio.h>
#include <fsl_port.h>


#if BOARD_CLOUDCHASER

#define CC_NUM_DEVICES              2
#define CC_DEV_MIN                  0
#define CC_DEV_MAX                  1

#define ISR_KINETIS_HAVE_PORTC
#define ISR_KINETIS_HAVE_PORTD
//#define ISR_KINETIS_HAVE_PORTE

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


#define SYS_PORT_VALID(SYS_PORT)    ( ((SYS_PORT) >= SYS_PORT_A) && ((SYS_PORT) <= SYS_PORT_E) )
#define SYS_PORT_PORTN(SYS_PORT)    ((PORT_Type*[])PORT_BASE_PTRS)[(SYS_PORT)-1]
#define SYS_PORT_GPION(SYS_PORT)    ((GPIO_Type*[])GPIO_BASE_PTRS)[(SYS_PORT)-1]
#define SYS_PORT_IRQN(SYS_PORT)     (((IRQn_Type[])PORT_IRQS)[(SYS_PORT)-1])
#define SYS_PORT_MASK(SYS_PORT)     ((u8)1<<((SYS_PORT)-1))

typedef enum {
    SYS_PORT_NONE,
    SYS_PORT_A,
    SYS_PORT_B,
    SYS_PORT_C,
    SYS_PORT_D,
    SYS_PORT_E
} sys_port_t;

typedef struct {
    const sys_port_t port;
    const u32 pin;
} sys_gpio_t;

typedef struct {
    SPI_Type *const spi;
    const u32 pcs;
} spi_config_t;

typedef union {
    const sys_gpio_t gpio[3];

    struct {
        const sys_gpio_t hgm;
        const sys_gpio_t lna;
        const sys_gpio_t pa;
    };
} amp_config_t;

typedef sys_gpio_t isr_config_t;

typedef struct interface {
    const spi_config_t spi;
    const amp_config_t amp;
    const isr_config_t isr[3];

} interface_t;