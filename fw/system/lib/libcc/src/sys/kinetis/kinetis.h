#pragma once

#include <cc/common.h>

#include <fsl_dspi.h>
#include <fsl_gpio.h>
#include <fsl_port.h>

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

extern const interface_t cc_interface[CC_NUM_DEVICES];