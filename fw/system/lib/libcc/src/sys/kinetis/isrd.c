#include "kinetis.h"
#include <cc/sys/kinetis/isrd.h>

#include <fsl_gpio.h>
#include <fsl_port.h>

#include <FreeRTOS.h>
#include <task.h>

#include <stdatomic.h>

#ifdef ISRD_PORTA
#define IF_PORTA(THEN)          THEN
#define IFE_PORTA(THEN,ELSE)    THEN
#else
#define IF_PORTA(THEN)
#define IFE_PORTA(THEN,ELSE)    ELSE
#endif

#ifdef ISRD_PORTB
#define IF_PORTB(THEN)          THEN
#define IFE_PORTB(THEN,ELSE)    THEN
#else
#define IF_PORTB(THEN)
#define IFE_PORTB(THEN,ELSE)    ELSE
#endif

#ifdef ISRD_PORTC
#define IF_PORTC(THEN)          THEN
#define IFE_PORTC(THEN,ELSE)    THEN
#else
#define IF_PORTC(THEN)
#define IFE_PORTC(THEN,ELSE)    ELSE
#endif

#ifdef ISRD_PORTD
#define IF_PORTD(THEN)          THEN
#define IFE_PORTD(THEN,ELSE)    THEN
#else
#define IF_PORTD(THEN)
#define IFE_PORTD(THEN,ELSE)    ELSE
#endif

#ifdef ISRD_PORTE
#define IF_PORTE(THEN)          THEN
#define IFE_PORTE(THEN,ELSE)    THEN
#else
#define IF_PORTE(THEN)
#define IFE_PORTE(THEN,ELSE)    ELSE
#endif

#define IF_PORT(P,THEN)             IF_PORT##P(THEN)
#define IFE_PORT(P,THEN,ELSE)       IFE_PORT##P(THEN,ELSE)

#define IF_PORT_FN(P,THEN)          IF_PORT##P(THEN(P))
#define IFE_PORT_FN(P,THEN,ELSE)    IFE_PORT##P(THEN(P),ELSE(P))

#define IF_PORTS_FN(THEN)  \
    IF_PORT_FN(A,THEN)     \
    IF_PORT_FN(B,THEN)     \
    IF_PORT_FN(C,THEN)     \
    IF_PORT_FN(D,THEN)     \
    IF_PORT_FN(E,THEN)

#define IFE_PORTS_FN(THEN,ELSE)  \
    IFE_PORT_FN(A,THEN,ELSE)     \
    IFE_PORT_FN(B,THEN,ELSE)     \
    IFE_PORT_FN(C,THEN,ELSE)     \
    IFE_PORT_FN(D,THEN,ELSE)     \
    IFE_PORT_FN(E,THEN,ELSE)

#define ISRD_MAX_PIN(P)         ISRD_PORT##P##_MAX_PIN
#define ISRD_MAX_ISR(P)         ISRD_PORT##P##_MAX_ISR



static void isr_noop(void) {}

#define DECL_PORT_ISR_MAP(P)     \
    static isr_t isr_map_port##P[ISRD_PORT##P##_MAX_PIN+1] = { isr_noop };

IF_PORTS_FN(DECL_PORT_ISR_MAP)

#define DECL_PORT_MAP_ELEM(P)        isr_map_port ## P,
#define DECL_PORT_MAP_ELEM_NULL(P)   NULL,

static isr_t *const isr_maps[] = {
        IFE_PORTS_FN(DECL_PORT_MAP_ELEM, DECL_PORT_MAP_ELEM_NULL)
};

static PORT_Type *const ports[] = PORT_BASE_PTRS;
static GPIO_Type *const gpios[] = GPIO_BASE_PTRS;
static IRQn_Type const irqns[] = PORT_IRQS;

void isrd_configure(const u8 port, const u8 pin, const port_interrupt_t type, const isr_t isr)
{
    if (isr) isr_maps[port][pin] = isr;
    else isr_maps[port][pin] = isr_noop;

    const port_pin_config_t port_pin_config = {
            kPORT_PullDisable,
            kPORT_FastSlewRate,
            kPORT_PassiveFilterDisable,
            kPORT_OpenDrainDisable,
            kPORT_LowDriveStrength,
            kPORT_MuxAsGpio,
            kPORT_LockRegister,
    };

    PORT_SetPinConfig(ports[port], pin, &port_pin_config);
    PORT_SetPinInterruptConfig(ports[port], pin, type);

    NVIC_SetPriority(irqns[port], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1/*+2*/);

    EnableIRQ(irqns[port]);

    const gpio_pin_config_t gpio_pin_config = {
            .pinDirection = kGPIO_DigitalInput,
            .outputLogic = 0
    };

    GPIO_PinInit(gpios[port], pin, &gpio_pin_config);
}

#define DECL_PORT_ISR(P) \
    __attribute__((interrupt,noclone)) void PORT##P##_IRQHandler(void) { \
        u32 isfr = PORT##P->ISFR;                       \
        PORT##P->ISFR = isfr;                           \
        isr_t *const isr_table = isr_map_port ## P;     \
        u32 bit_nr;                                     \
        while (isfr) {                                  \
            bit_nr = __builtin_ctz(isfr);               \
            isr_table[bit_nr]();                        \
            isfr = isfr & (isfr-1);                     \
            if (!isfr) return;                          \
        }                                               \
    }

IF_PORTS_FN(DECL_PORT_ISR)
