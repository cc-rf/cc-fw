#include "kinetis.h"
#include "sys/isr.h"

#include <fsl_gpio.h>
#include <fsl_port.h>

#include <FreeRTOS.h>


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


typedef struct __packed {
    ccrf_isr_t isr;
    void *param;

} isr_map_t;

static void isr_noop(void *param __unused) { }

#define DECL_PORT_ISR_MAP(P)     \
    static isr_map_t isr_map_port##P[ISRD_PORT##P##_MAX_PIN+1] __ccrf_data;

#define DECL_PORT_MAP_ELEM(P)        isr_map_port ## P,
#define DECL_PORT_MAP_ELEM_NULL(P)   NULL,

#define DECL_PORT_IRQ_FN(P)             void __used PORT##P##_IRQHandler(void) __ccrf_isr;
#define DECL_PORT_IRQ_FN_ELEM(P)        PORT##P##_IRQHandler,
#define DECL_PORT_IRQ_FN_ELEM_NULL(P)   NULL,


IF_PORTS_FN(DECL_PORT_IRQ_FN)

IF_PORTS_FN(DECL_PORT_ISR_MAP)


static isr_map_t *const isr_maps[] = {
        IFE_PORTS_FN(DECL_PORT_MAP_ELEM, DECL_PORT_MAP_ELEM_NULL)
};


static PORT_Type *const ports[] = PORT_BASE_PTRS;
static GPIO_Type *const gpios[] = GPIO_BASE_PTRS;
static IRQn_Type const irqns[] = PORT_IRQS;


void ccrf_isr_configure(rdio_t rdio, ccrf_isr_src_t src, ccrf_isr_edge_t edge, ccrf_isr_t isr, void *param)
{
    const sys_port_t port = cc_interface[rdio_id(rdio)].isr[src].port - 1;
    const u32 pin = cc_interface[rdio_id(rdio)].isr[src].pin;

    isr_maps[port][pin].isr = isr ? isr : isr_noop;
    isr_maps[port][pin].param = param;

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

    port_interrupt_t type = kPORT_InterruptOrDMADisabled;

    switch (edge) {
        case CCRF_ISR_EDGE_FALLING:
            type = kPORT_InterruptFallingEdge;
            break;

        case CCRF_ISR_EDGE_RISING:
            type = kPORT_InterruptRisingEdge;
            break;

        case CCRF_ISR_EDGE_BOTH:
            type = kPORT_InterruptEitherEdge;
            break;

        case CCRF_ISR_EDGE_HIGH:
            type = kPORT_InterruptLogicOne;
            break;

        case CCRF_ISR_EDGE_LOW:
            type = kPORT_InterruptLogicZero;
            break;
    }

    PORT_SetPinInterruptConfig(ports[port], pin, type);

    NVIC_SetPriority(irqns[port], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    EnableIRQ(irqns[port]);

    const gpio_pin_config_t gpio_pin_config = {
            .pinDirection = kGPIO_DigitalInput,
            .outputLogic = 0
    };

    GPIO_PinInit(gpios[port], pin, &gpio_pin_config);
}


bool ccrf_isr_state(rdio_t rdio, ccrf_isr_src_t src)
{
    const sys_port_t port = cc_interface[rdio_id(rdio)].isr[src].port - 1;
    const u32 pin = cc_interface[rdio_id(rdio)].isr[src].pin;
    return GPIO_ReadPinInput(gpios[port], pin) != 0;
}

// TODO: Compare this method with https://lemire.me/blog/2018/02/21/iterating-over-set-bits-quickly/
#define DECL_PORT_ISR(P) \
    void PORT##P##_IRQHandler(void) { \
        u32 isfr = PORT##P->ISFR;                           \
        PORT##P->ISFR = isfr;                               \
        static const isr_map_t *const isr_table = isr_map_port ## P;     \
        u32 bit_nr;                                         \
        while (isfr) {                                      \
            bit_nr = __builtin_ctz(isfr);                   \
            isr_table[bit_nr].isr(isr_table[bit_nr].param); \
            isfr = isfr & (isfr-1);                         \
            if (!isfr) return;                              \
        }                                                   \
    }

IF_PORTS_FN(DECL_PORT_ISR)
