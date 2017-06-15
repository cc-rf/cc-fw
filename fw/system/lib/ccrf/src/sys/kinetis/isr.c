#include "kinetis.h"
#include <cc/isr.h>

#include <fsl_gpio.h>
#include <fsl_port.h>

#include <FreeRTOS.h>
#include <task.h>

#include <stdatomic.h>


#define PORT_MAPS_ENTRY(P) isr_pending_task_##P
#define PORT_MAPS_EMPTY(P) NULL

/*TODO: Use _Atomic and/or stdatomic.h functions to check/set atomically*/
#define PORT_IRQ_HANDLER(P) \
        /*static*/ /*_Atomic*/ /*volatile?*/ xTaskHandle isr_pending_task_##P[/*32*/1] = { NULL };\
        __attribute__((interrupt)) void PORT##P##_IRQHandler(void){\
            const u32 flag = GPIO_GetPinsInterruptFlags(GPIO##P); \
            GPIO_ClearPinsInterruptFlags(GPIO##P, flag); \
            return isr_common(SYS_PORT_##P, flag, isr_pending_task_##P);\
        }

typedef xTaskHandle *const isr_port_vecs_t;

static const isr_port_vecs_t isr_port_vecs[];

static inline void isr_common(const sys_port_t port, u32 flag, xTaskHandle *tasks);

static inline const isr_config_t *const get_isr_config(cc_dev_t dev, enum isr_pin *pin);


static const port_interrupt_t EDGE_MAP[] = {
        /*ISR_EDGE_SETUP*/      kPORT_InterruptOrDMADisabled,
        /*ISR_EDGE_FALLING*/    kPORT_InterruptFallingEdge,
        /*ISR_EDGE_RISING*/     kPORT_InterruptRisingEdge,
        /*ISR_EDGE_BOTH*/       kPORT_InterruptEitherEdge,
        /*ISR_EDGE_HIGH*/       kPORT_InterruptLogicOne,
        /*ISR_EDGE_LOW*/        kPORT_InterruptLogicZero,
};


isr_handle_t cc_isr_setup(cc_dev_t dev, enum isr_pin *pin, enum isr_edge edge)
{
    if (!pin) return NULL;
    const isr_config_t *const cfg = get_isr_config(dev, pin);
    if (!cfg) return NULL;

    if (edge != ISR_EDGE_SETUP) {
        // TODO: Maybe start task here on-demand

        //rtos[dev].port_mask |= SYS_PORT_MASK(cfg->port);

        const port_pin_config_t port_pin_config = {
                kPORT_PullDisable,
                kPORT_FastSlewRate,
                kPORT_PassiveFilterDisable,
                kPORT_OpenDrainDisable,
                kPORT_LowDriveStrength,
                kPORT_MuxAsGpio,
                kPORT_LockRegister,
        };

        PORT_SetPinConfig(SYS_PORT_PORTN(cfg->port), cfg->pin, &port_pin_config);
        PORT_SetPinInterruptConfig(SYS_PORT_PORTN(cfg->port), cfg->pin, EDGE_MAP[edge]);

        NVIC_SetPriority(SYS_PORT_IRQN(cfg->port), configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1/*+2*/);

        EnableIRQ(SYS_PORT_IRQN(cfg->port));

        const gpio_pin_config_t gpio_pin_config = {
                .pinDirection = kGPIO_DigitalInput,
                .outputLogic = 0
        };

        GPIO_PinInit(SYS_PORT_GPION(cfg->port), cfg->pin, &gpio_pin_config);
    }

    return (isr_handle_t) &isr_port_vecs[cfg->port - 1][cfg->pin];
}

void cc_isr_wait(isr_handle_t isr_handle)
{
    *(xTaskHandle *)isr_handle = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

static inline const isr_config_t *const get_isr_config(cc_dev_t dev, enum isr_pin *pin)
{
    assert(CC_DEV_VALID(dev));
    assert(pin && ((*pin < ISR_PIN_COUNT) || (*pin == ISR_PIN_ANY)));

    if (*pin == ISR_PIN_ANY) {
        for (enum isr_pin p = ISR_PIN_BASE; p < ISR_PIN_COUNT; ++p) {
            if (SYS_PORT_VALID(cc_interface[dev].isr[p].port)) {
                *pin = p;
                if (!isr_port_vecs[cc_interface[dev].isr[p].port - 1][cc_interface[dev].isr[p].pin - 1]) break;
            }
        }

        if (*pin == ISR_PIN_ANY) return NULL;
    }

    const isr_config_t *const cfg = &cc_interface[dev].isr[*pin];
    assert(SYS_PORT_VALID(cfg->port));
    return cfg;
}


static inline void isr_common(const sys_port_t port, u32 flag, xTaskHandle *tasks)
{
    BaseType_t xHigherPriorityTaskWoken, xHigherPriorityTaskWokenAll = pdFALSE;
    xTaskHandle task;

    while (flag) {
        if ((flag & 1) && (task = *tasks)) {
            *tasks = NULL;
            vTaskNotifyGiveFromISR(task, &xHigherPriorityTaskWoken);
            xHigherPriorityTaskWokenAll |= xHigherPriorityTaskWoken;
            // TODO: clear individual flag bits as they are notified? (if so need to keep track of bit #s)
        }

        flag >>= 1;
        ++tasks;
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWokenAll);
}

#ifdef ISR_KINETIS_HAVE_PORTA
PORT_IRQ_HANDLER(A)
#endif

#ifdef ISR_KINETIS_HAVE_PORTB
PORT_IRQ_HANDLER(B)
#endif

#ifdef ISR_KINETIS_HAVE_PORTC
PORT_IRQ_HANDLER(C)
#endif

#ifdef ISR_KINETIS_HAVE_PORTD
PORT_IRQ_HANDLER(D)
#endif

#ifdef ISR_KINETIS_HAVE_PORTE
PORT_IRQ_HANDLER(E)
#endif


static const isr_port_vecs_t isr_port_vecs[] = {
#ifdef ISR_KINETIS_HAVE_PORTA
        PORT_MAPS_ENTRY(A),
#else
        PORT_MAPS_EMPTY(A),
#endif

#ifdef ISR_KINETIS_HAVE_PORTB
        PORT_MAPS_ENTRY(B),
#else
        PORT_MAPS_EMPTY(B),
#endif

#ifdef ISR_KINETIS_HAVE_PORTC
        PORT_MAPS_ENTRY(C),
#else
        PORT_MAPS_EMPTY(C),
#endif

#ifdef ISR_KINETIS_HAVE_PORTD
        PORT_MAPS_ENTRY(D),
#else
        PORT_MAPS_EMPTY(D),
#endif

#ifdef ISR_KINETIS_HAVE_PORTE
        PORT_MAPS_ENTRY(E),
#else
        PORT_MAPS_EMPTY(E),
#endif
};
