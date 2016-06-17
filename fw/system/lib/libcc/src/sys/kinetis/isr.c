#include "kinetis.h"
#include <cc/isr.h>

#include <fsl_gpio.h>
#include <fsl_port.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>


#define PORT_IRQ_HANDLER(P) \
        static volatile u32 isr_count_##P, isr_flag_##P;\
        __attribute__((interrupt)) void PORT##P##_IRQHandler(void){\
            const u32 flag = GPIO_GetPinsInterruptFlags(GPIO##P); \
            GPIO_ClearPinsInterruptFlags(GPIO##P, flag); \
            return isr_common(SYS_PORT_##P, flag); \
        }

static inline const isr_config_t *const get_isr_config(cc_dev_t dev, enum isr_pin *pin);
static void isr_task(void *param);
static inline void isr_common(const sys_port_t port, const u32 flag);

static isr_t isr_map[CC_NUM_DEVICES][ISR_PIN_COUNT] = {{ 0 }};

static const port_interrupt_t EDGE_MAP[] = {
        /*ISR_EDGE_SETUP*/      kPORT_InterruptOrDMADisabled,
        /*ISR_EDGE_FALLING*/    kPORT_InterruptFallingEdge,
        /*ISR_EDGE_RISING*/     kPORT_InterruptRisingEdge,
        /*ISR_EDGE_BOTH*/       kPORT_InterruptEitherEdge,
        /*ISR_EDGE_HIGH*/       kPORT_InterruptLogicOne,
        /*ISR_EDGE_LOW*/        kPORT_InterruptLogicZero,
};

typedef struct {
    sys_port_t port;
    u32 flag;
} isr_queue_t;


typedef struct {
    xQueueHandle isr_queue_handle;
    xTaskHandle isr_task_handle;
    u8 port_mask;

} isr_rtos_t;

static isr_rtos_t rtos[CC_NUM_DEVICES] = {{NULL}};



bool cc_isr_init(cc_dev_t dev)
{
    assert(CC_DEV_VALID(dev));

    if (!rtos[dev].isr_queue_handle) {
        rtos[dev].port_mask = 0;

        if (!(rtos[dev].isr_queue_handle = xQueueCreate(24, sizeof(isr_queue_t)))) {
            cc_dbg("[%u] error: queue create failed", dev);
            return false;
        }
    }

    if (!rtos[dev].isr_task_handle) {
        xTaskCreate(
                isr_task, "isr_task", TASK_STACK_SIZE_DEFAULT,
                (void *)dev, TASK_PRIO_DEFAULT, &rtos[dev].isr_task_handle
        );

        if (!rtos[dev].isr_task_handle) {
            cc_dbg("[%u] error: task create failed", dev);
            return false;
        }
    }

    /*rtc_config_t rtc_config;
    RTC_GetDefaultConfig(&rtc_config);
    RTC_Init(RTC, &rtc_config);
    rtc_datetime_t rtc_datetime = { 0 };
    RTC_SetDatetime(RTC, &rtc_datetime);*/

    return true;
}

enum isr_pin cc_isr(cc_dev_t dev, enum isr_pin pin, enum isr_edge edge, isr_t isr)
{
    const isr_config_t *const cfg = get_isr_config(dev, &pin);

    if (!cfg) return ISR_PIN_NONE;

    isr_map[dev][pin] = isr;

    if (edge != ISR_EDGE_SETUP) {
        // TODO: Maybe start task here on-demand

        rtos[dev].port_mask |= SYS_PORT_MASK(cfg->port);

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

    } else if (!isr) {
        PORT_SetPinInterruptConfig(SYS_PORT_PORTN(cfg->port), cfg->pin, EDGE_MAP[edge]);
    }

    return pin;
}

bool cc_isr_state(cc_dev_t dev, enum isr_pin pin)
{
    const isr_config_t *const cfg = get_isr_config(dev, &pin);
    if (pin == ISR_PIN_NONE) return false;
    return GPIO_ReadPinInput(SYS_PORT_GPION(cfg->port), cfg->pin) != 0;
}

static inline const isr_config_t *const get_isr_config(cc_dev_t dev, enum isr_pin *pin)
{
    assert(CC_DEV_VALID(dev));
    assert(pin && ((*pin < ISR_PIN_COUNT) || (*pin == ISR_PIN_ANY)));

    if (*pin == ISR_PIN_ANY) {
        for (enum isr_pin p = ISR_PIN_BASE; p < ISR_PIN_COUNT; ++p) {
            if (SYS_PORT_VALID(cc_interface[dev].isr[p].port)) {
                *pin = p;
                if (!isr_map[dev][p]) break;
            }
        }

        if (*pin == ISR_PIN_ANY) return NULL;
    }

    const isr_config_t *const cfg = &cc_interface[dev].isr[*pin];
    assert(SYS_PORT_VALID(cfg->port));
    return cfg;
}

static void isr_task(void *param)
{
    const cc_dev_t dev = (cc_dev_t)param; assert(CC_DEV_VALID(dev));
    const isr_config_t *const isrs = cc_interface[dev].isr; assert(isrs);
    const isr_t *const maps = isr_map[dev]; assert(maps);
    const xQueueHandle queue = rtos[dev].isr_queue_handle; assert(queue);

    isr_queue_t evt;

    while (1) {
        if (xQueueReceive(queue, &evt, portMAX_DELAY) != pdTRUE) continue;

        for (u8 i = 0; i < ISR_PIN_COUNT; ++i) {
            if (evt.port == isrs[i].port) {
                const u32 pf = 1u << isrs[i].pin;
                if (evt.flag & pf) {
                    evt.flag &= ~pf;
                    if (maps[i]) maps[i](dev);
                }
            }
        }

        if (evt.flag) {
            cc_dbg("[%u] unhandled: port=%u flag=0x%08X", dev, evt.port, evt.flag);
        }
    }
}

static inline void isr_common(sys_port_t port, u32 flag)
{
    BaseType_t xHigherPriorityTaskWoken, xHigherPriorityTaskWokenAll = pdFALSE;

    const isr_queue_t evt = {
            .port = port,
            .flag = flag
    };

    for (cc_dev_t dev = CC_DEV_MIN; dev <= CC_DEV_MAX; ++dev) {
        if (rtos[dev].port_mask & SYS_PORT_MASK(port)) {
            if (xQueueSendFromISR(rtos[dev].isr_queue_handle, &evt, &xHigherPriorityTaskWoken) != pdTRUE) {
                cc_dbg("[%u] interrupt queue full!", dev);
                assert(false);
            }

            xHigherPriorityTaskWokenAll |= xHigherPriorityTaskWoken;
        }
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
