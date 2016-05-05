#include <cc/isr.h>
//#include <FreeRTOS.h>
//#include <task.h>
//#include <queue.h>
//#include <semphr.h>
#include <fsl_rtc.h>
#include <itm.h>
#include <cc/interface/kinetis/kinetis.h>

/*
static struct isr_map ISR_MAP[] = {
#if BOARD_FRDM_K66F

#define PORT            PORTB
#define GPIO            GPIOB
#define IRQ_N           PORTB_IRQn
#define IRQ_HANDLER     PORTB_IRQHandler

        {ISR_PIN_GPIO2, NULL, PORT, GPIO, IRQ_N, 2},
        {ISR_PIN_GPIO3, NULL, PORT, GPIO, IRQ_N, 3},
        {ISR_PIN_GPIO0, NULL, PORT, GPIO, IRQ_N, 4},

#elif BOARD_FRDM_K22F

#define PORT            PORTC
#define GPIO            GPIOC
#define IRQ_N           PORTC_IRQn
#define IRQ_HANDLER     PORTC_IRQHandler

        {ISR_PIN_GPIO2, NULL, PORT, GPIO, IRQ_N, 2},
        */
/* Also possible: PTC0, PTD2 *//*


#elif BOARD_TWR_K65F180M

#define PORT            PORTC
#define GPIO            GPIOC
#define IRQ_N           PORTC_IRQn
#define IRQ_HANDLER     PORTC_IRQHandler

        {ISR_PIN_GPIO2, NULL, PORT, GPIO, IRQ_N, 19},

#elif BOARD_CLOUDCHASER

#define PORT            PORTC
#define GPIO            GPIOC
#define IRQ_N           PORTC_IRQn
#define IRQ_HANDLER     PORTC_IRQHandler

        {ISR_PIN_GPIO3, NULL, PORT, GPIO, IRQ_N, 10},
        {ISR_PIN_GPIO2, NULL, PORT, GPIO, IRQ_N, 11},
        {ISR_PIN_GPIO0, NULL, PORT, GPIO, IRQ_N, 12},

#else
#error unknown board
#endif
};
*/

#define PORT_IRQ_HANDLER(P) \
        static volatile u32 isr_count_##P, isr_flag_##P;\
        __attribute__((interrupt)) void PORT##P##_IRQHandler(void){\
            isr_flag_##P = GPIO_GetPinsInterruptFlags(GPIO##P); \
            GPIO_ClearPinsInterruptFlags(GPIO##P, isr_flag_##P); \
            ++isr_count_##P; \
            /*return isr_common(SYS_PORT_##P, GPIO##P);*/ \
        }

#define PORT_ISR_TASK(P) \
        if (isr_count_##P) { \
            --isr_count_##P; \
            isr_common(SYS_PORT_##P, isr_flag_##P); \
        }

static inline const isr_config_t *const get_isr_config(cc_dev_t dev, enum isr_pin *pin);
//static void isr_task(void *param);

static isr_t isr_map[CC_NUM_DEVICES][ISR_PIN_COUNT] = {{ 0 }};

static const port_interrupt_t EDGE_MAP[] = {
        /*ISR_EDGE_SETUP*/      kPORT_InterruptOrDMADisabled,
        /*ISR_EDGE_FALLING*/    kPORT_InterruptFallingEdge,
        /*ISR_EDGE_RISING*/     kPORT_InterruptRisingEdge,
        /*ISR_EDGE_BOTH*/       kPORT_InterruptEitherEdge,
        /*ISR_EDGE_HIGH*/       kPORT_InterruptLogicOne,
        /*ISR_EDGE_LOW*/        kPORT_InterruptLogicZero,
};

/*volatile u32 port_isr_mask = 0;
volatile u32 port_isr_flags = 0;
volatile u64 port_isr_count = 0, isr_handle_count = 0;*/


/*typedef struct {
    xSemaphoreHandle isr_semaphore_handle;
    xTaskHandle isr_task_handle;

} isr_rtos_t;

static isr_rtos_t rtos[CC_NUM_DEVICES] = {{NULL}};*/

bool cc_isr_init(cc_dev_t dev)
{
    assert(CC_DEV_VALID(dev));
    /*port_isr_mask = 0;

    if (!rtos[dev].isr_semaphore_handle) {
        if (!(rtos[dev].isr_semaphore_handle = xSemaphoreCreateCounting(UINT32_MAX, 0))) {
            cc_dbg("[%u] error: semaphore create failed", dev);
            return false;
        }
    }

    if (!rtos[dev].isr_task_handle) {
        xTaskCreate(
                isr_task, "isr_task", configMINIMAL_STACK_SIZE*8,
                (void *)dev, 1, &rtos[dev].isr_task_handle
        );

        if (!rtos[dev].isr_task_handle) {
            cc_dbg("[%u] error: task create failed", dev);
            return false;
        }
    }*/

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
        const port_pin_config_t port_pin_config = {
                kPORT_PullDown,
                kPORT_FastSlewRate,
                kPORT_PassiveFilterDisable,
                kPORT_OpenDrainDisable,
                kPORT_LowDriveStrength,
                kPORT_MuxAsGpio,
                kPORT_UnlockRegister,
        };

        //port_isr_mask |= (1u << cfg->pin);
        PORT_SetPinConfig(SYS_PORT_PORTN(cfg->port), cfg->pin, &port_pin_config);
        PORT_SetPinInterruptConfig(SYS_PORT_PORTN(cfg->port), cfg->pin, EDGE_MAP[edge]);

//#if defined(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
        NVIC_SetPriority(SYS_PORT_IRQN(cfg->port), 1/*configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY-*//*CC_NUM_DEVICES+dev*/);
//#endif

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

/*static void isr_task(void *param)
{
    const cc_dev_t dev = (cc_dev_t)param; assert(CC_DEV_VALID(dev));
    const isr_config_t *const isrs = cc_interface[dev].isr; assert(isrs);
    const isr_t *const maps = isr_map[dev]; assert(maps);
    const xSemaphoreHandle sem = rtos[dev].isr_semaphore_handle; assert(sem);

    u32 flags;

    while (1) {
        //if (xSemaphoreTake(sem, portMAX_DELAY) != pdTRUE) continue;
        while (isr_handle_count == port_isr_count) { asm("nop"); }
        flags = port_isr_flags;
        ++isr_handle_count;
        //port_isr_flags &= ~flags;

        for (u8 i = 0; i < ISR_PIN_COUNT; ++i) {
            const u32 pf = 1u << isrs[i].pin;
            if (flags & pf) {
                flags &= ~pf;
                if (maps[i]) maps[i](dev);
            }
        }

        if (flags) {
            cc_dbg("[%u] unhandled: flags=0x%08X", dev, flags);
        }
    }
}*/

static inline void isr_common(sys_port_t port, u32 flags/*GPIO_Type *gpio*/);

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

void isr_task(void)
{
    #ifdef ISR_KINETIS_HAVE_PORTA
        PORT_ISR_TASK(A)
    #endif
    #ifdef ISR_KINETIS_HAVE_PORTB
        PORT_ISR_TASK(B)
    #endif
    #ifdef ISR_KINETIS_HAVE_PORTC
        PORT_ISR_TASK(C)
    #endif
    #ifdef ISR_KINETIS_HAVE_PORTD
        PORT_ISR_TASK(D)
    #endif
    #ifdef ISR_KINETIS_HAVE_PORTE
        PORT_ISR_TASK(E)
    #endif
}

static inline void isr_common(sys_port_t port, u32 flags/*GPIO_Type *gpio*/)
{
    /*bool need_context_switch = false;
    BaseType_t xHigherPriorityTaskWoken;

    const isr_queue_item_t evt = { port, GPIO_GetPinsInterruptFlags(gpio) };

    for (cc_dev_t dev = 0; dev < CC_NUM_DEVICES; ++dev) {
        const interface_t *const iface = &cc_interface[dev];

        for (u8 i = 0; i < ISR_PIN_COUNT; ++i) {
            const isr_config_t *const cfg = &iface->isr[i];

            if (port == cfg->port) {
                const u32 pin_flag = 1u << cfg->pin;

                if (evt.flag & pin_flag) {
                    xQueueSendFromISR(rtos[dev].isr_queue_handle, &evt, &xHigherPriorityTaskWoken);
                    need_context_switch |= xHigherPriorityTaskWoken == pdTRUE;
                    break;
                }
            }
        }
    }

    GPIO_ClearPinsInterruptFlags(gpio, evt.flag);

    if (need_context_switch) taskYIELD(); // portEND_SWITCHING_ISR()?? */

    /*const u32 flags = GPIO_GetPinsInterruptFlags(gpio) & port_isr_mask;

    if (flags) {
        GPIO_ClearPinsInterruptFlags(gpio, flags);
        ++port_isr_count;
        port_isr_flags |= flags;
        //BaseType_t xHigherPriorityTaskWoken;
        //xSemaphoreGiveFromISR(rtos[0].isr_semaphore_handle, &xHigherPriorityTaskWoken);
        //portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

    }*/

    //const u32 flags = GPIO_GetPinsInterruptFlags(gpio);

    for (cc_dev_t dev = CC_DEV_MIN; dev <= CC_DEV_MAX; ++dev) {
        const interface_t *const iface = &cc_interface[dev];

        for (u8 i = 0; i < ISR_PIN_COUNT; ++i) {
            const isr_config_t *const cfg = &iface->isr[i];

            if (port == cfg->port) {
                const u32 pin_flag = 1u << cfg->pin;

                if (flags & pin_flag) {
                    //GPIO_ClearPinsInterruptFlags(gpio, pin_flag);
                    if (isr_map[dev][i]) isr_map[dev][i](dev);
                    return;
                }
            }
        }
    }
}

