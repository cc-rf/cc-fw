#include <board/board.h>
#include <board/trace.h>

#include <kio/itm.h>
#include <kio/uid.h>
#include <kio/flsh.h>

#include <composite.h>

#include <FreeRTOS.h>
#include <task.h>
#include <fsl_gpio.h>
#include <fsl_port.h>
#include <fsl_lptmr.h>
#include <fsl_tickless_generic.h>

#include "cloudchaser.h"
#include <board/led.h>

#define MAIN_TASK_STACK_SIZE    TASK_STACK_SIZE_MEDIUM


#if BOARD_REVISION <= 1

    #define PFLAG_PORT          PORTB
    #define PFLAG_GPIO          GPIOB
    #define PFLAG_PIN           5

    #define UFLAG1_ON_PORT      PORTA
    #define UFLAG1_ON_GPIO      GPIOA
    #define UFLAG1_ON_PIN       16

    #define UFLAG1_PORT         PORTA
    #define UFLAG1_GPIO         GPIOA
    #define UFLAG1_PIN          17

#else

    #define PFLAG_PORT          PORTD
    #define PFLAG_GPIO          GPIOD
    #define PFLAG_PIN           11

    #define UFLAG1_ON_PORT      PORTB
    #define UFLAG1_ON_GPIO      GPIOB
    #define UFLAG1_ON_PIN       2

    #define UFLAG1_PORT         PORTB
    #define UFLAG1_GPIO         GPIOB
    #define UFLAG1_PIN          3

#endif

#define UFLAG2_ON_PORT      PORTE
#define UFLAG2_ON_GPIO      GPIOE
#define UFLAG2_ON_PIN       0
#define UFLAG2_ON_MUX_ORIG  kPORT_MuxAlt2

#define UFLAG2_PORT         PORTE
#define UFLAG2_GPIO         GPIOE
#define UFLAG2_PIN          4


static void main_task(void *param);
static void pin_flag_init(void);
extern void usb_recv(u8 port, mbuf_t *mbuf) __fast_code;


StackType_t main_task_stack[MAIN_TASK_STACK_SIZE] __fast_data;
StaticTask_t main_task_static __fast_data;

bool __pflag_set, __uflag1_set/*, __uflag2_set*/;


int main(void)
{
    board_init();

    pin_flag_init();

    xTaskCreateStatic(
            main_task, "main", MAIN_TASK_STACK_SIZE, NULL, TASK_PRIO_DEFAULT,
            main_task_stack, &main_task_static
    );

    vTaskStartScheduler();
}


static void main_task(void *param)
{
    (void)param;

    board_rtos_init();

    if (!usb_composite_init(usb_recv)) {
        board_trace("usb: init fail");
        goto _end;
    }

    /*while (1) {
        board_trace_f("hello! t=%lu", pdMS_TO_TICKS(1000));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }*/


    /**
     * SPI interconnect (mcuc) testing
     */

    /*if (pflag_set()) {

        extern void spi_master_init(SPI_Type *spi);
        extern void spi_master_io(size_t size, u8 *tx, u8 *rx);

        spi_master_init(SPI1);
        u8 ch[] = { 6, 'h', 'e', 'l', 'l', 'o', '\0', '\0' };
        u8 r = 0;

        //do {
            spi_master_io(1, &ch[0], &r);
        //} while (r != 0x7e);

        for (int i = 0; i < 100000; ++i) asm("nop");

        spi_master_io(ch[0], &ch[1], &ch[1]);
        board_trace_f("r=0x%x ch=%s", r, &ch[1]);

        //board_trace("done.");
        vTaskDelay(portMAX_DELAY);

    } else {

        extern void spi_slave_init(SPI_Type *spi);
        extern void spi_slave_io(size_t size, u8 *tx, u8 *rx);

        spi_slave_init(SPI1);

        u8 len;
        u8 ch[24];

        while (1) {
            len = 0xab;
            memset(ch, 'o', 24);

            spi_slave_io(1, &len, &len);
            spi_slave_io(len, ch, ch);

            board_trace_f("ch/%u=%s", len, ch);
        }

    }*/

    cloudchaser_main();

    while (1) vTaskDelay(portMAX_DELAY); // TODO: Allow exit from main task?

    _end:
    vTaskDelete(NULL);
}


static void pin_flag_init(void)
{
    const port_pin_config_t port_pin_config = {
            kPORT_PullDown,
            kPORT_FastSlewRate,
            kPORT_PassiveFilterDisable,
            kPORT_OpenDrainDisable,
            kPORT_LowDriveStrength,
            kPORT_MuxAsGpio,
            kPORT_LockRegister,
    };

    const port_pin_config_t port_pin_config_out = {
            kPORT_PullDisable,
            kPORT_SlowSlewRate,
            kPORT_PassiveFilterDisable,
            kPORT_OpenDrainDisable,
            kPORT_LowDriveStrength,
            kPORT_MuxAsGpio,
            kPORT_LockRegister,
    };

    const gpio_pin_config_t gpio_pin_config = {
            .pinDirection = kGPIO_DigitalInput,
            .outputLogic = 0
    };

    const gpio_pin_config_t gpio_pin_config_out = {
            .pinDirection = kGPIO_DigitalOutput,
            .outputLogic = 1
    };

    PORT_SetPinConfig(PFLAG_PORT, PFLAG_PIN, &port_pin_config);
    GPIO_PinInit(PFLAG_GPIO, PFLAG_PIN, &gpio_pin_config);
    __pflag_set = GPIO_ReadPinInput(PFLAG_GPIO, PFLAG_PIN) != 0;

    PORT_SetPinConfig(UFLAG1_ON_PORT, UFLAG1_ON_PIN, &port_pin_config_out);
    PORT_SetPinConfig(UFLAG1_PORT, UFLAG1_PIN, &port_pin_config);
    GPIO_PinInit(UFLAG1_ON_GPIO, UFLAG1_ON_PIN, &gpio_pin_config_out);
    GPIO_PinInit(UFLAG1_GPIO, UFLAG1_PIN, &gpio_pin_config);
    __uflag1_set = GPIO_ReadPinInput(UFLAG1_GPIO, UFLAG1_PIN) != 0;
    GPIO_WritePinOutput(UFLAG1_ON_GPIO, UFLAG1_ON_PIN, 0);

    /*PORT_SetPinConfig(UFLAG2_ON_PORT, UFLAG2_ON_PIN, &port_pin_config_out);
    PORT_SetPinConfig(UFLAG2_PORT, UFLAG2_PIN, &port_pin_config);
    GPIO_PinInit(UFLAG2_ON_GPIO, UFLAG2_ON_PIN, &gpio_pin_config_out);
    GPIO_PinInit(UFLAG2_GPIO, UFLAG2_PIN, &gpio_pin_config);
    __uflag2_set = GPIO_ReadPinInput(UFLAG2_GPIO, UFLAG2_PIN) != 0;
    GPIO_WritePinOutput(UFLAG2_ON_GPIO, UFLAG2_ON_PIN, 0);
    PORT_SetPinMux(UFLAG2_ON_PORT, UFLAG2_ON_PIN, UFLAG2_ON_MUX_ORIG);*/
}

bool pflag_set(void)
{
    return __pflag_set;
}

bool uflag1_set(void)
{
    return __uflag1_set;
}

bool uflag2_set(void)
{
    return false; //__uflag2_set;
}


//// ------


// begin: write.c

/*static inline bool isInterrupt()
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}*/

_READ_WRITE_RETURN_TYPE __used _write(int __fd, const void *__buf, size_t __nbyte)
{
    if (__buf == 0)
        return -1;

    if ((__fd != 1) && (__fd != 2))
    {
        return -1;
    }

    /*BaseType_t xHigherPriorityTaskWokenAll = pdFALSE, xHigherPriorityTaskWoken;
    const bool is_interrupt = isInterrupt();

    if (!is_interrupt) xSemaphoreTake(write_sem, portMAX_DELAY);
    else {
        while (!xSemaphoreTakeFromISR(write_sem, &xHigherPriorityTaskWoken));
        xHigherPriorityTaskWokenAll |= xHigherPriorityTaskWoken;
    }*/

    itm_write(0, (const u8 *) __buf, __nbyte);
    usb_write(0, (u8 *) __buf, __nbyte); // TODO: Map to SERF_USB_PORT or CONSOLE_USB_PORT or both?

    /*if (!is_interrupt) xSemaphoreGive(write_sem);
    else {
        xSemaphoreGiveFromISR(write_sem, &xHigherPriorityTaskWoken);
        xHigherPriorityTaskWokenAll |= xHigherPriorityTaskWoken;
    };

    if (is_interrupt) portEND_SWITCHING_ISR(xHigherPriorityTaskWokenAll)*/

    return __nbyte;
}


// begin: sbrk.c
#include <sys/types.h>
#include <errno.h>

/*!
 * @brief Function to override ARMGCC default function _sbrk
 *
 * _sbrk is called by malloc. ARMGCC default _sbrk compares "SP" register and
 * heap end, if heap end is larger than "SP", then _sbrk returns error and
 * memory allocation failed. This function changes to compare __HeapLimit with
 * heap end.
 */
caddr_t _sbrk(int incr)
{
    extern char end __asm("end");
    extern char heap_limit __asm("__HeapLimit");
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == NULL)
        heap_end = &end;

    prev_heap_end = heap_end;

    if (heap_end + incr > &heap_limit)
    {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;

    return (caddr_t)prev_heap_end;
}


// -
#include "fault.c"
