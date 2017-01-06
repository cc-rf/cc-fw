#include <board.h>
#include <pin_mux.h>

#include <FreeRTOS.h>
#include <task.h>

#include <stdio.h>
#include <itm.h>


#include <cc/spi.h>
#include <cc/io.h>
#include <cc/nphy.h>
#include <cc/freq.h>
#include <cc/cc1200.h>
#include <cc/isr.h>
#include <cc/tmr.h>
#include <cc/mac.h>
#include <cc/chan.h>
#include <core_cm4.h>
#include <cc/sys/kinetis/pit.h>
#include <cc/type.h>
#include <timers.h>
#include <cc/amp.h>
#include <cc/cfg.h>
#include <fsl_port.h>
#include <semphr.h>


#define PFLAG_PORT PORTB
#define PFLAG_GPIO GPIOB
#define PFLAG_PIN  5


extern void vcom_init(void);

static void main_task(void *param);

static SemaphoreHandle_t write_sem = NULL;

int main(void)
{
    BOARD_InitPins();
    LED_A_ON();

    BOARD_BootClockRUN();
    LED_B_ON();

    write_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(write_sem);

    BOARD_InitDebugConsole();
    itm_init();
    printf("<boot>\r\n");

    /*printf("how about a fault.\r\n");
    *(int *)(0xfffefafe) = 42;
    while (1) {};*/


    printf("\nclocks:\n  core\t\t\t= %lu\n  bus\t\t\t= %lu\n  flexbus\t\t= %lu\n  flash\t\t\t= %lu\n  pllfllsel\t\t= %lu\n  osc0er\t\t= %lu\n  osc0erundiv\t\t= %lu\n  mcgfixedfreq\t\t= %lu\n  mcginternalref\t= %lu\n  mcgfll\t\t= %lu\n  mcgpll0\t\t= %lu\n  mcgirc48m\t\t= %lu\n  lpo\t\t\t= %lu\n\n",
       CLOCK_GetFreq(kCLOCK_CoreSysClk),
       CLOCK_GetFreq(kCLOCK_BusClk),
       CLOCK_GetFreq(kCLOCK_FlexBusClk),
       CLOCK_GetFreq(kCLOCK_FlashClk),
       CLOCK_GetFreq(kCLOCK_PllFllSelClk),
       CLOCK_GetFreq(kCLOCK_Osc0ErClk),
       CLOCK_GetFreq(kCLOCK_Osc0ErClkUndiv),
       CLOCK_GetFreq(kCLOCK_McgFixedFreqClk),
       CLOCK_GetFreq(kCLOCK_McgInternalRefClk),
       CLOCK_GetFreq(kCLOCK_McgFllClk),
       CLOCK_GetFreq(kCLOCK_McgPll0Clk),
       CLOCK_GetFreq(kCLOCK_McgIrc48MClk),
       CLOCK_GetFreq(kCLOCK_LpoClk)
    );

    // setup the rx/tx flag input pin
    const port_pin_config_t port_pin_config = {
            kPORT_PullDown,
            kPORT_FastSlewRate,
            kPORT_PassiveFilterDisable,
            kPORT_OpenDrainDisable,
            kPORT_LowDriveStrength,
            kPORT_MuxAsGpio,
            kPORT_LockRegister,
    };

    PORT_SetPinConfig(PFLAG_PORT, PFLAG_PIN, &port_pin_config);

    const gpio_pin_config_t gpio_pin_config = {
            .pinDirection = kGPIO_DigitalInput,
            .outputLogic = 0
    };

    GPIO_PinInit(PFLAG_GPIO, PFLAG_PIN, &gpio_pin_config);


    xTaskCreate(main_task, "main", TASK_STACK_SIZE_DEFAULT, NULL, TASK_PRIO_HIGHEST, NULL);

    vcom_init();
    printf("<vcom init>\r\n");
    LED_C_ON();

    // Theoretically this will make sure the sub-priority on all interrupt configs is zero.
    //   Not sure it's actually really needed or what it does in the long run.
    //   See comment in FreeRTOS port.c.
    //NVIC_SetPriorityGrouping(0);

    vTaskStartScheduler();
}

static bool pflag_set(void)
{
    return GPIO_ReadPinInput(PFLAG_GPIO, PFLAG_PIN) != 0;
}

static const struct cc_cfg_reg CC_CFG_MAC[] = {
        {CC1200_SETTLING_CFG, 0x3}, // Defaults except never auto calibrate
};


#define MSG_LEN 38

static pit_t xsec_timer_0;
static pit_t xsec_timer;

u32 sync_timestamp(void)
{
    return pit_get_elapsed(xsec_timer);
}

static void handle_rx(u8 flag, u8 *buf, u8 len);

static void main_task(void *param)
{
    (void)param;
    printf("<main task>\r\n");

    //xTimerHandle timer = xTimerCreate(NULL, pdMS_TO_TICKS(100), pdTRUE, NULL, timer_task);

    #if 1

    amp_init(0);

    xsec_timer_0 = pit_alloc(&(pit_cfg_t){
            .period = pit_nsec_tick(1000000)
    });

    xsec_timer = pit_chain(xsec_timer_0, &(pit_cfg_t){
            .period = UINT32_MAX
    });

    pit_start(xsec_timer);
    pit_start(xsec_timer_0);

    amp_ctrl(0, AMP_LNA, true);
    amp_ctrl(0, AMP_PA, true);
    amp_ctrl(0, AMP_HGM, true);

    if (nphy_init((nphy_rx_t)handle_rx, pflag_set())) {
        printf("nphy init successful.\r\n");

        /*if (!cc_cfg_regs(0, CC_CFG_MAC, COUNT_OF(CC_CFG_MAC))) {
            printf("warn: could not configure (mac)\n");
        }*/

        if (!pflag_set()) {
            printf("mode: receive\r\n");

            vTaskDelay(portMAX_DELAY);

        } else {
            printf("mode: transmit\r\n");

            u8 data[MSG_LEN] = {'a'};

            u32 start_time = sync_timestamp();
            u32 sum_lengths = 0;
            u32 num_packets = 0;
            u32 time_diff;

            while (1) {
                //pkt.chn = (u8) chan_cur;
                nphy_tx(0, data, MSG_LEN);
                //printf("tx/%lu: seq=%u\r\n", chan_cur, pkt.seq);

                sum_lengths += MSG_LEN;// + 3 + /*being generous: 2 sync, 2 preamble*/4;
                num_packets++;
                time_diff = sync_timestamp() - start_time;

                if (time_diff >= 1000) {
                    printf("tx: rate = %lu bps\t\t\tpkts = %lu\r\n", (1000 * (sum_lengths * 8)) / time_diff, num_packets);
                    num_packets = 0;
                    sum_lengths = 0;
                    start_time = sync_timestamp();
                }

                //const u32 pkt_time = cc_get_tx_time(0, pkt.len);

                //pkt.len = (u8)((pkt.len + 1) % (MSG_LEN + 2));
                //if (!pkt.len) pkt.len = 2;

                LED_D_TOGGLE();
                //vTaskDelay(pdMS_TO_TICKS(/*1137*/20 + (pkt.len % 2)*3 ));
                //vTaskDelay(pdMS_TO_TICKS(pkt_time*3));
            }

        }


    }

    #endif

    vTaskDelay(portMAX_DELAY);
    vTaskDelete(NULL);
    while (1) {}
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, const char *pcTaskName)
{
    printf("stack overflow in task '%s'\r\n", pcTaskName);
}


static u32 start_time = 0;
static u32 sum_lengths = 0;
static u32 num_packets = 0;
static u32 time_diff;

static void handle_rx(u8 flag, u8 *buf, u8 len)
{
    (void)flag;
    (void)buf;

    sum_lengths += len;// + 3 + /*being generous: 2 sync, 2 preamble*/4;
    num_packets++;
    if (!start_time) start_time = sync_timestamp();
    time_diff = sync_timestamp() - start_time;

    if (time_diff >= 1000) {
        printf("rx: rate = %lu bps\t\t\tpkts = %lu\r\n", (1000 * (sum_lengths * 8)) / time_diff, num_packets);
        num_packets = 0;
        sum_lengths = 0;
        start_time = 0;
    }

    //printf("\t\t\t\t\trx: seq=%u len=%u t=%lu\r\n", pkt->seq, pkt->len, sync_timestamp());

    LED_D_TOGGLE();
    //if (!pflag_set()) nphy_tx((pkt_t *)pkt);
}


extern void usb_write(char *buf, size_t len);

static inline bool isInterrupt()
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}


int _write(int handle, char *buffer, int size)
{
    if (buffer == 0)
        return -1;

    if ((handle != 1) && (handle != 2))
    {
        return -1;
    }

    BaseType_t xHigherPriorityTaskWokenAll = pdFALSE, xHigherPriorityTaskWoken;
    const bool is_interrupt = isInterrupt();

    if (!is_interrupt) xSemaphoreTake(write_sem, portMAX_DELAY);
    else {
        while (!xSemaphoreTakeFromISR(write_sem, &xHigherPriorityTaskWoken));
        xHigherPriorityTaskWokenAll |= xHigherPriorityTaskWoken;
    }

    itm_write(0, (const u8 *)buffer, (size_t)size);
    usb_write(buffer, (size_t)size);

    if (!is_interrupt) xSemaphoreGive(write_sem);
    else {
        xSemaphoreGiveFromISR(write_sem, &xHigherPriorityTaskWoken);
        xHigherPriorityTaskWokenAll |= xHigherPriorityTaskWoken;
    };

    if (is_interrupt) portEND_SWITCHING_ISR(xHigherPriorityTaskWokenAll)

    return size;
}

#include "fault.c"

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
