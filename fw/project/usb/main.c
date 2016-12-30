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


#define PFLAG_PORT PORTB
#define PFLAG_GPIO GPIOB
#define PFLAG_PIN  5


extern void vcom_init(void);

static void main_task(void *param);

int main(void)
{
    BOARD_InitPins();
    LED_A_ON();

    BOARD_BootClockRUN();
    LED_B_ON();

    BOARD_InitDebugConsole();
    itm_init();
    printf("<boot>\r\n");


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


    xTaskCreate(main_task, "main", TASK_STACK_SIZE_DEFAULT, NULL, TASK_PRIO_DEFAULT, NULL);

    //vcom_init();
    //printf("<vcom init>\r\n");
    //LED_C_ON();

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

#define FREQ_BASE   905000000
#define FREQ_BW     800000

#define CHAN_COUNT  25

static const u32 freq_base      = FREQ_BASE;
static const u32 freq_side_bw   = FREQ_BW / 2;
static const u32 chan_count     = CHAN_COUNT;
static const u32 chan_time      = 200;//30;

static struct {
    chan_grp_t group;
    chan_inf_t chan[CHAN_COUNT];
    chan_t hop_table[CHAN_COUNT];

} chnl = {
        .group = {
                .dev = 0,
                .freq = {
                        .base = FREQ_BASE,
                        .bw   = FREQ_BW
                },
                .size = CHAN_COUNT
        }
};

static volatile u32 chan_cur = UINT32_MAX;

static inline u32 chan_freq(const u32 chan)
{
    //NOTE: for debug purposes, only use 2 channels
    return (freq_base + freq_side_bw * (1+2*/*chan*/(12+chan%2)));
}

static inline void chan_set(const u32 chan)
{
    if (chan != chan_cur) {
        chan_cur = chan;
        //cc_set_freq(0, chan_freq(chan_cur));

        //NOTE: for debug purposes, only use 3 channels
        //chan_select(&chnl.group, (chan_t) (10 + chan%3));

        chan_select(&chnl.group, (chan_t)chan);
    }
}

static inline void chan_next(void)
{
    chan_set((chan_cur + 1) % chan_count);
}

static void timer_task(xTimerHandle timer __unused)
{

}

#define MSG_LEN 120

typedef struct __packed {
    u8 len;
    u8 chn;
    u8 seq;
    u8 data[MSG_LEN];

} app_pkt_t;

static pit_t xsec_timer_0;
static pit_t xsec_timer;

static inline u32 sync_timestamp(void)
{
    return pit_get_elapsed(xsec_timer);
}

static void main_task(void *param)
{
    (void)param;
    printf("<main task>\r\n");

    //xTimerHandle timer = xTimerCreate(NULL, pdMS_TO_TICKS(100), pdTRUE, NULL, timer_task);

#if 1
    if (nphy_init()) {
        printf("nphy init successful.\r\n");

        if (!cc_cfg_regs(0, CC_CFG_MAC, COUNT_OF(CC_CFG_MAC))) {
            printf("warn: could not configure (mac)\n");
        }

        amp_init(0);

        chan_grp_init(&chnl.group, NULL);
        chan_grp_calibrate(&chnl.group);
        chan_set(0);

        xsec_timer_0 = pit_alloc(&(pit_cfg_t){
                .period = pit_nsec_tick(1000000)
        });

        xsec_timer = pit_chain(xsec_timer_0, &(pit_cfg_t){
                .period = UINT32_MAX
        });

        pit_start(xsec_timer);
        pit_start(xsec_timer_0);

        u32 ticks = 0;
        u32 sync_time = 0;
        u32 remaining = portMAX_DELAY;
        u32 chan_ticks;

        if (!pflag_set()) {
            printf("mode: receive\r\n");

            app_pkt_t *pkt;

            amp_ctrl(0, AMP_LNA, true);
            amp_ctrl(0, AMP_HGM, true);

            while (1) {
                if (sync_time) {
                    ticks = sync_timestamp() - sync_time;
                    remaining = chan_time - (ticks % chan_time);
                    chan_set((ticks / chan_time) % chan_count);
                }

                //printf("rx: [wait] chn=%lu rem=%lu\n", chan_cur, remaining);
                pkt = (app_pkt_t *) nphy_rx(pdMS_TO_TICKS(remaining));

                if (pkt && pkt->len) {
                    if (!sync_time) sync_time = sync_timestamp();
                    LED_D_TOGGLE();
                    //printf("rx: [recv] chn=%lu seq=%u\n", chan_cur, pkt->seq);
                }
            }

        } else {
            printf("mode: transmit\r\n");

            u32 pkt_time;

            app_pkt_t pkt = {.len = /*MSG_LEN + */2, .seq = 0, .chn = (u8) chan_cur, .data = {[0 ... MSG_LEN - 1] = 'a'}};

            amp_ctrl(0, AMP_PA, true);
            amp_ctrl(0, AMP_HGM, true);

            while (1) {

                if (sync_time) {
                    ticks = sync_timestamp() - sync_time;
                    chan_ticks = (ticks % chan_time);
                    remaining = chan_time - /*(ticks % chan_time)*/chan_ticks;
                    pkt_time = cc_get_tx_time(0, pkt.len);

                    if (chan_ticks < 10) {
                        vTaskDelay(pdMS_TO_TICKS(10-chan_ticks));
                        continue;
                    }

                    if (remaining <= pkt_time || remaining < 16) {
                        vTaskDelay(pdMS_TO_TICKS(10+remaining));
                        continue;
                    }

                    chan_set((ticks / chan_time) % chan_count);
                }

                pkt.chn = (u8) chan_cur;
                //ticks = sync_timestamp();
                nphy_tx((cc_pkt_t *) &pkt);
                //ticks = sync_timestamp() - ticks;
                //printf("tx/%lu: seq=%u t=%lu/%lu\r\n", chan_cur, pkt.seq, pkt_time, ticks);

                if (!sync_time) sync_time = sync_timestamp();

                ++pkt.seq;

                pkt.len = (u8)((pkt.len + 1) % (MSG_LEN + 2));
                if (!pkt.len) pkt.len = 2;

                LED_D_TOGGLE();
                vTaskDelay(pdMS_TO_TICKS(/*1137*/7 + (pkt.len % 2)*3 ));
            }

        }


    }
    #endif

    vTaskDelay(portMAX_DELAY);
    vTaskDelete(NULL);
    while (1) {}
}


int _write(int handle, char *buffer, int size)
{
    if (buffer == 0)
        return -1;

    if ((handle != 1) && (handle != 2))
    {
        return -1;
    }

    itm_write(0, (const u8 *)buffer, (size_t)size);

    return size;
}
