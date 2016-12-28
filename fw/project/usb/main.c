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

    //xTaskCreate(main_task, "main", TASK_STACK_SIZE_DEFAULT, NULL, TASK_PRIO_DEFAULT, NULL);

    vcom_init();
    printf("<vcom init>\r\n");
    LED_C_ON();

    // Theoretically this will make sure the sub-priority on all interrupt configs is zero.
    //   Not sure it's actually really needed or what it does in the long run.
    //   See comment in FreeRTOS port.c.
    //NVIC_SetPriorityGrouping(0);

    vTaskStartScheduler();
}

static void main_task(void *param)
{
    (void)param;
    printf("<main task>\r\n");

    #if 0
    if (nphy_init()) {
        printf("nphy init successful.\r\n");

        cc_set_freq(0, 915000000);

        #if 0

        while (1) {
            cc_pkt_t *pkt = nphy_rx();

            if (!pkt->len) {
                printf("rx: <null>\r\n");
            } else {
                printf("rx:");

                for (u8 i = 0; i < pkt->len; ++i)
                    printf(" %02X", pkt->data[i]);

                printf("\r\n");
            }
        }

        #else

        static cc_pkt_t pkt = { .len = 1, .data = { '!' }};

        while (1) {
            nphy_tx(&pkt);
            printf("tx: sent\r\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        #endif
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
