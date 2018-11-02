#include <board/board.h>
#include <usr/type.h>
#include <kio/itm.h>
#include <board/led.h>
#include <fsl_common.h>
#include "clock.h"
#include "pins.h"


void board_boot(void)
{
    InstallIRQHandler(0, 0);

    extern u32 __fast_text_begin[];
    extern u32 __fast_text_end[];
    extern u32 __fast_code_begin[];

    u32 *src = __fast_text_begin;
    u32 *dst = __fast_code_begin;

    while (src < __fast_text_end) {
        *dst++ = *src++;
    }

    boot_pins_init();
    boot_clock_run_hs_oc();

    itm_init();
    itm_puts(0, "<boot>\r\n");

    /*itm_printf(0, "\nclocks:\n  core\t\t\t= %lu\n  bus\t\t\t= %lu\n  flexbus\t\t= %lu\n  flash\t\t\t= %lu\n  pllfllsel\t\t= %lu\n  osc0er\t\t= %lu\n  osc0erundiv\t\t= %lu\n  mcgfixedfreq\t\t= %lu\n  mcginternalref\t= %lu\n  mcgfll\t\t= %lu\n  mcgpll0\t\t= %lu\n  mcgirc48m\t\t= %lu\n  lpo\t\t\t= %lu\n\n",
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
    );*/

    #if configUSE_TICKLESS_IDLE && configUSE_LPTMR
    lptmr_config_t config;
    LPTMR_GetDefaultConfig(&config);
    config.prescalerClockSource = kLPTMR_PrescalerClock_1; // Clock 1 == LPO?
    config.bypassPrescaler = true;

    LPTMR_Init(TICKLESS_LPTMR_BASE_PTR, &config);
    LPTMR_EnableInterrupts(TICKLESS_LPTMR_BASE_PTR, kLPTMR_TimerInterruptEnable);
    #endif
}


void board_rtos_init(void)
{
    led_init();
}
