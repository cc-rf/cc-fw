#include <board/board.h>
#include <board/trace.h>
#include <usr/type.h>
#include <kio/itm.h>
#include <kio/flsh.h>
#include <board/led.h>
#include <fsl_common.h>
#include "clock.h"
#include "pins.h"


void board_boot(void)
{
    extern u32 __fast_text_begin[];
    extern u32 __fast_text_end[];
    extern u32 __fast_code_begin[];

    wcpy(__fast_text_begin, __fast_text_end, __fast_code_begin);

    /*
    extern u32 __fast_data_rom_begin[];
    extern u32 __fast_data_rom_end[];
    extern u32 __fast_data_begin[];

    wcpy(__fast_data_rom_begin, __fast_data_rom_end, __fast_data_begin);
    */

    InstallIRQHandler(0, 0);

    boot_pins_init();
    boot_clock_run_hs_oc();

    itm_init();
    itm_puts(0, "<boot>\r\n");

    /*board_trace_f("\nclocks:\n  core\t\t\t= %lu\n  bus\t\t\t= %lu\n  flexbus\t\t= %lu\n  flash\t\t\t= %lu\n  pllfllsel\t\t= %lu\n  osc0er\t\t= %lu\n  osc0erundiv\t\t= %lu\n  mcgfixedfreq\t\t= %lu\n  mcginternalref\t= %lu\n  mcgfll\t\t= %lu\n  mcgpll0\t\t= %lu\n  mcgirc48m\t\t= %lu\n  lpo\t\t\t= %lu\n\n",
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

void vApplicationStackOverflowHook(TaskHandle_t xTask, const char *pcTaskName)
{
    board_trace_f("stack overflow in task %p '%s'", xTask, pcTaskName);
    while (1) asm("nop");
}

void vApplicationMallocFailedHook(void)
{
    board_trace("malloc failed!");
    while (1) asm("nop");
}

StaticTask_t xIdleTaskTCB;
StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

static StaticTask_t xTimerTaskTCB __fast_data;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH] __fast_data;

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

#if configUSE_TICKLESS_IDLE

#if configUSE_LPTMR

LPTMR_Type *vPortGetLptrmBase(void)
{
    return TICKLESS_LPTMR_BASE_PTR;
}

IRQn_Type vPortGetLptmrIrqn(void)
{
    return TICKLESS_LPTMR_IRQn;
}

#endif

void rtos_sleep_pre(TickType_t xExpectedIdleTime)
{
    //board_trace_f("sleep-pre %lu", xExpectedIdleTime);
}

void rtos_sleep_post(TickType_t xExpectedIdleTime)
{
    //board_trace_f("sleep-post %lu", xExpectedIdleTime);
}

#endif
