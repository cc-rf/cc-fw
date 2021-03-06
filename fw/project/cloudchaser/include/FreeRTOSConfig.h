#pragma once

#include <usr/type.h>

/* Phillip: Additional configs from FreeRTOS.h */
#define configUSE_NEWLIB_REENTRANT          0/*1*/
#define configSUPPORT_STATIC_ALLOCATION     1

/* Phillip: specific task stack sizes and priorities */

#define configMINIMAL_STACK_SIZE    ((unsigned short) (768u >> 2u))
#define TASK_STACK_SIZE_SMALL       configMINIMAL_STACK_SIZE
#define TASK_STACK_SIZE_HUGE        (TASK_STACK_SIZE_SMALL * 3)
#define TASK_STACK_SIZE_LARGE       (TASK_STACK_SIZE_SMALL * 2)
#define TASK_STACK_SIZE_MEDIUM      (TASK_STACK_SIZE_SMALL * 3 / 2)
#define TASK_STACK_SIZE_DEFAULT     TASK_STACK_SIZE_SMALL
#define TASK_STACK_SIZE_TIMER       TASK_STACK_SIZE_MEDIUM

#define TASK_PRIO_MAX               (configMAX_PRIORITIES - 1)
#define TASK_PRIO_MIN               0
#define TASK_PRIO_HIGHEST           (TASK_PRIO_MAX - 1)
#define TASK_PRIO_HIGH              ((TASK_PRIO_DEFAULT + TASK_PRIO_HIGHEST) / 2)
#define TASK_PRIO_DEFAULT           ((TASK_PRIO_MAX - TASK_PRIO_MIN)/2)
#define TASK_PRIO_LOW               ((TASK_PRIO_DEFAULT + TASK_PRIO_LOWEST) / 2)
#define TASK_PRIO_LOWEST            (TASK_PRIO_MIN + 1)

/* Phillip: Additional macros */

#define pdUS_TO_TICKS( xTimeInUs ) ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInUs ) * ( TickType_t ) configTICK_RATE_HZ ) / ( TickType_t ) 1000000 ) )

#define pdSEC_TO_TICKS( xTimeInSec ) ( ( TickType_t ) ( ( TickType_t ) ( xTimeInSec ) * ( TickType_t ) configTICK_RATE_HZ ) )

#define pdTICKS_TO_US( xTicks ) ( ( TickType_t ) ( ( ( TickType_t ) ( xTicks ) * ( TickType_t ) 1000000 ) / ( TickType_t ) configTICK_RATE_HZ ) )

#define pdTICKS_TO_MS( xTicks ) ( ( TickType_t ) ( ( ( TickType_t ) ( xTicks ) * ( TickType_t ) 1000 ) / ( TickType_t ) configTICK_RATE_HZ ) )

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION 0U
#define configUSE_IDLE_HOOK 0U
#define configUSE_TICK_HOOK 0U
#define configUSE_TICKLESS_IDLE 1U
#define configUSE_LPTMR 0U // Mine

#if configUSE_LPTMR
#define configLPTMR_CLOCK_HZ CLOCK_GetFreq(kCLOCK_LpoClk)

#define vPortLptmrIsr LPTMR0_IRQHandler
#define TICKLESS_LPTMR_BASE_PTR LPTMR0
#define TICKLESS_LPTMR_IRQn LPTMR0_IRQn
#endif

#if configUSE_TICKLESS_IDLE && 0
extern void rtos_sleep_pre(uint32_t xExpectedIdleTime) __fast_code;
extern void rtos_sleep_post(uint32_t xExpectedIdleTime) __fast_code;
#define configPRE_SLEEP_PROCESSING(x) rtos_sleep_pre(x)
#define configPOST_SLEEP_PROCESSING(x) rtos_sleep_post(x)
#endif

#define configCPU_CLOCK_HZ (CLOCK_GetCoreSysClkFreq())
#define configTICK_RATE_HZ ((TickType_t) 10000U)
#define configMAX_PRIORITIES (18U)
#define configTOTAL_HEAP_SIZE ((size_t)(171U * 1024U))

#define configMAX_TASK_NAME_LEN (10U)
#define configUSE_TRACE_FACILITY 1U
#define configUSE_16_BIT_TICKS 0U
#define configIDLE_SHOULD_YIELD 1U
#define configUSE_MUTEXES 1U
#define configQUEUE_REGISTRY_SIZE 8U
#define configCHECK_FOR_STACK_OVERFLOW 2U
#define configUSE_RECURSIVE_MUTEXES 1U
#define configUSE_MALLOC_FAILED_HOOK 1U
#define configUSE_APPLICATION_TASK_TAG 0U
#define configUSE_COUNTING_SEMAPHORES 1U
#define configUSE_TIME_SLICING 0U

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 0U
#define configMAX_CO_ROUTINE_PRIORITIES (2U)

/* Software timer definitions. */
#define configUSE_TIMERS 1U
#define configTIMER_TASK_PRIORITY (configMAX_PRIORITIES - 1U)
#define configTIMER_QUEUE_LENGTH 10U
#define configTIMER_TASK_STACK_DEPTH TASK_STACK_SIZE_TIMER

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet 0U
#define INCLUDE_uxTaskPriorityGet 0U
#define INCLUDE_vTaskDelete 1U
#define INCLUDE_vTaskCleanUpResources 1U
#define INCLUDE_vTaskSuspend 1U
#define INCLUDE_vTaskDelayUntil 1U
#define INCLUDE_vTaskDelay 1U
#define INCLUDE_xSemaphoreGetMutexHolder 0U
#define INCLUDE_xEventGroupSetBitFromISR 0U
#define INCLUDE_xTimerPendFunctionCall 0U
#define INCLUDE_uxTaskGetStackHighWaterMark 1U

/* This demo makes use of one or more example stats formatting functions.  These
format the raw data provided by the uxTaskGetSystemState() function in to human
readable ASCII form.  See the notes in the implementation of vTaskList() within
FreeRTOS/Source/tasks.c for limitations. */
#define configUSE_STATS_FORMATTING_FUNCTIONS 0U

/* Run time stats gathering definitions. */
#ifdef __ICCARM__
/* The #ifdef just prevents this C specific syntax from being included in
assembly files. */
void vMainConfigureTimerForRunTimeStats(void);
unsigned long ulMainGetRunTimeCounterValue(void);
#endif
#define configGENERATE_RUN_TIME_STATS 0U
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() // vMainConfigureTimerForRunTimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE()         // ulMainGetRunTimeCounterValue()

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 4 /* 15 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 0x0F

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 2

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
#define configASSERT(x)           \
    if ((x) == 0U)                \
    {                             \
        taskDISABLE_INTERRUPTS(); \
        for (;;)                  \
            ;                     \
    }

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler
