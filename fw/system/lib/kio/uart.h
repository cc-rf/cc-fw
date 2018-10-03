#pragma once

#include <kio/uart.h>
#include <fsl_uart.h>
#include <fsl_uart_edma.h>
#include <fsl_dma_manager.h>
#include <fsl_clock.h>
#include <fsl_device_registers.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>


//#define UART_LOCK // TODO: Need separate RX/TX locks for this
//#define UART_DMA
//#define UART_NOTIFY ((1u << 28) | (1u << 27))


#ifndef CONFIG_KIO_UART_COUNT
#error CONFIG_KIO_UART_COUNT undefined
#endif

#define SYSTEM_UARTS        ((UART_Type *[])UART_BASE_PTRS)
#define SYSTEM_UART_COUNT   (sizeof(SYSTEM_UARTS)/sizeof(SYSTEM_UARTS[0]))

#if defined(UART_NOTIFY)
#define UART_NOTIFY_SUCCESS     ((UART_NOTIFY << 1) & UART_NOTIFY)
#define UART_NOTIFY_FAIL        ((UART_NOTIFY >> 1) & UART_NOTIFY)
#endif

typedef struct uart {
    UART_Type *base;

    #ifdef UART_DMA
        uart_edma_handle_t edma_handle;
        edma_handle_t tx_handle;
        edma_handle_t rx_handle;
    #else
        uart_handle_t handle;
    #endif

    #if defined(UART_LOCK)

        xSemaphoreHandle rx_mtx;
        StaticSemaphore_t rx_mtx_static;
        xSemaphoreHandle tx_mtx;
        StaticSemaphore_t tx_mtx_static;

    #endif

    #if !defined(UART_NOTIFY)

        xSemaphoreHandle rx_sem;
        StaticSemaphore_t rx_sem_static;
        xSemaphoreHandle tx_sem;
        StaticSemaphore_t tx_sem_static;

        volatile bool rx_pending;

    #else

        volatile TaskHandle_t rx_task;
        volatile TaskHandle_t tx_task;

    #endif

    volatile size_t recv_size;

} *uart_t;

static inline uart_t uart_mem_alloc(void);
static inline void uart_mem_free(uart_t const uart);


#ifdef K66F18_SERIES

// TODO: Find a way to determine if this/any CPU has separate DMAs here (#4 has only 1, for example)

// NOTE: fsl_clock.h defines UARTn_CLK_SRC

#define SYSTEM_UART_CLOCK_SRCS          ((clock_name_t[]){kCLOCK_CoreSysClk,kCLOCK_CoreSysClk,kCLOCK_BusClk,kCLOCK_BusClk,kCLOCK_BusClk,kCLOCK_BusClk})
#define SYSTEM_UART_TX_DMA_REQ_SRCS     ((dma_request_source_t[]){kDmaRequestMux0UART0Tx,kDmaRequestMux0UART1Tx,kDmaRequestMux0UART2Tx,kDmaRequestMux0UART3Tx,kDmaRequestMux0UART4})
#define SYSTEM_UART_RX_DMA_REQ_SRCS     ((dma_request_source_t[]){kDmaRequestMux0UART0Rx,kDmaRequestMux0UART1Rx,kDmaRequestMux0UART2Rx,kDmaRequestMux0UART3Rx,kDmaRequestMux0UART4})

#else
#error Cannot determine UART clocks or DMA channels for this CPU
#endif


#if CONFIG_KIO_UART_COUNT == 0
#include <malloc.h>

static inline uart_t uart_mem_alloc(void)
{
    return calloc(1, sizeof(struct uart));
}

static inline void uart_mem_free(uart_t const uart)
{
    vPortFree(uart);
}

#elif (CONFIG_KIO_UART_COUNT == -1) || (CONFIG_KIO_UART_COUNT > 0)
#include <string.h>

#if CONFIG_KIO_UART_COUNT == -1
#define UART_COUNT SYSTEM_UART_COUNT
#else
#define UART_COUNT CONFIG_KIO_UART_COUNT
#endif

static struct uart uarts[UART_COUNT] = {{NULL}};

static inline uart_t uart_mem_alloc(void)
{
    for (size_t i = 0; i < UART_COUNT; ++i)
        if (!uarts[i].base) return &uarts[i];

    return NULL;
}

static inline void uart_mem_free(uart_t const uart)
{
    uart->base = NULL;
}

#else
#error CONFIG_KIO_UART_COUNT invalid
#endif
