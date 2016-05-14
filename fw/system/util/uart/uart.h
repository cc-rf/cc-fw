#pragma once

#include <util/uart.h>
#include <fsl_uart.h>
#include <fsl_uart_edma.h>
#include <fsl_dma_manager.h>
#include <fsl_clock.h>
#include <fsl_device_registers.h>

#include <FreeRTOS.h>
#include <semphr.h>


#ifndef CONFIG_UTIL_UART_COUNT
#error CONFIG_UTIL_UART_COUNT undefined
#endif

#define SYSTEM_UARTS        ((UART_Type *[])UART_BASE_PTRS)
#define SYSTEM_UART_COUNT   (sizeof(SYSTEM_UARTS)/sizeof(SYSTEM_UARTS[0]))


typedef struct uart {
    UART_Type *base;
    uart_edma_handle_t edma_handle;
    edma_handle_t tx_handle;
    edma_handle_t rx_handle;

    xSemaphoreHandle rx_mtx;
    xSemaphoreHandle rx_sem;
    xSemaphoreHandle tx_sem;

    volatile bool rx_pending;
    volatile u8 *tx_buf;

} *uart_t;

static inline uart_t uart_mem_alloc(void);
static inline void uart_mem_free(uart_t const uart);


#ifdef K66F18_SERIES

// TODO: Find a way to determine if this/any CPU has separate DMAs here (#4 has only 1, for example)

#define SYSTEM_UART_CLOCK_SRCS          ((clock_name_t[]){kCLOCK_CoreSysClk,kCLOCK_CoreSysClk,kCLOCK_BusClk,kCLOCK_BusClk,kCLOCK_BusClk,kCLOCK_BusClk})
#define SYSTEM_UART_TX_DMA_REQ_SRCS     ((dma_request_source_t[]){kDmaRequestMux0UART0Tx,kDmaRequestMux0UART1Tx,kDmaRequestMux0UART2Tx,kDmaRequestMux0UART3Tx,kDmaRequestMux0UART4})
#define SYSTEM_UART_RX_DMA_REQ_SRCS     ((dma_request_source_t[]){kDmaRequestMux0UART0Rx,kDmaRequestMux0UART1Rx,kDmaRequestMux0UART2Rx,kDmaRequestMux0UART3Rx,kDmaRequestMux0UART4})

#else
#error Cannot determine UART clocks or DMA channels for this CPU
#endif


#if CONFIG_UTIL_UART_COUNT == 0
#include <malloc.h>

static inline uart_t uart_mem_alloc(void)
{
    return calloc(1, sizeof(struct uart));
}

static inline void uart_mem_free(uart_t const uart)
{
    free(uart);
}

#elif (CONFIG_UTIL_UART_COUNT == -1) || (CONFIG_UTIL_UART_COUNT > 0)
#include <string.h>

#if CONFIG_UTIL_UART_COUNT == -1
#define UART_COUNT SYSTEM_UART_COUNT
#else
#define UART_COUNT CONFIG_UTIL_UART_COUNT
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
#error CONFIG_UTIL_UART_COUNT invalid
#endif