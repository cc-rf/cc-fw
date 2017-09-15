#include "uart.h"
#include <stdio.h>
#include <stdlib.h>


static void uart_dma_callback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData);

static dmamanager_handle_t *dmaManagerHandle;

uart_t uart_init(const uart_id_t id, const baud_t baud)
{
    assert(id < SYSTEM_UART_COUNT);

    uart_t const uart = uart_mem_alloc();

    assert(uart);
    assert(SYSTEM_UART_TX_DMA_REQ_SRCS[id] != kDmaRequestMux0Disable);
    assert(SYSTEM_UART_RX_DMA_REQ_SRCS[id] != kDmaRequestMux0Disable);

    uart->base = SYSTEM_UARTS[id];

    uart_config_t config;
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = baud;
    config.enableRx = true;
    config.enableTx = true;


    UART_Init(uart->base, &config, CLOCK_GetFreq(SYSTEM_UART_CLOCK_SRCS[id]));

    if (!uart->rx_mtx) uart->rx_mtx = xSemaphoreCreateMutexStatic(&uart->rx_mtx_static);

    if (!uart->rx_sem) uart->rx_sem = xSemaphoreCreateBinaryStatic(&uart->rx_sem_static);

    if (!uart->tx_sem) {
        uart->tx_sem = xSemaphoreCreateBinaryStatic(&uart->tx_sem_static);
        xSemaphoreGive(uart->tx_sem);
    }
    
    dmaManagerHandle = DMAMGR_Handle();

    status_t status;

    // NOTE: See FSL_FEATURE_UART_HAS_SEPARATE_DMA_RX_TX_REQn

    status = DMAMGR_RequestChannel(dmaManagerHandle, SYSTEM_UART_TX_DMA_REQ_SRCS[id], DMAMGR_DYNAMIC_ALLOCATE, &uart->tx_handle);
    assert(status == kStatus_Success);

    status = DMAMGR_RequestChannel(dmaManagerHandle, SYSTEM_UART_RX_DMA_REQ_SRCS[id], DMAMGR_DYNAMIC_ALLOCATE, &uart->rx_handle);
    assert(status == kStatus_Success);

    NVIC_SetPriority(((IRQn_Type[][FSL_FEATURE_EDMA_MODULE_CHANNEL])DMA_CHN_IRQS)[0][uart->tx_handle.channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_SetPriority(((IRQn_Type[][FSL_FEATURE_EDMA_MODULE_CHANNEL])DMA_CHN_IRQS)[0][uart->rx_handle.channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);

    UART_TransferCreateHandleEDMA(
            uart->base, &uart->edma_handle, uart_dma_callback, uart,
            &uart->tx_handle, &uart->rx_handle
    );

    return uart;
}

void uart_free(uart_t const uart)
{
    if (uart->base) {
        DMAMGR_ReleaseChannel(dmaManagerHandle, &uart->rx_handle);
        DMAMGR_ReleaseChannel(dmaManagerHandle, &uart->tx_handle);
        //vSemaphoreDelete(uart->rx_mtx);
        //vSemaphoreDelete(uart->rx_sem);
        //vSemaphoreDelete(uart->tx_sem);
        UART_Deinit(uart->base);
        uart->base = NULL;
    }

    return uart_mem_free(uart);
}

void uart_write(uart_t const uart, const u8 *buf, size_t len)
{
    assert(buf > (u8 *)1ul);
    assert(len);

    uart_transfer_t xfer = {
            .data = malloc(len),
            .dataSize = len
    };

    assert(xfer.data);
    memcpy(xfer.data, (void *)buf, len);

    while (!xSemaphoreTake(uart->tx_sem, portMAX_DELAY)) {
        UART_TransferAbortSendEDMA(uart->base, &uart->edma_handle);
        printf("[uart-tx] timeout: aborting\n");
        if (uart->tx_buf) {
            free((void *)uart->tx_buf);
            uart->tx_buf = NULL;
        }

        xSemaphoreGive(uart->tx_sem);
    }

    uart->tx_buf = xfer.data;
    //if (uart->rx_pending) UART_TransferAbortReceiveEDMA(uart->base, &uart->edma_handle);
    const status_t status = UART_SendEDMA(uart->base, &uart->edma_handle, &xfer);

    if (status != kStatus_Success) {
        printf("[uart-tx] fail: status=%li\n", status);
        xSemaphoreGive(uart->tx_sem);
    }
}

void uart_read(uart_t const uart, u8 *buf, size_t len)
{
    //TODO :Look into UART_TransferGetReceiveCountEDMA to choose the transfer size intelligently

    uart_transfer_t xfer = {
            .data = buf,
            .dataSize = len
    };

    xSemaphoreTake(uart->rx_mtx, portMAX_DELAY);

    uart->rx_pending = true;
    const status_t status = UART_ReceiveEDMA(uart->base, &uart->edma_handle, &xfer);

    if (status == kStatus_Success) {
        xSemaphoreTake(uart->rx_sem, portMAX_DELAY);
    } else {
        printf("[uart-rx] fail: status=%li\n", status);
    }

    xSemaphoreGive(uart->rx_mtx);
}

size_t uart_read_frame(uart_t const uart, u8 **buf)
{
    assert(buf);

    u8 len = 0;

    uart_read(uart, &len, 1);

    if (!len) {
        *buf = NULL;
    } else {
        *buf = malloc(len);
        assert(*buf);
        uart_read(uart, *buf, len);
    }

    return len;
}

size_t uart_readline(uart_t const uart, char **buf)
{
    assert(buf);

    size_t len_max = 60;
    u8 len = 0;
    u8 ch;

    *buf = malloc(len_max);
    assert(*buf);

    while (1) {
        uart_read(uart, &ch, 1);

        if (ch == '\r' || ch == '\n') {
            uart_puts(uart, "\r\n");
            (*buf)[len] = 0;
            break;
        }

        uart_write(uart, &ch, 1);

        (*buf)[len++] = ch;

        if (len >= len_max) {
            len_max *= 2;
            *buf = realloc(*buf, len_max);
            assert(*buf);
        }
    };

    return len;
}

void uart_puts(uart_t const uart, const char *str)
{
    uart_write(uart, (u8 *)str, strlen(str));
}

void uart_putch(uart_t const uart, const char ch)
{
    uart_write(uart, (const u8 *)&ch, 1);
}

size_t uart_gets(uart_t const uart, char *str, size_t len)
{
    size_t count = 0;

    /*while (len-- && !UART_ReadBlocking(TRACE_UART, (u8*)str++, 1) && str[count])
        ++count;*/

    while (len--) {
        uart_read(uart, (u8 *)str, 1);
        if (!*str++) break;
        count++;
    }

    return count;
}

void uart_getch(uart_t const uart, char *ch)
{
    uart_read(uart, (u8 *)ch, 1);
}

static void uart_dma_callback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData)
{
    uart_t const uart = (uart_t)userData;

    if (uart && uart->base) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        switch (status) {
            case kStatus_UART_TxIdle:
                if (uart->tx_buf) {
                    free((void *) uart->tx_buf);
                    uart->tx_buf = NULL;
                    xSemaphoreGiveFromISR(uart->tx_sem, &xHigherPriorityTaskWoken);
                }
                break;

            case kStatus_UART_RxIdle:
                if (uart->rx_pending) {
                    uart->rx_pending = false;
                    xSemaphoreGiveFromISR(uart->rx_sem, &xHigherPriorityTaskWoken);
                } else {
                    printf("[uart] no rx pending!\n");
                }
                break;

            default:
                printf("[uart] unhandled: status=%li\n", status);
                break;
        }

        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
}
