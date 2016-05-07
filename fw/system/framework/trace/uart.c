#include <uart.h>
#include <string.h>

#include <fsl_uart_edma.h>
#include <fsl_dma_manager.h>
#include <itm.h>
#include <malloc.h>

/**
 * UART0 and UART1 operate off the core system clock, while all others use the bus clock.
 */

#define TRACE_UART              UART0
#define TRACE_UART_CLKSRC       SYS_CLK

#define UART_TX_DMA_CHANNEL 0U
#define UART_RX_DMA_CHANNEL 1U
#define UART_TX_DMA_REQUEST kDmaRequestMux0UART0Tx
#define UART_RX_DMA_REQUEST kDmaRequestMux0UART0Rx


void uart_cb(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData);

static uart_config_t uart_cfg;
static uart_edma_handle_t uart_edma_handle;
static edma_handle_t uart_tx_handle;
//static edma_handle_t uart_rx_handle;

void uart_init(void)
{
    UART_GetDefaultConfig(&uart_cfg);

    uart_cfg.baudRate_Bps = 230400;
    uart_cfg.enableTx = true;
    uart_cfg.enableRx = false;

    UART_Init(TRACE_UART, &uart_cfg, CLOCK_GetFreq(TRACE_UART_CLKSRC));

     // TODO: Guard to check if already initialized

    // TODO: Check status
    DMAMGR_RequestChannel(UART_TX_DMA_REQUEST, UART_TX_DMA_CHANNEL, &uart_tx_handle);
    //DMAMGR_RequestChannel(UART_RX_DMA_REQUEST, UART_RX_DMA_CHANNEL, &uart_rx_handle);

    //NVIC_SetPriority(DMA0_DMA16_IRQn, 0);

    UART_TransferCreateHandleEDMA(TRACE_UART, &uart_edma_handle, uart_cb, NULL, &uart_tx_handle, NULL);
}

void uart_write(const u8 *buf, size_t len)
{
    uart_transfer_t xfer = {
            .data = malloc(len),
            .dataSize = len
    };

    //assert(xfer.data);
    memcpy(xfer.data, (void *)buf, len);

    while (uart_edma_handle.txState) asm("nop"); // need to do this?

    if (uart_edma_handle.userData) {
        free(uart_edma_handle.userData);
    }

    uart_edma_handle.userData = xfer.data;

    UART_SendEDMA(TRACE_UART, &uart_edma_handle, &xfer);
}

void uart_read(u8 *buf, size_t len)
{
    UART_ReadBlocking(TRACE_UART, buf, len);
}

void uart_puts(const char *str)
{
    uart_write((u8 *)str, strlen(str)+1);
}

size_t uart_gets(char *str, size_t len)
{
    size_t count = 0;

    /*while (len-- && !UART_ReadBlocking(TRACE_UART, (u8*)str++, 1) && str[count])
        ++count;*/
    while (len--) {
        if (UART_ReadBlocking(TRACE_UART, (u8*)str, 1) != kStatus_Success) {
            break;
        }

        if (!*str++) break;

        count++;
    }

    return count;
}

void uart_cb(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_UART_TxIdle)
    {
        if (handle && handle->userData) {
            free(handle->userData);
            handle->userData = NULL;
        }
    }
}