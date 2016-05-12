#include <uart.h>
#include <string.h>

#include <fsl_uart_edma.h>
#include <fsl_dma_manager.h>
#include <itm.h>
#include <malloc.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>

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
static edma_handle_t uart_rx_handle;

static struct {
    xSemaphoreHandle rx_mut;
    xSemaphoreHandle rx_sem;
    xSemaphoreHandle tx_sem;
    volatile u8 *tx_buf;
    volatile bool rx_pending;

} uart_rtos;

void uart_init(void)
{
    memset(&uart_rtos, 0, sizeof(uart_rtos));
    uart_rtos.rx_mut = xSemaphoreCreateMutex(); assert(uart_rtos.rx_mut != NULL);
    uart_rtos.rx_sem = xSemaphoreCreateBinary(); assert(uart_rtos.rx_sem != NULL);
    uart_rtos.tx_sem = xSemaphoreCreateBinary(); assert(uart_rtos.tx_sem != NULL);
    xSemaphoreGive(uart_rtos.tx_sem);

    UART_GetDefaultConfig(&uart_cfg);

    uart_cfg.baudRate_Bps = 230400;
    uart_cfg.enableTx = true;
    uart_cfg.enableRx = true;

    UART_Init(TRACE_UART, &uart_cfg, CLOCK_GetFreq(TRACE_UART_CLKSRC));

     // TODO: Guard to check if already initialized

    // TODO: Check status
    DMAMGR_RequestChannel(UART_TX_DMA_REQUEST, UART_TX_DMA_CHANNEL, &uart_tx_handle);
    DMAMGR_RequestChannel(UART_RX_DMA_REQUEST, UART_RX_DMA_CHANNEL, &uart_rx_handle);

    // TODO: Dynamically determine the DMA IRQn
    NVIC_SetPriority(DMA0_DMA16_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(DMA1_DMA17_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    UART_TransferCreateHandleEDMA(TRACE_UART, &uart_edma_handle, uart_cb, NULL,
                                  &uart_tx_handle, &uart_rx_handle);
}

void uart_write(const u8 *buf, size_t len)
{
    uart_transfer_t xfer = {
            .data = malloc(len),
            .dataSize = len
    };

    assert(xfer.data);
    memcpy(xfer.data, (void *)buf, len);

    while (xSemaphoreTake(uart_rtos.tx_sem, portMAX_DELAY) != pdTRUE);
    uart_rtos.tx_buf = xfer.data;
    UART_SendEDMA(TRACE_UART, &uart_edma_handle, &xfer);
}

void uart_read(u8 *buf, size_t len)
{
    uart_transfer_t xfer = {
            .data = buf,
            .dataSize = len
    };

    while (xSemaphoreTake(uart_rtos.rx_mut, portMAX_DELAY) != pdTRUE);
    uart_rtos.rx_pending = true;
    UART_ReceiveEDMA(TRACE_UART, &uart_edma_handle, &xfer);
    while (xSemaphoreTake(uart_rtos.rx_sem, portMAX_DELAY) != pdTRUE);
    xSemaphoreGive(uart_rtos.rx_mut);
}

size_t uart_read_frame(u8 **buf)
{
    assert(buf);

    u8 len = 0;

    uart_read(&len, 1);

    if (!len) {
        *buf = NULL;
    } else {
        *buf = malloc(len);
        assert(*buf);
        uart_read(*buf, len);
    }

    return len;
}

void uart_puts(const char *str)
{
    uart_write((u8 *)str, strlen(str)+1);
}

void uart_putch(const char ch)
{
    uart_write((const u8 *)&ch, 1);
}

size_t uart_gets(char *str, size_t len)
{
    size_t count = 0;

    /*while (len-- && !UART_ReadBlocking(TRACE_UART, (u8*)str++, 1) && str[count])
        ++count;*/

    while (len--) {
        uart_read((u8 *)str, 1);
        if (!*str++) break;
        count++;
    }

    return count;
}

void uart_getch(char *ch)
{
    uart_read((u8 *)ch, 1);
}

void uart_cb(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (status) {
        case kStatus_UART_TxIdle:
            if (uart_rtos.tx_buf) {
                free((void *)uart_rtos.tx_buf);
                uart_rtos.tx_buf = NULL;
                xSemaphoreGiveFromISR(uart_rtos.tx_sem, &xHigherPriorityTaskWoken);
            }
            break;
        case kStatus_UART_RxIdle:
            if (uart_rtos.rx_pending) {
                uart_rtos.rx_pending = false;
                xSemaphoreGiveFromISR(uart_rtos.rx_sem, &xHigherPriorityTaskWoken);
            }
            break;

        default:
            assert(false);
            break;
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}