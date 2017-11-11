#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <kio/itm.h>

#ifdef UART_DMA
static void uart_dma_callback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData);
#else
static void uart_callback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData);
#endif

#ifdef UART_DMA
static dmamanager_handle_t *dmaManagerHandle;
#endif

uart_t uart_init(const uart_id_t id, const baud_t baud)
{
    assert(id < SYSTEM_UART_COUNT);

    uart_t const uart = uart_mem_alloc();
    status_t status;

    assert(uart);
    //assert(SYSTEM_UART_TX_DMA_REQ_SRCS[id] != kDmaRequestMux0Disable);
    //assert(SYSTEM_UART_RX_DMA_REQ_SRCS[id] != kDmaRequestMux0Disable);

    uart->base = SYSTEM_UARTS[id];

    uart_config_t config;
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = baud;
    config.enableRx = true;
    config.enableTx = true;
    //config.parityMode = kUART_ParityOdd;
    //config.stopBitCount = kUART_TwoStopBit;

    status = UART_Init(uart->base, &config, CLOCK_GetFreq(SYSTEM_UART_CLOCK_SRCS[id]));
    assert(status == kStatus_Success);

    #if defined(UART_LOCK)

        if (!uart->tx_mtx) uart->tx_mtx = xSemaphoreCreateMutexStatic(&uart->tx_mtx_static);
        if (!uart->rx_mtx) uart->rx_mtx = xSemaphoreCreateMutexStatic(&uart->rx_mtx_static);

    #endif

    #if defined(UART_NOTIFY)

        uart->rx_task = NULL;
        uart->tx_task = NULL;

    #else

        if (!uart->rx_sem) uart->rx_sem = xSemaphoreCreateBinaryStatic(&uart->rx_sem_static);
        if (!uart->tx_sem) uart->tx_sem = xSemaphoreCreateBinaryStatic(&uart->tx_sem_static);

    #endif

    #ifdef UART_DMA

    dmaManagerHandle = DMAMGR_Handle();

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

    #else

    UART_TransferCreateHandle(uart->base, &uart->handle, uart_callback, uart);

    const IRQn_Type irqn = ((IRQn_Type [])UART_RX_TX_IRQS)[id];
    NVIC_SetPriority(irqn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    #endif

    return uart;
}

/*void __used UART0_RX_TX_IRQHandler(void)
{
    extern void UART0_RX_TX_DriverIRQHandler(void);
    const static uart_t uart = &uarts[0];
    static u8 in[UART_RX_BUFFER_SIZE];
    static u8 in_size = 0;
    u8 ch;

    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART0))
    {
        in[in_size++] = ch = UART_ReadByte(UART0);

        if (!ch) {
            memcpy(uart->rx, in, in_size);
            uart->rx_size = in_size;
            in_size = 0;

            if (uart->rx_pending) {
                uart->rx_pending = false;
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xSemaphoreGiveFromISR(uart->rx_sem, &xHigherPriorityTaskWoken);
                portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
            } else {
                itm_printf(0, "[uart] no rx pending!\n");
            }
        }

        if (in_size >= UART_RX_BUFFER_SIZE) {
            itm_puts(0, "uart/0: rx buffer overflow\n");
            in_size = 0;
        }

    } else {
        UART0_RX_TX_DriverIRQHandler();
    }
}*/


void uart_free(uart_t const uart)
{
    if (uart->base) {

        #ifdef UART_DMA
            DMAMGR_ReleaseChannel(dmaManagerHandle, &uart->rx_handle);
            DMAMGR_ReleaseChannel(dmaManagerHandle, &uart->tx_handle);
        #else
            // TODO: How to release transfer handle?
        #endif

        UART_Deinit(uart->base);
        uart->base = NULL;
    }

    return uart_mem_free(uart);
}

void uart_write(uart_t const uart, const u8 *buf, u8 len)
{
    assert(buf > (u8 *)1ul);
    assert(len);

    /*u8 out_buf[len+1];
    memcpy(&out_buf[1], buf, len);
    out_buf[0] = len;*/

    uart_transfer_t xfer = {
            .data = buf,
            .dataSize = len
    };

    #if defined(UART_LOCK)
        while (!xSemaphoreTake(uart->tx_mtx, portMAX_DELAY));
    #endif

    // send length first
    //UART_WriteBlocking(uart->base, &len, 1);

    #if defined(UART_NOTIFY)
        _retry:
        uart->tx_task = xTaskGetCurrentTaskHandle();
    #endif

    #ifdef UART_DMA
        const status_t status = UART_SendEDMA(uart->base, &uart->edma_handle, &xfer);
    #else
        //UART_WriteBlocking(uart->base, buf, len);
        const status_t status = UART_TransferSendNonBlocking(uart->base, &uart->handle, &xfer);
    #endif

    if (status != kStatus_Success) {
        itm_printf(0, "[uart-tx] fail: status=%li\n", status);

        #if defined(UART_LOCK)
            xSemaphoreGive(uart->tx_mtx);
        #endif

        return;
    }

    #if defined(UART_NOTIFY)

        u32 notify = 0;

        do {
            xTaskNotifyWait(UART_NOTIFY, UART_NOTIFY, &notify, portMAX_DELAY);

        } while (!(notify & UART_NOTIFY));

        if (notify & UART_NOTIFY_FAIL) {
            goto _retry;
        }

    #else
        xSemaphoreTake(uart->tx_sem, portMAX_DELAY);
    #endif

    #if defined(UART_LOCK)
        xSemaphoreGive(uart->tx_mtx);
    #endif
}

size_t uart_read(uart_t const uart, u8 *buf, size_t len)
{
    //TODO :Look into UART_TransferGetReceiveCountEDMA to choose the transfer size intelligently

    uart_transfer_t xfer = {
            .data = buf,
            .dataSize = len
    };

    #if defined(UART_LOCK)
        while (!xSemaphoreTake(uart->rx_mtx, portMAX_DELAY));
    #endif

    #if defined(UART_NOTIFY)
        uart->rx_task = xTaskGetCurrentTaskHandle();
    #else
        uart->rx_pending = true;
    #endif

    #ifdef UART_DMA
        const status_t status = UART_ReceiveEDMA(uart->base, &uart->edma_handle, &xfer);
    #else
        const status_t status = UART_TransferReceiveNonBlocking(uart->base, &uart->handle, &xfer, NULL);
    #endif

    if (status != kStatus_Success) {
        itm_printf(0, "[uart-rx] fail: status=%li\n", status);

        #if defined(UART_LOCK)
            xSemaphoreGive(uart->rx_mtx);
        #endif

        return 0;
    }

    #if defined(UART_NOTIFY)

        u32 notify = 0;

        do {
            xTaskNotifyWait(UART_NOTIFY, UART_NOTIFY, &notify, portMAX_DELAY);

        } while (!(notify & UART_NOTIFY));

        if (notify & UART_NOTIFY_FAIL) {
            return 0;
        }

    #else
        xSemaphoreTake(uart->rx_sem, portMAX_DELAY);
    #endif

    #if defined(UART_LOCK)
        xSemaphoreGive(uart->rx_mtx);
    #endif

    return len;
}

size_t uart_read_frame(uart_t const uart, serf_t *frame, size_t size)
{
    assert(frame);

    u8 in[size];
    u8 ch = 0;
    size_t in_size = 0;
    size_t frame_size;
    u32 status;

    while (1) {
        // read length first
        /*if (UART_ReadBlocking(uart->base, &ch, 1)) {
            status = UART_GetStatusFlags(uart->base);
            itm_printf(0, "uart: rx-size fail, status=0x%08x\n", status);
            UART_ClearStatusFlags(uart->base, status);
            continue;
        }

        if (!ch) continue;
        else in_size = ch;

        if (!uart_read(uart, in, in_size)) {
            in_size = 0;
            continue;
        }*/

        if (!uart_read(uart, &ch, 1)) {
            in_size = 0; // ?
            continue;
        }

        in[in_size++] = ch;

        if (!ch/*in[in_size - 1]*/) {

            if (in_size > size) {
                itm_printf(0, "[uart-rx] fail: frame too big (%u > %u)\n", in_size, size);
                in_size = 0;
                continue;
            }

            frame_size = serf_decode(in, &in_size, frame, in_size);
            return frame_size;

        } /*else {
            itm_puts(0, "[uart-rx] out of sync\n");

            while (ch) {
                if (UART_ReadBlocking(uart->base, &ch, 1)) {
                    status = UART_GetStatusFlags(uart->base);
                    itm_printf(0, "uart: rx-dump fail, status=0x%08x\n", status);
                    UART_ClearStatusFlags(uart->base, status);
                    continue;
                }
            }

            // TODO: read until zero?
            continue;
        }*/

        if (in_size >= UINT8_MAX) {
            itm_puts(0, "[uart-rx] fail: overflow\n");
            in_size = 0;
        }
    }
}

size_t uart_readline(uart_t const uart, char **buf)
{
    assert(buf);

    size_t len_max = 60;
    u8 len = 0;
    u8 ch;

    *buf = pvPortMalloc(len_max);
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
            *buf = realloc(*buf, len_max); // TODO: Fix this (not currently used)
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

u8 uart_getch(uart_t const uart)
{
    return UART_ReadByte(uart->base);
}

#ifdef UART_DMA
static void uart_dma_callback(UART_Type *base, uart_edma_handle_t *handle, status_t status, void *userData)
#else
static void uart_callback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
#endif
{
    uart_t const uart = (uart_t)userData;

    if (uart && uart->base) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        switch (status) {
            case kStatus_UART_TxIdle:
                #if defined(UART_NOTIFY)
                    if (uart->tx_task) {
                        TaskHandle_t task = uart->tx_task;
                        uart->tx_task = NULL;
                        xTaskNotifyFromISR(task, UART_NOTIFY_SUCCESS, eSetBits, &xHigherPriorityTaskWoken);
                    } else {
                        itm_puts(0, "[uart] no tx pending!\n");
                    }
                #else
                    xSemaphoreGiveFromISR(uart->tx_sem, &xHigherPriorityTaskWoken);
                #endif
                break;

            case kStatus_UART_RxIdle:
                #if defined(UART_NOTIFY)
                    if (uart->rx_task) {
                        TaskHandle_t task = uart->rx_task;
                        uart->rx_task = NULL;
                        xTaskNotifyFromISR(task, UART_NOTIFY_SUCCESS, eSetBits, &xHigherPriorityTaskWoken);
                    } else {
                        itm_puts(0, "[uart] no rx pending!\n");
                    }
                #else
                    if (uart->rx_pending) {
                        uart->rx_pending = false;
                        xSemaphoreGiveFromISR(uart->rx_sem, &xHigherPriorityTaskWoken);
                    } else {
                        itm_puts(0, "[uart] no rx pending!\n");
                    }
                #endif
                break;

            case kStatus_UART_RxHardwareOverrun:
                itm_puts(0, "[uart] overflow\n");
                UART_ClearStatusFlags(uart->base, kUART_RxOverrunFlag);

                #if defined(UART_DMA)
                UART_TransferAbortReceiveEDMA(uart->base, &uart->edma_handle);
                #else
                UART_TransferAbortReceive(uart->base, &uart->handle);
                #endif

                #if defined(UART_NOTIFY)
                    if (uart->rx_task) {
                        TaskHandle_t task = uart->rx_task;
                        uart->rx_task = NULL;
                        xTaskNotifyFromISR(task, UART_NOTIFY_FAIL, eSetBits, &xHigherPriorityTaskWoken);
                    } else {
                        itm_puts(0, "[uart] no rx pending! (/e)\n");
                    }
                #else
                    if (uart->rx_pending) {
                        uart->rx_pending = false;
                        xSemaphoreGiveFromISR(uart->rx_sem, &xHigherPriorityTaskWoken);
                    } else {
                        itm_puts(0, "[uart] no rx pending! (/e)\n");
                    }
                #endif

                break;

            default:
                itm_printf(0, "[uart] unhandled: status=%li\n", status);
                break;
        }

        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
}
