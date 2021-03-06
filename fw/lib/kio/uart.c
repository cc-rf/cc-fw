#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <kio/itm.h>
#include <board/trace.h>


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


void uart_write(uart_t const uart, size_t size, const u8 *data)
{
    assert(data > (u8 *)1ul);
    assert(size);

    /*u8 out_buf[len+1];
    memcpy(&out_buf[1], buf, len);
    out_buf[0] = len;*/

    uart_transfer_t xfer = {
            .data = (u8 *)data,
            .dataSize = size
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
        board_trace_f("[uart-tx] fail: status=%li\n", status);

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


size_t uart_read(uart_t const uart, size_t size, u8 *data)
{
    uart_transfer_t xfer = {
            .data = data,
            .dataSize = size
    };

    #if defined(UART_LOCK)
        while (!xSemaphoreTake(uart->rx_mtx, portMAX_DELAY));
    #endif

    #if defined(UART_NOTIFY)
        uart->rx_task = xTaskGetCurrentTaskHandle();
    #else
        uart->rx_pending = true;
    #endif

    uart->recv_size = size;

    #ifdef UART_DMA
        UART_EnableInterrupts(uart->base, kUART_IdleLineInterruptEnable);

        const status_t status = UART_ReceiveEDMA(uart->base, &uart->edma_handle, &xfer);
    #else
        const status_t status = UART_TransferReceiveNonBlocking(uart->base, &uart->handle, &xfer, NULL);
    #endif

    if (status != kStatus_Success) {
        board_trace_f("[uart-rx] fail: status=%li\n", status);

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

    return uart->recv_size;
}


void uart_puts(uart_t const uart, const char *str)
{
    uart_write(uart, strlen(str), (u8 *)str);
}


void uart_putch(uart_t const uart, const char ch)
{
    uart_write(uart, 1u, (const u8 *)&ch);
}


size_t uart_gets(uart_t const uart, char *str, size_t len)
{
    size_t count = 0;

    /*while (len-- && !UART_ReadBlocking(TRACE_UART, (u8*)str++, 1) && str[count])
        ++count;*/

    while (len--) {
        uart_read(uart, 1, (u8 *)str);
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

            case kStatus_UART_FramingError:
            case kStatus_UART_NoiseError:
            case kStatus_UART_ParityError:
                // TODO: See if UART can be reset before overrun, just resetting from here
                //       causes a weird buffer offset issue...
                board_trace_f("[uart] error=%li\n", status);
                break;

            case kStatus_UART_RxHardwareOverrun:
                itm_puts(0, "[uart] overrun\n");

            case kStatus_UART_IdleLineDetected:
                #if defined(UART_NOTIFY)
                    if (uart->rx_task) {
                        uart->recv_size = 0;

                        #if defined(UART_DMA)
                            if (status == kStatus_UART_IdleLineDetected) UART_TransferGetReceiveCountEDMA(uart->base, &uart->edma_handle, (u32 *)&uart->recv_size);
                            UART_TransferAbortReceiveEDMA(uart->base, &uart->edma_handle);
                        #else
                            if (status == kStatus_UART_IdleLineDetected) UART_TransferGetReceiveCount(uart->base, &uart->handle, (u32 *)&uart->recv_size);
                            UART_TransferAbortReceive(uart->base, &uart->handle);
                        #endif

                        TaskHandle_t task = uart->rx_task;
                        uart->rx_task = NULL;

                        xTaskNotifyFromISR(
                                task, status == kStatus_UART_IdleLineDetected ? UART_NOTIFY_SUCCESS : UART_NOTIFY_FAIL,
                                eSetBits, &xHigherPriorityTaskWoken
                        );
                    } else {
                        itm_puts(0, "[uart] no rx pending! (/e)\n");
                    }
                #else
                    if (uart->rx_pending) {
                         uart->recv_size = 0;

                        #if defined(UART_DMA)
                            if (status == kStatus_UART_IdleLineDetected) UART_TransferGetReceiveCountEDMA(uart->base, &uart->edma_handle, (u32 *)&uart->recv_size);
                            UART_TransferAbortReceiveEDMA(uart->base, &uart->edma_handle);
                        #else
                            if (status == kStatus_UART_IdleLineDetected) UART_TransferGetReceiveCount(uart->base, &uart->handle, (u32 *)&uart->recv_size);
                            UART_TransferAbortReceive(uart->base, &uart->handle);
                        #endif

                        uart->rx_pending = false;

                        xSemaphoreGiveFromISR(uart->rx_sem, &xHigherPriorityTaskWoken);
                    } else {
                        itm_puts(0, "[uart] no rx pending! (/e)\n");
                    }
                #endif

                break;

            default:
                board_trace_f("[uart] unknown status=%li\n", status);
                break;
        }

        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
}
