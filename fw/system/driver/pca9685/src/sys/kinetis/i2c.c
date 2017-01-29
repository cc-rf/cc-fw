#include "i2c.h"
#include <fsl_i2c_edma.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdlib.h>
#include <fsl_dma_manager.h>

typedef struct {
    I2C_Type *bus;
    xSemaphoreHandle mtx;
    xSemaphoreHandle sem;
    edma_handle_t edma_handle;
    i2c_master_edma_handle_t handle;

} i2c_t;


void i2c_callback(I2C_Type *base, i2c_master_edma_handle_t *handle, status_t status, void *userData);


i2c_handle_t i2c_init(u8 bus)
{
    i2c_t *i2c = malloc(sizeof(i2c_t)); assert(i2c);
    i2c->bus = ((I2C_Type*[])I2C_BASE_PTRS)[bus];
    i2c->mtx = xSemaphoreCreateMutex(); assert(i2c->mtx);
    i2c->sem = xSemaphoreCreateBinary(); assert(i2c->sem);

    status_t status;

    DMAMGR_Init();

    if ((status = DMAMGR_RequestChannel(kDmaRequestMux0I2C1, DMAMGR_DYNAMIC_ALLOCATE, &i2c->edma_handle))) {
        printf("<i2c> init: DMAMGR_RequestChannel(): status=%li\r\n", status);
        goto _fail;

    } else {
        i2c_master_config_t config;
        I2C_MasterGetDefaultConfig(&config);
        config.baudRate_Bps = 100000U;

        I2C_MasterInit(i2c->bus, &config, CLOCK_GetFreq(I2C1_CLK_SRC));
        I2C_MasterCreateEDMAHandle(i2c->bus, &i2c->handle, i2c_callback, (void *) i2c, &i2c->edma_handle);

        NVIC_SetPriority(((IRQn_Type[]) DMA_CHN_IRQS)[i2c->edma_handle.channel],
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    }

    return (i2c_handle_t)i2c;

    _fail:

    if (i2c) {
        free(i2c);
        i2c = NULL;
    }

    return NULL;
}

void i2c_io(i2c_handle_t handle, i2c_rw_t rw, u8 addr, u8 cmd, size_t size, void *data)
{
    i2c_t *const i2c = (i2c_t *)handle;

    i2c_master_transfer_t xfer = {
            .flags = kI2C_TransferDefaultFlag,
            .slaveAddress = addr,
            .direction = (i2c_direction_t)rw,
            .subaddress = cmd,
            .subaddressSize = sizeof(cmd),
            .data = (u8 *)data,
            .dataSize = size,
    };

    xSemaphoreTake(i2c->mtx, portMAX_DELAY);
    const status_t status = I2C_MasterTransferEDMA(i2c->bus, &i2c->handle, &xfer);

    if (status == kStatus_Success) {
        if (size > 1) {
            xSemaphoreTake(i2c->sem, portMAX_DELAY);
        }
    } else {
        printf("<i2c> fail: st=%li\r\n", status);
    }

    xSemaphoreGive(i2c->mtx);
}

void i2c_callback(I2C_Type *base, i2c_master_edma_handle_t *handle, status_t status, void *userData)
{
    const i2c_t *const i2c = (i2c_t *)userData;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (!uxQueueMessagesWaitingFromISR(i2c->mtx)) {
        xSemaphoreGiveFromISR(i2c->sem, &xHigherPriorityTaskWoken);
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
