#include "iic.h"


#ifdef IIC_DMA
static void iic_dma_callback(I2C_Type *base, i2c_master_edma_handle_t *handle, status_t status, void *userData);
#elif !defined(IIC_POLL)
static void iic_irq_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData);
#endif


static struct iic iics[IIC_MAX_BUS_COUNT] = { [ 0 ... (IIC_MAX_BUS_COUNT-1)] .base = NULL };


iic_t iic_init(u8 bus, u32 baud)
{
    iic_t iic = NULL;

    for (u8 i = 0; i < IIC_MAX_BUS_COUNT; ++i) {
        if (!iics[i].base) {
            iic = &iics[i];
            break;
        } else if (bus == iics[i].bus) {
            return &iics[i];
        }
    }

    if (!iic) return NULL;

    iic->base = ((I2C_Type *[])I2C_BASE_PTRS)[bus];
    iic->bus = bus;

    #ifdef IIC_LOCK
        iic->mtx = xSemaphoreCreateBinaryStatic(&iic->mtx_static);
        xSemaphoreGive(iic->mtx);
    #endif

    #if !defined(IIC_NOTIFY) && !defined(IIC_POLL)
        iic->sem = xSemaphoreCreateBinaryStatic(&iic->sem_static);
    #endif

    i2c_master_config_t iic_config;
    I2C_MasterGetDefaultConfig(&iic_config);

    iic_config.baudRate_Bps = baud;

    I2C_MasterInit(iic->base, &iic_config, CLOCK_GetBusClkFreq());

    #ifdef IIC_DMA

        dma_request_source_t dreq = ((dma_request_source_t []){kDmaRequestMux0I2C0, kDmaRequestMux0I2C1, kDmaRequestMux0I2C2, kDmaRequestMux0I2C3})[iic->bus];
        status_t status;

        iic->dmam_handle = DMAMGR_Handle();

        status = DMAMGR_RequestChannel(iic->dmam_handle, dreq, DMAMGR_DYNAMIC_ALLOCATE, &iic->dma_handle);
        assert(status == kStatus_Success);

        NVIC_SetPriority(((IRQn_Type [][FSL_FEATURE_EDMA_MODULE_CHANNEL])DMA_CHN_IRQS)[0][iic->dma_handle.channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);

        #ifdef IIC_NOTIFY
            I2C_MasterCreateEDMAHandle(
                    iic->base, &iic->iic_dma_handle, iic_dma_callback, NULL, &iic->dma_handle
            );
        #else
            I2C_MasterCreateEDMAHandle(
                    iic->base, &iic->iic_dma_handle, iic_dma_callback, (void *) iic->sem, &iic->dma_handle
            );
        #endif

    #else

        #ifdef IIC_NOTIFY
            I2C_MasterTransferCreateHandle(iic->base, &iic->master_handle, iic_irq_callback, NULL);
        #elif !defined(IIC_POLL)
            I2C_MasterTransferCreateHandle(iic->base, &iic->master_handle, iic_irq_callback, (void *)iic);
        #endif

        #ifndef IIC_POLL
            const IRQn_Type irqn = ((IRQn_Type [])I2C_IRQS)[iic->bus];
            NVIC_SetPriority(irqn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
        #endif

    #endif

    return iic;
}

/*static void print_hex(u8 *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        cc_dbg_printf_v("0x%02X ", buf[i]);
    }

    cc_dbg_printf_v("\r\n");
}*/


status_t iic_io(iic_t iic, iic_rw_t rw, u8 addr, u8 cmd, void *data, size_t size)
{
    status_t status = kStatus_Success;
    assert(iic);

    i2c_master_transfer_t xfer = {
            .flags = kI2C_TransferDefaultFlag,
            .slaveAddress = addr,
            .direction = (i2c_direction_t)rw,
            .subaddress = cmd,
            .subaddressSize = sizeof(cmd),
            .data = (u8 *)data,
            .dataSize = size,
    };

    #ifdef IIC_LOCK
        xSemaphoreTake(spi[dev].mtx, portMAX_DELAY);
    #endif

    #ifdef IIC_NOTIFY
        iic->master_handle.userData = xTaskGetCurrentTaskHandle();
    #endif

    #if defined(IIC_DMA)
        status = I2C_MasterTransferEDMA(iic->base, &iic->iic_dma_handle, &xfer);
    #elif defined(IIC_POLL)
        status = I2C_MasterTransferBlocking(iic->base, &xfer);
    #else
        status = I2C_MasterTransferNonBlocking(iic->base, &iic->master_handle, &xfer);
    #endif

    if (status == kStatus_Success) {
        #ifdef IIC_DMA
            if (size > 1) {
        #endif

        #ifdef IIC_NOTIFY

            u32 notify = 0;

            do {
                if (!xTaskNotifyWait(0, IIC_NOTIFY, &notify, pdMS_TO_TICKS(100))) {
                    #ifdef IIC_DMA
                    I2C_MasterTransferAbortEDMA(iic->base, &iic->iic_dma_handle);
                    #else
                    I2C_MasterTransferAbort(iic->base, &iic->master_handle);
                    #endif
                    notify = IIC_NOTIFY_FAIL;
                    break;
                }

            } while (!(notify & IIC_NOTIFY));

            if (notify & IIC_NOTIFY_FAIL) {
                status = kStatus_Fail;
            }

        #elif !defined(IIC_POLL)

            if (!xSemaphoreTake(iic->sem, pdMS_TO_TICKS(100))) {
                status = kStatus_Fail;
                #ifdef IIC_DMA
                    I2C_MasterTransferAbortEDMA(iic->base, &iic->iic_dma_handle);
                #else
                    I2C_MasterTransferAbort(iic->base, &iic->master_handle);
                #endif
            } else {
                status = iic->status;
            }

        #endif

        #ifdef IIC_DMA
            }
        #endif
    }

    #ifdef IIC_LOCK
        xSemaphoreGive(iic->mtx);
    #endif

    return status;
}


#if !defined(IIC_POLL)

#ifdef IIC_DMA
static void iic_dma_callback(I2C_Type *base, i2c_master_edma_handle_t *handle, status_t status, void *userData)
#else
static void iic_irq_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
#endif
{
    BaseType_t xHigherPriorityTaskWoken;

    #ifndef IIC_NOTIFY
        iic_t iic = (iic_t) userData;
        iic->status = status;
        xSemaphoreGiveFromISR(iic->sem, &xHigherPriorityTaskWoken);
    #else
        xTaskNotifyFromISR((xTaskHandle)userData, status ? IIC_NOTIFY_FAIL : IIC_NOTIFY_SUCCESS, eSetBits, &xHigherPriorityTaskWoken);
    #endif

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

#endif
