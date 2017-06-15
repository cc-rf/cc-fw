#pragma once

#include <kio/iic.h>
#include <kio/itm.h>

#include <stdlib.h>

#include <fsl_i2c.h>
#include <fsl_i2c_edma.h>
#include <fsl_dma_manager.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>


#define IIC_DMA
//#define IIC_LOCK
#define IIC_NOTIFY   (1u<<30)
//#define IIC_POLL


typedef struct iic {
    I2C_Type *base;
    u8 bus;

    #ifdef IIC_DMA
        dmamanager_handle_t *dmam_handle;
        i2c_master_edma_handle_t iic_dma_handle;
        edma_handle_t dma_handle;
    #elif !defined(IIC_POLL)
        i2c_master_handle_t master_handle;
    #endif
    #ifdef IIC_LOCK
        xSemaphoreHandle mtx;
        StaticQueue_t mtx_static;
    #endif
    #if !defined(IIC_NOTIFY) && !defined(IIC_POLL)
        xSemaphoreHandle sem;
        StaticQueue_t sem_static;
    #endif

} *iic_t;
