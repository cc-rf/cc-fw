#include <usr/type.h>

#include <fsl_dspi.h>
#include <fsl_dspi_edma.h>
#include <fsl_dma_manager.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <itm.h>

//#define CC_SPI_DMA
//#define CC_SPI_LOCK
//#define CC_SPI_NOTIFY   (1u<<29)


#ifdef CC_SPI_DMA
static void spi_dma_callback(SPI_Type *base, dspi_slave_edma_handle_t *handle, status_t status, void *userData);
#else
static void spi_irq_callback(SPI_Type *base, dspi_slave_handle_t *handle, status_t status, void *userData);
#endif

static struct {
    SPI_Type *spi;

    #ifdef CC_SPI_DMA
        dspi_slave_edma_handle_t edma_handle;
        edma_handle_t rx_handle;
        edma_handle_t tx_handle;
    #else
        dspi_slave_handle_t slave_handle;
    #endif

    #ifdef CC_SPI_LOCK
        xSemaphoreHandle mtx;
        StaticQueue_t mtx_static;
    #endif

    #ifndef CC_SPI_NOTIFY
        xSemaphoreHandle sem;
        StaticQueue_t sem_static;
    #endif

} slave;

void spi_slave_init(SPI_Type *spi)
{
    #ifdef CC_SPI_LOCK
        slave.mtx = xSemaphoreCreateBinaryStatic(&slave.mtx_static);
        xSemaphoreGive(slave.mtx);
    #endif

    #ifndef CC_SPI_NOTIFY
        slave.sem = xSemaphoreCreateBinaryStatic(&slave.sem_static);
    #endif

    dspi_slave_config_t spi_config;
    DSPI_SlaveGetDefaultConfig(&spi_config);

    slave.spi = spi;

    DSPI_SlaveInit(slave.spi, &spi_config);

    #ifdef CC_SPI_DMA

        dma_request_source_t dreq_rx, dreq_tx;
        status_t status;

        DMAMGR_Init();

        if      (slave.spi == SPI0) { dreq_rx = kDmaRequestMux0SPI0Rx; dreq_tx = kDmaRequestMux0SPI0Tx; }
        else if (slave.spi == SPI1) { dreq_rx = kDmaRequestMux0SPI1Rx; dreq_tx = kDmaRequestMux0SPI1Tx; }
        else if (slave.spi == SPI2) { dreq_rx = kDmaRequestMux0SPI2Rx; dreq_tx = kDmaRequestMux0SPI2Tx; }
        #ifdef SPI3
            else if (slave.spi == SPI3) { dreq_rx = kDmaRequestMux0SPI3Rx; dreq_tx = kDmaRequestMux0SPI3Tx; }
        #endif
        #ifdef SPI4
            else if (slave.spi == SPI4) { dreq_rx = kDmaRequestMux0SPI4Rx; dreq_tx = kDmaRequestMux0SPI4Tx; }
        #endif
        #ifdef SPI5
            else if (slave.spi == SPI5) { dreq_rx = kDmaRequestMux0SPI5Rx; dreq_tx = kDmaRequestMux0SPI5Tx; }
        #endif

        DMAMUX_SetSource(DMAMUX0, (dev*3u)+0u, dreq_rx);
        DMAMUX_EnableChannel(DMAMUX0, (dev*3u)+0u);
        DMAMUX_SetSource(DMAMUX0, (dev*3u)+1u, dreq_tx);
        DMAMUX_EnableChannel(DMAMUX0, (dev*3u)+1u);

        EDMA_CreateHandle(&slave.rx_handle, DMA0, (dev*3u)+0u);
        EDMA_CreateHandle(&slave.tx_handle, DMA0, (dev*3u)+1u);

        NVIC_SetPriority(((IRQn_Type [])DMA_CHN_IRQS)[slave.tx_handle.channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
        NVIC_SetPriority(((IRQn_Type [])DMA_CHN_IRQS)[slave.rx_handle.channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

        DSPI_SlaveTransferCreateHandleEDMA(
                slave.spi, &slave.edma_handle, spi_dma_callback, (void *) slave.sem,
                &slave.rx_handle, &slave.tx_handle
        );

    #else

    #ifndef CC_SPI_NOTIFY
        DSPI_SlaveTransferCreateHandle(
                slave.spi, &slave.slave_handle, spi_irq_callback, (void *) slave.sem
        );
    #else
        DSPI_SlaveTransferCreateHandle(
                slave.spi, &slave.slave_handle, spi_irq_callback, NULL
        );
    #endif

    IRQn_Type irqn = NotAvail_IRQn;

    if      (spi == SPI0) irqn = SPI0_IRQn;
    else if (spi == SPI1) irqn = SPI1_IRQn;
    else if (spi == SPI2) irqn = SPI2_IRQn;
    #ifdef SPI3
    else if (cfg->spi == SPI3) irqn = SPI3_IRQn;
    #endif
    #ifdef SPI4
    else if (cfg->spi == SPI4) irqn = SPI4_IRQn;
    #endif
    #ifdef SPI5
    else if (cfg->spi == SPI5) irqn = SPI5_IRQn;
    #endif

    if (irqn == NotAvail_IRQn) {
        //cc_dbg("irq for SPI@%p unavailable", spi);
        return;
    }

    NVIC_SetPriority(irqn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    #endif
}

void spi_slave_io(size_t size, u8 *tx, u8 *rx)
{
    dspi_transfer_t xfer = {
            .txData = tx,
            .rxData = rx,
            .dataSize = size,
            .configFlags = kDSPI_SlaveCtar0
    };


    #ifdef CC_SPI_LOCK
        xSemaphoreTake(slave.mtx, portMAX_DELAY);
    #endif

    #ifdef CC_SPI_NOTIFY
        slave.slave_handle.userData = xTaskGetCurrentTaskHandle();
    #endif

    #ifdef CC_SPI_DMA
        DSPI_SlaveTransferEDMA(cfg->spi, &slave.edma_handle, &xfer);
    #else
        DSPI_SlaveTransferNonBlocking(slave.spi, &slave.slave_handle, &xfer);
    #endif

    #ifndef CC_SPI_NOTIFY
        xSemaphoreTake(slave.sem, portMAX_DELAY);
    #else
        while (!xTaskNotifyWait(CC_SPI_NOTIFY, CC_SPI_NOTIFY, NULL, portMAX_DELAY));
    #endif

    #ifdef CC_SPI_LOCK
        xSemaphoreGive(slave.mtx);
    #endif
}

#ifdef CC_SPI_DMA
static void spi_dma_callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
#else
static void spi_irq_callback(SPI_Type *base, dspi_slave_handle_t *handle, status_t status, void *userData)
#endif
{
    BaseType_t xHigherPriorityTaskWoken;

    #ifndef CC_SPI_NOTIFY
        xSemaphoreGiveFromISR((xSemaphoreHandle)userData, &xHigherPriorityTaskWoken);
    #else
        xTaskNotifyFromISR((xTaskHandle)userData, CC_SPI_NOTIFY, eSetBits, &xHigherPriorityTaskWoken);
    #endif

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

