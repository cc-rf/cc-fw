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
static void spi_dma_callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData);
#else
static void spi_irq_callback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData);
#endif

static struct {
    SPI_Type *spi;

    #ifdef CC_SPI_DMA
        dspi_master_edma_handle_t edma_handle;
        edma_handle_t rx_handle;
        edma_handle_t tx_handle;
        edma_handle_t im_handle;
    #else
        dspi_master_handle_t master_handle;
    #endif

    #ifdef CC_SPI_LOCK
        xSemaphoreHandle mtx;
        StaticQueue_t mtx_static;
    #endif

    #ifndef CC_SPI_NOTIFY
        xSemaphoreHandle sem;
        StaticQueue_t sem_static;
    #endif

} master;

void spi_master_init(SPI_Type *spi)
{
    #ifdef CC_SPI_LOCK
        master.mtx = xSemaphoreCreateBinaryStatic(&master.mtx_static);
        xSemaphoreGive(master.mtx);
    #endif

    #ifndef CC_SPI_NOTIFY
        master.sem = xSemaphoreCreateBinaryStatic(&master.sem_static);
    #endif

    dspi_master_config_t spi_config;

    DSPI_MasterGetDefaultConfig(&spi_config);

    spi_config.whichPcs = kDSPI_Pcs0;
    spi_config.ctarConfig.pcsToSckDelayInNanoSec = 1000;
    spi_config.ctarConfig.lastSckToPcsDelayInNanoSec = 1000;
    spi_config.ctarConfig.betweenTransferDelayInNanoSec = 0;
    spi_config.ctarConfig.baudRate = 1000000;

    master.spi = spi;

    DSPI_MasterInit(master.spi, &spi_config, CLOCK_GetBusClkFreq());

    #ifdef CC_SPI_DMA

        dma_request_source_t dreq_rx, dreq_tx;
        status_t status;

        DMAMGR_Init();

        if      (master.spi == SPI0) { dreq_rx = kDmaRequestMux0SPI0Rx; dreq_tx = kDmaRequestMux0SPI0Tx; }
        else if (master.spi == SPI1) { dreq_rx = kDmaRequestMux0SPI1Rx; dreq_tx = kDmaRequestMux0SPI1Tx; }
        else if (master.spi == SPI2) { dreq_rx = kDmaRequestMux0SPI2Rx; dreq_tx = kDmaRequestMux0SPI2Tx; }
        #ifdef SPI3
            else if (master.spi == SPI3) { dreq_rx = kDmaRequestMux0SPI3Rx; dreq_tx = kDmaRequestMux0SPI3Tx; }
        #endif
        #ifdef SPI4
            else if (master.spi == SPI4) { dreq_rx = kDmaRequestMux0SPI4Rx; dreq_tx = kDmaRequestMux0SPI4Tx; }
        #endif
        #ifdef SPI5
            else if (master.spi == SPI5) { dreq_rx = kDmaRequestMux0SPI5Rx; dreq_tx = kDmaRequestMux0SPI5Tx; }
        #endif

        DMAMUX_SetSource(DMAMUX0, (dev*3u)+0u, dreq_rx);
        DMAMUX_EnableChannel(DMAMUX0, (dev*3u)+0u);
        DMAMUX_SetSource(DMAMUX0, (dev*3u)+1u, dreq_tx);
        DMAMUX_EnableChannel(DMAMUX0, (dev*3u)+1u);

        EDMA_CreateHandle(&master.rx_handle, DMA0, (dev*3u)+0u);
        EDMA_CreateHandle(&master.im_handle, DMA0, (dev*3u)+2u);
        EDMA_CreateHandle(&master.tx_handle, DMA0, (dev*3u)+1u);

        NVIC_SetPriority(((IRQn_Type [])DMA_CHN_IRQS)[master.tx_handle.channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
        NVIC_SetPriority(((IRQn_Type [])DMA_CHN_IRQS)[master.rx_handle.channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
        //NVIC_SetPriority(((IRQn_Type [])DMA_CHN_IRQS)[spi[dev].im_handle.channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

        DSPI_MasterTransferCreateHandleEDMA(
                master.spi, &master.edma_handle, spi_dma_callback, (void *) master.sem,
                &master.rx_handle, &master.im_handle, &master.tx_handle
        );

    #else

    #ifndef CC_SPI_NOTIFY
        DSPI_MasterTransferCreateHandle(
                master.spi, &master.master_handle, spi_irq_callback, (void *) master.sem
        );
    #else
        DSPI_MasterTransferCreateHandle(
                master.spi, &master.master_handle, spi_irq_callback, NULL
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

void spi_master_io(size_t size, u8 *tx, u8 *rx)
{
    dspi_transfer_t xfer = {
            .txData = tx,
            .rxData = rx,
            .dataSize = size,
            .configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous
    };


    #ifdef CC_SPI_LOCK
        xSemaphoreTake(master.mtx, portMAX_DELAY);
    #endif

    #ifdef CC_SPI_NOTIFY
        master.master_handle.userData = xTaskGetCurrentTaskHandle();
    #endif

    #ifdef CC_SPI_DMA
        DSPI_MasterTransferEDMA(cfg->spi, &master.edma_handle, &xfer);
    #else
        DSPI_MasterTransferNonBlocking(master.spi, &master.master_handle, &xfer);
    #endif

    #ifndef CC_SPI_NOTIFY
        xSemaphoreTake(master.sem, portMAX_DELAY);
    #else
        while (!xTaskNotifyWait(CC_SPI_NOTIFY, CC_SPI_NOTIFY, NULL, portMAX_DELAY));
    #endif

    #ifdef CC_SPI_LOCK
        xSemaphoreGive(master.mtx);
    #endif
}

#ifdef CC_SPI_DMA
static void spi_dma_callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
#else
static void spi_irq_callback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
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

