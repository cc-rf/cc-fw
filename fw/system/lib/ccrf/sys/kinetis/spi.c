#include "kinetis.h"
#include "sys/spi.h"

#include <fsl_dspi_edma.h>
#include <fsl_dma_manager.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

//#define CC_SPI_DMA
//#define CC_SPI_LOCK
//#define CC_SPI_NOTIFY   (1u<<29)
#define CC_SPI_POLL

// TODO: see if CC1200 will accept 8-bit addresses with leading zeroes in upper 8 bits

#ifdef CC_SPI_DMA
static void spi_dma_callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData);
#elif !defined(CC_SPI_POLL)
static void spi_irq_callback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData);
#endif

#if (defined(CC_SPI_DMA) && defined(CC_SPI_POLL)) || (defined(CC_SPI_NOTIFY) && defined(CC_SPI_POLL))
#error Bad SPI feature combination!
#endif

static struct ccrf_spi {
    SPI_Type *bus;
    u32 pcs;
    
    #ifdef CC_SPI_DMA
    dmamanager_handle_t *dmam_handle;
    dspi_master_edma_handle_t edma_handle;
    edma_handle_t rx_handle;
    edma_handle_t tx_handle;
    edma_handle_t im_handle;
    #elif !defined(CC_SPI_POLL)
    dspi_master_handle_t master_handle;
    #endif
    #ifdef CC_SPI_LOCK
    xSemaphoreHandle mtx;
    StaticQueue_t mtx_static;
    #endif
    #if !defined(CC_SPI_NOTIFY) && !defined(CC_SPI_POLL)
    xSemaphoreHandle sem;
    StaticQueue_t sem_static;
    #endif

} spis[CCRF_CONFIG_RDIO_COUNT] __used __ccrf_data;

ccrf_spi_t ccrf_spi_init(u8 rdio_id)
{
    const spi_config_t *const cfg = &cc_interface[rdio_id].spi;
    ccrf_spi_t spi = &spis[rdio_id];
    
    spi->bus = cfg->spi;
    spi->pcs = cfg->pcs;

    #ifdef CC_SPI_LOCK
        spi->mtx = xSemaphoreCreateBinaryStatic(&spi->mtx_static);
        xSemaphoreGive(spi->mtx);
    #endif

    #if !defined(CC_SPI_NOTIFY) && !defined(CC_SPI_POLL)
        spi->sem = xSemaphoreCreateBinaryStatic(&spi->sem_static);
    #endif

    dspi_master_config_t spi_config;

    DSPI_MasterGetDefaultConfig(&spi_config);

    spi_config.whichCtar = kDSPI_Ctar0;
    spi_config.whichPcs = (dspi_which_pcs_t)(1u << cfg->pcs);
    spi_config.ctarConfig.pcsToSckDelayInNanoSec = 0;
    spi_config.ctarConfig.lastSckToPcsDelayInNanoSec = 1000;
    spi_config.ctarConfig.betweenTransferDelayInNanoSec = 0;
    spi_config.ctarConfig.baudRate = 16000000;

    DSPI_MasterInit(cfg->spi, &spi_config, CLOCK_GetBusClkFreq());

    spi_config.whichCtar = kDSPI_Ctar1;
    spi_config.ctarConfig.baudRate = 8000000;

    DSPI_MasterInit(cfg->spi, &spi_config, CLOCK_GetBusClkFreq());

    #ifdef CC_SPI_DMA

        dma_request_source_t dreq_rx = kDmaRequestMux0Disable, dreq_tx = kDmaRequestMux0Disable;
        status_t status;

        if      (cfg->spi == SPI0) { dreq_rx = kDmaRequestMux0SPI0Rx; dreq_tx = kDmaRequestMux0SPI0Tx; }
        else if (cfg->spi == SPI1) { dreq_rx = kDmaRequestMux0SPI1Rx; dreq_tx = kDmaRequestMux0SPI1Tx; }
        else if (cfg->spi == SPI2) { dreq_rx = kDmaRequestMux0SPI2Rx; dreq_tx = kDmaRequestMux0SPI2Tx; }
        #ifdef SPI3
            else if (cfg->spi == SPI3) { dreq_rx = kDmaRequestMux0SPI3Rx; dreq_tx = kDmaRequestMux0SPI3Tx; }
        #endif
        #ifdef SPI4
            else if (cfg->spi == SPI4) { dreq_rx = kDmaRequestMux0SPI4Rx; dreq_tx = kDmaRequestMux0SPI4Tx; }
        #endif
        #ifdef SPI5
            else if (cfg->spi == SPI5) { dreq_rx = kDmaRequestMux0SPI5Rx; dreq_tx = kDmaRequestMux0SPI5Tx; }
        #endif

        spi->dmam_handle = DMAMGR_Handle();

        status = DMAMGR_RequestChannel(spi->dmam_handle, dreq_rx, DMAMGR_DYNAMIC_ALLOCATE, &spi->rx_handle);
        assert(status == kStatus_Success);

        status = DMAMGR_RequestChannel(spi->dmam_handle, kDmaRequestMux0Disable, DMAMGR_DYNAMIC_ALLOCATE, &spi->im_handle);
        assert(status == kStatus_Success);

        status = DMAMGR_RequestChannel(spi->dmam_handle, dreq_tx, DMAMGR_DYNAMIC_ALLOCATE, &spi->tx_handle);
        assert(status == kStatus_Success);

        /*DMAMUX_SetSource(DMAMUX0, (rdio_id(rdio)*3u)+0u, dreq_rx);
        DMAMUX_EnableChannel(DMAMUX0, (rdio_id(rdio)*3u)+0u);
        DMAMUX_SetSource(DMAMUX0, (rdio_id(rdio)*3u)+1u, dreq_tx);
        DMAMUX_EnableChannel(DMAMUX0, (rdio_id(rdio)*3u)+1u);

        EDMA_CreateHandle(&spi->rx_handle, DMA0, (rdio_id(rdio)*3u)+0u);
        EDMA_CreateHandle(&spi->im_handle, DMA0, (rdio_id(rdio)*3u)+2u);
        EDMA_CreateHandle(&spi->tx_handle, DMA0, (rdio_id(rdio)*3u)+1u);*/

        NVIC_SetPriority(((IRQn_Type [][FSL_FEATURE_EDMA_MODULE_CHANNEL])DMA_CHN_IRQS)[0][spi->tx_handle.channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
        NVIC_SetPriority(((IRQn_Type [][FSL_FEATURE_EDMA_MODULE_CHANNEL])DMA_CHN_IRQS)[0][spi->rx_handle.channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

        #ifdef CC_SPI_NOTIFY
            DSPI_MasterTransferCreateHandleEDMA(
                    cfg->spi, &spi->edma_handle, spi_dma_callback, (void *) NULL,
                    &spi->rx_handle, &spi->im_handle, &spi->tx_handle
            );
        #else
            DSPI_MasterTransferCreateHandleEDMA(
                    cfg->spi, &spi->edma_handle, spi_dma_callback, (void *) spi->sem,
                    &spi->rx_handle, &spi->im_handle, &spi->tx_handle
            );
        #endif

    #else

        #ifdef CC_SPI_NOTIFY
            DSPI_MasterTransferCreateHandle(
                    cfg->spi, &spi->master_handle, spi_irq_callback, NULL
            );
        #elif !defined(CC_SPI_POLL)
            DSPI_MasterTransferCreateHandle(
                    cfg->spi, &spi->master_handle, spi_irq_callback, (void *) spi->sem
            );
        #endif

        #ifndef CC_SPI_POLL
            IRQn_Type irqn = NotAvail_IRQn;

            if      (cfg->spi == SPI0) irqn = SPI0_IRQn;
            else if (cfg->spi == SPI1) irqn = SPI1_IRQn;
            else if (cfg->spi == SPI2) irqn = SPI2_IRQn;
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
                return NULL;
            }

            NVIC_SetPriority(irqn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
        #endif

    #endif

    return spi;
}

/*static void print_hex(u8 *buf, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        cc_dbg_printf_v("0x%02X ", buf[i]);
    }

    cc_dbg_printf_v("\r\n");
}*/

u8 ccrf_spi_io(ccrf_spi_t spi, u8 flag, u16 addr, u8 *tx, u8 *rx, size_t size)
{
    const u8 ahi = (u8) ((addr >> 8) & 0xFF);
    const u8 alo = (u8) (addr & 0xFF);

    const u8 hlen = ahi ? (u8)2 : (u8)1;
    const u32 xlen = size + hlen;
    u8 xbuf[xlen];

    if (hlen == 1) {
        xbuf[0] = flag | alo;
    } else {
        xbuf[0] = flag | ahi;
        xbuf[1] = alo;
    }

    if (size) {
        if (tx) {
            memcpy(&xbuf[hlen], tx, size);
        } else {
            //memset(&xbuf[hlen], 0, size);
        }
    }

    dspi_transfer_t xfer = {
            .txData = xbuf,
            .rxData = xbuf,
            .dataSize = xlen,
            .configFlags = (addr < 0x2F00 ? kDSPI_MasterCtar0 : kDSPI_MasterCtar1)
                    | (spi->pcs << DSPI_MASTER_PCS_SHIFT) | kDSPI_MasterPcsContinuous
    };

    #ifdef CC_SPI_DMA
        #ifdef CC_SPI_LOCK
            xSemaphoreTake(spi->mtx, portMAX_DELAY);
        #endif

        #ifdef CC_SPI_NOTIFY
            spi->edma_handle.userData = xTaskGetCurrentTaskHandle();
        #endif

        DSPI_MasterTransferEDMA(spi->bus, &spi->edma_handle, &xfer);

        #ifdef CC_SPI_NOTIFY
            u32 notify;

            do {
                if (!xTaskNotifyWait(CC_SPI_NOTIFY, CC_SPI_NOTIFY, &notify, pdMS_TO_TICKS(100)))
                    notify = 0;

            } while (!(notify & CC_SPI_NOTIFY));
        #else
            xSemaphoreTake(spi->sem, portMAX_DELAY);
        #endif

        #ifdef CC_SPI_LOCK
            xSemaphoreGive(spi->mtx);
        #endif

    #else
        #ifdef CC_SPI_LOCK
            xSemaphoreTake(spi->mtx, portMAX_DELAY);
        #endif

        #ifdef CC_SPI_NOTIFY
            spi->master_handle.userData = xTaskGetCurrentTaskHandle();
        #endif

        #ifdef CC_SPI_POLL
            taskENTER_CRITICAL();
            DSPI_MasterTransferBlocking(spi->bus, &xfer);
            taskEXIT_CRITICAL();
        #else
            DSPI_MasterTransferNonBlocking(spi->bus, &spi->master_handle, &xfer);
        #endif

        #ifdef CC_SPI_NOTIFY

            u32 notify;

            do {
                if (!xTaskNotifyWait(CC_SPI_NOTIFY, CC_SPI_NOTIFY, &notify, pdMS_TO_TICKS(100)))
                    notify = 0;

            } while (!(notify & CC_SPI_NOTIFY));

        #elif !defined(CC_SPI_POLL)
            xSemaphoreTake(spi->sem, portMAX_DELAY);
        #endif

        #ifdef CC_SPI_LOCK
            xSemaphoreGive(spi->mtx);
        #endif

    #endif

    if (rx && size) {
        memcpy(rx, &xbuf[hlen], size);
    }

    return xbuf[0];
}

#ifdef CC_SPI_DMA

static void spi_dma_callback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken;

    #ifndef CC_SPI_NOTIFY
        xSemaphoreGiveFromISR((xSemaphoreHandle)userData, &xHigherPriorityTaskWoken);
    #else
        xTaskNotifyFromISR((xTaskHandle)userData, CC_SPI_NOTIFY, eSetBits, &xHigherPriorityTaskWoken);
    #endif

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

#elif !defined(CC_SPI_POLL)

static void spi_irq_callback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken;

    #ifndef CC_SPI_NOTIFY
        xSemaphoreGiveFromISR((xSemaphoreHandle)userData, &xHigherPriorityTaskWoken);
    #else
        xTaskNotifyFromISR((xTaskHandle)userData, CC_SPI_NOTIFY, eSetBits, &xHigherPriorityTaskWoken);
    #endif

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

#endif
