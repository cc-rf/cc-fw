#include "kinetis.h"
#include <cc/spi.h>

#include <fsl_dspi_edma.h>
#include <fsl_dma_manager.h>

#include <FreeRTOS.h>
#include <semphr.h>


// TODO: see if CC1200 will accept 8-bit addresses with leading zeroes in upper 8 bits

static void spi_irq_callback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData);

static struct {
#ifdef CC_SPI_DMA
    dspi_master_edma_handle_t edma_handle;
    edma_handle_t rx_handle;
    edma_handle_t tx_handle;
#else
    dspi_master_handle_t master_handle;
#endif
    xSemaphoreHandle mtx;
    xSemaphoreHandle sem;

} spi[CC_NUM_DEVICES];

void cc_spi_init(cc_dev_t dev)
{
    assert(CC_DEV_VALID(dev));
    const spi_config_t *const cfg = &cc_interface[dev].spi;

    spi[dev].mtx = xSemaphoreCreateMutex(); assert(spi[dev].mtx != NULL);
    spi[dev].sem = xSemaphoreCreateBinary(); assert(spi[dev].sem != NULL);

    dspi_master_config_t spi_config;
    DSPI_MasterGetDefaultConfig(&spi_config);
    spi_config.whichPcs = (dspi_which_pcs_t)(1u << cfg->pcs);
    spi_config.ctarConfig.pcsToSckDelayInNanoSec = 1000;
    spi_config.ctarConfig.lastSckToPcsDelayInNanoSec = 1000;
    spi_config.ctarConfig.betweenTransferDelayInNanoSec = 0;
    spi_config.ctarConfig.baudRate = 8000000;

    DSPI_MasterInit(cfg->spi, &spi_config, CLOCK_GetBusClkFreq());

#ifdef CC_SPI_DMA
    dma_request_source_t dreq_rx, dreq_tx;
    status_t status;

    DMAMGR_Init();

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

    status = DMAMGR_RequestChannel(dreq_rx, DMAMGR_DYNAMIC_ALLOCATE, &spi[dev].tx_handle);
    assert(status == kStatus_Success);

    status = DMAMGR_RequestChannel(dreq_tx, DMAMGR_DYNAMIC_ALLOCATE, &spi[dev].rx_handle);
    assert(status == kStatus_Success);

    NVIC_SetPriority(((IRQn_Type [])DMA_CHN_IRQS)[spi[dev].tx_handle.channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(((IRQn_Type [])DMA_CHN_IRQS)[spi[dev].rx_handle.channel], configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    DSPI_MasterTransferCreateHandleEDMA(
            cfg->spi, &spi[dev].edma_handle, spi_dma_callback, (void *)dev,
    );
#else

#endif

    DSPI_MasterTransferCreateHandle(
            cfg->spi, &spi[dev].master_handle, spi_irq_callback, (void *) spi[dev].sem
    );

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
        cc_dbg("[%u] irq for SPI@%p unavailable", dev, cfg->spi);
        return;
    }

    NVIC_SetPriority(irqn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    cc_dbg_v("[%u] initialized", dev);
}

static void print_hex(u8 *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        cc_dbg_printf_v("0x%02X ", buf[i]);
    }

    cc_dbg_printf_v("\r\n");
}

u8 cc_spi_io(cc_dev_t dev, u8 flag, u16 addr, u8 *tx, u8 *rx, u32 len)
{
    assert(CC_DEV_VALID(dev));
    const spi_config_t *const cfg = &cc_interface[dev].spi;

    assert((len && (tx || rx)) || (!len && !tx && !rx));

    const u8 ahi = (u8) ((addr >> 8) & 0xFF);
    const u8 alo = (u8) (addr & 0xFF);

    const u8 hlen = ahi ? (u8)2 : (u8)1;
    const u32 xlen = len + hlen;
    u8 xbuf[xlen];

    if (hlen == 1) {
        xbuf[0] = flag | alo;
    } else {
        xbuf[0] = flag | ahi;
        xbuf[1] = alo;
    }

    if (len) {
        if (tx) {
            memcpy(&xbuf[hlen], tx, len);
        } else {
            memset(&xbuf[hlen], 0, len);
        }
    }

    dspi_transfer_t xfer = {
            .txData = xbuf,
            .rxData = xbuf,
            .dataSize = xlen,
            .configFlags = kDSPI_MasterCtar0 | (cfg->pcs << DSPI_MASTER_PCS_SHIFT) | kDSPI_MasterPcsContinuous
    };

#if CC_DEBUG_VERBOSE
    cc_dbg_v("flag=0x%x addr=0x%x len=%lu hlen=%u", flag, addr, len, hlen);
    cc_dbg_printf_v("tx="); print_hex(xbuf, xlen);
#endif

    xSemaphoreTake(spi[dev].mtx, portMAX_DELAY);
    DSPI_MasterTransferNonBlocking(cfg->spi, &spi[dev].master_handle, &xfer);
    xSemaphoreTake(spi[dev].sem, portMAX_DELAY);
    xSemaphoreGive(spi[dev].mtx);

    if (rx && len) {
        memcpy(rx, &xbuf[hlen], len);
    }

    u8 st = xbuf[0];
    return st;
}

static void spi_irq_callback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR((xSemaphoreHandle)userData, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
