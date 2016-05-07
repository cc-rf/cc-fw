#include <cc/spi.h>

#include <FreeRTOS.h>
#include <portmacro.h>
#include <cc/interface/kinetis/kinetis.h>
#include <malloc.h>

//#include "fsl_dspi_freertos.h"

// TODO: see if CC1200 will accept 8-bit addresses with leading zeroes in upper 8 bits

static void spi_cb(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData);

static dspi_master_handle_t spi_handle[CC_NUM_DEVICES];

void cc_spi_init(cc_dev_t dev)
{
    assert(CC_DEV_VALID(dev));
    const spi_config_t *const cfg = &cc_interface[dev].spi;
    dspi_master_config_t spi_config;
    DSPI_MasterGetDefaultConfig(&spi_config);
    spi_config.whichPcs = (dspi_which_pcs_t)(1u << cfg->pcs);
    spi_config.ctarConfig.pcsToSckDelayInNanoSec = 1000;
    spi_config.ctarConfig.lastSckToPcsDelayInNanoSec = 1000;
    spi_config.ctarConfig.betweenTransferDelayInNanoSec = 0;
    spi_config.ctarConfig.baudRate = 8000000;


    // TODO: Make this board-specific (also decide on RTOS usage or not...)
    //DSPI_RTOS_Init(&spi, SPI1, &spi_config, CLOCK_GetBusClkFreq());

    DSPI_MasterInit(cfg->spi, &spi_config, CLOCK_GetBusClkFreq());
    DSPI_MasterTransferCreateHandle(cfg->spi, &spi_handle[dev], spi_cb, NULL);
    //DMAMGR_RequestChannel(UART_TX_DMA_REQUEST, UART_TX_DMA_CHANNEL, &uart_tx_handle);


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

    NVIC_SetPriority(irqn, 0);
    //EnableIRQ(irqn);

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
    //u8 xbuf[xlen];
    u8 *xbuf = malloc(xlen);
    assert(xbuf);

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

    //DSPI_RTOS_Transfer(&spi, &xfer);

#if CC_DEBUG_VERBOSE
    cc_dbg_v("flag=0x%x addr=0x%x len=%lu hlen=%u", flag, addr, len, hlen);
    cc_dbg_printf_v("tx="); print_hex(xbuf, xlen);
#endif

    //portDISABLE_INTERRUPTS();
    //DSPI_MasterTransferBlocking(cfg->spi, &xfer);
    //portENABLE_INTERRUPTS();

    //while ((volatile void *)spi_handle[dev].userData) asm("nop");
    //spi_handle[dev].userData = xbuf;
    DSPI_MasterTransferNonBlocking(cfg->spi, &spi_handle[dev], &xfer);

    do {
        //asm("nop");
    } while (spi_handle[dev].state != kStatus_Success);

#if CC_DEBUG_VERBOSE
    //cc_dbg_v("flag=0x%x addr=0x%x len=%lu hlen=%u st=0x%02X", flag, addr, len, hlen, xbuf[0]);
    cc_dbg_printf_v("rx="); print_hex(xbuf, xlen);
#endif

    if (rx && len) {
        memcpy(rx, &xbuf[hlen], len);
    }

    u8 st = xbuf[0];
    free(xbuf);
    return st;
}

static void spi_cb(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
{
    //assert(status == kStatus_Success);
    /*if (status == kStatus_Success) {
        if (handle && handle->userData) {
            handle->userData = NULL;
        }
    }*/
}









#include <cc/cc1200.h>

/* Always transfer 2-byte header by making the first byte a NOP strobe.
 * TODO: look at timing improvements from not varying header size.
 * TODO: look at making op-specific versions to increase performance.
 * */
/*
u8 cc_spi_io_e(u8 flag, u16 addr, u8 *tx, u8 *rx, u32 len)
{
    const u8 ahi = (u8) ((addr >> 8) & 0xFF);
    const u8 alo = (u8) (addr & 0xFF);

    const u8 hlen = 2;
    const u32 xlen = len + hlen;
    u8 xbuf[xlen];
    u8 xbuf_rx[xlen];

    if (!ahi) {
        xbuf[0] = CC1200_SNOP;
        xbuf[1] = flag | alo;
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
            .rxData = xbuf_rx,
            .dataSize = xlen,
            .configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcsContinuous
    };

    //DSPI_RTOS_Transfer(&spi, &xfer);
    printf("cc_spi_io_e: flag=0x%x addr=0x%x len=%lu hlen=%u \r\n", flag, addr, len, hlen);
    printf("cc_spi_io_e: tx="); print_hex(xbuf, xlen);
    status_t res = DSPI_MasterTransferBlocking(SPI_BUS, &xfer);
    printf("cc_spi_io_e: rx="); print_hex(xbuf_rx, xlen);
    printf("cc_spi_io_e: res=%lu\r\n", res);


    if (rx && len) {
        memcpy(rx, &xbuf_rx[hlen], len);
    }

    return xbuf_rx[0];
}*/
