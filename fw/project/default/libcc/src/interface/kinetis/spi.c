#include <cc/spi.h>

#include <FreeRTOS.h>
#include <portmacro.h>

//#include "fsl_dspi_freertos.h"

// TODO: see if CC1200 will accept 8-bit addresses with leading zeroes in upper 8 bits

//static dspi_rtos_handle_t spi;

/*const static dspi_master_config_t spi_config = {
    .whichCtar = kDSPI_Ctar0,
    .ctarConfig = {
        .baudRate = 5000000,
        .bitsPerFrame = 8,
    },
    .whichPcs = kDSPI_Pcs0
};*/


/*const static spi_dev_t spi_dev[CC_NUM_DEVICES] =

#if BOARD_FRDM_K66F
    {
        {SPI0, 0},
    };

#elif BOARD_FRDM_K22F
    {
        {SPI1, 0},
    };

#elif BOARD_TWR_K65F180M
    {
        {SPI2, 1},
    };

#elif BOARD_CLOUDCHASER
    {
        {SPI0, 0},
        {SPI2, 0}
    };

#else
#error unknown board
#endif*/


static dspi_master_config_t spi_config;

void cc_spi_init(cc_dev_t dev)
{
    assert(CC_DEV_VALID(dev));
    const spi_config_t *const cfg = &cc_interface[dev].spi;
    DSPI_MasterGetDefaultConfig(&spi_config);
    spi_config.whichPcs = (dspi_which_pcs_t)(1u << cfg->pcs);
    spi_config.ctarConfig.pcsToSckDelayInNanoSec = 1000;
    spi_config.ctarConfig.lastSckToPcsDelayInNanoSec = 1000;
    spi_config.ctarConfig.betweenTransferDelayInNanoSec = 0;
    spi_config.ctarConfig.baudRate = 7000000;


    // TODO: Make this board-specific (also decide on RTOS usage or not...)
    //DSPI_RTOS_Init(&spi, SPI1, &spi_config, CLOCK_GetBusClkFreq());

    DSPI_MasterInit(cfg->spi, &spi_config, CLOCK_GetBusClkFreq());
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

    //DSPI_RTOS_Transfer(&spi, &xfer);

#if CC_DEBUG_VERBOSE
    cc_dbg_v("flag=0x%x addr=0x%x len=%lu hlen=%u", flag, addr, len, hlen);
    cc_dbg_printf_v("tx="); print_hex(xbuf, xlen);
#endif

    portDISABLE_INTERRUPTS();
    DSPI_MasterTransferBlocking(cfg->spi, &xfer);
    portENABLE_INTERRUPTS();

#if CC_DEBUG_VERBOSE
    //cc_dbg_v("flag=0x%x addr=0x%x len=%lu hlen=%u st=0x%02X", flag, addr, len, hlen, xbuf[0]);
    cc_dbg_printf_v("rx="); print_hex(xbuf, xlen);
#endif



    if (rx && len) {
        memcpy(rx, &xbuf[hlen], len);
    }

    return xbuf[0];
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
