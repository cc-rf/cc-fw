#include <cc/phy.h>
#include <cc/io.h>
#include <cc/cfg.h>

#include <cc/spi.h>
#include <alloca.h>
#include <string.h>
#include <assert.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <cc/sys/kinetis/pit.h>
#include <timers.h>
#include <malloc.h>
#include <cc/freq.h>


#include <cc/sys/kinetis/isrd.h>
#include <fsl_port.h>


static void isr_mcu_wake(void);
static void cca_setup(void);
static void cca_run(void);
static void cca_end(void);

static const struct cc_cfg_reg CC_CFG_PHY[] = {
        {CC1200_IOCFG3, CC1200_IOCFG_GPIO_CFG_HW0},
        {CC1200_IOCFG2, CC1200_IOCFG_GPIO_CFG_HW0},
        {CC1200_IOCFG1, CC1200_IOCFG_GPIO_CFG_HIGHZ},
        {CC1200_IOCFG0, CC1200_IOCFG_GPIO_CFG_HW0},

        /* NOTE: Enabling CC1200_RFEND_CFG1_RX_TIME_QUAL_M when using a RX timeout means that
         * there will be times when the radio stays in RX due to a valid CS or PQT but will hang there
         * regardless of whether a packet was received. It seems to linger on the channel potentially
         * forever despite the lack of a sync word detection.
         * For the timeout to always trigger an interrupt, RX_TIME_QUAL must be zero.
         * */
        {CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RXOFF_MODE_IDLE | (0x7<<1) /*| CC1200_RFEND_CFG1_RX_TIME_QUAL_M*/},
        {CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_IDLE | CC1200_RFEND_CFG0_TERM_ON_BAD_PACKET_EN /*| 1*/},

        // These may not necessarily be part of the phy handling

        //{CC1200_PREAMBLE_CFG1, /*0xD*/0x4 << 2},
        //{CC1200_SYNC_CFG1,  0x09 | CC1200_SYNC_CFG1_SYNC_MODE_16/* changed from 11 */},
        {CC1200_PKT_CFG2,   CC1200_PKT_CFG2_CCA_MODE_ALWAYS},
        {CC1200_PKT_CFG1,   CC1200_PKT_CFG1_CRC_CFG_ON_INIT_1D0F | CC1200_PKT_CFG1_ADDR_CHECK_CFG_OFF | CC1200_PKT_CFG1_APPEND_STATUS | CC1200_PKT_CFG1_WHITE_DATA},
        {CC1200_PKT_CFG0,   CC1200_PKT_CFG0_LENGTH_CONFIG_VARIABLE},
        {CC1200_PKT_LEN,    255},
        //{CC1200_SERIAL_STATUS, CC1200_SERIAL_STATUS_IOC_SYNC_PINS_EN}, // Enable access to GPIO state in CC1200_GPIO_STATUS.GPIO_STATE

        {CC1200_RNDGEN,     CC1200_RNDGEN_EN}, // Needed for random backoff for LBT CCA: https://e2e.ti.com/support/wireless_connectivity/f/156/t/370230
        {CC1200_FIFO_CFG,   0 /*!CC1200_FIFO_CFG_CRC_AUTOFLUSH*/},

        {CC1200_AGC_GAIN_ADJUST, (u8)(s8)(-81 - 15 + 11)}
};


static const cc_dev_t dev = 0;
static xTaskHandle waiting_task = NULL;

bool nphy_init(void)
{
    cc_spi_init(dev);

    cc_strobe(dev, CC1200_SRES);
    int i = 0;

    while ((i++ < 1000) && (cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_CHIP_RDYn));

    u8 pn = cc_get(dev, CC1200_PARTNUMBER);

    if (pn != 0x20) {
        cc_dbg("[%u] error: part number 0x%02X != 0x20", dev, pn);
        return false;
    }

    if (!cc_cfg_regs(dev, CC_CFG_DEFAULT, COUNT_OF(CC_CFG_DEFAULT))) {
        cc_dbg("[%u] error: could not configure (default)", dev);
        return false;
    }

    if (!cc_cfg_regs(dev, CC_CFG_PHY, COUNT_OF(CC_CFG_PHY))) {
        cc_dbg("[%u] error: could not configure", dev);
        return false;
    }

    isrd_configure(2, 10, kPORT_InterruptRisingEdge, isr_mcu_wake);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(0), CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP);

    return true;
}

static bool wait_mcu_wake(TickType_t ticks)
{
    waiting_task = xTaskGetCurrentTaskHandle();

    if (!ulTaskNotifyTake(pdTRUE, ticks)) {
        xTaskNotifyStateClear(waiting_task);
        waiting_task = NULL;
        return false;
    }

    return true;
}

static void isr_mcu_wake(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskHandle *const task = waiting_task;

    if (task) {
        //portDISABLE_INTERRUPTS();
        waiting_task = NULL;
        vTaskNotifyGiveFromISR(task, &xHigherPriorityTaskWoken);
        //portENABLE_INTERRUPTS();
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

cc_pkt_t *nphy_rx(u32 timeout)
{
    static u8 buf[256];
    static cc_pkt_t *const pkt = (cc_pkt_t *)buf;

    u8 st = cc_strobe(dev, CC1200_SNOP | CC1200_ACCESS_READ);

    pkt->len = 0;

    if ((st & CC1200_STATUS_STATE_M) != CC1200_STATE_RX) {
        if (st & CC1200_STATUS_EXTRA_M) {
            cc_dbg_v("[%u] SFRX: st=0x%02X", dev, st);
            cc_strobe(dev, CC1200_SFRX);
        }

        cc_dbg_v("[%u] SRX: st=0x%02X", dev, st);
        cc_strobe(dev, CC1200_SRX);
    } else {
        cc_dbg_v("[%u] st=0x%02X", dev, st);
    }

    _wait:
    if (!wait_mcu_wake(timeout)) return NULL;
    st = cc_get(dev, CC1200_MARC_STATUS1);

    switch (st) {
        case CC1200_MARC_STATUS1_NO_FAILURE:
            goto _wait;

        case CC1200_MARC_STATUS1_RX_FINISHED:
            break;

        case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
        case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:
            cc_dbg("[%u] rx fifo error: ms1=0x%02X", dev, st);
            cc_strobe(dev, CC1200_SFRX);
            return pkt;

        case CC1200_MARC_STATUS1_ADDRESS:
        case CC1200_MARC_STATUS1_CRC:
        case CC1200_MARC_STATUS1_MAXIMUM_LENGTH:
        default:
            cc_dbg("[%u] rx error: ms1=0x%02X", dev, st);
            return pkt;
    }

    /* Assumption (may change later): variable packet length, 2 status bytes -> 3 total. */
    const static u8 PKT_OVERHEAD = 3;

    u8 fifo_len = cc_get(dev, CC1200_NUM_RXBYTES);

    if (fifo_len > PKT_OVERHEAD) {
        //u8 rxf = cc_get(dev, CC1200_RXFIRST);
        //u8 rxl = cc_get(dev, CC1200_RXLAST);
        //cc_dbg("[%u] rxf=%u rxl=%u n=%u", dev, rxf, rxl, len);

        // (another assumption: only one packet in FIFO. TBD whether it ever might be more)
        cc_fifo_read(dev, (u8 *)pkt, fifo_len);

        const s8 rssi = (s8)pkt->data[pkt->len];
        const u8 crc_ok = pkt->data[pkt->len + 1] & (u8) CC1200_LQI_CRC_OK_BM;
        const u8 lqi = pkt->data[pkt->len + 1] & (u8) CC1200_LQI_EST_BM;

        if (!crc_ok) pkt->len = 0;

    } else if (fifo_len) {
        cc_strobe(dev, CC1200_SFRX);
    }

    return pkt;
}

void nphy_tx(cc_pkt_t *pkt)
{
    u8 st;

    while ((st = cc_strobe(dev, CC1200_SIDLE)) & CC1200_STATUS_CHIP_RDYn);

    if (st & CC1200_STATE_TXFIFO_ERROR) {
        cc_dbg("[%u] SFTX", dev);
        cc_strobe(dev, CC1200_SFTX);
    }

    cc_fifo_write(dev, (u8 *)pkt, pkt->len + 1);
    cca_setup();
    _cca:
    cca_run();
    cc_strobe(dev, CC1200_STX);

    _wait:
    cc_dbg("[%u] tx: wait", dev);
    wait_mcu_wake(portMAX_DELAY);
    st = cc_get(dev, CC1200_MARC_STATUS1);
    cc_dbg("[%u] tx: st=0x%02X", dev, st);

    switch (st) {
        case CC1200_MARC_STATUS1_NO_FAILURE:
            goto _wait;

        case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
            cc_dbg("[%u] tx cca fail", dev);
            goto _cca;

        case CC1200_MARC_STATUS1_TX_FINISHED:
            break;

        case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
        case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
            /*if (cc1200_strobe(CC1200_SFTX, NULL)) { DBG("fail: SFTX"); goto fail; }*/
            cc_dbg("[%u] tx fifo error: %s", dev, st == CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW ? "overflow" : "underflow");
            cc_strobe(dev, CC1200_SFTX);
            break;

        default:
            cc_dbg("[%u] tx error: ms1=0x%02X", dev, st);
            break;
    }

    cca_end();
}

#include <itm.h>


void cca_setup(void)
{
    itm_puts(0, "cca setup\r\n");

    //cc_update(dev, CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RXOFF_MODE_M, CC1200_RFEND_CFG1_RXOFF_MODE_TX);
    cc_update(dev, CC1200_PKT_CFG2, CC1200_PKT_CFG2_CCA_MODE_M, CC1200_PKT_CFG2_CCA_MODE_RSSI_THR_ETSI_LBT);
    //cc_update(dev, CC1200_MODCFG_DEV_E, CC1200_MODCFG_DEV_E_MODEM_MODE_M, CC1200_MODCFG_DEV_E_MODEM_MODE_CARRIER_SENSE);
    //cc_update(dev, CC1200_SYNC_CFG1, CC1200_SYNC_CFG1_SYNC_THR_M, 0);
}


void cca_run(void)
{
    u8 reg;
    cc_strobe(dev, CC1200_SRX);

    do {
        reg = cc_get(dev, CC1200_RSSI0) & (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID);
    } while (reg != (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID));
}

void cca_end(void)
{
    cc_update(dev, CC1200_PKT_CFG2, CC1200_PKT_CFG2_CCA_MODE_M, CC1200_PKT_CFG2_CCA_MODE_ALWAYS);
    //cc_update(dev, CC1200_MODCFG_DEV_E, CC1200_MODCFG_DEV_E_MODEM_MODE_M, CC1200_MODCFG_DEV_E_MODEM_MODE_NORMAL);
    //cc_update(dev, CC1200_SYNC_CFG1, CC1200_SYNC_CFG1_SYNC_THR_M, 7); // NOTE: cheating here and using known value
}




typedef enum {
    RF_STATE_NONE,
    RF_STATE_RX,
    RF_STATE_TX,
    RF_STATE_CCA

} rf_state_t;

rf_state_t state;

static void radio_loop(void)
{
    while (1) {

        /**
         * - handle status and initiate next action, OR
         * - update current channel AND
         * - if tx queued: expected: in RX, CCA set, strobe TX, CONTINUE
         * -
         */

        const u8 ms = cc_get(dev, CC1200_MARC_STATUS1);
        u8 st;

        switch (ms) {
            case CC1200_MARC_STATUS1_RX_FINISHED:
                // handle rx, should be idle
                break;

            case CC1200_MARC_STATUS1_RX_TIMEOUT:
            case CC1200_MARC_STATUS1_RX_TERMINATION:
            case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
            case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:
            case CC1200_MARC_STATUS1_ADDRESS:
            case CC1200_MARC_STATUS1_CRC:
            case CC1200_MARC_STATUS1_MAXIMUM_LENGTH:
                // sfrx?, ensure idle
                break;

            case CC1200_MARC_STATUS1_TX_FINISHED:
                // should already be idle
                break;

            case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
                // what state here? do nothing, retry happens below
                break;

            case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
            case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
                // sftx, ensure idle
                break;

            default:
                break;
        }

        // update channel

        if (xQueueReceive(nphy_tx_queue, &pkt, 0) == pdTRUE) {
            cc_strobe(dev, CC1200_SIDLE);
            cc_fifo_write(dev, (u8 *)pkt, pkt->len + 1);
            cc_strobe(dev, CC1200_STX);
        }

        // if tx queued or cca failed:
        //    if not cca failed, fill fifo
        //    if CCA, ensure RX
        //    start TX

        /**
         * - force idle
         * - set channel
         */


    }
}
