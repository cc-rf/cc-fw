#include <cc/phy.h>
#include <cc/io.h>
#include <cc/cfg.h>
#include <cc/isr.h>
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
        {CC1200_PKT_CFG1,   CC1200_PKT_CFG1_CRC_CFG_ON_INIT_1D0F | CC1200_PKT_CFG1_ADDR_CHECK_CFG_OFF | CC1200_PKT_CFG1_APPEND_STATUS},
        {CC1200_PKT_CFG0,   CC1200_PKT_CFG0_LENGTH_CONFIG_VARIABLE},
        {CC1200_PKT_LEN,    255},
        {CC1200_SERIAL_STATUS, CC1200_SERIAL_STATUS_IOC_SYNC_PINS_EN}, // Enable access to GPIO state in CC1200_GPIO_STATUS.GPIO_STATE

        {CC1200_RNDGEN,     CC1200_RNDGEN_EN}, // Needed for random backoff for LBT CCA: https://e2e.ti.com/support/wireless_connectivity/f/156/t/370230
        {CC1200_FIFO_CFG,   0 /*!CC1200_FIFO_CFG_CRC_AUTOFLUSH*/},
};

static bool phy_setup(cc_dev_t dev);
static void phy_tmr(TimerHandle_t xTimer);
static void isr_mcu_wake(cc_dev_t dev);
static void phy_rx_tmr_kick(cc_dev_t dev);
static void phy_rx_tmr_check(cc_dev_t dev);
static void handle_rx(cc_dev_t dev, u8 *buf, u8 len);
static inline void rx_resume(cc_dev_t dev);
static void tx_task(void *param);
static void phy_tx_execute(cc_dev_t dev, bool cca, u8 *buf, u32 len);
static void cca_enable(cc_dev_t dev/*, u8 cca_mode*/);
static void cca_disable(cc_dev_t dev);

typedef struct __packed {
    bool cca;
    u8 *buf;
    u32 len;

} tx_queue_item;

static struct {
    bool rx_enabled;
    volatile u8 tx_completion_status;
    volatile bool cca_configured;
    volatile u8 cca_saved_sync_thr;
    volatile u8 cca_saved_rx_time;
    phy_cfg_t phy_cfg;
    volatile pit_tick_t rx_tick;
    xSemaphoreHandle tx_sem;
    xQueueHandle tx_queue;
    xTaskHandle tx_task;

} phy[CC_NUM_DEVICES];

extern pit_t xsec_timer;

bool phy_init(cc_dev_t dev, phy_cfg_t *cfg)
{
    assert(CC_DEV_VALID(dev));
    assert(cfg);

    phy[dev].phy_cfg = *cfg;
    phy[dev].rx_enabled = false;
    phy[dev].tx_completion_status = 0;
    phy[dev].tx_sem = xSemaphoreCreateCounting(UINT32_MAX, 0); assert(phy[dev].tx_sem != NULL);
    /*phy[dev].tx_queue = xQueueCreate(8, sizeof(tx_queue_item)); assert(phy[dev].tx_queue != NULL);

    xTaskCreate(
            tx_task, "tx_task", TASK_STACK_SIZE_DEFAULT,
            (void *)dev, TASK_PRIO_HIGHEST, &phy[dev].tx_task
    );

    assert(phy[dev].tx_task);*/

    cc_spi_init(dev);
    if (!cc_isr_init(dev)) return false;
    if (!phy_setup(dev)) return false;

    return true;
}

static bool phy_setup(cc_dev_t dev)
{
    cc_strobe(dev, CC1200_SRES);
    int i = 0;

    while ((i++ < 1000) && (cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_CHIP_RDYn));

    u8 pn = cc_get(dev, CC1200_PARTNUMBER);

    if (pn != 0x20) {
        cc_dbg("[%u] error: part number 0x%02X != 0x20", dev, pn);
        return false;
    }

    //if (dev == CC_DEV_MIN) {
        if (!cc_cfg_regs(dev, CC_CFG_DEFAULT, COUNT_OF(CC_CFG_DEFAULT))) {
            cc_dbg("[%u] error: could not configure (default)", dev);
            return false;
        }
    //} else {
    //    if (!cc_cfg_regs(dev, CC_CFG_DEFAULT_NB, COUNT_OF(CC_CFG_DEFAULT_NB))) {
    //        cc_dbg("[%u] error: could not configure (default-nb)", dev);
    //        return false;
    //    }
    //}

    /*if (dev != CC_DEV_MIN) {
        cc_update(dev, CC1200_MODCFG_DEV_E, CC1200_MODCFG_DEV_E_MODEM_MODE_DSSS_PN, 0);
    }*/

    if (!cc_cfg_regs(dev, CC_CFG_PHY, COUNT_OF(CC_CFG_PHY))) {
        cc_dbg("[%u] error: could not configure", dev);
        return false;
    }

    enum isr_pin pin;

    /* TODO: Devise a different method so that the ISR can be set after the radio is configured. */

    if ((pin = cc_isr(dev, ISR_PIN_ANY, ISR_EDGE_RISING, isr_mcu_wake)) == ISR_PIN_NONE)
        return false;

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(pin), CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP);

    TimerHandle_t tmr = xTimerCreate("phy_tmr", 30 / portTICK_PERIOD_MS, pdTRUE, NULL, phy_tmr);

    if (tmr) {
        //xTimerStart(tmr, 41 / portTICK_PERIOD_MS);
    } else {
        cc_dbg("[%u] error: timer create failed", dev);
        return false;
    }

    return true;
}

static void phy_tmr(TimerHandle_t xTimer)
{
    (void)xTimer;
    for (cc_dev_t dev = CC_DEV_MIN; dev <= CC_DEV_MAX; ++dev)
        phy_rx_tmr_check(dev);
}

volatile bool rxtimeout[CC_NUM_DEVICES] = {false,false};

static void isr_mcu_wake(cc_dev_t dev)
{
    u8 st, ms1, len;
    bool flag = true;

    ms1 = cc_get(dev, CC1200_MARC_STATUS1);

#if CC_DEBUG_VERBOSE
    if (rxtimeout[dev]) {
        st = cc_strobe(dev, CC1200_SNOP | CC1200_ACCESS_READ);
        cc_dbg("[%u] st=0x%02X ms1=0x%02X", dev, st, ms1);
        rxtimeout[dev] = false;
    }
#else
    (void)st;
#endif

    switch (ms1) {
        case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
            /*if (phy[dev].tx_completion_status == 0xFF) {
                if (!(st & CC1200_STATE_IDLE)) cc_strobe(dev, CC1200_SIDLE);
                if (phy[dev].phy_cfg.signal) phy[dev].phy_cfg.signal(dev, ms1, &flag);
                if (flag) cc_strobe(dev, CC1200_SRX);
            }
            break;*/

        case CC1200_MARC_STATUS1_TX_FINISHED:
        case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
        case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
            if (phy[dev].tx_completion_status == 0xFF) {
                phy[dev].tx_completion_status = ms1;
                xSemaphoreGive(phy[dev].tx_sem);
            }

            if (phy[dev].phy_cfg.signal) phy[dev].phy_cfg.signal(dev, ms1, NULL);
            break;

        case CC1200_MARC_STATUS1_RX_FINISHED:
            phy_rx_tmr_kick(dev);
            len = cc_get(dev, CC1200_NUM_RXBYTES);

            if (len) {
                //u8 rxf = cc_get(dev, CC1200_RXFIRST);
                //u8 rxl = cc_get(dev, CC1200_RXLAST);
                //cc_dbg("[%u] rxf=%u rxl=%u n=%u", dev, rxf, rxl, len);
                u8 *buf = alloca(len);
                assert(buf);
                cc_fifo_read(dev, buf, len);

                if (phy[dev].phy_cfg.signal) phy[dev].phy_cfg.signal(dev, ms1, &flag);
                if (flag) rx_resume(dev);

                handle_rx(dev, buf, len);
            } else {
                cc_strobe(dev, CC1200_SFRX);
                if (phy[dev].phy_cfg.signal) phy[dev].phy_cfg.signal(dev, ms1, &flag);
                if (flag) rx_resume(dev);
            }
            break;

        case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
        case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:
            //cc_dbg("[%u] rx fifo error: ms1=0x%02X", dev, ms1);
            //cc_strobe(dev, CC1200_SFRX);

        case CC1200_MARC_STATUS1_ADDRESS:
        case CC1200_MARC_STATUS1_CRC:
        case CC1200_MARC_STATUS1_MAXIMUM_LENGTH:
            cc_dbg("[%u] rx error: ms1=0x%02X", dev, ms1);
        case CC1200_MARC_STATUS1_RX_TERMINATION:
        case CC1200_MARC_STATUS1_RX_TIMEOUT:
            phy_rx_tmr_kick(dev);
            //if (!(st & CC1200_STATE_IDLE)) cc_strobe(dev, CC1200_SIDLE);
            //cc_strobe(dev, CC1200_SFRX); // TODO: use read flag on SNOP to get rx fifo info bits

            if (phy[dev].phy_cfg.signal) phy[dev].phy_cfg.signal(dev, ms1, &flag);
            // TODO: make the SFRX conditional?
            if (flag) rx_resume(dev);
            break;

        case CC1200_MARC_STATUS1_EWOR_SYNC_LOST:
            /* TODO: what to do here? */
            if (phy[dev].phy_cfg.signal) phy[dev].phy_cfg.signal(dev, ms1, NULL);
            break;

        default:
            if (phy[dev].phy_cfg.signal) phy[dev].phy_cfg.signal(dev, ms1, &flag);
            if (flag) rx_resume(dev);
            break;
    }

    if (rxtimeout[dev]) rxtimeout[dev] = false;

}

static void phy_rx_tmr_kick(cc_dev_t dev)
{
    phy[dev].rx_tick = pit_get_current(xsec_timer);
}

static void phy_rx_tmr_check(cc_dev_t dev)
{
    if (phy[dev].rx_enabled) {
        const pit_tick_t rx_ticks = phy[dev].rx_tick - pit_get_current(xsec_timer);

        if (rx_ticks >= 50000) {
            rxtimeout[dev] = true;

            //const u8 ms = cc_get(dev, CC1200_MODEM_STATUS1);
            //const u8 st = cc_strobe(dev, CC1200_ACCESS_READ | CC1200_SNOP);

            // TODO: find ways to combine strobes easily, and measure benefit

            // TODO: Maybe always strobe sidle in case the isr needs to be triggered?
            //if (st & CC1200_STATE_RX) {
            const u8 st = cc_strobe(dev, CC1200_SNOP | CC1200_ACCESS_READ);
            const bool cs = cc_get(dev, CC1200_RSSI0) & (CC1200_RSSI0_CARRIER_SENSE_VALID | CC1200_RSSI0_CARRIER_SENSE) == (CC1200_RSSI0_CARRIER_SENSE_VALID | CC1200_RSSI0_CARRIER_SENSE);
            const u8 mo1 = cc_get(dev, CC1200_MODEM_STATUS1);
            const bool pqt = mo1 & (CC1200_MODEM_STATUS1_PQT_VALID | CC1200_MODEM_STATUS1_PQT_REACHED) == (CC1200_MODEM_STATUS1_PQT_VALID | CC1200_MODEM_STATUS1_PQT_REACHED);
            const bool sync = mo1 & CC1200_MODEM_STATUS1_SYNC_FOUND != 0;
            //}

            cc_dbg/*_v*/("[%u] rx timeout: st=0x%02X rssi=%i freq=%u cs=%u pqt=%u sync=%u", dev, st,
                         cc_get_rssi(dev), cc_get_freq(dev),
                         cs, pqt, sync
            );

            cc_strobe(dev, CC1200_SIDLE);

            if (st & CC1200_STATUS_EXTRA_M) {
                cc_strobe(dev, CC1200_SFRX);
            }

            rx_resume(dev);
            //phy_rx_tmr_kick(dev);

            //bool flag = true;
            //if (phy[dev].phy_cfg.signal) phy[dev].phy_cfg.signal(dev, CC1200_MARC_STATUS1_RX_TIMEOUT, &flag);
            //if (flag) rx_resume(dev);
        }
    }
}

static void handle_rx(cc_dev_t dev, u8 *buf, u8 len)
{
    assert(buf);

    /* Assumption (may change later): variable packet length, 2 status bytes -> 3 total. */
    const static u8 PKT_OVERHEAD = 3;

    if (len < PKT_OVERHEAD) {
        cc_dbg("[%u] len=%u < PKT_OVERHEAD=%u", dev, len, PKT_OVERHEAD);
    }

    size_t pkt_count = 0;

    while (len >= PKT_OVERHEAD) {
        ++pkt_count;
        u8 pkt_len = buf[0];

        //cc_dbg("[%u] c=%u len=%u pkt_len=%u", dev, pkt_count, len, pkt_len);

        if (pkt_len > (len - PKT_OVERHEAD)) {
            /* Bad length field, cannot continue */
            cc_dbg("[%u] c=%u underflow: len=%u < len[header]=%u", dev, pkt_count, len, pkt_len);
            return;
        }

        if (pkt_len) {
            const s8 rssi = (s8)buf[pkt_len + 1];
            const u8 crc_ok = buf[pkt_len + 2] & (u8) CC1200_LQI_CRC_OK_BM;
            const u8 lqi = buf[pkt_len + 2] & (u8) CC1200_LQI_EST_BM;

            //DBG("len=%i crc_ok=%i", (int)pkt_len, (int)crc_ok);

            if (!crc_ok) {
                cc_dbg("[%u] c=%u len=%u pkt_len=%u crc_ok=0", dev, pkt_count, len, pkt_len);
            }

            /* TODO: Potentially use the workqueue to post to packet rx callback */
            if (crc_ok && phy[dev].phy_cfg.rx) {
                /*u8 *pkt_buf = malloc(pkt_len);
                assert(pkt_buf);
                memcpy(pkt_buf, &buf[1], pkt_len);
                phy[dev].phy_cfg.rx(dev, pkt_buf, pkt_len);*/
                assert(buf > (u8 *) 1ul);
                assert(pkt_len);
                phy[dev].phy_cfg.rx(dev, &buf[1], pkt_len, rssi, lqi);
            }
        } else {
            cc_dbg("[%u] c=%u len=%u pkt_len=0", dev, pkt_count, len);
        }

        len -= pkt_len + PKT_OVERHEAD;
        buf += pkt_len + PKT_OVERHEAD;
    }
}

void phy_rx_enable(cc_dev_t dev)
{
    if (!phy[dev].rx_enabled) {
        cc_dbg_v("[%u] enabling", dev);
        phy[dev].rx_enabled = true;
        rx_resume(dev);
    }
}

void phy_rx_disable(cc_dev_t dev)
{
    if (phy[dev].rx_enabled) {
        phy[dev].rx_enabled = false;
        if (!phy[dev].cca_configured) {
            //u8 mst = cc_get(dev, CC1200_MARCSTATE) & CC1200_MARC_2PIN_STATE_M;
            cc_strobe(dev, CC1200_SIDLE);

            /*if (mst != CC1200_MARC_2PIN_STATE_IDLE) {
                cc_dbg_v("[%u] disabling: !cca_configured -> SIDLE", dev);
            } else {*/
                cc_dbg_v("[%u] disabling: mst=0x%02X", dev, mst);
            //}
        } else {
            cc_dbg_v("[%u] disabling: cca configured", dev);
        }
    }
}

bool phy_rx_enabled(cc_dev_t dev)
{
    return phy[dev].rx_enabled;
}

static inline void rx_resume(cc_dev_t dev)
{
    if (phy[dev].rx_enabled) {
        phy_rx_tmr_kick(dev);

        // TODO: Find a way to use cached state
        const u8 st = cc_strobe(dev, CC1200_SNOP | CC1200_ACCESS_READ);

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
    }
}

static void tx_task(void *param)
{
    const cc_dev_t dev = (cc_dev_t)param;
    const xQueueHandle queue = phy[dev].tx_queue; assert(queue);

    tx_queue_item txq;

    while (1) {
        if (xQueueReceive(queue, &txq, portMAX_DELAY) != pdTRUE) continue;
        phy_tx_execute(dev, txq.cca, txq.buf, txq.len);
        free(txq.buf);
    }
}

void phy_tx(cc_dev_t dev, bool cca, u8 *buf, u32 len)
{
    /*if (!len || len > 120) return;

    tx_queue_item txq = {
            .buf = malloc(len + 1),
            .len = len,
            .cca = cca
    };

    assert(txq.buf);
    txq.buf[0] = (u8)len;
    memcpy(&txq.buf[1], buf, len);

    if (!xQueueSend(phy[dev].tx_queue, &txq, portMAX_DELAY)) {
        cc_dbg("[%u] tx queue full!", dev);
    }*/

    u8 *pkt = alloca(len + 1);
    assert(pkt);
    pkt[0] = (u8)len;
    memcpy(&pkt[1], buf, len);
    phy_tx_execute(dev, cca, pkt, len);
}

static void phy_tx_execute(cc_dev_t dev, bool cca, u8 *buf, u32 len)
{
    if (phy_rx_enabled(dev)) {
        // TODO: Disable RX or add a flag to this or something
        phy_rx_tmr_kick(dev);
    }

    u8 *pkt, st, reg;
    //bool cb = false;

    pkt = buf;
    assert(pkt[0] == (u8)len);

    //cc_dbg_v("[%u] len=%lu", dev, len);
    while ((st = cc_strobe(dev, CC1200_SIDLE)) & CC1200_STATUS_CHIP_RDYn);
    //cc_dbg_v("[%u] st=0x%02X", dev, st);

    if (st & CC1200_STATE_TXFIFO_ERROR) {
        cc_dbg_v("[%u] SFTX", dev);
        st = cc_strobe(dev, CC1200_SFTX);
    }

    cc_fifo_write(dev, pkt, len + 1);
    st = cc_strobe(dev, CC1200_SNOP);

    phy[dev].tx_completion_status = 0xFF;

    if (cca) {
        cca_enable(dev);

        cca_begin:
        cc_dbg_v("[%u] cca begin: st=0x%02X", dev, st);

        if ((st & CC1200_STATUS_STATE_M) != CC1200_STATE_RX) {
            cc_dbg_v("[%u] SRX: st=0x%02X", dev, st);
            cc_strobe(dev, CC1200_SRX);
        }

        do {
            reg = cc_get(dev, CC1200_RSSI0) & (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID);
        } while (reg != (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID));

        cc_dbg_v("[%u] rssi & cs valid", dev);
    } else {
        /* disable CCA if about to go from RX to TX */
        if (st & CC1200_STATE_RX)
            cca_disable(dev);
    }

    cc_dbg_v("[%u] STX", dev);
    cc_strobe(dev, CC1200_STX);

    if (xSemaphoreTake(phy[dev].tx_sem, 100 / portTICK_PERIOD_MS) != pdTRUE) {
        cc_dbg("[%u] timeout, checking manually", dev);
        isr_mcu_wake(dev);
    }

    reg = phy[dev].tx_completion_status;
    phy[dev].tx_completion_status = 0;

    cc_dbg_v("[%u] tx_completion_status=0x%02X", dev, reg);

    // TODO: check tx fifo status even when no error?

    switch (reg) {
        case CC1200_MARC_STATUS1_TX_FINISHED:
            //DBG("sent: len=%i", (int)len);
            /* TODO: Potentially use the workqueue to post to packet tx callback */
            /*if (drv_cfg.cb.tx) {
                cb = true;
                drv_cfg.cb.tx(buf, len);
            }*/
            //cb = true;
            break;

        case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
            if (cca) {
                cc_dbg("cca retry");
                //if (cc1200_strobe(CC1200_SNOP, &st)) { DBG("fail: SNOP(2)"); goto fail; }*/
                st = cc_strobe(dev, CC1200_SIDLE);
                goto cca_begin;
            }
            break;

        case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
        case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
            /*if (cc1200_strobe(CC1200_SFTX, NULL)) { DBG("fail: SFTX"); goto fail; }*/
            cc_dbg("[%u] fifo error: %s", dev, reg == CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW ? "overflow" : "underflow");
            cc_strobe(dev, CC1200_SFTX);
            break;
    }

    /* TODO: Maybe move cca_disable to rx_resume */
    if (phy[dev].rx_enabled) {
        cca_disable(dev);
        rx_resume(dev);
    }

    /*fail:

    if (drv_cfg.cb.tx && !cb) {
        cb = true;
        *//* TODO: Potentially use the workqueue to post to packet tx callback *//*
        *//* TODO: Indicate status on failure (instead of zero length)
         *       (failure is presumed due to callback not already being called)
         *//*
        drv_cfg.cb.tx(buf, 0);
    }*/

    return;
}

static void cca_enable(cc_dev_t dev/*, u8 cca_mode*/)
{
    if (!phy[dev].cca_configured) {
        cc_dbg_v("[%u] enabling", dev);
        u8 pkt_cfg2 = cc_get(dev, CC1200_PKT_CFG2);
        u8 sync_cfg1 = cc_get(dev, CC1200_SYNC_CFG1);
        u8 rx_time = cc_get(dev, CC1200_RFEND_CFG1);
        phy[dev].cca_saved_sync_thr = sync_cfg1;
        phy[dev].cca_saved_rx_time = rx_time;
        rx_time = (rx_time & ~CC1200_RFEND_CFG1_RX_TIME_M) | (CC1200_RFEND_CFG1_RX_TIME_FOREVER << CC1200_RFEND_CFG1_RX_TIME_S);
        pkt_cfg2 &= ~(u8)CC1200_PKT_CFG2_CCA_MODE_M;
        pkt_cfg2 |= CC1200_PKT_CFG2_CCA_MODE_RSSI_THR_ETSI_LBT/*cca_mode*/ & CC1200_PKT_CFG2_CCA_MODE_M;
        sync_cfg1 &= ~(u8)CC1200_SYNC_CFG1_SYNC_THR_M;
        cc_set(dev, CC1200_RFEND_CFG1, rx_time);
        cc_set(dev, CC1200_PKT_CFG2, pkt_cfg2);
        phy[dev].cca_configured = true;
        cc_set(dev, CC1200_SYNC_CFG1, sync_cfg1);
    }
}

static void cca_disable(cc_dev_t dev)
{
    if (phy[dev].cca_configured) {
        cc_dbg_v("[%u] disabling", dev);
        u8 pkt_cfg2 = cc_get(dev, CC1200_PKT_CFG2);
        pkt_cfg2 &= ~(u8)CC1200_PKT_CFG2_CCA_MODE_M;
        pkt_cfg2 |= CC1200_PKT_CFG2_CCA_MODE_ALWAYS & CC1200_PKT_CFG2_CCA_MODE_M;
        cc_set(dev, CC1200_PKT_CFG2, pkt_cfg2);
        cc_set(dev, CC1200_RFEND_CFG1, phy[dev].cca_saved_rx_time);
        phy[dev].cca_configured = false;
        /* TODO: check potential for rx before tx commence due to this, if in rx and
         * about to strobe tx but not before a sync word is detected. */
        cc_set(dev, CC1200_SYNC_CFG1, phy[dev].cca_saved_sync_thr);
    }
}
