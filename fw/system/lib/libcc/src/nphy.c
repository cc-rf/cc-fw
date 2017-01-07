#include <cc/nphy.h>
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
#include <cc/type.h>


#define PHY_PKT_FLAG_SYNC       1   // is a sync packet
#define PHY_PKT_FLAG_NOSYNC     2   // not synced

typedef struct __packed {
    u8 len;
    u8 flag;

} phy_pkt_hdr_t;

typedef struct __packed {
    phy_pkt_hdr_t hdr;
    u8 data[];

} phy_pkt_t, phy_data_pkt_t;

typedef struct __packed {
    phy_pkt_hdr_t hdr;
    u32 ts;

} phy_sync_pkt_t;


static void isr_mcu_wake(void);
static void cca_setup(void);
static void cca_run(void);
static void cca_end(void);

static void nphy_rx(bool flush);

static void nphy_task(void *param);
static void nphy_dispatch_task(void *param);

#define CC_RSSI_OFFSET      (s8)(-81 - 15 + 11)

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
        {CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RXOFF_MODE_RX/*TODO: what should this be?*/ | (0x7<<1) /*| CC1200_RFEND_CFG1_RX_TIME_QUAL_M*/},
        {CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_RX | CC1200_RFEND_CFG0_TERM_ON_BAD_PACKET_EN /*| 1*/},

        // These may not necessarily be part of the phy handling

        //{CC1200_PREAMBLE_CFG1, /*0xD*/0x4 << 2},
        //{CC1200_SYNC_CFG1,  0x09 | CC1200_SYNC_CFG1_SYNC_MODE_16/* changed from 11 */},
        {CC1200_PKT_CFG2,   CC1200_PKT_CFG2_CCA_MODE_ALWAYS},
        {CC1200_PKT_CFG1,   CC1200_PKT_CFG1_CRC_CFG_ON_INIT_1D0F | CC1200_PKT_CFG1_ADDR_CHECK_CFG_OFF | CC1200_PKT_CFG1_APPEND_STATUS | CC1200_PKT_CFG1_WHITE_DATA},
        {CC1200_PKT_CFG0,   CC1200_PKT_CFG0_LENGTH_CONFIG_VARIABLE},
        {CC1200_PKT_LEN,    255},
        //{CC1200_SERIAL_STATUS, CC1200_SERIAL_STATUS_IOC_SYNC_PINS_EN}, // Enable access to GPIO state in CC1200_GPIO_STATUS.GPIO_STATE

        {CC1200_RNDGEN,     CC1200_RNDGEN_EN}, // Needed for random backoff for LBT CCA: https://e2e.ti.com/support/wireless_connectivity/f/156/t/370230
        {CC1200_FIFO_CFG,   CC1200_FIFO_CFG_CRC_AUTOFLUSH /*!CC1200_FIFO_CFG_CRC_AUTOFLUSH*/},


        {CC1200_AGC_GAIN_ADJUST, (u8)CC_RSSI_OFFSET},
        {CC1200_SETTLING_CFG, 0x3}, // Defaults except never auto calibrate

};


#undef cc_dbg_v
#define cc_dbg_v(format, ...) cc_dbg(format, ##__VA_ARGS__ ) //cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )


static const cc_dev_t dev = 0;
static xTaskHandle waiting_task = NULL;

static struct {
    xTaskHandle task;
    xTaskHandle disp;
    xQueueHandle txq;
    xQueueHandle rxq;
    nphy_rx_t rx;

} nphy = {NULL};




extern u32 sync_timestamp(void);
s32 sync_time = 0;

static void ensure_rx(void);


#define FREQ_BASE       905000000
#define FREQ_BW         800000
#define CHAN_COUNT      25
#define CHAN_TIME       400//100//200//60//400/*100*///200  //30
#define MAX_CCA_RETRY   2//4//3//2//3
#define MAX_CCA_TIME    11 //NOTE: When using LBT, backoff time minimum is 5
#define MAX_PACKET_LEN  120

static const u32 freq_base      = FREQ_BASE;
static const u32 freq_side_bw   = FREQ_BW / 2;
static const u32 chan_count     = CHAN_COUNT;
static const u32 chan_time      = CHAN_TIME;

#include <cc/chan.h>
#include <fsl_rnga.h>


static struct {
    chan_grp_t group;
    chan_inf_t chan[CHAN_COUNT];
    chan_t hop_table[CHAN_COUNT];

} chnl = {
        .group = {
                .dev = 0,
                .freq = {
                        .base = FREQ_BASE,
                        .bw   = FREQ_BW
                },
                .size = CHAN_COUNT
        }
};

volatile u32 chan_cur = UINT32_MAX;

void cca_disable(void);

static inline bool chan_set(const u32 chan)
{
    if (chan != chan_cur) {
        chan_cur = chan;

        u8 st;

        st = cc_strobe(dev, CC1200_SIDLE | CC1200_ACCESS_READ);

        // chance of switching during rx -- TODO: maybe try to extract valid packets?
        if (st & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFRX);

        do {
            st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;

        } while (st != CC1200_STATE_IDLE);

        //NOTE: for debug purposes, only use 2 channels
        chan_select(&chnl.group, (chan_t) (4 + chan%2));
        //chan_select(&chnl.group, (chan_t)chan);

        ensure_rx(); // this kills the in-progress tx

        return true;
    }

    return false;
}


static bool boss = false;

bool nphy_init(nphy_rx_t rx, bool sync_master)
{
    boss = sync_master;

    // TODO: eventually move elsewhere
    RNGA_Init(RNG);

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

    nphy.txq = xQueueCreate(3/*2*//*24*//*8*//*8*/, sizeof(void *));
    nphy.rxq = xQueueCreate(3/*2*//*16*//*4*//*8*/, sizeof(void *));
    nphy.rx = rx;

    if (!xTaskCreate(nphy_task, "nphy:main", TASK_STACK_SIZE_LARGE * 2, NULL, TASK_PRIO_HIGH+1, &nphy.task)) {
        cc_dbg("[%u] error: unable to create main task", dev);
        isrd_configure(2, 10, kPORT_InterruptOrDMADisabled, NULL);
        return false;
    }

    if (!xTaskCreate(nphy_dispatch_task, "nphy:disp", TASK_STACK_SIZE_DEFAULT, NULL, TASK_PRIO_HIGH-1, &nphy.disp)) {
        cc_dbg("[%u] error: unable to create dispatch task", dev);
        isrd_configure(2, 10, kPORT_InterruptOrDMADisabled, NULL);
        return false;
    }

    chan_grp_init(&chnl.group, NULL);
    chan_grp_calibrate(&chnl.group);
    chan_set(0);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(0), CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP);
    isrd_configure(2, 10, kPORT_InterruptRisingEdge, isr_mcu_wake);

    cc_strobe(dev, CC1200_SRX);
    return true;
}

#define NOTIFY_MASK_ISR    2
#define NOTIFY_MASK_TX     4

void nphy_tx(u8 flag, u8 *buf, u8 len)
{
    phy_data_pkt_t *qpkt = malloc(sizeof(phy_data_pkt_t) + len); assert(qpkt);
    if (len && buf) memcpy(qpkt->data, buf, len);
    qpkt->hdr.len = sizeof(phy_data_pkt_t) - 1 + len;
    qpkt->hdr.flag = flag & (u8)PHY_PKT_FLAG_IMMEDIATE;

    if (qpkt->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {
        if (xQueueSendToFront(nphy.txq, &qpkt, portMAX_DELAY/*pdMS_TO_TICKS(500)*/)) {
            xTaskNotify(nphy.task, NOTIFY_MASK_TX, eSetBits);
        } else {
            cc_dbg("tx pkt queue immediate fail");
        }

    } else if (xQueueSend(nphy.txq, &qpkt, portMAX_DELAY/*pdMS_TO_TICKS(500)*/)) {
        xTaskNotify(nphy.task, NOTIFY_MASK_TX, eSetBits);
    } else {
        cc_dbg("tx pkt queue fail");
    }
}

static void isr_mcu_wake(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(nphy.task, NOTIFY_MASK_ISR, eSetBits, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static bool sync_needed = false;

static void process_packet(rf_pkt_t *pkt)
{
    phy_sync_pkt_t *const spkt = (phy_sync_pkt_t *)pkt;

    if (spkt->hdr.flag & PHY_PKT_FLAG_SYNC) {
        // TODO: make sure size is big enough before reading flag

        if (!boss) {
            sync_time = sync_timestamp() - spkt->ts;

            //static s32 last_sync = 0;
            //u32 now = sync_timestamp();
            //sync_time = now - spkt->ts;
            //cc_dbg_v(
            //        "sync: now=%lu time=%i ts=%u last=%i diff=%i",
            //        now, sync_time, spkt->ts, last_sync, spkt->hdr.len, spkt->hdr.flag, last_sync
            //);
            //last_sync = sync_time;
        }

        // TODO: maybe work out some sort of intelligent synchronization

    } else if (spkt->hdr.flag & PHY_PKT_FLAG_NOSYNC) {
        if (boss && !sync_needed) {
            sync_needed = true;
        }
    } else {
        // TODO: check for size underflow vs. phy packet size
        phy_pkt_t *dpkt = malloc(1 + pkt->len);
        assert(dpkt);
        memcpy(dpkt, pkt, 1 + pkt->len);

        ////if (nphy.rx) nphy.rx(0, pkt->data, 1 + pkt->len - sizeof(phy_pkt_hdr_t));
        ////free(dpkt);

        if (!xQueueSend(nphy.rxq, &dpkt, pdMS_TO_TICKS(100))) {
            cc_dbg("rx pkt queue fail");
            free(dpkt);
            // TODO: anything else?
        }
    }
}

static void nphy_rx(bool flush)
{
    /* Assumption (may change later): variable packet length, 2 status bytes -> 3 total. */
    const static u8 PKT_OVERHEAD = 3;

    u8 len = cc_get(dev, CC1200_NUM_RXBYTES);

    if (len < PKT_OVERHEAD) {
        if (flush) cc_strobe(dev, CC1200_SFRX);

        if (len && flush) {
            cc_dbg("len=%u < PKT_OVERHEAD=%u", len, PKT_OVERHEAD);
        }

        return;
    }

    rf_pkt_t *spkt;
    u8 *buf = alloca(len);
    size_t pkt_count = 0;

    cc_fifo_read(dev, buf, len);

    while (len > PKT_OVERHEAD) {
        spkt = (rf_pkt_t *)buf;

        if (spkt->len > MAX_PACKET_LEN) {
            // NOTE: _v added newly, but this is a useful error to see when timing is off. same applies for below
            cc_dbg_v("[%u] c=%u malformed: len[header]=%u > len[max]=%u  (len[fifo]=%u)", dev, pkt_count+1, spkt->len, MAX_PACKET_LEN, len);
            break;
        }

        if (spkt->len > (len - PKT_OVERHEAD)) {
            cc_dbg_v("[%u] c=%u underflow: len[header]=%u > len[fifo]=%u", dev, pkt_count+1, spkt->len, len);
            break;
        }

        ++pkt_count;

        //u8 rxf = cc_get(dev, CC1200_RXFIRST);
        //u8 rxl = cc_get(dev, CC1200_RXLAST);
        //cc_dbg("[%u] rxf=%u rxl=%u n=%u", dev, rxf, rxl, len);

        if (spkt->len) {
            const s8 rssi = (s8) spkt->data[spkt->len];
            const u8 crc_ok = spkt->data[spkt->len + 1] & (u8) CC1200_LQI_CRC_OK_BM;
            const u8 lqi = spkt->data[spkt->len + 1] & (u8) CC1200_LQI_EST_BM;

            if (crc_ok) {
                process_packet(spkt);
            } else {
                cc_dbg_v("[%u] c=%u bad crc", dev, pkt_count);
                // NEW: don't trust anything else in the buffer
                break;
            }
        } else {
            cc_dbg_v("[%u] c=%u empty", dev, pkt_count);
            // NEW: this is weird, should def stop
            break;
        }

        len -= spkt->len + PKT_OVERHEAD;
        buf += spkt->len + PKT_OVERHEAD;
    }

    if (flush && len) cc_strobe(dev, CC1200_SFRX);
}

bool nphy_recv(u8 **buf, u8 *len, u32 timeout)
{
    phy_pkt_t *pkt;
    *buf = NULL;

    if (xQueueReceive(nphy.rxq, &pkt, pdMS_TO_TICKS(timeout))) {
        *buf = pkt->data;
        *len = pkt->hdr.len - sizeof(phy_pkt_t) + 1;
        return true;
    }

    return false;
}

void nphy_free_buf(u8 *buf)
{
    phy_pkt_t *pkt = buf - offsetof(phy_pkt_t, data);
    free(pkt);
}

static void ensure_rx(void)
{
    u8 st;

    do {
        st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;

    } while (st == CC1200_STATE_SETTLING || st == CC1200_STATE_CALIBRATE);

    switch (st) {
        case CC1200_STATE_RX:
            return;

        case CC1200_STATE_TX:
            cc_dbg("warning: was in TX");
            break;

        case CC1200_STATE_FSTXON:
            cc_dbg("warning: was in FSTXON");
            break;

        case CC1200_STATE_IDLE:
        default:
            break;

        case CC1200_STATE_RXFIFO_ERROR:
            cc_strobe(dev, CC1200_SIDLE); // seems necessary to prevent ISR error indication?
            cc_strobe(dev, CC1200_SFRX);
            cc_dbg_v("rx fifo error");
            break;

        case CC1200_STATE_TXFIFO_ERROR:
            cc_strobe(dev, CC1200_SIDLE); // seems necessary to prevent ISR error indication?
            cc_strobe(dev, CC1200_SFTX);
            cc_dbg_v("tx fifo error");
            break;
    }

    cc_strobe(dev, CC1200_SRX);

    do {
        st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;

    }  while (st != CC1200_STATE_RX);
}

static bool cca_enabled = false;

void cca_enable(void)
{
    if (!cca_enabled) {
        cca_enabled = true;

        /*u8 st = cc_strobe(dev, CC1200_SNOP | CC1200_ACCESS_READ);

        if ((st & CC1200_STATUS_STATE_M) != CC1200_STATE_IDLE) {
            //cc_dbg("SIDLE");
            cc_strobe(dev, CC1200_SIDLE);
        }

        if (st & CC1200_STATUS_EXTRA_M) {
            //cc_dbg("SFRX");
            cc_strobe(dev, CC1200_SFRX);
        }*/

        //cc_update(dev, CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RXOFF_MODE_M, CC1200_RFEND_CFG1_RXOFF_MODE_TX);
        cc_update(dev, CC1200_PKT_CFG2, CC1200_PKT_CFG2_CCA_MODE_M, CC1200_PKT_CFG2_CCA_MODE_RSSI_THR_ETSI_LBT);
        //cc_update(dev, CC1200_MODCFG_DEV_E, CC1200_MODCFG_DEV_E_MODEM_MODE_M, CC1200_MODCFG_DEV_E_MODEM_MODE_CARRIER_SENSE);
        cc_update(dev, CC1200_SYNC_CFG1, CC1200_SYNC_CFG1_SYNC_THR_M, 0);
    }
}

void cca_run(void)
{
    u8 reg;

    ensure_rx();

    do {
        reg = cc_get(dev, CC1200_RSSI0) & (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID);
    } while (reg != (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID));
}

void cca_disable(void)
{
    if (cca_enabled) {
        cca_enabled = false;
        cc_update(dev, CC1200_PKT_CFG2, CC1200_PKT_CFG2_CCA_MODE_M, CC1200_PKT_CFG2_CCA_MODE_ALWAYS);
        //cc_update(dev, CC1200_MODCFG_DEV_E, CC1200_MODCFG_DEV_E_MODEM_MODE_M, CC1200_MODCFG_DEV_E_MODEM_MODE_NORMAL);
        cc_update(dev, CC1200_SYNC_CFG1, CC1200_SYNC_CFG1_SYNC_THR_M, 7); // NOTE: cheating here and using known value
    }
}

static void nphy_task(void *param)
{
    rf_pkt_t *pkt = NULL;
    u8 ms, st, retry = 0;

    u32 ticks = 0;
    u32 remaining = 0;
    u32 tx_time = 0;
    u32 chan_ticks = 0;
    u32 notify;

    bool cca = false;
    /*chan_t*/u32 sync_chan = chan_cur - 1;

    phy_sync_pkt_t pkt_sync = {
            .hdr = {
                    .len = sizeof(phy_sync_pkt_t) - 1,
                    .flag = PHY_PKT_FLAG_SYNC
            },
            .ts = 0
    };

    const u32 start_time = sync_timestamp();

    if (boss) {
        sync_time = start_time;
    }

    while (1) {

        if (xTaskNotifyWait(0, UINT32_MAX, &notify, pdMS_TO_TICKS(remaining))) {

            if (notify & NOTIFY_MASK_ISR) {
                // handle isr
                ms = cc_get(dev, CC1200_MARC_STATUS1);

                //cc_dbg_v("ms1=0x%02X st=0x%02X t=%lu", ms, cc_strobe(dev,CC1200_SNOP), sync_timestamp());

                if (pkt) {
                    switch (ms) {
                        case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
                            if (!retry--) {
                                cc_dbg_v("tx: cca max retry t=%lu time=%lu", sync_timestamp(), sync_timestamp() - tx_time);
                                st = cc_strobe(dev, CC1200_SIDLE);
                                if (st & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);
                                tx_time = 0;
                                goto _end_tx;
                            }

                            /*RNGA_GetRandomData(RNG, &st, 1);
                            st = (u8)(3 + (st % 8));
                            vTaskDelay(pdMS_TO_TICKS(st));*/

                        _re_cca:
                            // would be able to update tx_time to remain correct if it werent for the need for this check...
                            /*if ((sync_timestamp() - tx_time) >= MAX_CCA_TIME) {
                                cc_dbg_v("tx: cca timeout t=%lu", sync_timestamp());
                                cc_strobe(dev, CC1200_SFTX);
                                tx_time = 0;
                                goto _end_tx;
                            }*/

                            // fifo check: new: go idle if a fifo refill is needed
                            if (cc_get(dev, CC1200_NUM_TXBYTES) != (pkt->len + 1)) {
                                // this seems to specifically happen when receving a packet during tx (pretty sure)
                                if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);
                                cc_fifo_write(dev, (u8 *)pkt, pkt->len + 1);
                                //cc_dbg_v("tx: fifo refill");
                            }

                            //cc_dbg("tx: cca retry t=%lu ch=%u st=0x%02X", sync_timestamp(), chan_cur, cc_strobe(dev,CC1200_SNOP));

                            // TODO: maybe make sure CCA is actually definitely enabled before this
                            assert(cca_enabled);
                            cca_run();
                            cc_strobe(dev, CC1200_STX); // TODO: Should this be SFTXON?
                            tx_time = sync_timestamp();

                            // update wait time but NOT channel
                            if (sync_time) {
                                ticks = sync_timestamp() - sync_time;
                                remaining = chan_time - (ticks % chan_time);
                            } else {
                                ticks = sync_timestamp() - start_time;
                                remaining = chan_time - (ticks % chan_time);
                            }

                            continue;

                        case CC1200_MARC_STATUS1_RX_FINISHED:
                            // UPDATE: keeping rx on during cca makes LBT not work. so this won't happen anymore.

                            // an awkward situation, sort of -- packet received during cca, which is totally possible in some
                            // cases. might as well handle it as 'normal'. RX during CCA can be disabled by disabling sync word
                            // detection, but we probably care about packets all the time.

                            cc_dbg("WHOA: RX DURING TX");

                            cc_strobe(dev, CC1200_SIDLE); // NOTE: could be unnecessary/impactful
                            nphy_rx(true);
                            //break;
                            goto _re_cca;

                        case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
                            // NEW: do not try to salvage packets ASDASDASDTGWE
                            //nphy_rx(false); // try to salvage packet(s)

                            // fall through for forced uncoditional flush

                        case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:
                            cc_strobe(dev, CC1200_SIDLE); // seems important (not sure yet)?
                            cc_strobe(dev, CC1200_SFRX);
                            cc_dbg_v("tx: rx fifo error");
                            // fall through

                        case CC1200_MARC_STATUS1_CRC:
                            // assumption: crc autoflush register set, nothing to do here other than continue tx
                            goto _re_cca;


                        case CC1200_MARC_STATUS1_NO_FAILURE:
                            // although other things could be happening, we are busy waiting for the result of a tx.
                            // it was assumed before (hoprefully) that there would be channel time to do so, so that means
                            // if the packet is sent close to the end of a slot, we'll be late to the next one. such is life.

                            // however: sometimes a cca does not generate a result (!) and no tx status is indicated, so check
                            // for timeout below (TODO: either use diff timeout when not cca or rename the 'max cca timeout' var)

                            // NEW: looks like we finally got to the point where this needs to happen elsewhere
                            //if ((sync_timestamp() - tx_time) >= MAX_CCA_TIME*2) {
                            //    cc_dbg("tx: timeout t=%lu", sync_timestamp());
                            //    if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);
                            //    goto _end_tx;
                            //}

                            // TODO: what about periodic checking of the same thing below when an outbound packet is pending
                            // and no interrupts are generated??
                            continue;


                        case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
                            // sometimes the fifo underflows during a cca when an rx or rx error happens.
                            // so just refill the fifo here and go again. this might be a degenerate case
                            // now that the NUM_TXBYTES check in the cca handling above is done.
                            cc_strobe(dev, CC1200_SFTX);
                            cc_fifo_write(dev, (u8 *)pkt, pkt->len);
                            cc_dbg("tx: tx fifo underflow");
                            goto _re_cca;

                        case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
                            cc_strobe(dev, CC1200_SIDLE); // seems important (more sure: rx fifo fails often shortly after)?
                            cc_strobe(dev, CC1200_SFRX); // ^ same as above reason
                            cc_strobe(dev, CC1200_SFTX);
                            tx_time = 0;
                            cc_dbg_v("tx: tx fifo overflow");
                            goto _end_tx;

                        default:
                            st = cc_get(dev, CC1200_SNOP);
                            cc_dbg("tx: weird outcome: ms=0x%02X st=0x%02X", ms, st);

                            // fall through

                        case CC1200_MARC_STATUS1_TX_FINISHED:
                            //if (!sync_time /*|| !chan_cur*/) sync_time = sync_timestamp();

                            // just sent a sync packet
                            //if (boss && (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_SYNC)) sync_time = sync_timestamp();
                            // maybe the boss never has a sync time?

                            //tx_time = sync_timestamp() - 1; // HUGE impact -- bad idea

                            // DEBUG: extra info
                            //printf("tx/%u: seq=%u len=%u t=%lu\r\n", (u8)chan_cur, ((app_pkt_t *)pkt)->seq, ((app_pkt_t *)pkt)->len, sync_timestamp());

                            //if ((void*)pkt == &pkt_sync) {
                            //    cc_dbg_v("sync: sent. flag=0x%02X", pkt_sync.hdr.flag);
                            //}

                        _end_tx:
                            /*if (tx_time) {
                                const u32 elapsed = sync_timestamp() - tx_time;

                                if (elapsed >= 15) {
                                    cc_dbg_v("tx: len=%u time=%lu", pkt->len, elapsed);
                                }
                            }*/

                            if ((void*)pkt != &pkt_sync) {

                                if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {
                                    //tx_time = 0; // hmm
                                }

                                free(pkt);

                            } else {
                                sync_needed = false;
                                tx_time = 0; // could be redundant depending on what done when sent
                            }

                            pkt = NULL;

                            // TODO: find a better place to restore the sync word settings?
                            cca_disable();

                            // don't srx or update channel or check for tx, let it happen below.
                            break;
                    }
                } else if (ms) {
                    switch (ms) {
                        case CC1200_MARC_STATUS1_RX_FINISHED:
                            tx_time = 0; // only ungate next tx if not currently busy with a tx
                            //cc_strobe(dev, CC1200_SIDLE); // NOTE: could be unnecessary/impactful (copied from RX_FIN above and recently moved to before nphy_rx)
                            nphy_rx(true);

                            if (boss && sync_needed) {
                                pkt_sync.ts = sync_timestamp() - sync_time;
                                pkt = (rf_pkt_t *) &pkt_sync;

                                //cc_strobe(dev, CC1200_SIDLE); // not needed, but slows the tx a tiny bit to improve chance of receipt

                                cca_disable();

                                if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);

                                cc_strobe(dev, CC1200_STX);
                                cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);

                                tx_time = sync_timestamp();
                                //cc_dbg_v("sync: ch=%lu ts=%lu", sync_chan, pkt_sync.ts);
                            }
                            break;

                        case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
                        case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
                            st = cc_strobe(dev, CC1200_SFTX);
                            cc_dbg("rx: tx fifo error: ms=0x%02X st=0x%02X", ms, st);
                            break;

                        case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
                            // When doing high-rate back-and-forth with two nodes using CCAs but not waiting after sending,
                            // packets seem to pile up 3 at a time in the RX queue while we wait for RX_FINISHED, meaning the
                            // first two get salvaged from the fifo here but the third is always a few bytes short. It seems
                            // that changing the RXOFF mode to RX prevents such situations. The alternative might be to watch
                            // for the SYNC_RXTX falling edge, but it's unclear whether the current FIFO handling logic can
                            // deal with it properly (reading a packet while receiving another one might require the use of
                            // RXFIRST/RXLAST).
                            // The RXOFF change is sensible so far and does not seem to have adverse side effects.

                            // NEW: do not try to salvage packets
                            //nphy_rx(false); // try to salvage packet(s)

                            // fall through for forced uncoditional flush

                        case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:
                            cc_strobe(dev, CC1200_SIDLE); // seems important (not sure yet)? (copied from above)
                            st = cc_strobe(dev, CC1200_SFRX);
                            cc_dbg_v("rx: rx fifo error: ms=0x%02X st=0x%02X", ms, st);
                            break;

                        case CC1200_MARC_STATUS1_CRC:
                            // assumption: crc autoflush register set, nothing to do here other than continue rx
                            break;

                        default:
                            st = cc_get(dev, CC1200_SNOP);
                            cc_dbg("task: weird outcome: ms=0x%02X st=0x%02X", ms, st);
                            break;
                    }
                }
            }

        } else {
            // timeout
        }


        if (sync_time) {
            ticks = sync_timestamp() - sync_time;
            chan_ticks = (ticks % chan_time);
            remaining = chan_time - chan_ticks;

            if (chan_set((ticks / chan_time) % chan_count)) {

                if (pkt) {
                    const u8 tx_bytes = cc_get(dev, CC1200_NUM_TXBYTES);

                    if (tx_bytes) cc_strobe(dev, CC1200_SFTX);

                    cc_dbg(
                            "chan: switched during TX! TXBYTES=%u plen=%u is_sync=%u elapsed=%lu",
                            tx_bytes, pkt->len, (void*)pkt == &pkt_sync, sync_timestamp() - tx_time
                    );

                    tx_time = 0;

                    if ((void*)pkt != &pkt_sync) free(pkt);
                    pkt = NULL;
                    cc_strobe(dev, CC1200_SIDLE); // may cause redundancies but probably safer...
                    cca_disable();
                }

                // shouldn't throw things off too much...
                if (sync_time == start_time) {
                    sync_time = sync_timestamp();
                }

                ticks = sync_timestamp() - sync_time;
                chan_ticks = (ticks % chan_time);
                remaining = chan_time - chan_ticks;

                if (boss) {
                    pkt_sync.ts = ticks;
                    pkt = (rf_pkt_t *) &pkt_sync;

                    //cc_strobe(dev, CC1200_SIDLE); // not needed, but slows the tx a tiny bit to improve chance of receipt

                    cca_disable();

                    if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);

                    cc_strobe(dev, CC1200_STX);
                    cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);

                    tx_time = sync_timestamp(); // maybe different for this? -- no; will affect ability to time out
                    //cc_dbg_v("sync: ch=%lu ts=%lu", sync_chan, pkt_sync.ts);
                }
            }

        } else {
            ticks = sync_timestamp() - start_time;
            chan_ticks = (ticks % chan_time);
            remaining = chan_time - chan_ticks;
        }

        if (pkt) {
            const u32 tx_elapsed = sync_timestamp() - tx_time;

            if (tx_elapsed >= MAX_CCA_TIME*2) {
                cc_dbg("tx: timeout t=%lu", sync_timestamp());
                if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);
                goto _end_tx;
            }


        } else {
            if (xQueuePeek(nphy.txq, &pkt, 0) && pkt/*for the ide...*/) {
                //ticks = sync_timestamp() - tx_time;
                u32 pkt_time = cc_get_tx_time(dev, pkt->len) /*+ 1*/;

                if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {

                    //pkt_time = 0; // is this quite right?

                    if (chan_ticks < (ticks=10) || remaining < (ticks=(10+pkt_time))) {
                        pkt = NULL;
                        remaining = ticks;
                        continue;
                    }

                } else {

                    // LBT minimum is 5ms

                    if ((sync_timestamp() - tx_time) < (ticks=pkt_time*2) || chan_ticks <= (ticks=20) || remaining <= (ticks=(20+pkt_time))) {
                        pkt = NULL;
                        remaining = ticks;
                        continue;
                    }

                }

                xQueueReceive(nphy.txq, &pkt, 0);

                cca = !(((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE);


                if (!sync_time) ((phy_pkt_t *)pkt)->hdr.flag |= PHY_PKT_FLAG_NOSYNC;

                if (cca) {
                    retry = MAX_CCA_RETRY;
                    //cc_strobe(dev, CC1200_SIDLE);
                    cca_enable();
                    cca_run();
                } else {
                    cca_disable();
                    cc_strobe(dev, CC1200_SIDLE); // is this needed? always?
                }

                cc_strobe(dev, CC1200_STX);
                cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);
                tx_time = sync_timestamp();
            }
        }

        if (!pkt) {
            ensure_rx();
        }

        if (sync_time) {
            ticks = sync_timestamp() - sync_time;
            chan_ticks = (ticks % chan_time);
            remaining = chan_time - chan_ticks;
        } else {
            ticks = sync_timestamp() - start_time;
            chan_ticks = (ticks % chan_time);
            remaining = chan_time - chan_ticks;
        }
    }
}

static void nphy_dispatch_task(void *param __unused)
{
    const xQueueHandle rxq = nphy.rxq;
    const nphy_rx_t rx = nphy.rx;
    phy_pkt_t *spkt = alloca(256);
    phy_pkt_t *pkt = NULL;

    while (1) {
        if (xQueueReceive(rxq, &pkt, portMAX_DELAY) && pkt) {
            memcpy(spkt, pkt, 1 + pkt->hdr.len);
            free(pkt);
            if (rx) rx(spkt->hdr.flag, spkt->data, spkt->hdr.len - sizeof(phy_pkt_t) + 1); // TODO: maybe require cb and remove check
        }
    }
}
