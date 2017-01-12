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
    s32 ts;

} phy_sync_pkt_t;


static void isr_mcu_wake(void);
static void isr_marc_2pin_status_0(void);
static void isr_marc_2pin_status_1(void);

static void cca_setup(void);
static void cca_run(void);
static void cca_end(void);

static void nphy_rx(bool flush);

static void nphy_task(void *param);
static void nphy_dispatch_task(void *param);

static void rf_task(void *param);
static void rf_task_NEW(void *param);

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
        {CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RXOFF_MODE_IDLE/*TODO: what should this be?*/ | (0x7<<1) /*| CC1200_RFEND_CFG1_RX_TIME_QUAL_M*/},
        {CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_RX| CC1200_RFEND_CFG0_TERM_ON_BAD_PACKET_EN /*| 1*/},

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
#define MAX_CCA_RETRY   3//4//3//2//3
#define MAX_CCA_TIME    11 //NOTE: When using LBT, backoff time minimum is 5
#define MAX_PACKET_LEN  120

static const u32 freq_base      = FREQ_BASE;
static const u32 freq_side_bw   = FREQ_BW / 2;
static const u32 chan_count     = CHAN_COUNT;
static const u32 chan_time      = CHAN_TIME;

#include <cc/chan.h>
#include <fsl_rnga.h>
#include <itm.h>
#include <stdatomic.h>


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

static inline bool chan_set(const u32 chan)
{
    if (chan != chan_cur) {
        //cc_dbg_v("@%u->%u\r\n", chan_cur, chan);
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

        // NEW: don't do this because we'll always be sending a sync packet!!
        //ensure_rx(); // this kills the in-progress tx

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

    if (!xTaskCreate(/*nphy_task*/rf_task_NEW, "nphy:main", TASK_STACK_SIZE_LARGE/*(TASK_STACK_SIZE_DEFAULT * 3) / 2*/, NULL, TASK_PRIO_HIGH+1, &nphy.task)) {
        cc_dbg("[%u] error: unable to create main task", dev);
        isrd_configure(2, 10, kPORT_InterruptOrDMADisabled, NULL);
        return false;
    }

    if (!xTaskCreate(nphy_dispatch_task, "nphy:disp", (TASK_STACK_SIZE_DEFAULT * 3) / 2, NULL, TASK_PRIO_HIGH-1, &nphy.disp)) {
        cc_dbg("[%u] error: unable to create dispatch task", dev);
        isrd_configure(2, 10, kPORT_InterruptOrDMADisabled, NULL);
        return false;
    }

    chan_grp_init(&chnl.group, NULL);
    chan_grp_calibrate(&chnl.group);
    chan_set(0);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(0), CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP);
    isrd_configure(2, 10, kPORT_InterruptRisingEdge, isr_mcu_wake);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(1), CC1200_IOCFG_GPIO_CFG_MARC_2PIN_STATUS0);
    isrd_configure(2, 11, kPORT_InterruptRisingEdge | kPORT_InterruptFallingEdge, isr_marc_2pin_status_0);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(2), CC1200_IOCFG_GPIO_CFG_MARC_2PIN_STATUS1);
    isrd_configure(2, 12, kPORT_InterruptRisingEdge | kPORT_DMAFallingEdge, isr_marc_2pin_status_1);

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
        // TEMPORARY!!!
        //qpkt->hdr.flag &= ~PHY_PKT_FLAG_IMMEDIATE;

        if (xQueueSendToFront(nphy.txq, &qpkt, pdMS_TO_TICKS(500))) {
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


typedef enum __packed {
    MARC_2PIN_STATUS_SETTLING,
    MARC_2PIN_STATUS_TX,
    MARC_2PIN_STATUS_IDLE,
    MARC_2PIN_STATUS_RX,

} marc_2pin_status_t;

static volatile atomic_uint marc_2pin_status_field = 0;

static void isr_marc_2pin_status_0(void)
{
    if (isrd_state(2, 11)) {
        atomic_fetch_or(&marc_2pin_status_field, (1u << 0));
    } else {
        atomic_fetch_and(&marc_2pin_status_field, ~(1u << 0));
    }
}

static void isr_marc_2pin_status_1(void)
{
    if (isrd_state(2, 12)) {
        atomic_fetch_or(&marc_2pin_status_field, (1u << 1));
    } else {
        atomic_fetch_and(&marc_2pin_status_field, ~(1u << 1));
    }
}

static marc_2pin_status_t marc_2pin_status(void) {
    return (marc_2pin_status_t)marc_2pin_status_field;
}

static bool sync_needed = false;

static void process_packet(rf_pkt_t *pkt)
{
    phy_sync_pkt_t *const spkt = (phy_sync_pkt_t *)pkt;

    if (spkt->hdr.flag & PHY_PKT_FLAG_SYNC) {
        // TODO: make sure size is big enough before reading flag

        if (!boss) {
            sync_time = sync_timestamp() - spkt->ts - cc_get_tx_time(dev, spkt->hdr.len)/*NEW: include pkt_time + fudge*/;
            //itm_puts(0, ".sync\r\n");

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
    if (marc_2pin_status() == MARC_2PIN_STATUS_RX)
        return;

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

    //itm_puts(0, "SRX\n");
    //printf("SRX: st=0x%02X t=%lu\n", st, sync_timestamp());
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
                    .flag = PHY_PKT_FLAG_SYNC | PHY_PKT_FLAG_IMMEDIATE
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

                //cc_dbg_v("ms1=0x%02X st=0x%02X t=%lu 2Ps=%u", ms, cc_strobe(dev,CC1200_SNOP), sync_timestamp(), marc_2pin_status());

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

                                if (elapsed >= 20 || (!(((phy_pkt_t *)pkt)->hdr.flag && PHY_PKT_FLAG_IMMEDIATE) && elapsed < 5)) {
                                    cc_dbg_v("tx: len=%u time=%lu", pkt->len, elapsed);
                                }
                            }*/


                            if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {
                                if ((void*)pkt == &pkt_sync) {
                                    sync_needed = false;
                                    tx_time = 0;
                                } else {
                                    //tx_time = 0; // hmm
                                }
                            } else {
                                //cc_strobe(dev, CC1200_SIDLE);
                                cca_disable();
                                //cc_strobe(dev, CC1200_SRX);
                                free(pkt);
                            }

                            /*if ((void*)pkt != &pkt_sync) {

                                if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {
                                    //tx_time = 0; // hmm
                                }

                                free(pkt);

                            } else {
                                sync_needed = false;
                                tx_time = 0; // could be redundant depending on what done when sent
                            }*/

                            pkt = NULL;

                            // TODO: find a better place to restore the sync word settings?
                            //cca_disable();

                            // don't srx or update channel or check for tx, let it happen below.
                            break;
                    }
                } else if (ms) {
                    switch (ms) {
                        case CC1200_MARC_STATUS1_RX_FINISHED:
                            tx_time = 0; // only ungate next tx if not currently busy with a tx
                            //cc_strobe(dev, CC1200_SIDLE); // NOTE: could be unnecessary/impactful (copied from RX_FIN above and recently moved to before nphy_rx)
                            nphy_rx(true);

                            if (/*boss &&*/ sync_needed) {
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
                            st = cc_strobe(dev, CC1200_SFRX);
                            cc_strobe(dev, CC1200_SIDLE); // seems important (not sure yet)? (copied from above)
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
                            "chan: switched during TX! TXBYTES=%u plen=%u is_sync=%u elapsed=%lu time=%lu",
                            tx_bytes, pkt->len, (void*)pkt == &pkt_sync, sync_timestamp() - tx_time, sync_timestamp()
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

                    if ((ticks=chan_ticks) < 10 || (ticks=remaining) < (10+pkt_time)) {
                        pkt = NULL;
                        remaining = ticks;
                        //if (ticks >= 1) ensure_rx();
                        continue;
                    }

                } else {

                    // LBT minimum is 5ms

                    if ((ticks=(sync_timestamp() - tx_time)) < (pkt_time*2) || (ticks=chan_ticks) <= 20 || (ticks=remaining) <= (20+pkt_time)) {
                        pkt = NULL;
                        remaining = ticks;
                        //if (ticks >= 1) ensure_rx();
                        continue;
                    }

                }

                xQueueReceive(nphy.txq, &pkt, 0);

                cca = !(((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE);


                if (!sync_time) ((phy_pkt_t *)pkt)->hdr.flag |= PHY_PKT_FLAG_NOSYNC;

                if (cca) {
                    retry = MAX_CCA_RETRY;

                    //if (!cca_enabled) {
                    //    cc_strobe(dev, CC1200_SIDLE);
                    //}

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

typedef enum {
    RF_STATE_NONE,
    RF_STATE_RX_RUN,
    RF_STATE_TX_RUN,
    RF_STATE_RX_END,
    RF_STATE_TX_END,
    RF_STATE_RX_ERR,
    RF_STATE_TX_ERR,

} rf_state_t;

const static char *const rf_state_str[7] = {
        "(NONE)",
        "RX/RUN",
        "TX/RUN",
        "RX/FIN",
        "TX/FIN",
        "RX/ERR",
        "TX/ERR"
};


static void rf_task(void *param __unused)
{
    rf_pkt_t *pkt = NULL;
    u8 ms, st, retry = 0;

    u32 ticks = 0;
    u32 remaining = 0;
    u32 tx_time = 0;
    u32 chan_ticks = 0;
    u32 notify;

    bool cca = false;

    phy_sync_pkt_t pkt_sync = {
            .hdr = {
                    .len = sizeof(phy_sync_pkt_t) - 1,
                    .flag = PHY_PKT_FLAG_SYNC | PHY_PKT_FLAG_IMMEDIATE
            },
            .ts = 0
    };

    const u32 start_time = sync_timestamp();

    if (boss) {
        sync_time = start_time;
    }

    rf_state_t rf_state = RF_STATE_NONE;
    rf_state_t rf_state_next = RF_STATE_NONE;
    rf_state_t rf_state_prev;

    u32 tx_next = 0;

    while (1) {
        rf_state_prev = rf_state;

        if (xTaskNotifyWait(0, UINT32_MAX, &notify, pdMS_TO_TICKS(remaining))) {

            if (notify & NOTIFY_MASK_ISR) {

                ms = cc_get(dev, CC1200_MARC_STATUS1);

                if (ms) {
                    //cc_dbg_v("isr: ms1=0x%02X st=0x%02X t=%lu", ms, cc_strobe(dev, CC1200_SNOP), sync_timestamp());
                    cc_dbg_v("isr: ms1=0x%02X st=0x%02X t=%lu", ms, cc_strobe(dev, CC1200_SNOP), sync_timestamp());
                }

                switch (ms) {
                    case CC1200_MARC_STATUS1_RX_FINISHED:
                        nphy_rx(true);
                        rf_state = RF_STATE_RX_END;
                        break;

                    case CC1200_MARC_STATUS1_TX_FINISHED:
                        rf_state = RF_STATE_TX_END;
                        break;

                    case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
                    case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:
                        cc_strobe(dev, CC1200_SFRX);

                    case CC1200_MARC_STATUS1_ADDRESS: // TODO: need a flush here? (current this never happens but maybe will in the future)
                    case CC1200_MARC_STATUS1_CRC:
                        rf_state = RF_STATE_RX_ERR;
                        break;

                    case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
                    case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
                        cc_strobe(dev, CC1200_SFTX);

                    case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
                        rf_state = RF_STATE_TX_ERR;
                        break;

                    default:
                    case CC1200_MARC_STATUS1_NO_FAILURE:
                        rf_state = rf_state_prev;
                        //break;
                        // NEW: do nothing, it was a SIDLE
                        continue;
                }
            } else if (notify & NOTIFY_MASK_TX) {
                if (tx_next) {
                    const u32 now = sync_timestamp();

                    if (now < tx_next) {
                        remaining = tx_next - now;
                        continue;
                    }
                }

                rf_state = RF_STATE_TX_RUN;
            }

        } else {
            // timeout

            switch (cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M) { // TODO: Maybe use MARCSTATE instead
                case CC1200_STATE_IDLE:
                    rf_state = RF_STATE_NONE;
                    break;

                case CC1200_STATE_RX:
                    rf_state = RF_STATE_RX_RUN;
                    break;

                case CC1200_STATE_TX:
                    rf_state = RF_STATE_TX_RUN;
                    break;

                case CC1200_STATE_RXFIFO_ERROR:
                    ms = cc_get(dev, CC1200_MARC_STATUS1);
                    cc_strobe(dev, CC1200_SFRX);
                    rf_state = RF_STATE_RX_ERR;
                    break;

                case CC1200_STATE_TXFIFO_ERROR:
                    ms = cc_get(dev, CC1200_MARC_STATUS1);
                    cc_strobe(dev, CC1200_SFTX);
                    rf_state = RF_STATE_TX_ERR;
                    break;

                default:
                    rf_state = rf_state_prev;
            }

        }

        if (sync_time) {
            ticks = sync_timestamp() - sync_time;
            chan_ticks = (ticks % chan_time);
            remaining = chan_time - chan_ticks;

            if (chan_set((ticks / chan_time) % chan_count)) {
                //cc_dbg_v("switch! chan=%lu time=%lu", chan_cur, sync_timestamp());
                //cc_dbg_v("switch! time=%lu", sync_timestamp());
                printf("#%lu\r\n", sync_timestamp());

                /*if (rf_state && rf_state != RF_STATE_RX_RUN) {
                    cc_dbg("chan: unexpectected rf_state=%u", rf_state);
                }*/

                rf_state = RF_STATE_NONE;

                if (pkt) {
                    const u8 tx_bytes = cc_get(dev, CC1200_NUM_TXBYTES);

                    if (tx_bytes) cc_strobe(dev, CC1200_SFTX);

                    cc_dbg(
                            "chan: switched during TX! TXBYTES=%u plen=%u is_sync=%u elapsed=%lu time=%lu",
                            tx_bytes, pkt->len, (void*)pkt == &pkt_sync, sync_timestamp() - tx_time, sync_timestamp()
                    );

                    tx_time = 0;

                    if ((void*)pkt != &pkt_sync) free(pkt);
                    pkt = NULL;
                    cc_strobe(dev, CC1200_SIDLE); // may cause redundancies but probably safer...
                    ///cca_disable();
                }

                // shouldn't throw things off too much...
                if (sync_time == start_time) {
                    sync_time = sync_timestamp();
                }

                ticks = sync_timestamp() - sync_time;
                chan_ticks = (ticks % chan_time);
                remaining = chan_time - chan_ticks;

                if (boss) {
                    sync_needed = true;
                    rf_state = RF_STATE_TX_RUN;
                }
            }

        } else {
            ticks = sync_timestamp() - start_time;
            chan_ticks = (ticks % chan_time);
            remaining = chan_time - chan_ticks;
        }

        rf_state_next = rf_state;

        //if (!rf_state || rf_state != rf_state_prev)
        do {
            rf_state = rf_state_next;

            //cc_dbg_v("rf: time=%05u notif=0x%02X prev=%s state=%s", sync_timestamp(), notify, rf_state_str[rf_state_prev], rf_state_str[rf_state]);
            switch (rf_state) {
                case RF_STATE_RX_RUN:
                    if (!pkt) {
                        if (tx_next) {
                            const u32 now = sync_timestamp();

                            if (tx_next <= now) {
                                itm_puts(0, "back to tx\r\n");
                                rf_state_next = RF_STATE_TX_RUN;
                                break;
                            }
                        }

                        ensure_rx();
                    }

                    rf_state_next = RF_STATE_RX_RUN;
                    break;

                case RF_STATE_TX_RUN:
                    if (sync_needed) {
                        assert(!pkt);

                        pkt_sync.ts = ticks;
                        pkt = (rf_pkt_t *) &pkt_sync;

                        //cc_strobe(dev, CC1200_SIDLE); // not needed, but slows the tx a tiny bit to improve chance of receipt

                        ///cca_disable();

                        // NEW: SNOP instead of SIDLE
                        if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);

                        do {
                            st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;

                        } while (st != CC1200_STATE_IDLE);


                        cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);
                        cc_strobe(dev, CC1200_STX);

                        tx_time = sync_timestamp(); // maybe different for this? -- no; will affect ability to time out
                        tx_next = tx_time + 2;

                        //cc_dbg_v("sync: ch=%lu ts=%lu t=%lu", chan_cur, pkt_sync.ts, sync_timestamp());
                        cc_dbg_v("sync: t=%lu n=%lu", tx_time, tx_next);

                        sync_needed = false;
                        rf_state_next = RF_STATE_TX_RUN;

                    } else if (!pkt && xQueuePeek(nphy.txq, &pkt, 0) && pkt/*for the ide...*/) {
                        //ticks = sync_timestamp() - tx_time;
                        u32 pkt_time = cc_get_tx_time(dev, pkt->len) /*+ 1*/;

                        if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {

                            //pkt_time = 0; // is this quite right?

                            if ((ticks=chan_ticks) < 10 || (ticks=remaining) < (10+pkt_time)) {
                                pkt = NULL;
                                //remaining = ticks;
                                tx_next = sync_timestamp() + ticks;
                                //ticks = UINT32_MAX; // indicates to not update remaining
                                rf_state_next = RF_STATE_TX_RUN;
                                break;
                            }

                            tx_next = 0;

                        } else {

                            // LBT minimum is 5ms

                            if ((ticks=(sync_timestamp() - tx_time)) < (pkt_time*2) || (ticks=chan_ticks) <= 20 || (ticks=remaining) <= (20+pkt_time)) {
                                pkt = NULL;
                                //remaining = ticks;
                                tx_next = sync_timestamp() + ticks;
                                //ticks = UINT32_MAX; // indicates to not update remaining
                                rf_state_next = RF_STATE_RX_RUN;
                                break;
                            }

                            tx_next = 2*pkt_time;
                        }

                        xQueueReceive(nphy.txq, &pkt, 0);

                        cca = false;//!(((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE);

                        if (!sync_time) ((phy_pkt_t *)pkt)->hdr.flag |= PHY_PKT_FLAG_NOSYNC;

                        /*if (cca) {
                            retry = MAX_CCA_RETRY;
                            //if (!cca_enabled) {
                            //    cc_strobe(dev, CC1200_SIDLE);
                            //}
                            cca_enable();
                            cca_run();
                        } else*/ {
                            //cca_disable();

                            // NEW: don't do this
                            //cc_strobe(dev, CC1200_SIDLE); // is this needed? always?
                        }

                        cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);
                        cc_strobe(dev, CC1200_STX);
                        tx_time = sync_timestamp();
                        tx_next = tx_time + tx_next;
                        cc_dbg_v("tx: sent t=%lu n=%lu", tx_time, tx_next);

                        rf_state_next = RF_STATE_TX_RUN;

                    } else {
                        rf_state_next = RF_STATE_RX_RUN;
                    }

                    break;

                case RF_STATE_RX_END:
                    tx_time = 0; // TODO: be more smart... when last received was immediate, block a while

                    //if (sync_needed) {
                        rf_state_next = RF_STATE_TX_RUN;
                    //} else {
                    //    rf_state_next = RF_STATE_RX_RUN;
                    //}
                    break;

                case RF_STATE_TX_END:
                    if (pkt) {
                        tx_next = 0;


                        if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {
                            if ((void*)pkt == &pkt_sync) {
                                //sync_needed = false; // taken care of right after it's sent, meaning it can get triggered to be sent again immediately anyway and no reason to gate until the tx-end moment in the case where we will maybe rx during tx although that is designed to hopefully never happen in this configuration
                                tx_time = 0;
                                cc_dbg_v("tx: <sync>sent t=%lu", sync_timestamp());
                            } else {
                                cc_dbg_v("tx: <immd>sent t=%lu", sync_timestamp());
                                free(pkt);
                                //tx_time = 0; // hmm
                            }

                            rf_state_next = RF_STATE_TX_RUN;
                        } else {
                            //cc_strobe(dev, CC1200_SIDLE);
                            cca_disable();
                            //cc_strobe(dev, CC1200_SRX);
                            free(pkt);

                            rf_state_next = RF_STATE_RX_RUN;
                            cc_dbg_v("tx: <norm>sent t=%lu", sync_timestamp());
                        }

                        pkt = NULL;
                    } else {
                        rf_state_next = RF_STATE_RX_RUN;
                    }
                    break;

                case RF_STATE_RX_ERR:
                    cc_dbg_v("rx error: ms=0x%02X st=0x%02X", ms, cc_strobe(dev, CC1200_SNOP));
                    rf_state_next = RF_STATE_RX_END; // not really needed
                    break;

                case RF_STATE_TX_ERR:
                    cc_dbg_v("tx error: ms=0x%02X st=0x%02X", ms, cc_strobe(dev, CC1200_SNOP));
                    // TODO: check for CCA retry needed
                    rf_state_next = RF_STATE_TX_END;
                    break;

                default:
                case RF_STATE_NONE:
                    switch (rf_state_prev) {
                        case RF_STATE_RX_RUN:
                            rf_state_next = RF_STATE_RX_RUN;
                            break;

                        case RF_STATE_NONE:
                        case RF_STATE_TX_RUN:
                            rf_state_next = RF_STATE_TX_RUN; // ? does this make sense ?
                            break;

                        default:
                        case RF_STATE_RX_END:
                        case RF_STATE_TX_END:
                        case RF_STATE_RX_ERR:
                        case RF_STATE_TX_ERR:
                            rf_state_next = rf_state;
                            assert(false); // should never happen
                            break;
                    }
                    break;
            }

            //cc_dbg_v(
            //        "rf: time=%05u notif=0x%02X prev=%s state=%s next=%s",
            //        sync_timestamp(), notify, rf_state_str[rf_state_prev], rf_state_str[rf_state], rf_state_str[rf_state_next]
            //);

            //?/ rf_state = rf_state_next;

        } while (rf_state_next != rf_state);

        if (!tx_next) {
            if (sync_time) {
                ticks = sync_timestamp() - sync_time;
                chan_ticks = (ticks % chan_time);
                remaining = chan_time - chan_ticks;
            } else {
                ticks = sync_timestamp() - start_time;
                chan_ticks = (ticks % chan_time);
                remaining = chan_time - chan_ticks;
            }
        } else {
            const u32 now = sync_timestamp();

            if (now < tx_next) {
                remaining = tx_next - now;
                cc_dbg_v("rf: wait=%lu", remaining);
            } else {
                remaining = 0;
                tx_next = 1;
            }
        }
    }
}

/*

void x()
{
    // channel check


    switch (my_state) {
        case MY_RX:

            break;

        case MY_TX:

            break;
    }
}
*/

typedef enum {
    LOOP_STATE_NONE,
    LOOP_STATE_RX,
    LOOP_STATE_TX,

} loop_state_t;

const static char *const loop_state_str[3] = {
        "NA",
        "RX",
        "TX",
};


static void rf_task_NEW(void *param __unused)
{
    rf_pkt_t *pkt = NULL;
    u8 ms = 0, st, retry = 0;

    u32 ts;
    s32 ticks = 0;
    u32 remaining = 0;
    u32 tx_time = 0;
    u32 chan_ticks = 0;
    u32 notify;

    bool cca = false;

    phy_sync_pkt_t pkt_sync = {
            .hdr = {
                    .len = sizeof(phy_sync_pkt_t) - 1,
                    .flag = PHY_PKT_FLAG_SYNC | PHY_PKT_FLAG_IMMEDIATE
            },
            .ts = 0
    };

    const s32 start_time = sync_timestamp();

    if (boss) {
        sync_time = start_time;
    }

    rf_state_t rf_state = RF_STATE_NONE;
    rf_state_t rf_state_prev;

    u32 tx_next = 0;

    loop_state_t loop_state = LOOP_STATE_NONE;

    while (1) {
        rf_state_prev = rf_state;

        if (remaining > chan_time) {
            cc_dbg("remaining=%lu > %lu !!", remaining, chan_time);
        }

        if (xTaskNotifyWait(0, UINT32_MAX, &notify, pdMS_TO_TICKS(remaining))) {

            if (notify & NOTIFY_MASK_ISR) {

                ms = cc_get(dev, CC1200_MARC_STATUS1);

                if (ms) {
                    //cc_dbg_v("isr: ms1=0x%02X st=0x%02X t=%lu 2Ps=%u", ms, cc_strobe(dev, CC1200_SNOP), sync_timestamp(), marc_2pin_status());
                }

                switch (ms) {
                    case CC1200_MARC_STATUS1_RX_FINISHED:
                        nphy_rx(true);
                        rf_state = RF_STATE_RX_END;
                        // NEW: Block any new tx for 1 ms
                        //tx_time = sync_timestamp();
                        //tx_next = tx_time + 1;
                        // NEW: Maybe the opposite
                        tx_time = 0;
                        tx_next = 0;
                        //tx_time = sync_timestamp();
                        //tx_next = tx_time + 5;

                        ///loop_state = LOOP_STATE_TX;
                        loop_state = LOOP_STATE_RX;
                        break;

                    case CC1200_MARC_STATUS1_TX_FINISHED:
                        rf_state = RF_STATE_TX_END;
                        loop_state = LOOP_STATE_TX;
                        break;

                    case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
                    case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:

                    case CC1200_MARC_STATUS1_ADDRESS: // TODO: need a flush here? (current this never happens but maybe will in the future)
                    case CC1200_MARC_STATUS1_CRC: // NOTE: This doesn't maybe need a flush but was getting weirdness after lots of bad crcs...
                        cc_strobe(dev, CC1200_SFRX);
                        rf_state = RF_STATE_RX_ERR;
                        break;

                    case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
                    case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
                        cc_strobe(dev, CC1200_SFTX);

                    case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
                        rf_state = RF_STATE_TX_ERR;
                        break;

                    default:
                    case CC1200_MARC_STATUS1_NO_FAILURE:
                        //rf_state = rf_state_prev; // may not be true....
                        rf_state = RF_STATE_NONE;
                        goto _loop_end;
                }
            } else if (notify & NOTIFY_MASK_TX) {
                if (tx_next) {
                    goto _loop_end;
                }

                rf_state = RF_STATE_NONE; // TODO: make more accurate?
            }

        } else {
            // timeout

            switch (cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M) { // TODO: Maybe use MARCSTATE instead
                case CC1200_STATE_IDLE:
                    rf_state = RF_STATE_NONE;
                    break;

                case CC1200_STATE_RX:
                    rf_state = RF_STATE_RX_RUN;
                    break;

                case CC1200_STATE_TX:
                    rf_state = RF_STATE_TX_RUN;
                    break;

                case CC1200_STATE_RXFIFO_ERROR:
                    ms = cc_get(dev, CC1200_MARC_STATUS1);
                    cc_strobe(dev, CC1200_SFRX);
                    rf_state = RF_STATE_RX_ERR;
                    break;

                case CC1200_STATE_TXFIFO_ERROR:
                    ms = cc_get(dev, CC1200_MARC_STATUS1);
                    cc_strobe(dev, CC1200_SFTX);
                    rf_state = RF_STATE_TX_ERR;
                    break;

                default:
                    //cc_dbg("WEIRD STATE");
                    rf_state = rf_state_prev;
                    // don't know, maybe settling?
                    goto _loop_end;
            }

        }


        if (sync_time) {
            ts = sync_timestamp();
            ticks = ts - sync_time;
            remaining = chan_time - (ticks % chan_time);

            if (remaining <= 1) {
                remaining = 1;
                continue;
            }

            if (chan_set((ticks / chan_time) % chan_count)) {
                //cc_dbg_v("switch! chan=%lu time=%lu ticks=%li", chan_cur, ts, ticks);
                //cc_dbg_v("switch! time=%lu", sync_timestamp());
                //printf("#%lu\r\n", ts);

                /*if (rf_state && rf_state != RF_STATE_RX_RUN) {
                    cc_dbg("chan: unexpectected rf_state=%u", rf_state);
                }*/

                const bool first = loop_state == LOOP_STATE_NONE;

                loop_state = LOOP_STATE_RX;

                if (pkt) {
                    const u8 tx_bytes = cc_get(dev, CC1200_NUM_TXBYTES);

                    if (tx_bytes) cc_strobe(dev, CC1200_SFTX);

                    cc_dbg(
                            "chan: switched during TX! TXBYTES=%u plen=%u is_sync=%u elapsed=%lu tx_time=%lu time=%lu next=%lu rem=%lu",
                            tx_bytes, pkt->len, (void *) pkt == &pkt_sync, sync_timestamp() - tx_time, tx_time, sync_timestamp(), tx_next, remaining
                    );

                    // NEW: don't bother with these
                    //tx_time = 0;
                    //tx_next = 0;

                    if ((void *) pkt != &pkt_sync) free(pkt);
                    pkt = NULL;
                    cc_strobe(dev, CC1200_SIDLE); // may cause redundancies but probably safer...
                    ///cca_disable();
                }

                if (boss) {
                    sync_needed = true;
                    loop_state = LOOP_STATE_TX;

                    // shouldn't throw things off too much...
                    // ACTUALLY..... Must not do this. It does. Of course.
                    //if (first /*proxy: first time around the loop*/) {
                    //    sync_time = sync_timestamp();
                    //    //cc_dbg_v("switch: sync_time=%lu->%lu", start_time, sync_time);
                    //}

                    ///ticks = 0;
                    ///remaining = chan_time;
                } else {
                    // TODO: Is this actually different?
                    ///ticks = sync_timestamp() - sync_time;
                    ///remaining = chan_time - (ticks % chan_time);
                }
            }
        }

        loop_state_t loop_state_next = loop_state;
        //itm_puts(0,"LOOP RUN\r\n");

        do {
            //cc_dbg_v("loop: state=%s next=%s t=%lu", loop_state_str[loop_state], loop_state_str[loop_state_next], sync_timestamp());
            loop_state = loop_state_next;
            loop_state_next = loop_state;

            switch (loop_state) {
                case LOOP_STATE_RX:
                    //assert(!pkt);
                    if (pkt) {
                        cc_dbg("WARNING: rx during tx. t=%lu len=%u is_sync=%u",
                               sync_timestamp(), pkt->len,
                               ((phy_pkt_hdr_t *)pkt)->flag & PHY_PKT_FLAG_SYNC != 0
                        );
                    }

                    if (!pkt) {
                        if (tx_next && sync_timestamp() >= tx_next) {
                            loop_state_next = LOOP_STATE_TX;
                            continue;
                        }
                    }

                    switch (rf_state) {
                        case RF_STATE_RX_RUN:
                            //cc_dbg_v("rx-run: st=0x%02X", cc_strobe(dev, CC1200_SNOP));
                            break;

                        case RF_STATE_RX_ERR:
                            cc_dbg("rx fail: ms=0x%02X t=%lu e_t=%lu", ms, sync_timestamp(), sync_timestamp() - sync_time);

                        default:
                            // TODO: Strobe here when below is not true
                            //cc_strobe(dev, CC1200_SRX);

                        case RF_STATE_RX_END:
                            // TODO: Strobe here when RX OFF mode is not RX
                            //cc_dbg_v("SRX");
                            //itm_puts(0, "^");
                            //cc_strobe(dev, CC1200_SRX);

                            // NEW: maybe time to tx now
                            rf_state = RF_STATE_RX_RUN;
                            loop_state_next = LOOP_STATE_TX;
                            continue;

                            break;
                    }

                    ensure_rx();

                    continue;

                case LOOP_STATE_TX:
                    switch (rf_state) {
                        default:
                            break;

                        case RF_STATE_TX_RUN:
                            // do nothing, busy
                            continue;

                        case RF_STATE_TX_ERR:
                            cc_dbg("tx fail: ms=0x%02X", ms);

                        case RF_STATE_TX_END:
                            if (pkt) {
                                //tx_next = 0;

                                if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {
                                    if ((void*)pkt == &pkt_sync) {
                                        //NEW/tx_time = 0;
                                        //tx_next = 0;
                                        //cc_dbg_v("tx: <sync>sent t=%lu", sync_timestamp());
                                    } else {
                                        //cc_dbg_v("tx: <immd>sent t=%lu", sync_timestamp());
                                        free(pkt);
                                        //tx_time = 0; // hmm
                                        //tx_next = 0; // allow after any immediate

                                    }
                                } else {
                                    assert(pkt != (void*)&pkt_sync);
                                    free(pkt);
                                    //cc_dbg_v("tx: <norm>sent t=%lu", sync_timestamp());

                                    //if (!sync_needed) {
                                        // NOTE: ^ This should actually always happen!

                                        //cc_strobe(dev, CC1200_SIDLE);

                                        loop_state_next = LOOP_STATE_RX;
                                        pkt = NULL;
                                        continue;
                                    //}
                                }

                                pkt = NULL;
                            } else {
                                cc_dbg("WARNING: tx completion indicated without a packet pending");
                                //?? (includes tx error condition continuation)
                                //loop_state_next = LOOP_STATE_RX;
                                //continue;
                            }

                            break;
                    }

                    if (sync_needed) {
                        assert(!pkt);

                        ticks = sync_timestamp() - (sync_time ? sync_time : start_time);
                        remaining = chan_time - (ticks % chan_time);

                        pkt_sync.ts = ticks; // TODO: update?
                        pkt = (rf_pkt_t *) &pkt_sync;

                        //cc_strobe(dev, CC1200_SIDLE); // not needed, but slows the tx a tiny bit to improve chance of receipt
                        ///cca_disable();

                        // NEW: SNOP instead of SIDLE?
                        if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);

                        do {
                            st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;

                        } while (st != CC1200_STATE_IDLE);

                        // NEW: go straight to idle after this for potential immediate tx
                        cc_update(dev, CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_M, CC1200_RFEND_CFG0_TXOFF_MODE_IDLE);


                        cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);
                        cc_strobe(dev, CC1200_STX);

                        // TODO: WHAT to do here?
                        //tx_time = 0;//sync_timestamp(); // maybe different for this? -- no; will affect ability to time out
                        //tx_next = 0;//to stay in line with imm send behav below -- //tx_time + 2;

                        // NEW: treat this and imm packets kinda like normal
                        tx_time = sync_timestamp();
                        tx_next = tx_time + cc_get_tx_time(dev, pkt->len);

                        //cc_dbg_v("sync: ch=%lu ts=%lu t=%lu", chan_cur, pkt_sync.ts, sync_timestamp());
                        //cc_dbg_v("sync: t=%lu n=%lu", tx_time, tx_next);

                        sync_needed = false;

                    } else if (!pkt && xQueuePeek(nphy.txq, &pkt, 0) && pkt/*for the ide...*/) {
                        //ticks = sync_timestamp() - tx_time;
                        u32 pkt_time = cc_get_tx_time(dev, pkt->len) /*+ 1*/; // NOTE: things work because this rounds up, and is never zero.

                        if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {

                            ts = sync_timestamp();

                            /*if (tx_next && tx_next > ts) {
                                pkt = NULL;
                                continue;
                            }*/

                            ticks = ts - (sync_time ? sync_time : start_time);
                            chan_ticks = (ticks % chan_time);
                            remaining = chan_time - chan_ticks;

                           /*if ((ts - tx_time) < (pkt_time * 1)) {

                                tx_next = ts + ((pkt_time * 1) - (ts - tx_time));
                                pkt = NULL;
                                continue;

                            } else if (chan_ticks < 1) {

                                tx_next = ts + (1 - chan_ticks);
                                pkt = NULL;
                                continue;

                            } else*/ if (remaining < (2 + pkt_time)) {

                                tx_next = ts + ((2 + pkt_time) - remaining);
                                pkt = NULL;
                                continue;
                            }

                            tx_next = pkt_time;
                            cc_update(dev, CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_M, CC1200_RFEND_CFG0_TXOFF_MODE_IDLE);

                        } else {
                            ts = sync_timestamp();

                            if (tx_next && tx_next > ts) {
                                pkt = NULL;
                                continue;
                            }

                            ticks = ts - (sync_time ? sync_time : start_time);
                            chan_ticks = (ticks % chan_time);
                            remaining = chan_time - chan_ticks;

                            // LBT minimum is 5ms

                            if ((ts - tx_time) < (pkt_time * 3)) {

                                tx_next = ts + ((pkt_time * 3) - (ts - tx_time));
                                pkt = NULL;
                                continue;

                            } else if (chan_ticks <= 5) {

                                tx_next = ts + (5 - chan_ticks);
                                pkt = NULL;
                                continue;

                            } else if (remaining <= (20 + pkt_time)) {

                                tx_next = ts + ((20 + pkt_time) - remaining);
                                pkt = NULL;
                                continue;
                            }

                            tx_next = 3 * pkt_time;
                            cc_update(dev, CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_M, CC1200_RFEND_CFG0_TXOFF_MODE_RX);
                        }

                        xQueueReceive(nphy.txq, &pkt, 0);

                        cca = false;//!(((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE);

                        if (!sync_time) ((phy_pkt_t *)pkt)->hdr.flag |= PHY_PKT_FLAG_NOSYNC;

                        /*if (cca) {
                            retry = MAX_CCA_RETRY;
                            //if (!cca_enabled) {
                            //    cc_strobe(dev, CC1200_SIDLE);
                            //}
                            cca_enable();
                            cca_run();
                        } else*/ {
                            //cca_disable();
                            // ARGH WTF DO I DO HERE
                            //cc_strobe(dev, CC1200_SIDLE); // is this needed? always?
                        }

                        cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);
                        cc_strobe(dev, CC1200_STX);
                        tx_time = sync_timestamp();
                        if (tx_next) tx_next = tx_time + tx_next;
                        //cc_dbg_v("tx: sent t=%lu n=%lu", tx_time, tx_next);

                    } else if (!pkt) {
                        if (tx_next) {
                            tx_next = 0;
                        }
                        loop_state_next = LOOP_STATE_RX;
                    }

                    continue;

                default:
                    loop_state_next = LOOP_STATE_TX;
                    continue;
            }

        } while (loop_state_next != loop_state);


        _loop_end:
        ts = sync_timestamp();

        ticks = ts - (sync_time ? sync_time : start_time);
        chan_ticks = (ticks % chan_time);
        remaining = chan_time - chan_ticks;

        if (tx_next) {
            if (ts >= tx_next) {
                // TODO: Does this happen?
                remaining = 0;
                //tx_next = 0;
                continue;
            }

            if ((ts + remaining) > tx_next) {
                remaining = tx_next - ts;
                //cc_dbg_v("rf: wait=%lu  ts=%lu tx_time=%lu tx_next=%lu ticks=%li chan_ticks=%lu", remaining, ts, tx_time, tx_next, ticks, chan_ticks);
            }
        }
    }
}
