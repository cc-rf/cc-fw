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



/**
 * TODO: At the PHY level, we're really dealing with FRAMES rather than PACKETS.
 *          Change the semantics here to reflect that.
 */

#define PHY_PKT_FLAG_SYNC       1   // is a sync packet
#define PHY_PKT_FLAG_NOSYNC     2   // not synced

typedef struct __packed {
    u8 len;
    u8 flag;

} phy_pkt_hdr_t;

typedef struct __packed {
    u8 chan;
    s8 rssi;
    u8 lqi;

} phy_pkt_meta_t;

typedef struct __packed {
    phy_pkt_hdr_t hdr;
    u8 data[];

} phy_pkt_t;

typedef struct __packed {
    phy_pkt_hdr_t hdr;
    s32 ts;

} phy_sync_pkt_t;

typedef struct __packed {
    phy_pkt_meta_t meta;
    phy_pkt_t pkt;

} phy_recv_queue_t;

static void isr_mcu_wake(void);
static void isr_ctl_lna(void);
static void isr_ctl_pa(void);

static void cca_setup(void);
static void cca_run(void);
static void cca_end(void);

static bool nphy_rx(bool flush);

static void nphy_dispatch_task(void *param);
static void nphy_task(void *param);

#define CC_RSSI_OFFSET      (s8)(-81 - 15 /*+ 11*/)

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

static struct {
    xTaskHandle task;
    xTaskHandle disp;
    xQueueHandle txq;
    xQueueHandle rxq;
    nphy_rx_t rx;

} nphy = {NULL};




extern u32 sync_timestamp(void);


static void ensure_rx(void);


#define FREQ_BASE       905000000
#define FREQ_BW         800000
#define CHAN_COUNT      25
#define CHAN_TIME       100000
#define MAX_CCA_RETRY   3
#define MAX_CCA_TIME    13 //NOTE: When using LBT, backoff time minimum is 5

static const u32 freq_base      = FREQ_BASE;
static const u32 freq_side_bw   = FREQ_BW / 2;
static const u32 chan_count     = CHAN_COUNT;
static const u32 chan_time      = CHAN_TIME;

#include <cc/chan.h>
#include <fsl_rnga.h>
#include <itm.h>
#include <stdatomic.h>
#include <malloc.h>
#include <cc/amp.h>


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

static bool boss;
static bool sync_needed;
static bool sync_slow;
static u32 sync_last;
static s32 sync_time;



static inline bool chan_set(u32 chan)
{
    if (chan >= chan_count) {
        assert(chan < (2 * chan_count));
        chan = (2 * chan_count) - chan - 1;
    }

    if (chan != chan_cur) {
        chan_cur = chan;

        u8 st = cc_strobe(dev, CC1200_SIDLE | CC1200_ACCESS_READ);

        // chance of switching during rx -- TODO: maybe try to extract valid packets?
        if (st & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFRX);

        do {
            st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;

        } while (st != CC1200_STATE_IDLE);

        chan_select(&chnl.group, chnl.hop_table[chan]);
        //cc_dbg_v("#%u\t%lu\t%lu\t\t%lu", chan, chnl.hop_table[chan], sync_timestamp(), sync_timestamp() - sync_time);

        return true;
    }

    return false;
}

s32 nphy_mem_count = 0;

static inline void *nphy_malloc(size_t size)
{
    void *ptr = malloc(size);
    if (ptr) ++nphy_mem_count;
    return ptr;
}

static inline void nphy_free(void *ptr)
{
    if (ptr) --nphy_mem_count;
    return free(ptr);
}


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


    nphy.txq = xQueueCreate(5, sizeof(void *)); assert(nphy.txq);
    nphy.rxq = xQueueCreate(7, sizeof(phy_recv_queue_t *)); assert(nphy.rxq);
    nphy.rx = rx;


    amp_init(dev);
    amp_ctrl(dev, AMP_HGM, true);

    amp_ctrl(dev, AMP_LNA, true);
    chan_grp_init(&chnl.group, chnl.hop_table);
    chan_grp_calibrate(&chnl.group);
    amp_ctrl(dev, AMP_LNA, false);


    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(0), CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP);
    isrd_configure(2, 10, kPORT_InterruptRisingEdge, isr_mcu_wake, 0);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(1), CC1200_IOCFG_GPIO_CFG_LNA_PD);
    isrd_configure(2, 11, kPORT_InterruptEitherEdge, isr_ctl_lna, 1);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(2), CC1200_IOCFG_GPIO_CFG_PA_PD);
    isrd_configure(2, 12, kPORT_InterruptEitherEdge, isr_ctl_pa, 1);


    if (!xTaskCreate(nphy_task, "nphy:main", TASK_STACK_SIZE_LARGE, NULL, TASK_PRIO_HIGH + 1, &nphy.task)) {
        cc_dbg("[%u] error: unable to create main task", dev);
        return false;
    }

    if (!xTaskCreate(nphy_dispatch_task, "nphy:disp", TASK_STACK_SIZE_LARGE, NULL, TASK_PRIO_HIGH, &nphy.disp)) {
        cc_dbg("[%u] error: unable to create dispatch task", dev);
        return false;
    }

    // ensure the tasks run even if their priority is lower
    vTaskDelay(pdMS_TO_TICKS(chan_time/4));

    return true;
}

static nphy_hook_t hook_sync = NULL;

void nphy_hook_sync(nphy_hook_t hook)
{
    hook_sync = hook;
}


#define NOTIFY_MASK_ISR    2
#define NOTIFY_MASK_TX     4

void nphy_tx(u8 flag, u8 *buf, u8 len)
{
    phy_pkt_t *qpkt = nphy_malloc(len + sizeof(phy_pkt_t)); assert(qpkt);
    if (len && buf) memcpy(qpkt->data, buf, len);
    qpkt->hdr.len = len + (sizeof(phy_pkt_t) - sizeof(rf_pkt_t)); // NOTE: upon receive, this reflects size minus phy header, but here it is all the data bytes
    qpkt->hdr.flag = flag & (u8)PHY_PKT_FLAG_IMMEDIATE;

    if (qpkt->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {

        if (xQueueSendToFront(nphy.txq, &qpkt, pdMS_TO_TICKS(500))) {
            xTaskNotify(nphy.task, NOTIFY_MASK_TX, eSetBits);
        } else {
            cc_dbg("tx pkt queue immediate fail");
            nphy_free(qpkt);
        }

    } else if (xQueueSend(nphy.txq, &qpkt, portMAX_DELAY/*pdMS_TO_TICKS(500)*/)) {
        xTaskNotify(nphy.task, NOTIFY_MASK_TX, eSetBits);
    } else {
        cc_dbg("tx pkt queue fail");
        nphy_free(qpkt);
    }
}

static void isr_mcu_wake(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(nphy.task, NOTIFY_MASK_ISR, eSetBits, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void isr_ctl_lna(void)
{
    amp_ctrl(dev, AMP_LNA, !isrd_state(2, 11));
}

static void isr_ctl_pa(void)
{
    amp_ctrl(dev, AMP_PA, !isrd_state(2, 12));
}


/*typedef enum __packed {
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
}*/

static bool process_packet(rf_pkt_t *pkt, s8 rssi, u8 lqi)
{
    phy_pkt_t *const ppkt = (phy_pkt_t *)pkt;

    if (ppkt->hdr.len < sizeof(phy_pkt_hdr_t)) {
        cc_dbg("rx: pkt too small for header");
        return false;
    }

    if (ppkt->hdr.flag & PHY_PKT_FLAG_SYNC) {
        const u32 ts = sync_timestamp();
        const phy_sync_pkt_t *const spkt = (phy_sync_pkt_t *) pkt;

        if (!boss) {
            //itm_puts(0, ".");
            const bool ppkt_sync_slow = (ppkt->hdr.flag & PHY_PKT_FLAG_NOSYNC) != 0;

            //if (!ppkt_sync_slow || sync_slow || ((ts - sync_last) >= (CHAN_TIME*CHAN_COUNT))) {
                sync_slow = ppkt_sync_slow;
                sync_last = sync_timestamp();
                sync_time = sync_last - spkt->ts;
                if (hook_sync) hook_sync(chan_cur);
            //}
        }

        return true;
    }

    ppkt->hdr.len -= (sizeof(phy_pkt_t) - sizeof(rf_pkt_t));

    if (ppkt->hdr.flag & PHY_PKT_FLAG_NOSYNC) {
        if (boss && !sync_needed) {
            sync_needed = true;
        }
    }

    // TODO: check for size underflow vs. phy packet size
    phy_recv_queue_t *recv = nphy_malloc(sizeof(phy_recv_queue_t) + ppkt->hdr.len); assert(recv);

    recv->meta = (phy_pkt_meta_t){
            .chan = (u8)chnl.group.cur->id,
            .rssi = rssi,
            .lqi = lqi
    };

    memcpy(&recv->pkt, ppkt, sizeof(phy_pkt_hdr_t) + ppkt->hdr.len);

    if (recv->pkt.hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {
        if (!xQueueSendToFront(nphy.rxq, &recv, pdMS_TO_TICKS(100))) {
            cc_dbg("rx pkt queue immediate fail");
            nphy_free(recv);
        }

        return true;
    } else {
        if (!xQueueSend(nphy.rxq, &recv, pdMS_TO_TICKS(100))) {
            cc_dbg("rx pkt queue fail");
            nphy_free(recv);
        }

        return false;
    }
}

static bool nphy_rx(bool flush)
{
    /* Assumption (may change later): variable packet length, 2 status bytes -> 3 total. */
    const static u8 PKT_OVERHEAD = 3;

    u8 len = cc_get(dev, CC1200_NUM_RXBYTES);

    if (len < PKT_OVERHEAD) {
        if (flush) cc_strobe(dev, CC1200_SFRX);

        if (len && flush) {
            cc_dbg("len=%u < PKT_OVERHEAD=%u", len, PKT_OVERHEAD);
        }

        return false;
    }

    rf_pkt_t *spkt;
    u8 *buf = alloca(len);
    size_t pkt_count = 0;
    bool got_imm = false;

    cc_fifo_read(dev, buf, len);

    while (len > PKT_OVERHEAD) {
        spkt = (rf_pkt_t *)buf;

        if (spkt->len > PHY_FRAME_SIZE_MAX) {
            // NOTE: _v added newly, but this is a useful error to see when timing is off. same applies for below
            cc_dbg_v("[%u] c=%u malformed: len[header]=%u > len[max]=%u  (len[fifo]=%u)", dev, pkt_count+1, spkt->len, PHY_FRAME_SIZE_MAX, len);
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
                got_imm |= process_packet(spkt, rssi, lqi);
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

    return got_imm;
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

static void ensure_rx(void)
{
    //if (marc_2pin_status() == MARC_2PIN_STATUS_RX) {
    //    return;
    //}

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


static void nphy_dispatch_task(void *param __unused)
{
    const xQueueHandle rxq = nphy.rxq;
    const nphy_rx_t rx = nphy.rx; assert(rx);
    phy_recv_queue_t *recv = NULL;

    while (1) {
        if (xQueueReceive(rxq, &recv, portMAX_DELAY) && recv) {
            rx(recv->pkt.hdr.flag, recv->pkt.hdr.len, recv->pkt.data, recv->meta.rssi, recv->meta.lqi); // TODO: maybe require cb and remove check
            nphy_free(recv);
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


static void nphy_task(void *param __unused)
{
    rf_pkt_t *pkt = NULL;
    u8 ms = 0, st;

    u32 ts;
    s32 ticks;
    u32 chan_ticks;
    u32 remaining = 0;
    u32 tx_next = 0;
    u32 notify;

    //bool cca = false;

    rf_state_t rf_state = RF_STATE_NONE;
    loop_state_t loop_state = LOOP_STATE_NONE;

    phy_sync_pkt_t pkt_sync = {
            .hdr = {
                    .len = sizeof(phy_sync_pkt_t) - 1,
                    .flag = (u8)(PHY_PKT_FLAG_IMMEDIATE | PHY_PKT_FLAG_SYNC | (!boss ? PHY_PKT_FLAG_NOSYNC : 0))
            },
            .ts = 0
    };

    sync_slow = !boss;
    sync_needed = false;
    sync_last = 0;
    sync_time = 0;

    const s32 start_time = sync_timestamp();

    if (boss) {
        sync_needed = true;
        loop_state = LOOP_STATE_TX;
    } else {
        chan_set(0);
        cc_strobe(dev, CC1200_SRX);
    }


    while (1) {
        if (xTaskNotifyWait(0, UINT32_MAX, &notify, pdMS_TO_TICKS((remaining+1000)/1000))) {

            if (notify & NOTIFY_MASK_ISR) {

                ms = cc_get(dev, CC1200_MARC_STATUS1);

                if (ms) {
                    //cc_dbg_v("isr: ms1=0x%02X st=0x%02X t=%lu 2Ps=%u", ms, cc_strobe(dev, CC1200_SNOP), sync_timestamp(), marc_2pin_status());
                }

                switch (ms) {
                    case CC1200_MARC_STATUS1_RX_FINISHED:
                        nphy_rx(true);

                        //if (!nphy_rx(true)) {
                        //    if (!pkt) tx_next = 0;
                        //} else {
                        //    if (!pkt || !tx_next) tx_next = sync_timestamp() + 2000; // TODO: Use pkt time or find a good value for this
                        //}

                        rf_state = RF_STATE_RX_END;
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
                    goto _loop_end;
            }

        }

        if (sync_time) {
            ts = sync_timestamp();
            ticks = ts - sync_time;
            const u32 chan_time_cur = chan_time * (sync_slow ? 2 : 1);

            if (!boss && ((ts - sync_last) >= (CHAN_TIME * CHAN_COUNT)) && sync_last) {
                cc_dbg("sync dropped: last=%lu now=%lu", sync_last, ts);
                sync_time = 0;
                ticks = 0;
                remaining = chan_time;
            } else {
                remaining = chan_time_cur - (ticks % chan_time_cur);

                /*if (remaining <= 1000) {
                    continue;
                }*/
            }

            if (chan_set((ticks / chan_time_cur) % (chan_count * 2))) {
                if (pkt) {
                    const u8 tx_bytes = cc_get(dev, CC1200_NUM_TXBYTES);

                    if (tx_bytes) cc_strobe(dev, CC1200_SFTX);

                    cc_dbg(
                            "chan: switched during TX! TXBYTES=%u plen=%u is_sync=%u time=%lu next=%lu rem=%lu",
                            tx_bytes, pkt->len, (void *) pkt == &pkt_sync, sync_timestamp(), tx_next, remaining
                    );

                    if ((void *) pkt != &pkt_sync) nphy_free(pkt);
                    pkt = NULL;
                    cc_strobe(dev, CC1200_SIDLE); // may cause redundancies but probably safer...
                    ///cca_disable();
                }

                if (boss) {
                    sync_needed = true;
                    loop_state = LOOP_STATE_TX;
                } else {
                    loop_state = LOOP_STATE_RX;
                }
            }
        } else if (!boss) {
            ts = sync_timestamp();
            ticks = ts - start_time;
            remaining = (2 * chan_time) - (ticks % (2 * chan_time));

            /*if (remaining <= 1000) {
                continue;
            }*/

            if (chan_set((ticks / (2 * chan_time)) % (chan_count * 2))) {
                if (pkt) {
                    const u8 tx_bytes = cc_get(dev, CC1200_NUM_TXBYTES);

                    if (tx_bytes) cc_strobe(dev, CC1200_SFTX);

                    cc_dbg(
                            "chan: switched during TX! TXBYTES=%u plen=%u is_sync=%u time=%lu next=%lu rem=%lu <NOSYNC>",
                            tx_bytes, pkt->len, (void *) pkt == &pkt_sync, sync_timestamp(), tx_next, remaining
                    );

                    if ((void *) pkt != &pkt_sync) nphy_free(pkt);
                    pkt = NULL;
                    cc_strobe(dev, CC1200_SIDLE); // may cause redundancies but probably safer...
                    ///cca_disable();
                }

                sync_needed = true;
                loop_state = LOOP_STATE_TX;
            }
        }

        loop_state_t loop_state_next = loop_state;

        do {
            //cc_dbg_v("loop: state=%s next=%s t=%lu", loop_state_str[loop_state], loop_state_str[loop_state_next], sync_timestamp());
            loop_state = loop_state_next;
            loop_state_next = loop_state;

            switch (loop_state) {
                case LOOP_STATE_RX:
                    if (pkt) {
                        // This seems to now happen from time to time when ACKs turn around really fast

                        //cc_dbg("WARNING: rx during tx. t=%lu len=%u is_sync=%u tx_next=%lu",
                        //       sync_timestamp(), pkt->len,
                        //       ((phy_pkt_hdr_t *)pkt)->flag & PHY_PKT_FLAG_SYNC != 0, tx_next
                        //);

                        loop_state_next = LOOP_STATE_TX;
                        continue;
                    }

                    switch (rf_state) {
                        case RF_STATE_RX_ERR:
                            //cc_dbg("rx fail: ms=0x%02X t=%lu e_t=%lu", ms, sync_timestamp(), sync_timestamp() - sync_time);
                        case RF_STATE_RX_RUN:
                            break;

                        default:
                        case RF_STATE_RX_END:
                            rf_state = RF_STATE_RX_RUN;
                            // TODO: perhaps make this choice a little more smart

                            //if (!tx_next) {
                                loop_state_next = LOOP_STATE_TX;
                                continue;
                            //}

                            break;
                    }

                    if (sync_needed || (tx_next && sync_timestamp() >= tx_next)) {
                        loop_state_next = LOOP_STATE_TX;
                        continue;
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
                                if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {
                                    if ((void*)pkt == &pkt_sync) {
                                        //cc_dbg_v("tx: <sync>sent t=%lu", sync_timestamp());
                                    } else {
                                        //cc_dbg_v("tx: <immd>sent t=%lu", sync_timestamp());
                                        nphy_free(pkt);
                                        //tx_next = 0; // allow after any immediate
                                    }
                                } else {
                                    assert(pkt != (void*)&pkt_sync);
                                    nphy_free(pkt);
                                    //cc_dbg_v("tx: <norm>sent t=%lu", sync_timestamp());

                                    //if (!sync_needed) {
                                        // NOTE: ^ This should actually always happen!

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
                        //assert(sync_time);

                        pkt = (rf_pkt_t *) &pkt_sync;

                        //cc_strobe(dev, CC1200_SIDLE); // not needed, but slows the tx a tiny bit to improve chance of receipt
                        ///cca_disable();

                        // NEW: SNOP instead of SIDLE?
                        if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);

                        do {
                            st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;

                        } while (st != CC1200_STATE_IDLE);

                        //cc_update(dev, CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_M, CC1200_RFEND_CFG0_TXOFF_MODE_RX);

                        if (boss && !sync_time) {
                            chan_set(0);
                            sync_time = sync_timestamp();
                            ticks = 0;
                        } else {
                            ticks = sync_timestamp() - sync_time;
                        }

                        tx_next = cc_get_tx_time(dev, pkt->len);
                        pkt_sync.ts = ticks + tx_next + 1000;

                        cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);
                        cc_strobe(dev, CC1200_STX);

                        tx_next += sync_timestamp();
                        //cc_dbg_v("sync: ch=%lu ts=%lu now=%lu next=%lu", chan_cur, pkt_sync.ts, sync_timestamp(), tx_next);

                        sync_needed = false;
                        if (hook_sync) hook_sync(chan_cur);

                    } else if (!pkt && xQueuePeek(nphy.txq, &pkt, 0) && pkt/*for the ide...*/) {
                        u32 pkt_time = cc_get_tx_time(dev, pkt->len); // NOTE: things work because this rounds up, and is never zero.

                        if (sync_time && ((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {

                            ts = sync_timestamp();

                            if (tx_next > ts) {
                                pkt = NULL;
                                if (rf_state == RF_STATE_TX_END) loop_state_next = LOOP_STATE_RX;
                                continue;
                            }

                            ticks = ts - sync_time;
                            chan_ticks = (ticks % chan_time);
                            remaining = chan_time - chan_ticks;

                            if (chan_ticks < 20000) {

                                tx_next = ts + (20000 - chan_ticks);
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;
                            }

                            if (remaining < (20000 + pkt_time)) {

                                tx_next = ts + ((20000 + pkt_time) - remaining);
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;
                            }

                            tx_next = (3 * pkt_time) / 2;
                            //cc_update(dev, CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_M, CC1200_RFEND_CFG0_TXOFF_MODE_IDLE);

                        } else {
                            ts = sync_timestamp();

                            if (tx_next > ts) {
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;
                            }

                            if (!sync_time) {
                                ((phy_pkt_t *)pkt)->hdr.flag |= PHY_PKT_FLAG_NOSYNC;

                                // NEW: this will trigger a response, so give some time and make not immediate flagged
                                if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE)
                                    ((phy_pkt_t *)pkt)->hdr.flag &= ~PHY_PKT_FLAG_IMMEDIATE;
                            }

                            ticks = ts - (sync_time ? sync_time : start_time);
                            chan_ticks = (ticks % chan_time);
                            remaining = chan_time - chan_ticks;

                            // LBT minimum is 5ms

                            if (chan_ticks < 20000) {

                                tx_next = ts + (20000 - chan_ticks);
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;

                            }

                            if (remaining < (20000 + pkt_time)) {

                                tx_next = ts + ((20000 + pkt_time) - remaining);
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;
                            }

                            tx_next = 3 * pkt_time;
                            //cc_update(dev, CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_M, CC1200_RFEND_CFG0_TXOFF_MODE_RX);
                        }

                        xQueueReceive(nphy.txq, &pkt, 0);

                        //cca = false;//!(((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE);

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

                        //cc_strobe(dev, CC1200_SIDLE); // This is not required but an RXTX_SWITCH seems to be too fast for our receiver logic

                        cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);
                        cc_strobe(dev, CC1200_STX);
                        if (tx_next) tx_next = sync_timestamp() + tx_next;
                        //cc_dbg_v("tx: sent now=%lu next=%lu", sync_timestamp(), tx_next);

                    } else if (!pkt) {

                        if (tx_next) {
                            tx_next = 0;
                        }

                        loop_state_next = LOOP_STATE_RX;
                        continue;

                    } else if (pkt) {

                        if (!tx_next) {
                            cc_dbg("WARNING: cannot check for frame timeout due to zero tx_next");
                        } else {
                            ts = sync_timestamp();

                            if (ts >= (tx_next+7000)) {
                                cc_dbg("tx: timed out: now=%lu tx_next=%lu", ts, tx_next);
                                if ((void *)pkt != &pkt_sync) nphy_free(pkt);
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;
                            }
                        }
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
                //cc_dbg("ts=%lu >= tx_next=%lu", ts, tx_next);
                remaining = 0;
                //tx_next = 0;
                continue;
            }

            if ((ts + remaining) > tx_next) {
                remaining = tx_next - ts;
                //cc_dbg_v("rf: wait=%lu now=%lu tx_next=%lu ticks=%li chan_ticks=%lu", remaining, ts, tx_next, ticks, chan_ticks);
                continue;
            }
        }
    }
}
