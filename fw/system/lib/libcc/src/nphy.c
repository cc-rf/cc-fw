#include <cc/nphy.h>
#include <cc/io.h>
#include <cc/cfg.h>

#include <cc/spi.h>
#include <alloca.h>
#include <string.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <malloc.h>
#include <cc/freq.h>

#include <cc/sys/kinetis/isrd.h>
#include <sclk.h>

#include <cc/chan.h>
#include <fsl_rnga.h>
#include <itm.h>
#include <stdatomic.h>
#include <malloc.h>
#include <cc/amp.h>
#include <cc/sys/kinetis/pit.h>
#include <cc/type.h>


#define PHY_PKT_FLAG_SYNC       1   // is a sync packet
#define PHY_PKT_FLAG_NOSYNC     2   // not synced
#define PHY_PKT_FLAG_RECAL      8

#define PHY_TXQ_LEN             3

typedef struct __packed {
    u8 len;
    u8 cell;
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
    u8 data[PHY_FRAME_SIZE_MAX];

} phy_static_pkt_t;

typedef struct __packed {
    phy_pkt_hdr_t hdr;
    u8 padding[1];

} phy_sync_pkt_t;

typedef struct __packed {
    phy_pkt_meta_t meta;
    phy_pkt_t pkt;

} phy_recv_queue_t;

static void isr_mcu_wake(void);
static void isr_ctl_lna(void);
static void isr_ctl_pa(void);
static void isr_chan_hop(pit_t pit __unused, void *param __unused);

static void cca_run(void);

static bool nphy_rx(bool flush);

static void nphy_task(void *param);

#define CC_RSSI_OFFSET      (s8)(-81 - 3)

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
        {CC1200_PKT_CFG2,   /*CC1200_PKT_CFG2_CCA_MODE_ALWAYS*/CC1200_PKT_CFG2_CCA_MODE_NOT_RX/*CC1200_PKT_CFG2_CCA_MODE_RSSI_THR_ETSI_LBT*/},
        {CC1200_PKT_CFG1,   CC1200_PKT_CFG1_CRC_CFG_ON_INIT_1D0F | CC1200_PKT_CFG1_ADDR_CHECK_CFG_ON_NO_BCAST | CC1200_PKT_CFG1_APPEND_STATUS | CC1200_PKT_CFG1_WHITE_DATA},
        {CC1200_PKT_CFG0,   CC1200_PKT_CFG0_LENGTH_CONFIG_VARIABLE},
        {CC1200_PKT_LEN,    125}, // CRC Autoflush limits this to 127 in variable packet length mode, and then there needs to be room for the two status bytes
        {CC1200_DEV_ADDR,   0x00},
        //{CC1200_SERIAL_STATUS, CC1200_SERIAL_STATUS_IOC_SYNC_PINS_EN}, // Enable access to GPIO state in CC1200_GPIO_STATUS.GPIO_STATE

        {CC1200_RNDGEN,     CC1200_RNDGEN_EN}, // Needed for random backoff for LBT CCA: https://e2e.ti.com/support/wireless_connectivity/f/156/t/370230
        {CC1200_FIFO_CFG,   CC1200_FIFO_CFG_CRC_AUTOFLUSH /*!CC1200_FIFO_CFG_CRC_AUTOFLUSH*/},

        {CC1200_AGC_GAIN_ADJUST,    (u8)CC_RSSI_OFFSET},
        {CC1200_AGC_CS_THR,         (u8)(-CC_RSSI_OFFSET - (s8)90)},


        {CC1200_SETTLING_CFG, 0x3}, // Defaults except never auto calibrate

};


#undef cc_dbg_v
#define cc_dbg_v(format, ...) cc_dbg(format, ##__VA_ARGS__ ) //cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )

#undef assert
#define assert(x) if (!(x)) { itm_printf(0, "ASSERT FAIL: ( " #x " ) on line %u of %s\r\n", __LINE__, __FILE__); asm("bkpt #0"); while (1) { asm("nop"); } }


static const cc_dev_t dev = 0;

#define PHY_TASK_STACK_SIZE     (TASK_STACK_SIZE_HUGE / sizeof(StackType_t))

static struct {
    xTaskHandle task;
    StaticTask_t task_static;
    StackType_t task_stack[PHY_TASK_STACK_SIZE];

    //xTaskHandle disp;

    xQueueHandle txq;
    StaticQueue_t txq_static;
    phy_static_pkt_t txq_buf[PHY_TXQ_LEN];

    //xQueueHandle rxq;
    nphy_rx_t rx;

} nphy;


static void ensure_rx(void);


#define FREQ_BASE       902125000u
#define FREQ_BW         950000u
#define CHAN_COUNT      25u
#define CHAN_TIME       20000u
#define RECAL_CYCLES    5

#define MAX_CCA_RETRY   3
#define MAX_CCA_TIME    13 //NOTE: When using LBT, backoff time minimum is 5

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

/*static*/ u32 tx_times[PHY_FRAME_SIZE_MAX + 1];

/*static*/ chan_t chan_cur = (chan_t)(-1);
chan_t chan_prev = (chan_t)(-1);

static bool boss;
static bool sync_needed;
//static bool recal_needed = false;
static sclk_t sync_last;
static sclk_t sync_time;
static u8 cell_cur;
/*static*/ pit_t pit_hop;
static volatile phy_static_pkt_t pkt_ack = { 0 };

static inline bool chan_set(chan_t chan)
{
    if (chan >= CHAN_COUNT) {
        assert(chan < (2 * CHAN_COUNT - 2));
        chan = (chan_t)(2 * CHAN_COUNT - 1) - chan - (chan_t)1;
    }

    if (chan != chan_cur) {
        chan_cur = chan;
        cc_strobe(dev, CC1200_SIDLE);
        chan_select(&chnl.group, chnl.hop_table[chan]);
        return true;
    }

    return false;
}

static inline void chan_recal(chan_t chan)
{
    chan_recalibrate(&chnl.group, chnl.hop_table[chan]);
}

bool nphy_init(u8 cell, bool sync_master, nphy_rx_t rx)
{
    sclk_init();

    nphy.rx = rx;
    cell_cur = cell;
    boss = sync_master;

    for (u8 i = 0; i <= PHY_FRAME_SIZE_MAX; ++i) {
        tx_times[i] = cc_get_tx_time(dev, i);
    }

    nphy.txq = xQueueCreateStatic(PHY_TXQ_LEN, sizeof(nphy.txq_buf[0]), (u8 *)nphy.txq_buf, &nphy.txq_static); assert(nphy.txq);

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

    cc_set(dev, CC1200_DEV_ADDR, cell);

    amp_init(dev);
    amp_ctrl(dev, AMP_HGM, true);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(0), CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP);
    isrd_configure(2, 10, kPORT_InterruptRisingEdge, isr_mcu_wake, 0);
    // For use with SPI1/external radio:
    //isrd_configure(4, 4, kPORT_InterruptRisingEdge, isr_mcu_wake, 0);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(1), CC1200_IOCFG_GPIO_CFG_LNA_PD);
    isrd_configure(2, 11, kPORT_InterruptEitherEdge, isr_ctl_lna, 1);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(2), CC1200_IOCFG_GPIO_CFG_PA_PD);
    isrd_configure(2, 12, kPORT_InterruptEitherEdge, isr_ctl_pa, 1);

    chan_grp_init(&chnl.group, chnl.hop_table);
    chan_table_reorder(&chnl.group, cell, chnl.hop_table);
    chan_grp_calibrate(&chnl.group);

    pit_init();

    pit_hop = pit_alloc(&(pit_cfg_t){
            .handler = isr_chan_hop,
            .period = pit_nsec_tick(CHAN_TIME * 1000)
    });

    nphy.task = xTaskCreateStatic(nphy_task, "nphy:main", PHY_TASK_STACK_SIZE, NULL, TASK_PRIO_HIGHEST, nphy.task_stack, &nphy.task_static);

    // ensure the tasks run even if their priority is lower
    vTaskDelay(pdMS_TO_TICKS(CHAN_TIME/4000));

    return true;
}

static nphy_hook_t hook_sync = NULL;

void nphy_hook_sync(nphy_hook_t hook)
{
    hook_sync = hook;
}


#define NOTIFY_MASK_ISR    (1<<1)
#define NOTIFY_MASK_TX     (1<<2)
#define NOTIFY_MASK_HOP    (1<<3)
#define NOTIFY_MASK_ALL    (NOTIFY_MASK_ISR | NOTIFY_MASK_TX | NOTIFY_MASK_HOP)

void nphy_tx(u8 flag, u8 *buf, u8 len)
{
    if (len > PHY_FRAME_SIZE_MAX) {
        cc_dbg("frame too big: %u > %u", len, PHY_FRAME_SIZE_MAX);
        return;
    }

    phy_static_pkt_t sqpkt = {
            .hdr = {
                    // NOTE: upon receive, this reflects size minus phy header, but here it is all the data bytes
                    .len = len + (sizeof(phy_pkt_t) - sizeof(rf_pkt_t)),
                    .cell = cell_cur,
                    .flag = flag & (u8)PHY_PKT_FLAG_IMMEDIATE/*The only flag allowed currently*/
            }
    };

    if (len && buf) memcpy(sqpkt.data, buf, len);

    if (sqpkt.hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {

        if (xTaskGetCurrentTaskHandle() == nphy.task && !pkt_ack.hdr.flag) {

            memcpy((void *) &pkt_ack, &sqpkt, sizeof(pkt_ack.hdr) + len);

        } else {

            if (xQueueSendToFront(nphy.txq, &sqpkt, portMAX_DELAY)) {
                xTaskNotify(nphy.task, NOTIFY_MASK_TX, eSetBits);
            } else {
                cc_dbg("tx pkt queue immediate fail");
            }
        }

        //taskYIELD();

    } else if (xQueueSend(nphy.txq, &sqpkt, portMAX_DELAY/*pdMS_TO_TICKS(500)*/)) {
        xTaskNotify(nphy.task, NOTIFY_MASK_TX, eSetBits);
    } else {
        cc_dbg("tx pkt queue fail");
    }
}

u32 nphy_delay(u8 len)
{
    /**
     * Max delay: Sync packet (850us) + sync lead time (500us) + gate (800us) + TX time + timing resolution? (~300us) + CCA failure ...
     */
    return 2500 + tx_times[len];
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

static void isr_chan_hop(pit_t pit __unused, void *param __unused)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(nphy.task, NOTIFY_MASK_HOP, eSetBits, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
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
        return false;
    }

    if (ppkt->hdr.flag & PHY_PKT_FLAG_SYNC) {
        //const phy_sync_pkt_t *const spkt = (phy_sync_pkt_t *) pkt;

        if (!boss) {
            //if (!ppkt_sync_slow || sync_slow || ((ts - sync_last) >= (CHAN_TIME*CHAN_COUNT))) {
                sync_last = sclk_time();
                sync_time = sync_last;
                pit_restart(pit_hop);

                /*if (!sync_time || (llabs((s64)sync_time - sync_time_new) > 1000)) {
                    cc_dbg("realigning channel hop timer");

                    //s64 ticks = sclk_time() - sync_time;
                    //u32 chan_ticks = (u32)(ticks % CHAN_TIME);
                    //u32 remaining = CHAN_TIME - chan_ticks;
                    //pit_stop(pit_hop);
                    //pit_set_period(remaining);
                }*/

                /*if (ppkt->hdr.flag & PHY_PKT_FLAG_RECAL) {
                    recal_needed = true;
                }*/

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
    /*phy_recv_queue_t *recv = nphy_malloc(sizeof(phy_recv_queue_t) + ppkt->hdr.len); assert(recv);

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
    }*/

    nphy.rx(ppkt->hdr.flag, ppkt->hdr.len, ppkt->data, rssi, lqi);

    return (ppkt->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) != 0;
}

static bool nphy_rx(bool flush)
{
    const static u8 PKT_OVERHEAD = 3; // length, 2x status

    u8 len = cc_get(dev, CC1200_NUM_RXBYTES);

    if (len < PKT_OVERHEAD) {
        if (flush) cc_strobe(dev, CC1200_SFRX);
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
            //cc_dbg_v("[%u] c=%u malformed: len[header]=%u > len[max]=%u  (len[fifo]=%u)", dev, pkt_count+1, spkt->len, PHY_FRAME_SIZE_MAX, len);
            break;
        }

        if (spkt->len > (len - PKT_OVERHEAD)) {
            //cc_dbg_v("[%u] c=%u underflow: len[header]=%u > len[fifo]=%u", dev, pkt_count+1, spkt->len, len);
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
                //cc_dbg_v("[%u] c=%u bad crc", dev, pkt_count);
                // NEW: don't trust anything else in the buffer
                break;
            }
        } else {
            //cc_dbg_v("[%u] c=%u empty", dev, pkt_count);
            // NEW: this is weird, should def stop
            break;
        }

        len -= spkt->len + PKT_OVERHEAD;
        buf += spkt->len + PKT_OVERHEAD;
    }

    if (flush && len) cc_strobe(dev, CC1200_SFRX);

    return got_imm;
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
            if (cc_strobe(dev, CC1200_SIDLE | CC1200_ACCESS_READ) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFRX);
            cc_dbg_v("rx fifo error");
            break;

        case CC1200_STATE_TXFIFO_ERROR:
            if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);
            cc_dbg_v("tx fifo error");
            break;
    }

    cc_strobe(dev, CC1200_SRX);

    /*do {
        st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;

    }  while (st != CC1200_STATE_RX);*/
}

//static bool cca_enabled = false;

void cca_enable(void)
{
    /*if (!cca_enabled) {
        cca_enabled = true;
        //cc_update(dev, CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RXOFF_MODE_M, CC1200_RFEND_CFG1_RXOFF_MODE_TX);
        //cc_update(dev, CC1200_PKT_CFG2, CC1200_PKT_CFG2_CCA_MODE_M, CC1200_PKT_CFG2_CCA_MODE_RSSI_THR_ETSI_LBT);
        //cc_update(dev, CC1200_MODCFG_DEV_E, CC1200_MODCFG_DEV_E_MODEM_MODE_M, CC1200_MODCFG_DEV_E_MODEM_MODE_CARRIER_SENSE);
        cc_update(dev, CC1200_SYNC_CFG1, CC1200_SYNC_CFG1_SYNC_THR_M, 0);
    }*/
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
    /*if (cca_enabled) {
        cca_enabled = false;
        //cc_update(dev, CC1200_PKT_CFG2, CC1200_PKT_CFG2_CCA_MODE_M, CC1200_PKT_CFG2_CCA_MODE_ALWAYS);
        //cc_update(dev, CC1200_MODCFG_DEV_E, CC1200_MODCFG_DEV_E_MODEM_MODE_M, CC1200_MODCFG_DEV_E_MODEM_MODE_NORMAL);
        cc_update(dev, CC1200_SYNC_CFG1, CC1200_SYNC_CFG1_SYNC_THR_M, 0xA); // NOTE: cheating here and using known value
    }*/
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
    phy_static_pkt_t spkt;
    rf_pkt_t *pkt = NULL;
    u8 ms = 0;

    sclk_t ts;
    u32 chan_ticks;
    chan_t chan_cycle_cur = (chan_t)(-1);
    u32 remaining = 0;
    sclk_t tx_next = 0;
    sclk_t tx_time = 0;
    sclk_t hop_time = (sclk_t)(-1);
    //u32 recal_cycles = RECAL_CYCLES + 1;
    u32 notify;

    //bool tx_off_idle = false;
    //bool cca = false;

    rf_state_t rf_state = RF_STATE_NONE;
    loop_state_t loop_state = LOOP_STATE_NONE;

    phy_sync_pkt_t pkt_sync = {
            .hdr = {
                    .len = sizeof(phy_sync_pkt_t) - 1,
                    .flag = (u8)(PHY_PKT_FLAG_IMMEDIATE | PHY_PKT_FLAG_SYNC | (!boss ? PHY_PKT_FLAG_NOSYNC : 0))
            },
    };

    sync_needed = false;
    sync_last = 0;
    sync_time = 0;

    //const sclk_t start_time = sclk_time();

    const pit_tick_t pit_hop_period = pit_nsec_tick(CHAN_TIME * 1000);
    const pit_tick_t pit_hop_period_adj = pit_nsec_tick((CHAN_TIME + 950) * 1000);

    if (boss) {
        loop_state = LOOP_STATE_TX;
        xTaskNotify(xTaskGetCurrentTaskHandle(), NOTIFY_MASK_HOP, eSetBits);
    } else {
        chan_set(0);
        chan_cycle_cur = 0;
        cc_strobe(dev, CC1200_SRX);
        loop_state = LOOP_STATE_RX;
    }

    /*sclk_t loop_time_start = sclk_time();
    sclk_t loop_time;
    sclk_t loop_time_prev = loop_time_start;
    sclk_t loop_time_sum = 0;
    u32 loop_time_count = 0;*/

    while (1) {

        /*ts = sclk_time();
        loop_time = ts - loop_time_prev;
        loop_time_sum += loop_time;

        if (++loop_time_count == 10000) {
            const u32 elapsed = (u32)(ts - loop_time_start);
            const u32 loop_time_avg_usec = (u32)(loop_time_sum / loop_time_count);
            const u32 loops_per_sec = (u32)(1000000ul * (u64)loop_time_count / elapsed);
            itm_printf(0, "(loop) time=%lu us\tfreq=%lu Hz\r\n", loop_time_avg_usec, loops_per_sec);
            loop_time_sum = 0;
            loop_time_count = 0;
            loop_time_start = sclk_time();
        }

        loop_time_prev = ts;*/

        #define pdUS_TO_TICKS( xTimeInUs ) ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInUs ) * ( TickType_t ) configTICK_RATE_HZ ) / ( TickType_t ) 1000000 ) )

        if (xTaskNotifyWait(0, NOTIFY_MASK_ALL, &notify, remaining ? pdUS_TO_TICKS(remaining) : portMAX_DELAY)) {
            loop_state = LOOP_STATE_NONE;

            if (notify & NOTIFY_MASK_ISR) {
                ms = cc_get(dev, CC1200_MARC_STATUS1);

                switch (ms) {
                    case CC1200_MARC_STATUS1_RX_FINISHED:
                        nphy_rx(true);//if (nphy_rx(true) /* && sync_time ?? */) tx_next = 0;
                        rf_state = RF_STATE_RX_END;
                        loop_state = LOOP_STATE_RX;
                        break;

                    case CC1200_MARC_STATUS1_TX_FINISHED:
                        rf_state = RF_STATE_TX_END;
                        loop_state = LOOP_STATE_TX;
                        break;

                    case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
                    case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:

                    case CC1200_MARC_STATUS1_ADDRESS:
                    case CC1200_MARC_STATUS1_CRC:
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
                        if (!(notify & ~NOTIFY_MASK_ISR)) goto _loop_end;
                        rf_state = RF_STATE_NONE;
                        break;
                }
            }

            if ((notify & NOTIFY_MASK_HOP) && (boss || sync_time)) {
                hop_time = ts = sclk_time();
                loop_state = LOOP_STATE_TX;

                if (!boss && !chan_cycle_cur && ((ts - sync_last) >= 5*(CHAN_TIME * CHAN_COUNT))) {
                    cc_dbg_v("sync lost: last=%lu now=%lu", SCLK_MSEC(sync_last), SCLK_MSEC(ts));
                    sync_time = 0;
                    goto _inner_loop;
                }

                chan_cycle_cur = (chan_t)(chan_cycle_cur + 1) % (chan_t)(CHAN_COUNT * 2 - 2);
                chan_prev = chan_cur;
                chan_set(chan_cycle_cur);
                //itm_printf(0, "hop: chan_cycle_cur=%u chan_cur=%u\r\n", chan_cycle_cur, chan_cur);

                if (boss) {
                    /**
                     * Dilemma: It would be nice to do this stuff immediately after the event to ensure
                     *  everything is synchronized, but a non-deterministic optimization within chan_set
                     *  causes some offsets on the order of <100us.
                     *  The current strategy removes that optimization to keep this deterministic but then
                     *  offsets the timing to put the boss a little bit behind. This is done by changing
                     *  the value of pit_hop_period_adj. +950us gives perfect timing sync, whereas +1000us
                     *  will have the boss be up to ~100us behind right here, deterministically.
                     */
                    if (!chan_cur) {
                        sync_time = ts;
                        sync_needed = true;

                        pit_stop(pit_hop);
                        pit_set_period(pit_hop, pit_hop_period_adj);
                        pit_start(pit_hop);

                    } else if (!chan_prev && chan_cur == 1) {
                        pit_stop(pit_hop);
                        pit_set_period(pit_hop, pit_hop_period);
                        pit_start(pit_hop);
                    }
                }

                if (hook_sync) hook_sync(chan_cur);

                /*if (!chan_cur) {
                    if (!--recal_cycles) {
                        recal_cycles = RECAL_CYCLES;
                        recal_needed = true;
                    }
                } else if (recal_needed && chan_cycle_cur == CHAN_COUNT) {
                    recal_needed = false;
                }

                if (recal_needed) {
                    chan_recal(chan_cur);
                    loop_state = LOOP_STATE_RX;
                } else {
                    loop_state = LOOP_STATE_TX;
                }*/

                if (pkt) {
                    const u8 tx_bytes = cc_get(dev, CC1200_NUM_TXBYTES);

                    /*if (tx_bytes)*/ {
                        // May have been done sending, TXBYTES will be zero if so.

                        if (!tx_bytes) {
                            pkt = NULL;
                            cca_disable();
                            itm_puts(0, "chan: hop tx clear (was done)\r\n");

                        } else if ((tx_bytes == pkt->len + 1) && (boss || chan_cur)) {
                            itm_puts(0, "chan: hop re-tx\r\n");
                            cc_strobe(dev, CC1200_STX);
                        } else {
                            cc_strobe(dev, CC1200_SFTX);

                            cc_dbg(
                                    "tx during chan hop canned: txbytes=%u len=%u chan=%u elapsed=%lu sent=%lu now=%lu rem=%lu",
                                    tx_bytes, pkt->len, chan_cur, (u32) (ts - tx_time), (u32) tx_time,
                                    (u32) ts, (u32) (pit_tick_nsec(pit_get_current(pit_hop)) / 1000u)
                            );

                            pkt = NULL;
                            cca_disable();
                        }
                    }
                }
            }

            if (notify & NOTIFY_MASK_TX) {
                if (!(notify & ~NOTIFY_MASK_TX)) {
                    if (pkt || tx_next) goto _loop_end;
                    rf_state = RF_STATE_NONE;
                    loop_state = LOOP_STATE_TX;
                    goto _inner_loop;
                }
            }

        } else {
            // timeout
            if (pkt) goto _loop_end;
            rf_state = RF_STATE_NONE;
            loop_state = LOOP_STATE_TX;

            /*switch (cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M) { // TODO: Maybe use MARCSTATE instead
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
            }*/

        }

        _inner_loop:;

        if (remaining) remaining = 0;

        loop_state_t loop_state_next = loop_state;

        do {
            //cc_dbg_v("loop: state=%s next=%s t=%lu", loop_state_str[loop_state], loop_state_str[loop_state_next], sclk_time());
            loop_state = loop_state_next;
            loop_state_next = loop_state;

            switch (loop_state) {
                case LOOP_STATE_RX:
                    if (pkt) {
                        // This seems to now happen from time to time when ACKs turn around really fast

                        //cc_dbg("WARNING: rx during tx. t=%lu len=%u is_sync=%u tx_next=%lu",
                        //       sclk_time(), pkt->len,
                        //       ((phy_pkt_hdr_t *)pkt)->flag & PHY_PKT_FLAG_SYNC != 0, tx_next
                        //);

                        loop_state_next = LOOP_STATE_TX;
                        continue;
                    }

                    switch (rf_state) {
                        case RF_STATE_RX_ERR:
                            //cc_dbg("rx fail: ms=0x%02X", ms);
                            rf_state = RF_STATE_NONE;
                            break;

                        default:
                        case RF_STATE_RX_RUN:
                            break;

                        case RF_STATE_RX_END:
                            rf_state = RF_STATE_NONE;
                            loop_state_next = LOOP_STATE_TX;
                            continue;
                    }

                    if (!pkt && (sync_needed || (tx_next && sclk_time() >= tx_next))) {
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
                            continue;

                        case RF_STATE_TX_ERR:
                            if ((ms == CC1200_MARC_STATUS1_TX_ON_CCA_FAILED) && pkt) {
                                itm_printf(0, "cca fail: p=0x%p\n", pkt);
                                cca_run();
                                cc_strobe(dev, CC1200_STX);
                                rf_state = RF_STATE_TX_RUN;
                                continue;
                            }

                            cc_dbg("tx fail: ms=0x%02X", ms);
                            if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);

                        case RF_STATE_TX_END:
                            if (pkt) {
                                if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {
                                    if ((void*)pkt == &pkt_ack) {
                                        pkt_ack.hdr.flag = 0;
                                        pkt_ack.hdr.len = 0;
                                    }
                                    if ((void*)pkt == &pkt_sync) {
                                        /*if (pkt_sync.hdr.flag & PHY_PKT_FLAG_RECAL) {
                                            pkt_sync.hdr.flag &= ~PHY_PKT_FLAG_RECAL;
                                        }*/
                                    }

                                    /*pkt = NULL;
                                    tx_next = sclk_time() + 1000;
                                    loop_state_next = LOOP_STATE_RX;
                                    rf_state = RF_STATE_RX_RUN;
                                    continue;*/

                                } else {
                                    pkt = NULL;
                                    cca_disable();
                                    tx_next = sclk_time() + 2 * tx_times[pkt->len] /*+ 1000*/;
                                    // Already in RX.
                                    //loop_state_next = LOOP_STATE_RX;
                                    rf_state = RF_STATE_RX_RUN;
                                    continue;
                                }

                                pkt = NULL;
                            } else {
                                cc_dbg("tx completion indicated but no packet pending");
                                // ^ This will now occur when the channel hop code above handles the lindering packet
                                cca_disable();
                            }

                            rf_state = RF_STATE_NONE;
                            break;
                    }

                    if (sync_needed) {
                        sync_needed = false;

                        assert(!pkt);
                        pkt = (rf_pkt_t *) &pkt_sync;

                        //if ((cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M) != CC1200_STATE_IDLE)
                        cc_strobe(dev, CC1200_SIDLE);

                        pkt_sync.hdr.cell = cell_cur;

                        /*if (recal_needed) {
                            pkt_sync.hdr.flag |= PHY_PKT_FLAG_RECAL;
                        }*/

                        cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);
                        cc_strobe(dev, CC1200_STX);
                        tx_time = sclk_time();

                    } else if (!pkt && (pkt_ack.hdr.flag || xQueuePeek(nphy.txq, &spkt, 0))) {
                        pkt = pkt_ack.hdr.flag ? (rf_pkt_t *)&pkt_ack : (rf_pkt_t *)&spkt;

                        const bool synced = (sync_time != 0);
                        const bool imm = synced && ((((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) != 0);

                        ts = sclk_time();
                        chan_ticks = (u32)(ts - hop_time);

                        if (chan_ticks > (CHAN_TIME - 300)) {
                            pkt = NULL;
                            continue;
                        }

                        remaining = CHAN_TIME - chan_ticks;

                        if (imm) {
                            if (!boss && !chan_cur) {

                                const u32 lead_time = 500 + tx_times[pkt_sync.hdr.len];

                                if (chan_ticks < lead_time) {

                                    tx_next = ts + (lead_time - chan_ticks);
                                    pkt = NULL;
                                    loop_state_next = LOOP_STATE_RX;
                                    continue;
                                }
                            }

                            if (remaining < (800 + tx_times[pkt->len])) {

                                tx_next = ts + remaining; // + 300;
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;
                            }

                        } else {

                            if (tx_next > (ts + 100)) {
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;
                            }

                            if (!boss && !chan_cur) {

                                const u32 lead_time = 500 + tx_times[pkt_sync.hdr.len];

                                if (chan_ticks < lead_time) {

                                    tx_next = ts + (lead_time - chan_ticks);
                                    pkt = NULL;
                                    loop_state_next = LOOP_STATE_RX;
                                    continue;
                                }
                            }

                            if (remaining < (800 + tx_times[pkt->len])) {

                                tx_next = ts + remaining; // + 300;
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;
                            }

                            if (!synced) {
                                ((phy_pkt_t *)pkt)->hdr.flag |= PHY_PKT_FLAG_NOSYNC;

                                if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE)
                                    ((phy_pkt_t *)pkt)->hdr.flag &= ~PHY_PKT_FLAG_IMMEDIATE;
                            }
                        }

                        if (pkt != (rf_pkt_t *)&pkt_ack) xQueueReceive(nphy.txq, &spkt, 0);

                        if (imm) {
                            if ((cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M) != CC1200_STATE_IDLE)
                                cc_strobe(dev, CC1200_SIDLE);
                        }

                        cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);

                        //if (!imm) {
                        //    cca_enable();
                        //    cca_run();
                        //}

                        cc_strobe(dev, CC1200_STX);
                        tx_time = sclk_time();
                        tx_next = 0;
                        //itm_printf(0, "tx: now=%lu rem=%lu\n", tx_time, remaining);

                    } else if (!pkt) {
                        if (tx_next && tx_next <= sclk_time()) tx_next = 0;
                        loop_state_next = LOOP_STATE_RX;
                    }

                    continue;

                default:
                    loop_state_next = LOOP_STATE_TX;
                    continue;
            }

        } while (loop_state_next != loop_state);


        _loop_end:

        if (tx_next /*&& uxQueueMessagesWaiting(nphy.txq)*/) {

            ts = sclk_time();
            remaining = (u32)(pit_tick_nsec(pit_get_current(pit_hop)) / 1000u);

            if ((ts + remaining) > tx_next) {
                remaining = (u32)(tx_next - ts);
            } else {
                remaining = 0;
            }

        } else {

            remaining = 0;
        }
    }
}
