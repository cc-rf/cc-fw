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


#define PHY_PKT_FLAG_SYNC       1   // is a sync packet
#define PHY_PKT_FLAG_NOSYNC     2   // not synced


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
    s32 ts;

} phy_sync_pkt_t;

typedef struct __packed {
    phy_pkt_meta_t meta;
    phy_pkt_t pkt;

} phy_recv_queue_t;

static void isr_mcu_wake(void);
static void isr_ctl_lna(void);
static void isr_ctl_pa(void);

static void cca_run(void);

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
        {CC1200_PKT_CFG2,   CC1200_PKT_CFG2_CCA_MODE_ALWAYS/*CC1200_PKT_CFG2_CCA_MODE_RSSI_THR*//*CC1200_PKT_CFG2_CCA_MODE_RSSI_THR_ETSI_LBT*/},
        {CC1200_PKT_CFG1,   CC1200_PKT_CFG1_CRC_CFG_ON_INIT_1D0F | CC1200_PKT_CFG1_ADDR_CHECK_CFG_ON_NO_BCAST | CC1200_PKT_CFG1_APPEND_STATUS | CC1200_PKT_CFG1_WHITE_DATA},
        {CC1200_PKT_CFG0,   CC1200_PKT_CFG0_LENGTH_CONFIG_VARIABLE},
        {CC1200_PKT_LEN,    125}, // CRC Autoflush limits this to 127 in variable packet length mode, and then there needs to be room for the two status bytes
        {CC1200_DEV_ADDR,   0x00},
        //{CC1200_SERIAL_STATUS, CC1200_SERIAL_STATUS_IOC_SYNC_PINS_EN}, // Enable access to GPIO state in CC1200_GPIO_STATUS.GPIO_STATE

        {CC1200_RNDGEN,     CC1200_RNDGEN_EN}, // Needed for random backoff for LBT CCA: https://e2e.ti.com/support/wireless_connectivity/f/156/t/370230
        {CC1200_FIFO_CFG,   CC1200_FIFO_CFG_CRC_AUTOFLUSH /*!CC1200_FIFO_CFG_CRC_AUTOFLUSH*/},


        {CC1200_AGC_GAIN_ADJUST, (u8)CC_RSSI_OFFSET},
        {CC1200_SETTLING_CFG, 0x3}, // Defaults except never auto calibrate

};


#undef cc_dbg_v
#define cc_dbg_v(format, ...) cc_dbg(format, ##__VA_ARGS__ ) //cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )

#undef assert
#define assert(x) if (!(x)) { itm_printf(0, "ASSERT FAIL: ( " #x " ) on line %u of %s\r\n", __LINE__, __FILE__); asm("bkpt #0"); while (1) { asm("nop"); } }




static const cc_dev_t dev = 0;

static struct {
    xTaskHandle task;
    xTaskHandle disp;
    xQueueHandle txq;
    xQueueHandle rxq;
    nphy_rx_t rx;

} nphy = {NULL};


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
static sclk_t sync_last;
static sclk_t sync_time;
static u8 cell_cur, cell_new;



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

        //do {
        //    st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;
        //} while (st != CC1200_STATE_IDLE);

        chan_select(&chnl.group, chnl.hop_table[chan]);
        //cc_dbg_v("#%u\t%lu\t%lu\t\t%lu", chan, chnl.hop_table[chan], sync_timestamp(), sync_timestamp() - sync_time);

        return true;
    }

    return false;
}

/*s32 nphy_mem_count = 0;

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
}*/

#define nphy_malloc     malloc
#define nphy_free       free

bool nphy_init(u8 cell, bool sync_master, nphy_rx_t rx)
{
    sclk_init();

    cell_new = cell_cur = cell;
    boss = sync_master;

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

    nphy.txq = xQueueCreate(7, sizeof(void *)); assert(nphy.txq);
    //nphy.rxq = xQueueCreate(7, sizeof(phy_recv_queue_t *)); assert(nphy.rxq);
    nphy.rx = rx;


    amp_init(dev);
    amp_ctrl(dev, AMP_HGM, true);

    amp_ctrl(dev, AMP_LNA, true);
    chan_grp_init(&chnl.group, chnl.hop_table);
    chan_table_reorder(&chnl.group, cell, chnl.hop_table);
    chan_grp_calibrate(&chnl.group);
    amp_ctrl(dev, AMP_LNA, false);


    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(0), CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP);
    isrd_configure(2, 10, kPORT_InterruptRisingEdge, isr_mcu_wake, 0);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(1), CC1200_IOCFG_GPIO_CFG_LNA_PD);
    isrd_configure(2, 11, kPORT_InterruptEitherEdge, isr_ctl_lna, 0);

    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(2), CC1200_IOCFG_GPIO_CFG_PA_PD);
    isrd_configure(2, 12, kPORT_InterruptEitherEdge, isr_ctl_pa, 0);


    if (!xTaskCreate(nphy_task, "nphy:main", TASK_STACK_SIZE_LARGE, NULL, TASK_PRIO_HIGH + 1, &nphy.task)) {
        cc_dbg("[%u] error: unable to create main task", dev);
        return false;
    }

    /*if (!xTaskCreate(nphy_dispatch_task, "nphy:disp", TASK_STACK_SIZE_LARGE, NULL, TASK_PRIO_HIGH + 1, &nphy.disp)) {
        cc_dbg("[%u] error: unable to create dispatch task", dev);
        return false;
    }*/

    // ensure the tasks run even if their priority is lower
    vTaskDelay(pdMS_TO_TICKS(chan_time/4000));

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
    qpkt->hdr.cell = cell_cur;
    qpkt->hdr.flag = flag & (u8)PHY_PKT_FLAG_IMMEDIATE;

    if (qpkt->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {

        if (xQueueSendToFront(nphy.txq, &qpkt, pdMS_TO_TICKS(500))) {
            xTaskNotify(nphy.task, NOTIFY_MASK_TX, eSetBits);
        } else {
            cc_dbg("tx pkt queue immediate fail");
            nphy_free(qpkt);
        }

        taskYIELD();

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
        return false;
    }

    if (ppkt->hdr.flag & PHY_PKT_FLAG_SYNC) {
        const phy_sync_pkt_t *const spkt = (phy_sync_pkt_t *) pkt;

        if (!boss) {
            //itm_puts(0, ".");
            const bool ppkt_sync_slow = (ppkt->hdr.flag & PHY_PKT_FLAG_NOSYNC) != 0;

            //if (!ppkt_sync_slow || sync_slow || ((ts - sync_last) >= (CHAN_TIME*CHAN_COUNT))) {
                sync_slow = ppkt_sync_slow;
                sync_last = sclk_time();
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
    /*u8 reg;

    ensure_rx();

    do {
        reg = cc_get(dev, CC1200_RSSI0) & (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID);
    } while (reg != (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID));*/
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

    sclk_t ts;
    sclk_t ticks;
    u32 chan_ticks;
    u32 remaining = 0;
    sclk_t tx_next = 0;
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
            .ts = 0
    };

    sync_slow = !boss;
    sync_needed = false;
    sync_last = 0;
    sync_time = 0;

    sclk_t start_time = sclk_time();

    if (boss) {
        sync_needed = true;
        loop_state = LOOP_STATE_TX;
    } else {
        chan_set(0);
        cc_strobe(dev, CC1200_SRX);
    }


    while (1) {
        if (xTaskNotifyWait(0, UINT32_MAX, &notify, pdMS_TO_TICKS(((remaining/2)+1001)/1000))) {

            if (notify & NOTIFY_MASK_ISR) {

                ms = cc_get(dev, CC1200_MARC_STATUS1);

                if (ms) {
                    //cc_dbg_v("isr: ms1=0x%02X st=0x%02X t=%lu 2Ps=%u", ms, cc_strobe(dev, CC1200_SNOP), SCLK_MSEC(sclk_time()), marc_2pin_status());
                }

                switch (ms) {
                    case CC1200_MARC_STATUS1_RX_FINISHED:
                        nphy_rx(true);
                        //if (!pkt) tx_next = 0;

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
            ts = sclk_time();
            ticks = ts - sync_time;
            u32 chan_time_cur = chan_time * (sync_slow ? 2 : 1);
            assert(ts > sync_last);

            if (!boss && ((ts - sync_last) >= 3*(CHAN_TIME * CHAN_COUNT)) && sync_last) {
                if (!sync_slow) {
                    sync_slow = true;
                    chan_time_cur *= 2;
                    cc_dbg_v("sync lost: last=%lu now=%lu", SCLK_MSEC(sync_last), SCLK_MSEC(ts));
                } else {

                    if (((ts - sync_last) >= 5*(CHAN_TIME * CHAN_COUNT))) {
                        cc_dbg_v("sync dropped: last=%lu now=%lu", SCLK_MSEC(sync_last), SCLK_MSEC(ts));
                        sync_time = 0;
                        //start_time = sclk_time(); // NEW: advance start time. TODO: Check this!
                        // ^ doesn't seem promising so far

                        goto _unsynced_check;
                    }
                }
            } else {
                remaining = chan_time_cur - (u32)(ticks % chan_time_cur);
            }

            if (chan_set((u32)((ticks / chan_time_cur) % (chan_count * 2)))) {
                if (pkt) {
                    const u8 tx_bytes = cc_get(dev, CC1200_NUM_TXBYTES);

                    if (tx_bytes) cc_strobe(dev, CC1200_SFTX);

                    cc_dbg(
                            "chan: switched during TX! TXBYTES=%u plen=%u is_sync=%u time=%lu next=%lu rem=%lu",
                            tx_bytes, pkt->len, (void *) pkt == &pkt_sync, SCLK_MSEC(sclk_time()), SCLK_MSEC(tx_next), remaining
                    );

                    if ((void *) pkt != &pkt_sync) nphy_free(pkt);
                    pkt = NULL;
                    cca_disable();
                    cc_strobe(dev, CC1200_SIDLE);
                }

                tx_next = 0;

                if (boss) {
                    sync_needed = true;
                    loop_state = LOOP_STATE_TX;
                } else {
                    loop_state = LOOP_STATE_RX;
                }
            }
        } else if (!boss) {
            _unsynced_check:

            ts = sclk_time();
            ticks = ts - start_time;
            remaining = (2 * chan_time) - (u32)(ticks % (2 * chan_time));

            if (chan_set((u32)((ticks / (2 * chan_time)) % (chan_count * 2)))) {
                if (pkt) {
                    const u8 tx_bytes = cc_get(dev, CC1200_NUM_TXBYTES);

                    if (tx_bytes) cc_strobe(dev, CC1200_SFTX);

                    cc_dbg(
                            "chan: switched during TX! TXBYTES=%u plen=%u is_sync=%u time=%lu next=%lu rem=%lu <NOSYNC>",
                            tx_bytes, pkt->len, (void *) pkt == &pkt_sync, SCLK_MSEC(sclk_time()), SCLK_MSEC(tx_next), remaining
                    );

                    if ((void *) pkt != &pkt_sync) nphy_free(pkt);
                    pkt = NULL;
                    cca_disable();
                    cc_strobe(dev, CC1200_SIDLE);
                }

                tx_next = 0;

                sync_needed = true;
                loop_state = LOOP_STATE_TX;
            }
        }

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
                            //cc_dbg("rx fail: ms=0x%02X t=%lu e_t=%lu", ms, sclk_time(), sclk_time() - sync_time);
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

                    if (sync_needed || (tx_next && sclk_time() >= tx_next)) {
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

                                //cc_dbg_v("cca fail: pkt=0x%p\n", pkt);

                                cca_run();
                                cc_strobe(dev, CC1200_STX);
                                continue;

                            }

                            cc_dbg("tx fail: ms=0x%02X", ms);
                            if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);

                        case RF_STATE_TX_END:
                            if (pkt) {
                                if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {
                                    if ((void*)pkt != &pkt_sync) {
                                        nphy_free(pkt);
                                    }
                                } else {
                                    assert(pkt != (void*)&pkt_sync);
                                    nphy_free(pkt);
                                    pkt = NULL;
                                    cca_disable();
                                    loop_state_next = LOOP_STATE_RX;
                                    continue;
                                }

                                pkt = NULL;
                            } else {
                                cc_dbg("WARNING: tx completion indicated without a packet pending");
                                cca_disable();
                            }

                            break;
                    }

                    if (sync_needed) {
                        assert(!pkt);

                        pkt = (rf_pkt_t *) &pkt_sync;

                        if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);

                        do {
                            st = cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M;

                        } while (st != CC1200_STATE_IDLE);

                        if (boss && !sync_time) {
                            chan_set(0);
                            sync_time = sclk_time();
                            ticks = 0;
                        } else {
                            ticks = sclk_time() - sync_time;

                            if (boss && !chan_cur && ticks >= (CHAN_TIME*CHAN_COUNT)) {
                                // TODO: Need to also realign the sync window for non-boss nodes sending provisional sync packets
                                sync_time = sclk_time();
                                ticks = 0;
                            }
                        }

                        pkt_sync.hdr.cell = cell_cur;

                        tx_next = cc_get_tx_time(dev, pkt->len);
                        pkt_sync.ts = (u32)(ticks + tx_next*2 + 1750);

                        cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);
                        cc_strobe(dev, CC1200_STX);

                        tx_next += sclk_time();

                        sync_needed = false;
                        if (hook_sync) hook_sync(chan_cur);

                    } else if (!pkt && xQueuePeek(nphy.txq, &pkt, 0) && pkt/*for the ide...*/) {

                        u32 pkt_time = cc_get_tx_time(dev, pkt->len);

                        const bool synced = (sync_time != 0) && !sync_slow;
                        const bool imm = synced && ((((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) != 0);

                        if (imm) {

                            ts = sclk_time();

                            /*if (tx_next > ts) {
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;
                            }*/

                            ticks = ts - sync_time;
                            chan_ticks = (u32)(ticks % chan_time);
                            remaining = chan_time - chan_ticks;

                            if (chan_ticks < 2500) {

                                tx_next = ts + (2500 - chan_ticks);
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;
                            }

                            if (remaining < (2500 + pkt_time)) {

                                tx_next = ts + ((2500 + pkt_time) - remaining);
                                pkt = NULL;
                                continue;
                            }

                            tx_next = pkt_time;
                            //cc_strobe(dev, CC1200_SIDLE);

                        } else {

                            ts = sclk_time();

                            if (tx_next > ts) {
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;
                            }

                            ticks = ts - (sync_time ? sync_time : start_time);
                            chan_ticks = (u32)(ticks % chan_time);
                            remaining = chan_time - chan_ticks;

                            if (chan_ticks < 5000) {

                                tx_next = ts + (5000 - chan_ticks);
                                pkt = NULL;
                                loop_state_next = LOOP_STATE_RX;
                                continue;
                            }

                            if (remaining < (5000 + pkt_time)) {

                                tx_next = ts + ((5000 + pkt_time) - remaining);
                                pkt = NULL;
                                continue;
                            }

                            tx_next = 3 * pkt_time;

                            if (!synced) {
                                ((phy_pkt_t *)pkt)->hdr.flag |= PHY_PKT_FLAG_NOSYNC;

                                if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE)
                                    ((phy_pkt_t *)pkt)->hdr.flag &= ~PHY_PKT_FLAG_IMMEDIATE;
                            }
                        }

                        xQueueReceive(nphy.txq, &pkt, 0);

                        cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);

                        if (!imm) {
                            cca_enable();
                            cca_run();
                        } else {
                            if ((cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M) != CC1200_STATE_IDLE)
                                cc_strobe(dev, CC1200_SIDLE);
                        }

                        cc_strobe(dev, CC1200_STX);
                        if (tx_next) tx_next = sclk_time() + tx_next;
                        //cc_dbg_v("tx: sent now=%lu next=%lu", SCLK_MSEC(sclk_time()), SCLK_MSEC(tx_next));

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
                            ts = sclk_time();

                            if (ts >= (tx_next+17000)) {
                                cc_dbg("tx: timed out: now=%lu tx_next=%lu", SCLK_MSEC(ts), SCLK_MSEC(tx_next));
                                if ((void *)pkt != &pkt_sync) nphy_free(pkt);
                                pkt = NULL;
                                if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);
                                cca_disable();
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
        ts = sclk_time();

        ticks = ts - (sync_time ? sync_time : start_time);
        chan_ticks = (u32)(ticks % chan_time);
        remaining = chan_time - chan_ticks;

        if (tx_next) {
            if (ts >= tx_next) {
                //cc_dbg("ts=%lu >= tx_next=%lu", SCLK_MSEC(ts), SCLK_MSEC(tx_next));
                remaining = 0;
                //tx_next = 0;
                continue;
            }

            if ((ts + remaining) > tx_next) {
                remaining = (u32)(tx_next - ts);
                //cc_dbg_v("rf: wait=%lu now=%lu tx_next=%lu ticks=%li chan_ticks=%lu", remaining, SCLK_MSEC(ts), SCLK_MSEC(tx_next), SCLK_MSEC(ticks), SCLK_MSEC(chan_ticks));
                continue;
            }
        }
    }
}
