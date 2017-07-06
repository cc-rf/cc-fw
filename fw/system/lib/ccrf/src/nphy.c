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
#include <kio/sclk.h>

#include <cc/chan.h>
#include <fsl_rnga.h>
#include <kio/itm.h>
#include <stdatomic.h>
#include <malloc.h>
#include <cc/amp.h>
#include <cc/sys/kinetis/pit.h>
#include <cc/type.h>


#define PHY_TASK_STACK_SIZE     (TASK_STACK_SIZE_HUGE / sizeof(StackType_t))


#define PHY_PKT_FLAG_SYNC       (0x01)   // is a sync packet
#define PHY_PKT_FLAG_NOSYNC     (0x02)   // not synced
#define PHY_PKT_FLAG_RECAL      (0x08)

#define PHY_TXQ_LEN             3

#define NOTIFY_MASK_ISR    (1<<1)
#define NOTIFY_MASK_TX     (1<<2)
#define NOTIFY_MASK_HOP    (1<<3)
#define NOTIFY_MASK_ALL    (NOTIFY_MASK_ISR | NOTIFY_MASK_TX | NOTIFY_MASK_HOP)

#define CALLER_NOTIFY_TX_DONE   (1<<4)
#define CALLER_NOTIFY_TX_FAIL   (1<<5)
#define CALLER_NOTIFY_TX_MASK   (CALLER_NOTIFY_TX_DONE | CALLER_NOTIFY_TX_FAIL)

#define CC_RSSI_OFFSET      (s8)(-81 - 3)


#define FREQ_BASE       902125000u
#define FREQ_BW         950000u
#define CHAN_COUNT      25u
#define CHAN_TIME       20000u
#define RECAL_CYCLES    5

#define MAX_CCA_RETRY   3
#define MAX_CCA_TIME    13 //NOTE: When using LBT, backoff time minimum is 5


#undef cc_dbg_v
#define cc_dbg_v(format, ...) cc_dbg(format, ##__VA_ARGS__ ) //cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )

#undef assert
#define assert(x) if (!(x)) { itm_printf(0, "ASSERT FAIL: ( " #x " ) on line %u of %s\r\n", __LINE__, __FILE__); asm("bkpt #0"); while (1) { asm("nop"); } }


typedef struct __packed {
    u8 len;
    u8 data[];

} rf_pkt_t;

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
    phy_static_pkt_t pkt;
    TaskHandle_t task;

} phy_send_queue_t;


static void isr_mcu_wake(void);
static void isr_chan_hop(pit_t pit __unused, void *param __unused);

static inline void chan_set(chan_t chan);
static inline void chan_next(volatile chan_t *chan);
static inline void chan_recal(chan_t chan);

static void cca_run(bool check);
static bool process_packet(rf_pkt_t *pkt, s8 rssi, u8 lqi);
static bool nphy_rx(bool flush);
static inline void ensure_rx(void);
static void nphy_task(void *param);


static const cc_dev_t dev = 0;

static struct {
    xTaskHandle task;
    StaticTask_t task_static;
    StackType_t task_stack[PHY_TASK_STACK_SIZE];

    xQueueHandle txq;
    StaticQueue_t txq_static;
    phy_send_queue_t txq_buf[PHY_TXQ_LEN];

    nphy_rx_t rx;

    nphy_hook_t hook_sync;

} nphy = {.rx = NULL, .hook_sync = NULL};


static const struct cc_cfg_reg CC_CFG_PHY[] = {
        {CC1200_IOCFG3, CC1200_IOCFG_GPIO_CFG_HW0},
        {CC1200_IOCFG2, CC1200_IOCFG_GPIO_CFG_HW0},
        {CC1200_IOCFG1, CC1200_IOCFG_GPIO_CFG_HIGHZ},
        {CC1200_IOCFG0, CC1200_IOCFG_GPIO_CFG_HW0},

        {CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RXOFF_MODE_IDLE | (CC1200_RFEND_CFG1_RX_TIME_FOREVER << CC1200_RFEND_CFG1_RX_TIME_S)},
        {CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_RX | CC1200_RFEND_CFG0_TERM_ON_BAD_PACKET_EN},

        {CC1200_PKT_CFG2,   CC1200_PKT_CFG2_CCA_MODE_ALWAYS},
        {CC1200_PKT_CFG1,   CC1200_PKT_CFG1_CRC_CFG_ON_INIT_1D0F | CC1200_PKT_CFG1_ADDR_CHECK_CFG_ON_NO_BCAST | CC1200_PKT_CFG1_APPEND_STATUS | CC1200_PKT_CFG1_WHITE_DATA},
        {CC1200_PKT_CFG0,   CC1200_PKT_CFG0_LENGTH_CONFIG_VARIABLE},
        {CC1200_PKT_LEN,    125}, // CRC Autoflush limits this to 127 in variable packet length mode, and then there needs to be room for the two status bytes
        {CC1200_DEV_ADDR,   0x00},

        {CC1200_RNDGEN,     CC1200_RNDGEN_EN}, // Needed for random backoff for LBT CCA: https://e2e.ti.com/support/wireless_connectivity/f/156/t/370230
        {CC1200_FIFO_CFG,   CC1200_FIFO_CFG_CRC_AUTOFLUSH},

        {CC1200_AGC_GAIN_ADJUST,    (u8)CC_RSSI_OFFSET},
        {CC1200_AGC_CS_THR,         (u8)(-61)},

        {CC1200_SETTLING_CFG, 0x3}, // Defaults except never auto calibrate
};

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

static struct {
    u32 rx_count;
    u32 tx_count;

} nphy_stat = {0};

static u32 tx_times[PHY_FRAME_SIZE_MAX + 1];

static bool boss;
static volatile bool sync_needed;
//static volatile bool recal_needed = false;
static volatile sclk_t sync_time;
static u8 cell_cur;
static pit_t pit_hop;
static volatile phy_static_pkt_t pkt_ack = { 0 };



static inline void chan_next(volatile chan_t *chan)
{
    if (++*chan >= CHAN_COUNT) *chan = 0;
    chan_set(*chan);
}

static inline void chan_set(chan_t chan)
{
    static volatile chan_t chan_prev = (chan_t) -1;

    if (chan != chan_prev) {
        chan_prev = chan;
        cc_strobe(dev, CC1200_SIDLE);
        chan_select(&chnl.group, chnl.hop_table[chan]);
    }
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

    chan_grp_init(&chnl.group, chnl.hop_table);
    chan_table_reorder(&chnl.group, cell, chnl.hop_table);
    chan_grp_calibrate(&chnl.group);

    pit_init();

    const pit_cfg_t pit_hop_cfg = {
            .handler = isr_chan_hop,
            .period = pit_nsec_tick(CHAN_TIME * 1000)
    };

    pit_hop = pit_alloc(&pit_hop_cfg);

    nphy.task = xTaskCreateStatic(nphy_task, "nphy:main", PHY_TASK_STACK_SIZE, NULL, TASK_PRIO_HIGHEST, nphy.task_stack, &nphy.task_static);

    // setup interrupts after task created
    cc_set(dev, (u16)CC1200_IOCFG_REG_FROM_PIN(0), CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP);
    isrd_configure(2, 10, kPORT_InterruptRisingEdge, isr_mcu_wake, 0);

    // ensure the tasks run even if their priority is lower
    vTaskDelay(pdMS_TO_TICKS(CHAN_TIME/4000));

    return true;
}


void nphy_hook_sync(nphy_hook_t hook)
{
    nphy.hook_sync = hook;
}


static inline u32 pit_hop_remaining(void) {
    return (u32) (pit_tick_nsec(pit_get_current(pit_hop)) / 1000u);
}


bool nphy_tx(u8 flag, u8 *buf, u8 len)
{
    if (len > PHY_FRAME_SIZE_MAX) {
        cc_dbg("frame too big: %u > %u", len, PHY_FRAME_SIZE_MAX);
        return false;
    }

    const bool block = (flag & PHY_PKT_FLAG_BLOCK) != 0 && (xTaskGetCurrentTaskHandle() != nphy.task);

    phy_send_queue_t sq = {
            .task = block ? xTaskGetCurrentTaskHandle() : NULL,
            .pkt = {
                    .hdr = {
                            // NOTE: upon receive, this reflects size minus phy header, but here it is all the data bytes
                            .len = len + (sizeof(phy_pkt_t) - sizeof(rf_pkt_t)),
                            .cell = cell_cur,
                            .flag = (u8)(flag & PHY_PKT_FLAG_IMMEDIATE) // block flag is local-only
                    }
            }
    };

    if (len && buf) memcpy(sq.pkt.data, buf, len);

    if (sq.pkt.hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {

        if (xTaskGetCurrentTaskHandle() == nphy.task && !pkt_ack.hdr.flag) {

            memcpy((void *) &pkt_ack, &sq.pkt, sizeof(pkt_ack.hdr) + len);

        } else {

            if (xQueueSendToFront(nphy.txq, &sq.pkt, portMAX_DELAY)) {
                xTaskNotify(nphy.task, NOTIFY_MASK_TX, eSetBits);
            } else {
                cc_dbg("tx pkt queue immediate fail");
            }
        }

    } else if (xQueueSend(nphy.txq, &sq.pkt, portMAX_DELAY/*pdMS_TO_TICKS(500)*/)) {
        xTaskNotify(nphy.task, NOTIFY_MASK_TX, eSetBits);
    } else {
        cc_dbg("tx pkt queue fail");
    }

    if (block) {
        //xTaskNotifyWait(CALLER_NOTIFY_TX_MASK, CALLER_NOTIFY_TX_MASK, NULL, portMAX_DELAY);
        u32 notify;

        do {
            if (!xTaskNotifyWait(CALLER_NOTIFY_TX_MASK, CALLER_NOTIFY_TX_MASK, &notify, portMAX_DELAY))
                notify = 0;

        } while (!(notify & CALLER_NOTIFY_TX_MASK));

        return (notify & CALLER_NOTIFY_TX_DONE) != 0;
    }

    return true;
}

u32 nphy_delay(u8 len)
{
    return 1000 + tx_times[len];
}


static void isr_mcu_wake(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(nphy.task, NOTIFY_MASK_ISR, eSetBits, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void isr_chan_hop(pit_t pit __unused, void *param __unused)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(nphy.task, NOTIFY_MASK_HOP, eSetBits, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

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
                sync_time = sclk_time();
                pit_restart(pit_hop);

                /*if (ppkt->hdr.flag & PHY_PKT_FLAG_RECAL) {
                    recal_needed = true;
                }*/

            //}
        }

        return false;
    }

    ++nphy_stat.rx_count;

    ppkt->hdr.len -= (sizeof(phy_pkt_t) - sizeof(rf_pkt_t));

    // NOTE: temporarily disabling this while other stuff gets debugged
    /*if (ppkt->hdr.flag & PHY_PKT_FLAG_NOSYNC) {
        if (boss && !sync_needed) {
            sync_needed = true;
        }
    }*/

    return nphy.rx(ppkt->hdr.flag, ppkt->hdr.len, ppkt->data, rssi, lqi);
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
    bool unblock = false;

    cc_fifo_read(dev, buf, len);

    while (len > PKT_OVERHEAD) {
        spkt = (rf_pkt_t *)buf;

        if (spkt->len > PHY_FRAME_SIZE_MAX) {
            // NOTE: _v added newly, but this is a useful error to see when timing is off. same applies for below
            cc_dbg("[%u] c=%u malformed: len[header]=%u > len[max]=%u  (len[fifo]=%u)", dev, pkt_count+1, spkt->len, PHY_FRAME_SIZE_MAX, len);
            break;
        }

        if (spkt->len > (len - PKT_OVERHEAD)) {
            cc_dbg("[%u] c=%u underflow: len[header]=%u > len[fifo]=%u", dev, pkt_count+1, spkt->len, len);
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
                unblock |= process_packet(spkt, rssi, lqi);
            } else {
                cc_dbg("[%u] c=%u bad crc", dev, pkt_count);
                // NEW: don't trust anything else in the buffer
                break;
            }
        } else {
            cc_dbg("[%u] c=%u empty", dev, pkt_count);
            // NEW: this is weird, should def stop
            break;
        }

        len -= spkt->len + PKT_OVERHEAD;
        buf += spkt->len + PKT_OVERHEAD;
    }

    if (flush && len) cc_strobe(dev, CC1200_SFRX);

    return unblock;
}


static inline void ensure_rx(void)
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
            if (cc_strobe(dev, CC1200_SIDLE | CC1200_ACCESS_READ) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFRX);
            cc_dbg_v("rx fifo error");
            break;

        case CC1200_STATE_TXFIFO_ERROR:
            if (cc_strobe(dev, CC1200_SIDLE) & CC1200_STATUS_EXTRA_M) cc_strobe(dev, CC1200_SFTX);
            cc_dbg_v("tx fifo error");
            break;
    }

    cc_strobe(dev, CC1200_SRX);
}


void cca_run(bool check)
{
    u8 reg;

    if (check) ensure_rx();

    do {
        reg = cc_get(dev, CC1200_RSSI0) & (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID);
    } while (reg != (CC1200_RSSI0_RSSI_VALID | CC1200_RSSI0_CARRIER_SENSE_VALID));
}


static inline void amp_tx_on(void)
{
    amp_ctrl(dev, AMP_LNA, false);
    amp_ctrl(dev, AMP_PA, true);
}


static inline void amp_tx_off(void)
{
    amp_ctrl(dev, AMP_LNA, true);
    amp_ctrl(dev, AMP_PA, false);
}


static void nphy_task(void *param __unused)
{
    phy_send_queue_t sq;
    rf_pkt_t *pkt = NULL;
    u8 ms = 0;

    sclk_t ts;
    u32 chan_ticks;
    u32 remaining = 0;
    volatile sclk_t tx_next = 0;
    //u32 recal_cycles = RECAL_CYCLES + 1;
    u32 notify;

    volatile chan_t chan_cur = (chan_t) -1;
    u8 cca_fail_count = 0;

    phy_sync_pkt_t pkt_sync = {
            .hdr = {
                    .len = sizeof(phy_sync_pkt_t) - 1,
                    .flag = (u8)(PHY_PKT_FLAG_IMMEDIATE | PHY_PKT_FLAG_SYNC | (!boss ? PHY_PKT_FLAG_NOSYNC : 0))
            },
    };

    sync_needed = false;
    sync_time = 0;

    amp_tx_off();

    if (boss) {
        xTaskNotify(xTaskGetCurrentTaskHandle(), NOTIFY_MASK_HOP, eSetBits);
    } else {
        chan_set(chan_cur = 0);
        cc_strobe(dev, CC1200_SRX);
    }

    /*sclk_t loop_time_start = sclk_time();
    sclk_t loop_time;
    sclk_t loop_time_prev = loop_time_start;
    sclk_t loop_time_sum = 0;
    u32 loop_time_count = 0;*/

    //sclk_t send_time;

    while (1) {

        /*ts = sclk_time();
        loop_time = ts - loop_time_prev;
        loop_time_sum += loop_time;

        if (++loop_time_count == 1000) {
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

        if (xTaskNotifyWait(0, NOTIFY_MASK_ALL, &notify, tx_next ? pdUS_TO_TICKS(1000) : portMAX_DELAY)) {

            if (notify & NOTIFY_MASK_ISR) {
                ms = cc_get(dev, CC1200_MARC_STATUS1);

                switch (ms) {
                    case CC1200_MARC_STATUS1_RX_FINISHED:
                        if (nphy_rx(true) && (sync_time || boss)) {
                            tx_next = 1;
                        }

                        break;

                    case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
                        if (pkt) {
                            ++cca_fail_count;
                            itm_printf(0, "cca fail #%lu\n", cca_fail_count);

                            //cca_run(true);
                            // ^ for when cca mode isn't NOT_RX/ALWAYS, otherwise (maybe? untested):
                            //cc_strobe(dev, CC1200_SIDLE);

                            cc_strobe(dev, CC1200_STX);
                            continue;
                        }
                        break;

                    case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
                    case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
                        cc_strobe(dev, CC1200_SFTX);
                        cc_strobe(dev, CC1200_SIDLE);

                    case CC1200_MARC_STATUS1_TX_FINISHED:
                        amp_tx_off();

                        if (pkt) {
                            if ((void*)pkt != &pkt_sync) ++nphy_stat.tx_count;
                            cca_fail_count = 0;

                            if (pkt == (rf_pkt_t *)&sq.pkt && sq.task) {
                                //send_time = sclk_time() - send_time;
                                //itm_printf(0, "l=%03u d=%i\n", pkt->len, (s32)send_time - tx_times[pkt->len]);

                                xTaskNotify(
                                        sq.task,
                                        ms == CC1200_MARC_STATUS1_TX_FINISHED ? CALLER_NOTIFY_TX_DONE : CALLER_NOTIFY_TX_FAIL,
                                        eSetBits
                                );
                            }

                            if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {
                                if ((void*)pkt == &pkt_ack) {
                                    pkt_ack.hdr.flag = 0;
                                    pkt_ack.hdr.len = 0;
                                }

                                if ((void*)pkt == &pkt_sync) {
                                    //itm_printf(0, "sync0: r=%lu\n", pit_hop_remaining());
                                    /*if (pkt_sync.hdr.flag & PHY_PKT_FLAG_RECAL) {
                                        pkt_sync.hdr.flag &= ~PHY_PKT_FLAG_RECAL;
                                    }*/

                                    sync_time = sclk_time();
                                    pit_restart(pit_hop);
                                }

                            } else {
                                tx_next = sclk_time() + 1000;
                            }

                            pkt = NULL;

                        } else {
                            cc_dbg("tx completion indicated but no packet pending");
                        }

                        break;

                    case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
                    case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:

                    case CC1200_MARC_STATUS1_ADDRESS:
                    case CC1200_MARC_STATUS1_CRC:
                        cc_strobe(dev, CC1200_SFRX);
                        break;

                    default:
                    case CC1200_MARC_STATUS1_NO_FAILURE:
                        if (!(notify & ~NOTIFY_MASK_ISR)) continue; // usually the result of a SIDLE strobe
                        break;
                }
            }

            if ((notify & NOTIFY_MASK_HOP) && (boss || sync_time)) {
                ts = sclk_time();
                tx_next = 0; // This fixes a lot of corner cases that lead to poorly timed transmissions

                if (!boss && !chan_cur) {
                    if (((ts - sync_time) >= 3*(CHAN_TIME * CHAN_COUNT))) {
                        cc_dbg_v("sync lost: last=%lu now=%lu", SCLK_MSEC(sync_time), SCLK_MSEC(ts));
                        sync_time = 0;
                        goto _inner_loop;

                    } else if ((ts - sync_time) >= 2*CHAN_TIME) {
                        cc_dbg_v("sync missed: last=%lu now=%lu", SCLK_MSEC(sync_time), SCLK_MSEC(ts));
                        sync_time = 0;
                        goto _inner_loop;
                    }
                }

                chan_next(&chan_cur);

                chan_clock_trig();
                if (!chan_cur || (chan_cur == 1)) chan_cycle_trig();

                if (boss && !chan_cur) {
                    if (!sync_time) {
                        sync_time = ts;
                        pit_start(pit_hop);
                    }

                    sync_needed = true;
                }

                if (nphy.hook_sync) nphy.hook_sync(chan_cur);

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
                    // May have been done sending, TXBYTES will be zero if so.

                    if (!tx_bytes) {
                        if (pkt == (rf_pkt_t *)&sq.pkt && sq.task) {
                            xTaskNotify(sq.task, CALLER_NOTIFY_TX_FAIL, eSetBits);
                        }

                        pkt = NULL;
                        itm_puts(0, "chan: hop tx-done-clear\r\n");

                    } else if ((tx_bytes == pkt->len + 1) && (chan_cur || (boss && !sync_needed))) {
                        itm_puts(0, "chan: hop re-tx\r\n");
                        cc_strobe(dev, CC1200_STX);

                    } else {
                        cc_strobe(dev, CC1200_SFTX);

                        cc_dbg("chan: hop tx canned: len=%u txbytes=%u", pkt->len, tx_bytes);

                        if (pkt == (rf_pkt_t *)&sq.pkt && sq.task) {
                            xTaskNotify(sq.task, CALLER_NOTIFY_TX_FAIL, eSetBits);
                        }

                        pkt = NULL;
                    }
                }
            }

            //if (notify & NOTIFY_MASK_TX) {
                // Just fall through and try to send.
            //}

        }

        _inner_loop:;

        if (sync_needed) {
            sync_needed = false;

            assert(!pkt);
            pkt = (rf_pkt_t *) &pkt_sync;

            if ((cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M) != CC1200_STATE_IDLE)
                cc_strobe(dev, CC1200_SIDLE);

            pkt_sync.hdr.cell = cell_cur;

            /*if (recal_needed) {
                pkt_sync.hdr.flag |= PHY_PKT_FLAG_RECAL;
            }*/

            cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);
            amp_tx_on();
            cc_strobe(dev, CC1200_STX);

        } else if (!pkt && ((volatile u8)pkt_ack.hdr.flag || xQueuePeek(nphy.txq, &sq, 0))) {
            pkt = (volatile u8)pkt_ack.hdr.flag ? (rf_pkt_t *)&pkt_ack : (rf_pkt_t *)&sq.pkt;

            volatile bool synced = (sync_time != 0);
            volatile bool imm = synced && ((((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) != 0);

            if (synced) {
                remaining = pit_hop_remaining();
                chan_ticks = CHAN_TIME - remaining;

                u32 lead_time = 400 + ((!boss && !chan_cur) ? 200 + tx_times[pkt_sync.hdr.len] : 0);

                if (chan_ticks < lead_time) {
                    tx_next = sclk_time() + (lead_time - chan_ticks);
                    pkt = NULL;
                    goto _restart_rx;
                }

                if (remaining < (400 + tx_times[pkt->len])) {
                    tx_next = 0;
                    pkt = NULL;
                    goto _restart_rx;
                }
            }

            if (!imm && tx_next > 1) {
                ts = sclk_time();

                if (tx_next > (ts + 100)) {
                    pkt = NULL;
                    goto _restart_rx;
                }
            }

            if (pkt != (rf_pkt_t *)&pkt_ack) {
                xQueueReceive(nphy.txq, &sq, 0);

                if (!synced) {
                    // TODO: decide whether or not this should be done for pkt_ack

                    ((phy_pkt_t *)pkt)->hdr.flag |= PHY_PKT_FLAG_NOSYNC;

                    if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE)
                        ((phy_pkt_t *)pkt)->hdr.flag &= ~PHY_PKT_FLAG_IMMEDIATE;
                }
            }

            if ((cc_strobe(dev, CC1200_SNOP) & CC1200_STATUS_STATE_M) != CC1200_STATE_IDLE)
                cc_strobe(dev, CC1200_SIDLE);

            cc_fifo_write(dev, (u8 *) pkt, pkt->len + 1);

            // For when CCA mode isn't ALWAYS, but seems useful for NOT_RX
            /*if (!imm) {
                cca_enable();
                cca_run(true);
            }*/

            amp_tx_on();
            //send_time = sclk_time();
            cc_strobe(dev, CC1200_STX);
            tx_next = 0;

        } else if (!pkt) {
            goto _restart_rx;
        }

        goto _loop_end;

        _restart_rx:

        ensure_rx();

        _loop_end:;
    }
}
