#include "phy/phy.h"
#include "phy/config.h"
#include "sys/amp.h"
#include "sys/trace.h"
#include "sys/timer.h"
#include "sys/clock.h"
#include "sys/local.h"
#include "chan.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <assert.h>
#include <ccrf/phy.h>


#define phy_trace_info      ccrf_trace_info
#define phy_trace_warn      ccrf_trace_warn
#define phy_trace_error     ccrf_trace_error
#define phy_trace_debug     ccrf_trace_debug
#define phy_trace_verbose   ccrf_trace_verbose


#define FREQ_BASE       902456000u
#define FREQ_BW         503000u
#define CHAN_COUNT      PHY_CHAN_COUNT
#define CHAN_TIME       20000u

#define PHY_RF_FRAME_SIZE_MAX   (PHY_FRAME_SIZE_MAX + sizeof(phy_pkt_hdr_t) - 1)

#define PHY_PKT_FLAG_SYNC       (0x01)   // Is a sync packet. Same value as local block flag, as this is non-local.

#define PHY_TXQ_LEN             3

#define PHY_TASK_STACK_SIZE     (TASK_STACK_SIZE_LARGE / sizeof(StackType_t))

#define NOTIFY_MASK_ISR         (1<<1)
#define NOTIFY_MASK_TX          (1<<2)
#define NOTIFY_MASK_HOP         (1<<3)
#define NOTIFY_MASK_DIAG        (1<<4)
#define NOTIFY_MASK_ALL         (NOTIFY_MASK_ISR | NOTIFY_MASK_TX | NOTIFY_MASK_HOP | NOTIFY_MASK_DIAG)

#define CALLER_NOTIFY_TX_DONE   (1<<5)
#define CALLER_NOTIFY_TX_FAIL   (1<<6)
#define CALLER_NOTIFY_TX_MASK   (CALLER_NOTIFY_TX_DONE | CALLER_NOTIFY_TX_FAIL)

#define PHY_SYNC_MAGIC          ((u8) 0x69)

typedef struct __packed {
    u8 size;
    u8 data[];

} rf_pkt_t;

typedef struct __packed {
    u8 size;
    u8 cell;
    u8 flag;

} phy_pkt_hdr_t;

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
    u8 magic;

} phy_sync_pkt_t;

typedef struct __packed {
    phy_static_pkt_t pkt;
    TaskHandle_t task;

} phy_send_queue_t;

struct __packed phy {
    rdio_t rdio;
    bool boss;
    u8 cell;
    phy_sync_t sync;
    phy_recv_t recv;
    void *recv_param;

    struct __packed {
        chan_group_t group;
        chan_info_t channel[CHAN_COUNT];
        chan_id_t hop_table[CHAN_COUNT];
        volatile chan_id_t cur;

    } chan;

    struct __packed {
        bool boss;
        bool nosync;
        chan_id_t chan;
        bool cw;

    } diag;

    ccrf_timer_t hop_timer;
    volatile phy_static_pkt_t pkt_ack;
    volatile ccrf_clock_t sync_time;

    TaskHandle_t task;
    StaticTask_t task_static;
    StackType_t task_stack[PHY_TASK_STACK_SIZE];

    QueueHandle_t txq;
    StaticQueue_t txq_static;
    phy_send_queue_t txq_buf[PHY_TXQ_LEN];

    phy_stat_t stat;
};


static void phy_task(phy_t const restrict phy);

static bool phy_recv(phy_t phy, bool flush);
static bool phy_recv_packet(phy_t phy, rf_pkt_t *pkt, s8 rssi, u8 lqi);

static inline void phy_chan_next(phy_t phy);
static inline void phy_chan_set(phy_t phy, chan_id_t chan);
static inline chan_id_t phy_chan_set_base(phy_t phy, chan_id_t chan);

static void phy_rdio_isr(phy_t phy);
static void hop_timer_handler(ccrf_timer_t timer, phy_t phy);


u32 tx_times[PHY_RF_FRAME_SIZE_MAX+1] = {0};

static struct phy phys[CCRF_CONFIG_RDIO_COUNT];


phy_t phy_init(phy_config_t *config)
{
    phy_t phy = &phys[config->rdid];

    memset(phy, 0, sizeof(struct phy));

    phy->boss = config->boss;
    phy->cell = config->cell;
    phy->sync = config->sync;
    phy->recv = config->recv;
    phy->recv_param = config->recv_param;

    phy->diag.boss = phy->boss;
    phy->diag.nosync = false;
    phy->diag.chan = CHAN_ID_INVALID;
    phy->diag.cw = false;

    rdio_config_t rdio_config = {
            .id = config->rdid,
            .isr = (rdio_isr_t) phy_rdio_isr,
            .isr_param = phy,
            .reg_config = PHY_RDIO_REG_CONFIG_DEFAULT,
            .reg_config_size = COUNT_OF(PHY_RDIO_REG_CONFIG_DEFAULT)
    };

    if (!(phy->rdio = rdio_init(&rdio_config))) {
        goto _fail;
    }

    rdio_reg_set(phy->rdio, CC1200_DEV_ADDR, phy->cell);

    ccrf_amp_init(phy->rdio);
    ccrf_amp_mode_hgm(phy->rdio, true);

    phy->chan.group = (chan_group_t) {
            .rdio = phy->rdio,
            .freq = {
                    .base = FREQ_BASE,
                    .bw = FREQ_BW
            },
            .size = CHAN_COUNT
    };

    phy->chan.cur = CHAN_ID_INVALID;

    chan_group_init(&phy->chan.group, phy->chan.hop_table);
    chan_table_reorder(&phy->chan.group, phy->cell, phy->chan.hop_table);
    chan_group_calibrate(&phy->chan.group);

    if (!(phy->hop_timer = ccrf_timer_init(CHAN_TIME, (ccrf_timer_handler_t) hop_timer_handler, phy))) {
        phy_trace_error("timer init fail");
        goto _fail;
    }

    if (!tx_times[PHY_RF_FRAME_SIZE_MAX]) {
        for (u8 i = 0; i <= PHY_RF_FRAME_SIZE_MAX; ++i) {
            tx_times[i] = rdio_util_get_tx_time(phy->rdio, i);
        }
    }

    phy->txq = xQueueCreateStatic(
            PHY_TXQ_LEN, sizeof(phy->txq_buf[0]), (u8 *)phy->txq_buf, &phy->txq_static
    );

    phy->task = xTaskCreateStatic(
            (TaskFunction_t) phy_task, "phy", PHY_TASK_STACK_SIZE, phy, TASK_PRIO_HIGHEST, phy->task_stack, &phy->task_static
    );

    goto _done;
    _fail:

    if (phy) {
        phy = NULL;
    }

    _done:
    return phy;
}


void phy_stat(phy_t phy, phy_stat_t *stat)
{
    *stat = phy->stat;
}


chan_id_t phy_chan(phy_t phy, u32 *freq)
{
    const chan_id_t chan = phy->chan.hop_table[phy->chan.cur];
    if (freq) *freq = phy->chan.channel[chan].freq;
    return chan;
}


u32 phy_freq(phy_t phy, chan_id_t chan)
{
    return phy->chan.channel[chan].freq;
}


void phy_hops(phy_t phy, chan_id_t chan[])
{
    memcpy(chan, phy->chan.hop_table, sizeof(chan_id_t) * PHY_CHAN_COUNT);
}


rdio_t phy_rdio(phy_t phy)
{
    return phy->rdio;
}

phy_cell_t phy_cell(phy_t phy)
{
    return phy->cell;
}

bool phy_boss(phy_t phy)
{
    return phy->boss;
}

bool phy_sync(phy_t phy)
{
    return phy->boss || phy->sync_time != 0;
}

bool phy_hgm(phy_t phy)
{
    return ccrf_amp_stat(phy->rdio, CCRF_AMP_HGM);
}

u8 phy_pwr(phy_t phy)
{
    u8 pwr = rdio_reg_get(phy->rdio, CC1200_PA_CFG1, NULL) & CC1200_PA_CFG1_PA_POWER_RAMP_M;
    if (pwr >= 3) pwr -= 3;
    return pwr;
}

bool phy_diag_boss(phy_t phy, bool boss, bool nosync)
{
    if (boss != phy->diag.boss) {
        phy->diag.boss = boss;
        phy->diag.nosync = nosync;
        xTaskNotify(phy->task, NOTIFY_MASK_DIAG, eSetBits);
        return true;
    }

    return false;
}


bool phy_diag_chan(phy_t phy, chan_id_t chan, u32 *freq)
{
    if (freq) *freq = phy->chan.channel[chan].freq;

    if (chan != phy->diag.chan || phy->diag.boss) {
        phy->diag.boss = false;
        phy->diag.chan = chan;
        xTaskNotify(phy->task, NOTIFY_MASK_DIAG, eSetBits);
        return true;
    }

    return false;
}


bool phy_diag_cw(phy_t phy, bool cw)
{
    if (cw != phy->diag.cw) {
        phy->diag.cw = cw;
        xTaskNotify(phy->task, NOTIFY_MASK_DIAG, eSetBits);
        return true;
    }

    return false;
}


bool phy_diag_pwr(phy_t phy, u8 pwr)
{
    assert(pwr <= PHY_PWR_MAX);
    // This works directly when SPI is polling or locking
    pwr += 3;
    u8 prev = pwr;

    rdio_reg_update(
            phy_rdio(phy), CC1200_PA_CFG1, CC1200_PA_CFG1_PA_POWER_RAMP_M,
            pwr, &prev
    );

    return pwr != prev;
}


bool phy_diag_hgm(phy_t phy, bool hgm)
{
    if (hgm != phy_hgm(phy)) {
        ccrf_amp_mode_hgm(phy->rdio, hgm);
        return true;
    }

    return false;
}


u32 phy_delay(u8 size)
{
    return 1000 + tx_times[size];
}


bool phy_send(phy_t phy, u8 flag, u8 *data, u8 size)
{
    if (size > PHY_FRAME_SIZE_MAX) {
        phy_trace_error("frame too big: %u > %u", size, PHY_FRAME_SIZE_MAX);
        return false;
    }

    const bool block = (flag & PHY_PKT_FLAG_BLOCK) != 0 && (xTaskGetCurrentTaskHandle() != phy->task);

    phy_send_queue_t sq = {
            .task = block ? xTaskGetCurrentTaskHandle() : NULL,
            .pkt = {
                    .hdr = {
                            // NOTE: upon receive, this reflects size minus phy header, but here it is all the data bytes
                            .size = size + (sizeof(phy_pkt_t) - sizeof(rf_pkt_t)),
                            .cell = phy->cell,
                            .flag = (u8)(flag & (PHY_PKT_FLAG_IMMEDIATE|PHY_PKT_FLAG_USER_MASK)) // block flag is local-only
                    }
            }
    };

    if (size && data) memcpy(sq.pkt.data, data, size);

    if (sq.pkt.hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {

        if (xTaskGetCurrentTaskHandle() == phy->task && !phy->pkt_ack.hdr.flag) {

            memcpy((void *) &phy->pkt_ack, &sq.pkt, sizeof(phy->pkt_ack.hdr) + size);

        } else {

            if (xQueueSend(phy->txq, &sq, pdMS_TO_TICKS(100))) {
                xTaskNotify(phy->task, NOTIFY_MASK_TX, eSetBits);
            } else {
                phy_trace_error("tx pkt queue immediate fail");
            }
        }

    } else if (xQueueSend(phy->txq, &sq, pdMS_TO_TICKS(100))) {
        xTaskNotify(phy->task, NOTIFY_MASK_TX, eSetBits);
    } else {
        phy_trace_error("tx pkt queue fail");
    }

    if (block) {
        u32 notify;

        do {
            xTaskNotifyWait(CALLER_NOTIFY_TX_MASK, CALLER_NOTIFY_TX_MASK, &notify, portMAX_DELAY);

        } while (!(notify & CALLER_NOTIFY_TX_MASK));

        return (notify & CALLER_NOTIFY_TX_DONE) != 0;
    }

    return true;
}


static void phy_task(phy_t const restrict phy)
{
    phy_send_queue_t sq = {0};
    rf_pkt_t *pkt = NULL;
    u8 ms = 0;

    ccrf_clock_t ts;
    u32 chan_elapsed;
    u32 remaining = 0;
    volatile TickType_t tx_next = 0;
    TickType_t loop_wait = portMAX_DELAY;
    u32 notify;

    rdio_ccac_t ccac = 0;

    phy_sync_pkt_t pkt_sync = {
            .hdr = {
                    .size = sizeof(phy_sync_pkt_t) - 1,
                    .flag = (u8)(PHY_PKT_FLAG_IMMEDIATE | PHY_PKT_FLAG_SYNC)
            },
            .magic = PHY_SYNC_MAGIC
    };

    bool sync_needed = false;
    phy->sync_time = 0;

    if (phy->boss) {
        xTaskNotify(xTaskGetCurrentTaskHandle(), NOTIFY_MASK_HOP, eSetBits);
    } else {
        phy_chan_set(phy, 0);
        rdio_strobe_rx(phy->rdio);
    }

    /*ccrf_clock_t loop_time_start = ccrf_clock();
    ccrf_clock_t loop_time;
    ccrf_clock_t loop_time_prev = loop_time_start;
    ccrf_clock_t loop_time_sum = 0;
    u32 loop_time_count = 0;*/

    while (1) {

        /*ts = ccrf_clock();
        loop_time = ts - loop_time_prev;
        loop_time_sum += loop_time;

        if (++loop_time_count == 1000) {
            const u32 elapsed = (u32)(ts - loop_time_start);
            const u32 loop_time_avg_usec = (u32)(loop_time_sum / loop_time_count);
            const u32 loops_per_sec = (u32)(1000000ul * (u64)loop_time_count / elapsed);
            itm_printf(0, "(loop) time=%lu us\tfreq=%lu Hz\r\n", loop_time_avg_usec, loops_per_sec);
            loop_time_sum = 0;
            loop_time_count = 0;
            loop_time_start = ccrf_clock();
        }

        loop_time_prev = ts;*/

        if (xTaskNotifyWait(0, NOTIFY_MASK_ALL, &notify, loop_wait)) {

            if (notify & NOTIFY_MASK_ISR) {
                ms = rdio_reg_get(phy->rdio, CC1200_MARC_STATUS1, NULL);

                switch (ms) {
                    case CC1200_MARC_STATUS1_RX_FINISHED:
                        if (phy_recv(phy, true)) {
                            if (tx_next && (phy->sync_time || phy->boss)) tx_next = 0;
                        } else {
                            if (!tx_next) {
                                /**
                                 * This is meant for rare cases when we want to force
                                 * back into rx at the clear expectation of more packets.
                                 * In some cases, this does not matter, such as when an
                                 * immediate (e.g. ack) is sent after this.
                                 */
                                tx_next = xTaskGetTickCount() + pdMS_TO_TICKS(1 + (rand() % 6));
                            }
                        }

                        if (pkt) goto _cca_fail;
                        break;

                    case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
                    case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:
                    case CC1200_MARC_STATUS1_CRC:
                    case CC1200_MARC_STATUS1_ADDRESS:
                        rdio_mode_idle(phy->rdio);
                        rdio_strobe_rxfl(phy->rdio);
                        ++phy->stat.rx.errors;
                        if (pkt) goto _cca_fail;
                        break;

                    case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
                    _cca_fail:
                        // Fall through and let the packet be re-queued.
                        // If pkt is pkt_ack or pkt_sync (and we got here via goto), it disappears.
                        //   But that's unlikely because those are sent from idle.

                    case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
                    case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
                        rdio_mode_idle(phy->rdio);
                        rdio_strobe_txfl(phy->rdio);

                    case CC1200_MARC_STATUS1_TX_FINISHED:
                        if (pkt) {
                            if (pkt == (rf_pkt_t *)&sq.pkt && sq.task) {
                                xTaskNotify(
                                        sq.task,
                                        ms == CC1200_MARC_STATUS1_TX_FINISHED ? CALLER_NOTIFY_TX_DONE : CALLER_NOTIFY_TX_FAIL,
                                        eSetBits
                                );
                            }

                            if ((void*)pkt == &phy->pkt_ack) {
                                phy->pkt_ack.hdr.flag = 0;
                                phy->pkt_ack.hdr.size = 0;
                            }

                            if ((void*)pkt == &pkt_sync) {
                                phy->sync_time = ccrf_clock();
                                ccrf_timer_restart(phy->hop_timer);
                            } else {
                                if (ms == CC1200_MARC_STATUS1_TX_FINISHED) {
                                    ++phy->stat.tx.count;
                                    phy->stat.tx.bytes += pkt->size + 1;
                                } else {
                                    ++phy->stat.tx.errors;
                                }
                            }

                            if (!(((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE)) {
                                rdio_cca_end(phy->rdio, ccac);
                                tx_next = xTaskGetTickCount() + pdMS_TO_TICKS(1 + (rand() % 5));
                            }

                            if (phy->diag.cw) rdio_cw_set(phy->rdio, true);

                            pkt = NULL;

                        } else {
                            phy_trace_debug("tx completion indicated but no packet pending");
                        }

                        break;

                    default:
                    case CC1200_MARC_STATUS1_NO_FAILURE:
                        // Can be the result of a SIDLE strobe.
                        if (!(notify & ~NOTIFY_MASK_ISR)) goto _loop_end;
                        break;
                }
            }

            if (notify & NOTIFY_MASK_DIAG) {

                if (phy->boss != phy->diag.boss) {
                    phy->sync_time = 0;

                    if ((phy->boss = phy->diag.boss)) {
                        phy->chan.cur = CHAN_ID_INVALID;
                        notify |= NOTIFY_MASK_HOP;
                    } else {
                        ccrf_timer_stop(phy->hop_timer);
                    }
                }

                if (!phy->boss && phy->diag.chan != (chan_id_t) CHAN_ID_INVALID) {
                    // Repeated calls get ingored within these functions.
                    phy_chan_set_base(phy, phy->diag.chan);
                }

                rdio_cw_set(phy->rdio, phy->diag.cw);
            }

            if ((notify & NOTIFY_MASK_HOP) && (phy->boss || phy->sync_time)) {
                ts = ccrf_clock();
                tx_next = 0;

                if (!phy->boss && !phy->chan.cur) {
                    if (((ts - phy->sync_time) >= 5*(CHAN_TIME * CHAN_COUNT))) {
                        phy_trace_debug("phy/%u: sync loss: last=%lu now=%lu", rdio_id(phy->rdio), CCRF_CLOCK_MSEC(phy->sync_time), CCRF_CLOCK_MSEC(ts));
                        // TODO: Stop timer?
                        phy->sync_time = 0;
                        goto _inner_loop;

                    } else if ((ts - phy->sync_time) >= 2*CHAN_TIME) {
                        phy_trace_verbose("phy/%u: sync miss: last=%lu now=%lu", rdio_id(phy->rdio), CCRF_CLOCK_MSEC(phy->sync_time), CCRF_CLOCK_MSEC(ts));
                    }
                }

                phy_chan_next(phy);

                if (phy->boss && !phy->chan.cur) {
                    if (!phy->sync_time) {
                        phy->sync_time = ts;
                        ccrf_timer_start(phy->hop_timer);
                    }

                    sync_needed = !phy->diag.nosync;
                }

                if (phy->sync) phy->sync(phy->chan.cur);

                if (pkt) {
                    const u8 tx_bytes = rdio_reg_get(phy->rdio, CC1200_NUM_TXBYTES, NULL);

                    /*if (tx_bytes == pkt->size + 1) {

                    }*/

                    rdio_strobe_txfl(phy->rdio);

                    phy_trace_debug("chan: hop tx canned: len=%u txbytes=%u", pkt->size, tx_bytes);

                    if (pkt == (rf_pkt_t *)&sq.pkt && sq.task) {
                        xTaskNotify(sq.task, CALLER_NOTIFY_TX_FAIL, eSetBits);
                    } else {
                        // Packet disappears (pkt_ack or pkt_sync).
                    }

                    // TODO: Need common post-tx handler

                    if ((void *) pkt == &phy->pkt_ack) {
                        phy->pkt_ack.hdr.flag = 0;
                        phy->pkt_ack.hdr.size = 0;
                    }

                    ++phy->stat.tx.errors;

                    if (phy->diag.cw) rdio_cw_set(phy->rdio, false);

                    pkt = NULL;
                }
            }

            if (notify & NOTIFY_MASK_TX) {
                // Just fall through and try to send.
            }

        } else {
            // Timeout, now == tx_next
            tx_next = 0;
        }

        _inner_loop:;

        if (sync_needed) {
            sync_needed = false;

            assert(!pkt);
            pkt = (rf_pkt_t *) &pkt_sync;

            pkt_sync.hdr.cell = phy->cell;

            rdio_mode_idle(phy->rdio); // TODO: Perhaps always strobing here would be better/more predictable

            if (phy->diag.cw) rdio_cw_set(phy->rdio, false);

            rdio_fifo_write(phy->rdio, (u8 *) pkt, pkt->size + (u8)1);
            rdio_strobe_tx(phy->rdio);

        } else if (!pkt && ((volatile u8)phy->pkt_ack.hdr.flag || xQueuePeek(phy->txq, &sq, 0))) {
            pkt = (volatile u8)phy->pkt_ack.hdr.flag ? (rf_pkt_t *)&phy->pkt_ack : (rf_pkt_t *)&sq.pkt;

            volatile bool synced = (phy->sync_time != 0);
            volatile bool imm = synced && ((((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) != 0);

            if (!imm && tx_next) {
                ts = xTaskGetTickCount();

                if (tx_next > (ts + pdUS_TO_TICKS(100))) {
                    pkt = NULL;
                    goto _restart_rx;
                }
            }

            if (synced) {
                remaining = ccrf_timer_remaining(phy->hop_timer);
                chan_elapsed = CHAN_TIME - remaining;

                u32 lead_time = 200 + ((!phy->boss && !phy->chan.cur) ? 500 + tx_times[pkt_sync.hdr.size] : 0);

                if (chan_elapsed < lead_time) {
                    tx_next = xTaskGetTickCount() + pdUS_TO_TICKS(lead_time);
                    pkt = NULL;
                    goto _restart_rx;
                }

                if (remaining < (750 + tx_times[pkt->size])) { // TODO: Give more time for CCA-TX? (non-imm)
                    tx_next = 0;
                    pkt = NULL;
                    goto _restart_rx;
                }
            }

            if (pkt != (rf_pkt_t *)&phy->pkt_ack) {
                xQueueReceive(phy->txq, &sq, 0);

                if (!synced) {
                    if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE)
                        ((phy_pkt_t *)pkt)->hdr.flag &= ~PHY_PKT_FLAG_IMMEDIATE;
                }
            }

            if (imm) {
                rdio_mode_idle(phy->rdio);
            }

            if (phy->diag.cw) {
                rdio_mode_idle(phy->rdio);
                rdio_cw_set(phy->rdio, false);
            }

            rdio_fifo_write(phy->rdio, (u8 *) pkt, pkt->size + (u8)1);

            if (!imm) {
                rdio_cca_begin(phy->rdio, &ccac);
            }

            rdio_strobe_tx(phy->rdio);
            tx_next = 0;

        } else if (!pkt) {
            goto _restart_rx;
        }

        goto _loop_end;

        _restart_rx:

        if (!phy->diag.cw) {
            rdio_mode_rx(phy->rdio);
        } else {
            rdio_mode_tx(phy->rdio);
        }

        _loop_end:;

        if (tx_next) {
            loop_wait = xTaskGetTickCount();

            if (tx_next > loop_wait) {
                loop_wait = tx_next - loop_wait;

            } else {
                tx_next = 0;
                loop_wait = 0;
            }

        } else {
            loop_wait = portMAX_DELAY;
        }
    }
}


static bool phy_recv(phy_t phy, bool flush)
{
    const static u8 PKT_OVERHEAD = 3; // length, 2x status

    u8 len = rdio_reg_get(phy->rdio, CC1200_NUM_RXBYTES, NULL);

    if (len < PKT_OVERHEAD) {
        if (flush) rdio_strobe_rxfl(phy->rdio);
        return false;
    }

    rf_pkt_t *spkt;
    u8 *buf = alloca(len);
    bool unblock = false;

    rdio_fifo_read(phy->rdio, buf, len);

    while (len > PKT_OVERHEAD) {
        spkt = (rf_pkt_t *)buf;

        if (spkt->size > PHY_RF_FRAME_SIZE_MAX) {
            break;
        }

        if (spkt->size > (len - PKT_OVERHEAD)) {
            break;
        }

        if (spkt->size) {
            const s8 rssi = (s8) spkt->data[spkt->size];
            const u8 crc_ok = spkt->data[spkt->size + 1] & (u8) CC1200_LQI_CRC_OK_BM;
            const u8 lqi = spkt->data[spkt->size + 1] & (u8) CC1200_LQI_EST_BM;

            if (crc_ok) {
                unblock |= phy_recv_packet(phy, spkt, rssi, lqi);
            } else {
                break;
            }
        } else {
            break;
        }

        len -= spkt->size + PKT_OVERHEAD;
        buf += spkt->size + PKT_OVERHEAD;
    }

    if (flush && len) rdio_strobe_rxfl(phy->rdio);

    return unblock;
}


static bool phy_recv_packet(phy_t phy, rf_pkt_t *pkt, s8 rssi, u8 lqi)
{
    phy_pkt_t *const ppkt = (phy_pkt_t *)pkt;

    if (ppkt->hdr.size < sizeof(phy_pkt_hdr_t)) {
        ++phy->stat.rx.errors;
        return false;
    }

    if (ppkt->hdr.flag & PHY_PKT_FLAG_SYNC) {
        const phy_sync_pkt_t *const spkt = (phy_sync_pkt_t *) pkt;

        if (spkt->magic == PHY_SYNC_MAGIC) {
            if (!phy->boss) {
                phy->sync_time = ccrf_clock();
                ccrf_timer_restart(phy->hop_timer);
                return true;
            }
        }

        return false;
    }

    ++phy->stat.rx.count;
    phy->stat.rx.bytes += ppkt->hdr.size + 1;

    ppkt->hdr.size -= (sizeof(phy_pkt_t) - sizeof(rf_pkt_t));

    return phy->recv(phy->recv_param, ppkt->hdr.flag, ppkt->hdr.size, ppkt->data, (pkt_meta_t){rssi, lqi});
}


static inline void phy_chan_next(phy_t phy)
{
    // NOTE: Incrementing from CHAN_ID_INVALID wraps to zero.
    chan_id_t next = phy->chan.cur + (chan_id_t)1;
    if (next >= CHAN_COUNT) next = 0;
    phy_chan_set(phy, next);
}


static inline void phy_chan_set(phy_t phy, chan_id_t chan)
{
    if (chan != phy->chan.cur) {
        phy->chan.cur = chan;
        rdio_mode_idle(phy->rdio);
        chan_select(&phy->chan.group, phy->chan.hop_table[chan]);
    }
}


static inline chan_id_t phy_chan_set_base(phy_t phy, chan_id_t chan)
{
    for (chan_id_t c = 0; c < phy->chan.group.size; ++c) {
        if (phy->chan.hop_table[c] == chan) {
            phy_chan_set(phy, c);
            return phy->chan.hop_table[c];
        }
    }

    return CHAN_ID_INVALID;
}


static void phy_rdio_isr(phy_t phy)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (phy && phy->task) {
        xTaskNotifyFromISR(phy->task, NOTIFY_MASK_ISR, eSetBits, &xHigherPriorityTaskWoken);
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}


static void hop_timer_handler(ccrf_timer_t timer __unused, phy_t phy)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (phy && phy->task) {
        xTaskNotifyFromISR(phy->task, NOTIFY_MASK_HOP, eSetBits, &xHigherPriorityTaskWoken);
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
