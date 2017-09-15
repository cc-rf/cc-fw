#include "phy/phy.h"
#include "phy/config.h"
#include "sys/amp.h"
#include "sys/trace.h"
#include "sys/timer.h"
#include "sys/clock.h"
#include "sys/local.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <assert.h>


#define phy_trace_info      ccrf_trace_info
#define phy_trace_warn      ccrf_trace_warn
#define phy_trace_error     ccrf_trace_error
#define phy_trace_debug     ccrf_trace_debug
#define phy_trace_verbose   ccrf_trace_verbose


#define FREQ_BASE       902125000u
#define FREQ_BW         950000u
#define CHAN_COUNT      25u
#define CHAN_TIME       40000u


#define PHY_PKT_FLAG_SYNC       (0x01)   // is a sync packet

#define PHY_TXQ_LEN             3

#define PHY_TASK_STACK_SIZE     (TASK_STACK_SIZE_LARGE / sizeof(StackType_t))

#define NOTIFY_MASK_ISR         (1<<1)
#define NOTIFY_MASK_TX          (1<<2)
#define NOTIFY_MASK_HOP         (1<<3)
#define NOTIFY_MASK_ALL         (NOTIFY_MASK_ISR | NOTIFY_MASK_TX | NOTIFY_MASK_HOP)

#define CALLER_NOTIFY_TX_DONE   (1<<4)
#define CALLER_NOTIFY_TX_FAIL   (1<<5)
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

    } chan;

    ccrf_timer_t hop_timer;
    volatile phy_static_pkt_t pkt_ack;
    volatile ccrf_clock_t sync_time;
    bool sync_needed;

    TaskHandle_t task;
    StaticTask_t task_static;
    StackType_t task_stack[PHY_TASK_STACK_SIZE];

    QueueHandle_t txq;
    StaticQueue_t txq_static;
    phy_send_queue_t txq_buf[PHY_TXQ_LEN];
};


static void phy_task(phy_t const restrict phy);

static bool phy_recv(phy_t phy, bool flush);
static bool phy_recv_packet(phy_t phy, rf_pkt_t *pkt, s8 rssi, u8 lqi);

static inline void phy_chan_next(phy_t phy, chan_id_t *chan);
static inline void phy_chan_set(phy_t phy, chan_id_t chan);

static void phy_rdio_isr(phy_t phy);
static void hop_timer_handler(ccrf_timer_t timer, phy_t phy);


u32 tx_times[PHY_FRAME_SIZE_MAX+1] = {0};

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
    ccrf_amp_ctrl(phy->rdio, CCRF_AMP_HGM, true);

    phy->chan.group = (chan_group_t) {
            .rdio = phy->rdio,
            .freq = {
                    .base = FREQ_BASE,
                    .bw = FREQ_BW
            },
            .size = CHAN_COUNT
    };

    chan_group_init(&phy->chan.group, phy->chan.hop_table);
    chan_table_reorder(&phy->chan.group, phy->cell, phy->chan.hop_table);
    chan_group_calibrate(&phy->chan.group);

    if (!(phy->hop_timer = ccrf_timer_init(CHAN_TIME, (ccrf_timer_handler_t) hop_timer_handler, phy))) {
        phy_trace_error("timer init fail");
        goto _fail;
    }

    if (!tx_times[PHY_FRAME_SIZE_MAX]) {
        for (u8 i = 0; i <= PHY_FRAME_SIZE_MAX; ++i) {
            tx_times[i] = rdio_util_get_tx_time(phy->rdio, i);
        }
    }

    phy->txq = xQueueCreateStatic(
            PHY_TXQ_LEN, sizeof(phy->txq_buf[0]), (u8 *)phy->txq_buf, &phy->txq_static); assert(phy->txq
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


rdio_t phy_rdio(phy_t phy)
{
    return phy->rdio;
}

bool phy_boss(phy_t phy)
{
    return phy->boss;
}

phy_cell_t phy_cell(phy_t phy)
{
    return phy->cell;
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

            if (xQueueSendToFront(phy->txq, &sq, pdMS_TO_TICKS(10))) {
                xTaskNotify(phy->task, NOTIFY_MASK_TX, eSetBits);
            } else {
                phy_trace_error("tx pkt queue immediate fail");
            }
        }

    } else if (xQueueSend(phy->txq, &sq, pdMS_TO_TICKS(10))) {
        xTaskNotify(phy->task, NOTIFY_MASK_TX, eSetBits);
    } else {
        phy_trace_error("tx pkt queue fail");
    }

    if (block) {
        u32 notify;

        do {
            if (!xTaskNotifyWait(CALLER_NOTIFY_TX_MASK, CALLER_NOTIFY_TX_MASK, &notify, pdMS_TO_TICKS(5)))
                notify = 0;

        } while (!(notify & CALLER_NOTIFY_TX_MASK));

        return (notify & CALLER_NOTIFY_TX_DONE) != 0;
    }

    return true;
}


static void phy_task(phy_t const restrict phy)
{
    phy_send_queue_t sq;
    rf_pkt_t *pkt = NULL;
    u8 ms = 0;

    ccrf_clock_t ts;
    u32 chan_elapsed;
    u32 remaining = 0;
    volatile TickType_t tx_next = 0;
    TickType_t loop_wait = portMAX_DELAY;
    u32 notify;

    volatile chan_id_t chan_cur = (chan_id_t) -1;

    u16 cca_fail_count = 0;
    rdio_ccac_t ccac = 0;

    phy_sync_pkt_t pkt_sync = {
            .hdr = {
                    .size = sizeof(phy_sync_pkt_t) - 1,
                    .flag = (u8)(PHY_PKT_FLAG_IMMEDIATE | PHY_PKT_FLAG_SYNC)
            },
            .magic = PHY_SYNC_MAGIC
    };

    phy->sync_needed = false;
    phy->sync_time = 0;

    ///ccrf_amp_mode_rx(phy->rdio, true);

    if (phy->boss) {
        xTaskNotify(xTaskGetCurrentTaskHandle(), NOTIFY_MASK_HOP, eSetBits);
    } else {
        phy_chan_set(phy, chan_cur = 0);
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
                            //const u8 rand = rdio_reg_get(phy->rdio, CC1200_RNDGEN, NULL) & CC1200_RNDGEN_VALUE_M;
                            //tx_next = xTaskGetTickCount() + pdMS_TO_TICKS(1) + pdMS_TO_TICKS(rand % 9);
                        }

                        if (pkt) goto _cca_fail;
                        break;

                    case CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW:
                    case CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW:
                    case CC1200_MARC_STATUS1_ADDRESS:
                    case CC1200_MARC_STATUS1_CRC:
                        rdio_mode_idle(phy->rdio);
                        rdio_strobe_rxfl(phy->rdio);
                        if (pkt) goto _cca_fail;
                        break;

                    case CC1200_MARC_STATUS1_TX_ON_CCA_FAILED:
                    _cca_fail:
                        if (pkt) {
                            ++cca_fail_count;
                            //phy_trace_debug("cca fail #%u\n", cca_fail_count);

                            assert(pkt == (rf_pkt_t *)&sq.pkt);
                            assert(!(((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE));

                            //const u8 rand = rdio_reg_get(phy->rdio, CC1200_RNDGEN, NULL) & CC1200_RNDGEN_VALUE_M;
                            //tx_next = xTaskGetTickCount() + pdMS_TO_TICKS(3) + pdMS_TO_TICKS(rand % 8);
                        }

                    case CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW:
                    case CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW:
                        rdio_mode_idle(phy->rdio);
                        rdio_strobe_txfl(phy->rdio);

                    case CC1200_MARC_STATUS1_TX_FINISHED:
                        ///ccrf_amp_mode_tx(phy->rdio, false);

                        if (pkt) {
                            cca_fail_count = 0;

                            if (pkt == (rf_pkt_t *)&sq.pkt && sq.task) {
                                xTaskNotify(
                                        sq.task,
                                        ms == CC1200_MARC_STATUS1_TX_FINISHED ? CALLER_NOTIFY_TX_DONE : CALLER_NOTIFY_TX_FAIL,
                                        eSetBits
                                );
                            }

                            if (((phy_pkt_t *)pkt)->hdr.flag & PHY_PKT_FLAG_IMMEDIATE) {
                                if ((void*)pkt == &phy->pkt_ack) {
                                    phy->pkt_ack.hdr.flag = 0;
                                    phy->pkt_ack.hdr.size = 0;
                                }

                                if ((void*)pkt == &pkt_sync) {

                                    phy->sync_time = ccrf_clock();
                                    ccrf_timer_restart(phy->hop_timer);
                                }

                            } else {
                                rdio_cca_end(phy->rdio, ccac);

                                const u8 rand = rdio_reg_get(phy->rdio, CC1200_RNDGEN, NULL) & CC1200_RNDGEN_VALUE_M;
                                tx_next = xTaskGetTickCount() + pdMS_TO_TICKS(5) + pdMS_TO_TICKS(rand % 6);
                            }

                            pkt = NULL;

                        } else {
                            phy_trace_debug("tx completion indicated but no packet pending");
                        }

                        break;

                    default:
                    case CC1200_MARC_STATUS1_NO_FAILURE:
                        if (!(notify & ~NOTIFY_MASK_ISR)) continue; // usually the result of a SIDLE strobe
                        break;
                }
            }

            if ((notify & NOTIFY_MASK_HOP) && (phy->boss || phy->sync_time)) {
                ts = ccrf_clock();
                tx_next = 0;

                if (!phy->boss && !chan_cur) {
                    if (((ts - phy->sync_time) >= 5*(CHAN_TIME * CHAN_COUNT))) {
                        phy_trace_debug("phy/%u: sync loss: last=%lu now=%lu", rdio_id(phy->rdio), CCRF_CLOCK_MSEC(phy->sync_time), CCRF_CLOCK_MSEC(ts));
                        phy->sync_time = 0;
                        goto _inner_loop;

                    } else if ((ts - phy->sync_time) >= 2*CHAN_TIME) {
                        phy_trace_verbose("phy/%u: sync miss: last=%lu now=%lu", rdio_id(phy->rdio), CCRF_CLOCK_MSEC(phy->sync_time), CCRF_CLOCK_MSEC(ts));
                    }
                }

                phy_chan_next(phy, (chan_id_t *) &chan_cur);

                if (phy->boss && !chan_cur) {
                    if (!phy->sync_time) {
                        phy->sync_time = ts;
                        ccrf_timer_start(phy->hop_timer);
                    }

                    phy->sync_needed = true;
                }

                if (phy->sync) phy->sync(chan_cur);

                if (pkt) {
                    const u8 tx_bytes = rdio_reg_get(phy->rdio, CC1200_NUM_TXBYTES, NULL);

                    /*if (tx_bytes == pkt->size + 1) {

                    }*/

                    rdio_strobe_txfl(phy->rdio);

                    phy_trace_debug("chan: hop tx canned: len=%u txbytes=%u", pkt->size, tx_bytes);

                    if (pkt == (rf_pkt_t *)&sq.pkt && sq.task) {
                        xTaskNotify(sq.task, CALLER_NOTIFY_TX_FAIL, eSetBits);
                    }

                    // TODO: Need common post-tx handler

                    ///ccrf_amp_mode_tx(phy->rdio, false);

                    cca_fail_count = 0;

                    if ((void *) pkt == &phy->pkt_ack) {
                        phy->pkt_ack.hdr.flag = 0;
                        phy->pkt_ack.hdr.size = 0;
                    }

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

        if (phy->sync_needed) {
            phy->sync_needed = false;

            assert(!pkt);
            pkt = (rf_pkt_t *) &pkt_sync;

            pkt_sync.hdr.cell = phy->cell;

            rdio_mode_idle(phy->rdio); // TODO: Perhaps always strobing here would be better/more predictable
            rdio_fifo_write(phy->rdio, (u8 *) pkt, pkt->size + (u8)1);
            ///ccrf_amp_mode_tx(phy->rdio, true);
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

            /*if (!imm) {
                // Manual CCA.

                ///ccrf_amp_mode_rx(phy->rdio, true);
                s16 rssi;
                rdio_rssi_read(phy->rdio, &rssi);

                if (rssi >= -97) {
                    phy_trace_debug("rb\t%i", rssi);
                    pkt = NULL;

                    const u8 rand = rdio_reg_get(phy->rdio, CC1200_RNDGEN, NULL) & CC1200_RNDGEN_VALUE_M;
                    tx_next = xTaskGetTickCount() + pdMS_TO_TICKS(5) + pdMS_TO_TICKS(rand % 6);

                    goto _loop_end;
                }
            }*/

            if (synced) {
                remaining = ccrf_timer_remaining(phy->hop_timer);
                chan_elapsed = CHAN_TIME - remaining;

                u32 lead_time = 200 + ((!phy->boss && !chan_cur) ? 500 + tx_times[pkt_sync.hdr.size] : 0);

                if (chan_elapsed < lead_time) {
                    tx_next = xTaskGetTickCount() + pdUS_TO_TICKS(lead_time);
                    pkt = NULL;
                    goto _restart_rx;
                }

                if (remaining < (500 + tx_times[pkt->size])) {
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

            rdio_fifo_write(phy->rdio, (u8 *) pkt, pkt->size + (u8)1);

            if (!imm) {
                ///ccrf_amp_mode_rx(phy->rdio, true);
                rdio_cca_begin(phy->rdio, &ccac);
            }

            ///ccrf_amp_mode_tx(phy->rdio, true);
            rdio_strobe_tx(phy->rdio);
            tx_next = 0;

        } else if (!pkt) {
            goto _restart_rx;
        }

        goto _loop_end;

        _restart_rx:

        rdio_mode_rx(phy->rdio);

        _loop_end:;

        if (tx_next) {
            loop_wait = xTaskGetTickCount();

            if (tx_next > loop_wait) {
                loop_wait = tx_next - loop_wait;

            } else {
                tx_next = 0;
                loop_wait = portMAX_DELAY; //phy->sync_time ? portMAX_DELAY : pdUS_TO_TICKS(CHAN_TIME);
            }

        } else {
            loop_wait = portMAX_DELAY; //phy->sync_time ? portMAX_DELAY : pdUS_TO_TICKS(CHAN_TIME);
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
    size_t pkt_count = 0;
    bool unblock = false;

    rdio_fifo_read(phy->rdio, buf, len);

    while (len > PKT_OVERHEAD) {
        spkt = (rf_pkt_t *)buf;

        if (spkt->size > PHY_FRAME_SIZE_MAX) {
            break;
        }

        if (spkt->size > (len - PKT_OVERHEAD)) {
            break;
        }

        ++pkt_count;

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
        return false;
    }

    if (ppkt->hdr.flag & PHY_PKT_FLAG_SYNC) {
        const phy_sync_pkt_t *const spkt = (phy_sync_pkt_t *) pkt;

        if (spkt->magic == PHY_SYNC_MAGIC) {
            if (!phy->boss) {
                phy->sync_time = ccrf_clock();
                ccrf_timer_restart(phy->hop_timer);
            }
        }

        return false;
    }

    ppkt->hdr.size -= (sizeof(phy_pkt_t) - sizeof(rf_pkt_t));

    return phy->recv(phy->recv_param, ppkt->hdr.flag, ppkt->hdr.size, ppkt->data, rssi, lqi);
}


static inline void phy_chan_next(phy_t phy, chan_id_t *chan)
{
    if (++*chan >= CHAN_COUNT) *chan = 0;
    phy_chan_set(phy, *chan);
}


static inline void phy_chan_set(phy_t phy, chan_id_t chan)
{
    static volatile chan_id_t chan_prev = (chan_id_t) -1;

    if (chan != chan_prev) {
        chan_prev = chan;
        rdio_mode_idle(phy->rdio);
        chan_select(&phy->chan.group, phy->chan.hop_table[chan]);
    }
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
