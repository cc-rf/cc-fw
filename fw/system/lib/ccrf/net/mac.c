#include <ccrf/mac.h>
#include "phy/phy.h"
#include "sys/trace.h"
#include "sys/local.h"

#include <FreeRTOS.h>
#include <assert.h>
#include <task.h>
#include <queue.h>


#define mac_trace_info          ccrf_trace_info
#define mac_trace_warn          ccrf_trace_warn
#define mac_trace_error         ccrf_trace_error
#define mac_trace_debug         ccrf_trace_debug
#define mac_trace_verbose       ccrf_trace_verbose


#define MAC_PEER_MAX            10

#define MAC_PEND_TIME           7
#define MAC_PEND_RETRY          5

#define MAC_TXQ_COUNT           7 // This should eventually reflect the number of threads waiting on messages
#define MAC_TXQ_SIZE            (MAC_TXQ_COUNT)
#define MAC_RXQ_SIZE            7

#define MAC_TX_TASK_STACK_SIZE  (TASK_STACK_SIZE_LARGE / sizeof(StackType_t))
#define MAC_RX_TASK_STACK_SIZE  (TASK_STACK_SIZE_MEDIUM / sizeof(StackType_t))

#define MAC_NOTIFY_ACK          (1u<<10)

#define MAC_FLAG_PKT_IMM        (PHY_PKT_FLAG_USER_0 >> 1)  // Local
#define MAC_FLAG_PKT_BLK        (PHY_PKT_FLAG_USER_0 >> 2)  // Local
#define MAC_FLAG_ACK_REQ        (PHY_PKT_FLAG_USER_0)
#define MAC_FLAG_ACK_RSP        (PHY_PKT_FLAG_USER_1)
#define MAC_FLAG_ACK_RQR        (PHY_PKT_FLAG_USER_2)

#define MAC_SEQ_INIT            ((mac_seq_t){0,0,0,0})

typedef struct __packed {
    u8 num : 8;
    u8 msg : 4;
    u8 idx : 3;
    u8 end : 1;

} mac_seq_t;

typedef struct __packed {
    mac_addr_t addr;
    mac_addr_t dest;
    mac_seq_t seq;
    u16 size;
    u8  data[];

} mac_pkt_t;

typedef struct __packed {
    mac_addr_t addr;
    mac_addr_t dest;
    mac_seq_t seq;
    u16 size;
    u8  data[MAC_PKT_SIZE_MAX];

} mac_static_pkt_t;

typedef struct __packed {
    mac_static_pkt_t pkt;
    u8 flag;

} mac_send_queue_t;

typedef struct __packed {
    mac_addr_t dest;
    mac_size_t size;
    mac_seq_t seq;
    u8 data[];

} mac_part_t;

typedef struct __packed {
    mac_addr_t addr;
    mac_seq_t seq;
    mac_part_t *part;

} mac_peer_t;

typedef struct __packed {
    pkt_meta_t meta;
    mac_peer_t *peer;
    mac_addr_t dest;
    mac_size_t size;
    mac_seq_t seq;
    u8 data[MAC_PKT_SIZE_MAX];

} mac_recv_data_t;

typedef struct __packed {
    mac_pkt_t *pkt;
    TaskHandle_t task;
    volatile u8 flag;

} mac_pend_t;

struct __packed mac {
    phy_t phy;
    mac_addr_t addr;
    mac_recv_t recv;

    mac_seq_t seq;
    mac_peer_t peer[MAC_PEER_MAX];
    mac_pend_t pend;

    TaskHandle_t task;
    StaticTask_t task_static;
    StackType_t task_stack[MAC_TX_TASK_STACK_SIZE];

    TaskHandle_t rx_task;
    StaticTask_t rx_task_static;
    StackType_t rx_task_stack[MAC_RX_TASK_STACK_SIZE];
    
    QueueHandle_t txq;
    StaticQueue_t txq_static;
    mac_send_queue_t txq_buf[MAC_TXQ_SIZE];
    
    QueueHandle_t rxq;
    StaticQueue_t rxq_static;
    mac_recv_data_t rxq_buf[MAC_RXQ_SIZE];
};


static mac_peer_t *mac_peer_get(mac_t mac, mac_addr_t addr);

static bool mac_send_base(mac_t mac, mac_addr_t dest, u8 flag, mac_size_t size, u8 data[]);
static bool mac_send_packet(mac_t mac, u8 flag, mac_static_pkt_t *pkt);

static void mac_task_send(mac_t mac);
static void mac_task_recv(mac_t mac);

static bool mac_phy_recv(mac_t mac, u8 flag, u8 size, u8 data[], pkt_meta_t meta);


static struct mac macs[CCRF_CONFIG_RDIO_COUNT];


mac_t mac_init(mac_config_t *config)
{
    mac_t mac = &macs[config->rdid];

    memset(mac, 0, sizeof(struct mac));

    mac->addr = config->addr;
    mac->recv = config->recv;

    phy_config_t phy_config = {
            .rdid = config->rdid,
            .boss = config->boss,
            .cell = config->cell,
            .sync = config->sync,
            .recv = (phy_recv_t) mac_phy_recv,
            .recv_param = mac
    };

    if (!(mac->phy = phy_init(&phy_config))) {
        goto _fail;
    }

    // This is important for variability in rand() calls within phy.
    srand(mac->addr);

    mac->txq = xQueueCreateStatic(MAC_TXQ_SIZE, sizeof(mac->txq_buf[0]), (u8 *)mac->txq_buf, &mac->txq_static);
    mac->rxq = xQueueCreateStatic(MAC_RXQ_SIZE, sizeof(mac->rxq_buf[0]), (u8 *)mac->rxq_buf, &mac->rxq_static);

    mac->task = xTaskCreateStatic(
            (TaskFunction_t) mac_task_send, "mac:send", MAC_TX_TASK_STACK_SIZE, mac, TASK_PRIO_HIGHEST - 2, mac->task_stack, &mac->task_static
    );
    
    mac->rx_task = xTaskCreateStatic(
            (TaskFunction_t) mac_task_recv, "mac:recv", MAC_RX_TASK_STACK_SIZE, mac, TASK_PRIO_HIGH, mac->rx_task_stack, &mac->rx_task_static
    );

    goto _done;
    _fail:

    if (mac) {
        mac = NULL;
    }

    _done:
    return mac;
}


mac_addr_t mac_addr(mac_t mac)
{
    return mac->addr;
}


bool mac_boss(mac_t mac)
{
    return phy_boss(mac->phy);
}


phy_cell_t mac_cell(mac_t mac)
{
    return phy_cell(mac->phy);
}


static mac_peer_t *mac_peer_get(mac_t mac, mac_addr_t addr)
{
    mac_peer_t *pp = NULL;

    if (!addr) return NULL;

    for (u8 i = 0; i < MAC_PEER_MAX; ++i) {
        if (mac->peer[i].addr == addr) {
            return &mac->peer[i];
        } else if (!pp && !mac->peer[i].addr) { // TODO: Change this logic to support peer removal/expiry
            pp = &mac->peer[i];
        }
    }

    if (pp) {
        pp->addr = addr;
        pp->seq = MAC_SEQ_INIT;
        pp->part = NULL;
        return pp;
    }

    return NULL;
}


bool mac_send(mac_t mac, mac_send_t type, mac_addr_t dest, mac_size_t size, u8 data[])
{
    switch (type) {
        case MAC_SEND_DGRM:
            return mac_send_base(mac, dest, 0, size, data);

        case MAC_SEND_MESG:
            return mac_send_base(mac, dest, MAC_FLAG_ACK_REQ | MAC_FLAG_PKT_BLK, size, data);

        case MAC_SEND_TRXN:
            return mac_send_base(mac, dest, MAC_FLAG_ACK_REQ | MAC_FLAG_PKT_BLK, size, data);

        case MAC_SEND_STRM:
            return mac_send_base(mac, dest, MAC_FLAG_PKT_IMM | MAC_FLAG_PKT_BLK, size, data);

        default:
            return false;
    }
}


static bool mac_send_base(mac_t mac, mac_addr_t dest, u8 flag, mac_size_t size, u8 data[])
{
    mac_size_t pkt_size;
    mac_size_t remaining = size;

    mac_static_pkt_t pkt = {
            .addr = mac->addr,
            .dest = dest,
            .seq = MAC_SEQ_INIT,
    };

    do {
        if (!++mac->seq.num) mac->seq.num = 1;

        pkt.seq.num = mac->seq.num;

        if (size > MAC_PKT_SIZE_MAX) {
            if (!pkt.seq.msg) {
                if (!++mac->seq.msg) mac->seq.msg = 1;
                pkt.seq.msg = mac->seq.msg;
                pkt.seq.idx = 0;

            } else if (!++pkt.seq.idx) pkt.seq.idx = 1;
        }

        pkt.size = remaining;

        pkt_size = MIN(remaining, MAC_PKT_SIZE_MAX);

        if (pkt.seq.idx && pkt_size == remaining) pkt.seq.end = 1;

        if (pkt_size && data) memcpy(pkt.data, &data[size - remaining], pkt_size);

        /*mac_trace_verbose(
                "mac/%u: TX %04X->%04X #%04u %03u-%02u-%02u",
                rdio_id(phy_rdio(mac->phy)), pkt.addr, pkt.dest,
                pkt.size, pkt.seq.num, pkt.seq.msg, pkt.seq.idx
        );*/

        if (!mac_send_packet(mac, flag, &pkt)) return false;

        remaining -= pkt_size;

    } while (remaining);

    return true;
}


static bool mac_send_packet(mac_t mac, u8 flag, mac_static_pkt_t *pkt)
{
    assert((!pkt->seq.msg && pkt->size <= MAC_PKT_SIZE_MAX) || pkt->seq.msg);

    const u8 pkt_len = sizeof(mac_pkt_t) + MIN(pkt->size, MAC_PKT_SIZE_MAX);
    const bool imm = flag & (u8)MAC_FLAG_PKT_IMM;
    const bool needs_ack = flag & (u8)MAC_FLAG_ACK_REQ;
    u8 phy_flag = (u8)(flag & PHY_PKT_FLAG_USER_MASK) | (u8)(imm ? PHY_PKT_FLAG_IMMEDIATE : 0) | (u8)(!imm || needs_ack ? PHY_PKT_FLAG_BLOCK : 0);

    u8 retry = MAC_PEND_RETRY;

    if (!(flag & MAC_FLAG_PKT_BLK)) {
        mac_send_queue_t sq = {
                .flag = flag
        };

        memcpy(&sq.pkt, pkt, pkt_len);

        if (!xQueueSend(mac->txq, &sq, pdMS_TO_TICKS(100))) {
            mac_trace_debug("mac/tx: queue failed");
            return false;
        }

        // packet was queued
        return true;
    }

    if (needs_ack) {
        // TODO: This needs a lock
        mac->pend.pkt = (mac_pkt_t *)pkt;
        mac->pend.task = xTaskGetCurrentTaskHandle();
        mac->pend.flag = flag;
    }

    ///sclk_t sent;

    _retry_tx:

    ///sent = sclk_time();

    if (!phy_send(mac->phy, phy_flag, (u8 *)pkt, pkt_len)) {

        if (--retry) {
            mac_trace_warn("retry 0x%02X/%u [tx fail]", pkt->seq.num, pkt->size);
            goto _retry_tx;

        } else {
            mac_trace_warn("drop 0x%02X/%u [tx fail]", pkt->seq.num, pkt->size);

            // TODO: Better cleanup/return flow

            if (needs_ack) {
                mac->pend = (mac_pend_t){ NULL, NULL, 0 };
            }

            return false;
        }
    }

    if (needs_ack) {
        const u32 tx_time = MAC_PEND_TIME + phy_delay(1 + (u8)MAC_PKT_OVERHEAD) / 1000;
        //sclk_t elaps = sclk_time();
        u32 notify = 0;

        if (!xTaskNotifyWait(MAC_NOTIFY_ACK, MAC_NOTIFY_ACK, &notify, pdMS_TO_TICKS(tx_time)) || !(notify & MAC_NOTIFY_ACK)) {
            //elaps = sclk_time() - elaps;

            if (mac->pend.flag & MAC_FLAG_ACK_RSP) {
                mac_trace_debug("race condition: packet acked successfully");

            } else {
                if (--retry) {

                    if (!(phy_flag & MAC_FLAG_ACK_RQR)) {
                        phy_flag |= MAC_FLAG_ACK_RQR;
                    }

                    mac_trace_debug(
                            "retry: seq=%03u len=%03u",// elaps=%lu",
                            pkt->seq.num, pkt_len//, elaps
                    );

                    goto _retry_tx;

                } else {
                    mac_trace_warn("drop 0x%02X/%u", pkt->seq.num, pkt->size);
                    mac->pend = (mac_pend_t){ NULL, NULL, 0 };
                    return false;
                }
            }
        }

        mac->pend = (mac_pend_t){ NULL, NULL, 0 };
    }

    return true;
}


static void mac_task_send(mac_t mac)
{
    const QueueHandle_t txq = mac->txq;
    mac_send_queue_t sq;

    while (1) {
        while (!xQueueReceive(txq, &sq, portMAX_DELAY));
        sq.flag |= MAC_FLAG_PKT_BLK;
        mac_send_packet(mac, sq.flag, &sq.pkt);
    }
}


static void mac_task_recv(mac_t mac)
{
    const QueueHandle_t rxq = mac->rxq;
    mac_recv_data_t recv;

    while (1) {
        while (!xQueueReceive(rxq, &recv, portMAX_DELAY));

        /*mac_trace_verbose(
                "mac/%u: RX %04X->%04X #%04u %03u-%02u-%02u",
                rdio_id(phy_rdio(mac->phy)), recv.peer->addr, recv.dest,
                recv.size, recv.seq.num, recv.seq.msg, recv.seq.idx
        );*/

        if (recv.seq.msg) {

            if (!recv.peer->part) {
                if (recv.seq.idx != 0) {
                    mac_trace_warn("(rx) multi-packet stream sync error");
                    continue;
                }

                recv.peer->part = malloc(sizeof(mac_part_t) + recv.size); assert(recv.peer->part);
                recv.peer->part->dest = recv.dest;
                recv.peer->part->size = recv.size;
                recv.peer->part->seq = recv.seq;
                memcpy(recv.peer->part->data, recv.data, MIN(recv.size, MAC_PKT_SIZE_MAX));
            } else if (recv.size >= recv.peer->part->size || recv.seq.msg != recv.peer->part->seq.msg || recv.dest != recv.peer->part->dest) {
                mac_trace_warn("(rx) multi-packet stream error");
                free(recv.peer->part);
                recv.peer->part = NULL;
                continue;
            } else {
                if (!++recv.peer->part->seq.idx) recv.peer->part->seq.idx = 1;

                if (recv.seq.idx != recv.peer->part->seq.idx) {
                    mac_trace_warn("(rx) multi-packet stream lost");
                    free(recv.peer->part);
                    recv.peer->part = NULL;
                    continue;
                }

                memcpy(&recv.peer->part->data[recv.peer->part->size - recv.size], recv.data, MIN(recv.size, MAC_PKT_SIZE_MAX));
            }

            if (recv.seq.end) {
                mac->recv(mac, recv.peer->addr, recv.peer->part->dest, recv.peer->part->size, recv.peer->part->data, recv.meta);
                free(recv.peer->part);
                recv.peer->part = NULL;
            }

            continue;
        }

        mac->recv(mac, recv.peer->addr, recv.dest, recv.size, recv.data, recv.meta);
    }
}


static bool mac_phy_recv(mac_t mac, u8 flag, u8 size, u8 data[], pkt_meta_t meta)
{
    mac_pkt_t *pkt = (mac_pkt_t *)data;
    bool unblock = true;

    if (size != sizeof(mac_pkt_t) + MIN(pkt->size, MAC_PKT_SIZE_MAX)) {
        mac_trace_debug("(rx) bad length: len=%u != size=%u + base=%u", size, MIN(pkt->size, MAC_PKT_SIZE_MAX), sizeof(mac_pkt_t));

    } else {
        if (pkt->dest == 0 || pkt->dest == mac->addr) {
            mac_peer_t *const peer = mac_peer_get(mac, pkt->addr);

            if (peer) {

                if (pkt->dest == mac->addr && (flag & MAC_FLAG_ACK_REQ)) {
                    if (flag & MAC_FLAG_ACK_RQR) {
                        mac_trace_debug(
                                "rqr-ack-tx: seq=%03u pseq=%03u len=%03u",
                                pkt->seq.num, peer->seq.num, pkt->size + MAC_PKT_OVERHEAD
                        );
                    }

                    u8 seqn = pkt->seq.num;

                    mac_send_base(
                            mac, pkt->addr, MAC_FLAG_PKT_BLK | MAC_FLAG_PKT_IMM | MAC_FLAG_ACK_RSP,
                            sizeof(seqn), &seqn
                    );

                    unblock = false;
                }

                if (pkt->dest == mac->addr && (flag & MAC_FLAG_ACK_RSP)) {
                    const u8 seqn = *(u8 *)pkt->data;
                    mac_pkt_t *const pend = mac->pend.pkt;

                    if (pend && seqn == pend->seq.num && (!pend->dest || pkt->addr == pend->dest)) {
                        if (!(mac->pend.flag & MAC_FLAG_ACK_RSP)) {

                            mac->pend.flag = flag;

                            if (mac->pend.task)
                                xTaskNotify(mac->pend.task, MAC_NOTIFY_ACK, eSetBits);

                        } else {
                            mac_trace_verbose("ack-dup\n");
                        }
                    }

                } else {
                    // Assumption: packets always arrive in non-monotonic increasing sequence order, retransmits included
                    //  This will not be the case if there is ever more than one TX queue

                    if (!(flag & MAC_FLAG_ACK_RQR) || (peer->seq.num != pkt->seq.num)) {
                        peer->seq.num = pkt->seq.num;

                        if (unblock && ((flag & PHY_PKT_FLAG_IMMEDIATE) || (pkt->seq.msg && !pkt->seq.end)))
                            unblock = false;

                        mac_recv_data_t recv = {
                                .meta = meta,
                                .peer = peer,
                                .dest = pkt->dest,
                                .size = pkt->size,
                                .seq  = pkt->seq,
                        };

                        if (recv.size) memcpy(recv.data, pkt->data, MIN(recv.size, MAC_PKT_SIZE_MAX));

                        if (!xQueueSend(mac->rxq, &recv, 0)) {
                            mac_trace_debug("(rx) queue failed");
                        }
                    }

                }

            }
        }
    }

    return unblock;
}
