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

#define MAC_TX_TASK_STACK_SIZE  (TASK_STACK_SIZE_MEDIUM / sizeof(StackType_t))
#define MAC_RX_TASK_STACK_SIZE  (TASK_STACK_SIZE_MEDIUM / sizeof(StackType_t))

#define MAC_NOTIFY_ACK          (1u<<10)


typedef enum __packed {
    MAC_FLAG_PKT_IMM = 1 << 0,
    MAC_FLAG_PKT_BLK = 1 << 1,
    MAC_FLAG_ACK_REQ = 1 << 4,
    MAC_FLAG_ACK_RSP = 1 << 5,
    MAC_FLAG_ACK_RQR = 1 << 6,

} mac_flag_t;

typedef struct __packed {
    mac_addr_t addr;
    mac_addr_t dest;
    u8  seqn;
    u8  flag;
    u8  size;
    u8  data[];

} mac_pkt_t;

typedef struct __packed {
    mac_addr_t addr;
    mac_addr_t dest;
    u8  seqn;
    u8  flag;
    u8  size;
    u8  data[MAC_PKT_SIZE_MAX];

} mac_static_pkt_t;

typedef struct __packed {
    pkt_meta_t meta;
    mac_addr_t peer;
    mac_addr_t dest;
    mac_size_t size;
    u8 data[MAC_PKT_SIZE_MAX];

} mac_recv_data_t;

typedef struct __packed {
    mac_addr_t addr;
    u8 seqn;

} mac_peer_t;

typedef struct __packed {
    mac_pkt_t *pkt;
    TaskHandle_t task;

} mac_pend_t;

struct __packed mac {
    phy_t phy;
    mac_addr_t addr;
    mac_recv_t recv;

    u8 seqn;
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
    mac_static_pkt_t txq_buf[MAC_TXQ_SIZE];
    
    QueueHandle_t rxq;
    StaticQueue_t rxq_static;
    mac_recv_data_t rxq_buf[MAC_RXQ_SIZE];
};


static mac_peer_t *mac_peer_get(mac_t mac, mac_addr_t addr);

static bool mac_send_base(mac_t mac, mac_addr_t dest, u8 flag, mac_size_t size, u8 data[]);
static bool mac_send_packet(mac_t mac, mac_static_pkt_t *pkt);

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
    mac->seqn = 0;
    mac->pend = (mac_pend_t){ NULL, NULL };

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
        pp->seqn = 0;
        return pp;
    }

    return NULL;
}


bool mac_send(mac_t mac, mac_send_t type, mac_addr_t dest, mac_size_t size, u8 data[])
{
    switch (type) {
        case MAC_SEND_DGRM:
            return mac_send_base(mac, dest, MAC_FLAG_PKT_BLK/* NOTE: Experimental, plays better with UART relay. */, size, data);

        case MAC_SEND_MESG:
            return mac_send_base(mac, dest, MAC_FLAG_ACK_REQ, size, data);

        case MAC_SEND_TRXN:
            return mac_send_base(mac, dest, MAC_FLAG_ACK_REQ | MAC_FLAG_PKT_BLK, size, data);

        case MAC_SEND_STRM:
            return mac_send_base(mac, dest, MAC_FLAG_PKT_IMM, size, data);

        default:
            return false;
    }
}


static bool mac_send_base(mac_t mac, mac_addr_t dest, u8 flag, mac_size_t size, u8 data[])
{
    if (size > MAC_PKT_SIZE_MAX) {
        mac_trace_debug("(tx) packet too big, truncating");
        size = MAC_PKT_SIZE_MAX;
    }

    mac->seqn = (u8)((mac->seqn + 1) % UINT8_MAX);
    if (!mac->seqn) mac->seqn = 1;

    mac_static_pkt_t pkt = {
            .addr = mac->addr,
            .dest = dest,
            .seqn = mac->seqn,
            .flag = flag,
            .size = (u8)size
    };

    if (size && data) memcpy(pkt.data, data, size);

    return mac_send_packet(mac, &pkt);
}


static bool mac_send_packet(mac_t mac, mac_static_pkt_t *pkt)
{
    assert(pkt->size <= MAC_PKT_SIZE_MAX);

    const u8 pkt_len = sizeof(mac_pkt_t) + pkt->size;
    const bool imm = pkt->flag & MAC_FLAG_PKT_IMM;
    const bool needs_ack = pkt->flag & MAC_FLAG_ACK_REQ;
    const u8 phy_flag = (u8)(imm ? PHY_PKT_FLAG_IMMEDIATE : 0) | (u8)(!imm || needs_ack ? PHY_PKT_FLAG_BLOCK : 0);

    u8 retry = MAC_PEND_RETRY;

    if (!(pkt->flag & MAC_FLAG_PKT_BLK)) {
        if (!xQueueSend(mac->txq, pkt, pdMS_TO_TICKS(100)/*portMAX_DELAY*/)) {
            mac_trace_debug("mac/tx: queue failed");
            return false;
        }

        // packet was queued
        return true;
    }

    if (needs_ack) {
        mac->pend.pkt = (mac_pkt_t *)pkt;
        mac->pend.task = xTaskGetCurrentTaskHandle();
    }

    ///sclk_t sent;

    _retry_tx:

    ///sent = sclk_time();

    if (!phy_send(mac->phy, phy_flag, (u8 *)pkt, pkt_len)) {

        if (--retry) {
            //mac_trace_warn("retry 0x%02X/%u [tx fail]", pkt->seqn, pkt->size);
            goto _retry_tx;

        } else {
            mac_trace_warn("drop 0x%02X/%u [tx fail]", pkt->seqn, pkt->size);

            // TODO: Better cleanup/return flow

            if (needs_ack) {
                mac->pend = (mac_pend_t){ NULL, NULL };
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

            if (((volatile u8)pkt->flag & MAC_FLAG_ACK_RSP)) {
                mac_trace_debug("race condition: packet acked successfully");

            } else {
                if (--retry) {

                    if (!(pkt->flag & MAC_FLAG_ACK_RQR)) {
                        pkt->flag |= MAC_FLAG_ACK_RQR;
                        //pkt->flag |= MAC_FLAG_PKT_IMM; // retries are urgent
                    }

                    mac_trace_debug(
                            "retry: seq=%03u len=%03u",// elaps=%lu",
                            pkt->seqn, pkt_len//, elaps
                    );

                    goto _retry_tx;
                } else {
                    mac_trace_warn("drop 0x%02X/%u", pkt->seqn, pkt->size);
                }
            }
        }

        mac->pend = (mac_pend_t){ NULL, NULL };
    }

    return true;
}


static void mac_task_send(mac_t mac)
{
    const QueueHandle_t txq = mac->txq;
    mac_static_pkt_t pkt;

    while (1) {
        if (xQueueReceive(txq, &pkt, portMAX_DELAY)) {
            assert(pkt.size <= MAC_PKT_SIZE_MAX && pkt.addr == mac->addr);
            pkt.flag |= MAC_FLAG_PKT_BLK;
            mac_send_packet(mac, &pkt);
        }
    }
}


static void mac_task_recv(mac_t mac)
{
    const QueueHandle_t rxq = mac->rxq;
    mac_recv_data_t recv;

    while (1) {
        if (xQueueReceive(rxq, &recv, portMAX_DELAY)) {
            mac->recv(mac, recv.peer, recv.dest, recv.size, recv.data, recv.meta);
        }
    }
}


static bool mac_phy_recv(mac_t mac, u8 flag, u8 size, u8 data[], pkt_meta_t meta)
{
    mac_pkt_t *const pkt = (mac_pkt_t *)data;
    bool unblock = false;

    if (size != sizeof(mac_pkt_t) + pkt->size) {
        mac_trace_debug("(rx) bad length: len=%u != size=%u + base=%u", size, pkt->size, sizeof(mac_pkt_t));

    } else {

        if (pkt->dest == 0 || pkt->dest == mac->addr) {

            if (pkt->dest == mac->addr) {
                unblock = true;
            }

            mac_peer_t *const peer = mac_peer_get(mac, pkt->addr);

            if (peer) {

                if (pkt->dest == mac->addr && (pkt->flag & MAC_FLAG_ACK_REQ)) {
                    if (pkt->flag & MAC_FLAG_ACK_RQR) {
                        mac_trace_debug(
                                "rqr-ack-tx: seq=%03u pseq=%03u len=%03u",
                                pkt->seqn, peer->seqn, pkt->size + MAC_PKT_OVERHEAD
                        );
                    }

                    mac_send_base(
                            mac, pkt->addr, MAC_FLAG_PKT_BLK | MAC_FLAG_PKT_IMM | MAC_FLAG_ACK_RSP,
                            sizeof(pkt->seqn), &pkt->seqn
                    );
                }

                if (pkt->dest == mac->addr && (pkt->flag & MAC_FLAG_ACK_RSP)) {
                    const u8 seqn = *(u8 *)pkt->data;
                    mac_pkt_t *const pend = mac->pend.pkt;

                    if (pend && seqn == pend->seqn && (!pend->dest || pkt->addr == pend->dest)) {
                        if (!(pend->flag & MAC_FLAG_ACK_RSP)) {

                            pend->flag |= MAC_FLAG_ACK_RSP;

                            if (mac->pend.task)
                                xTaskNotify(mac->pend.task, MAC_NOTIFY_ACK, eSetBits);

                        } else {
                            mac_trace_verbose("ack-dup\n");
                        }
                    }

                } else {
                    // Assumption: packets always arrive in non-monotonic increasing sequence order, retransmits included
                    //  This will not be the case if there is ever more than one TX queue
                    if (!(pkt->flag & MAC_FLAG_ACK_RQR) || (peer->seqn != pkt->seqn)) {
                        peer->seqn = pkt->seqn;

                        mac_recv_data_t recv = {
                                .meta = meta,
                                .peer = pkt->addr,
                                .dest = pkt->dest,
                                .size = pkt->size,
                        };

                        if (recv.size) memcpy(recv.data, pkt->data, recv.size);

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
