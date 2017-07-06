#include <cc/nmac.h>
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
#include <kio/itm.h>
#include <kio/sclk.h>
#include <cc/chan.h>


#define NMAC_PEER_MAX       10

#define NMAC_PEND_TIME      7
#define NMAC_PEND_RETRY     3

#define NMAC_TXQ_COUNT      3 // This should eventually reflect the number of threads waiting on messages
#define NMAC_TXQ_SIZE       (NMAC_TXQ_COUNT)
#define NMAC_RXQ_SIZE       7

#define MAC_TX_TASK_STACK_SIZE  (TASK_STACK_SIZE_MEDIUM / sizeof(StackType_t))
#define MAC_RX_TASK_STACK_SIZE  (TASK_STACK_SIZE_MEDIUM / sizeof(StackType_t))

#define MAC_NOTIFY_ACK      (1u<<10)

#define nmac_warn(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ )
#define nmac_debug(format, ...) itm_printf(0, format "\r\n", ##__VA_ARGS__ )
#define nmac_debug_pkt(format, ...) /*itm_printf(0, "<itm> " format "\r\n", ##__VA_ARGS__ )*/ /*cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )*/
#define nmac_debug_v(format, ...)

#undef assert
#define assert(x) if (!(x)) { itm_printf(0, "ASSERT FAIL: ( " #x " ) on line %u of %s\r\n", __LINE__, __FILE__); asm("bkpt #0"); while (1) { asm("nop"); } }


typedef enum __packed {
    MAC_FLAG_PKT_IMM = 1 << 0,
    MAC_FLAG_PKT_BLK = 1 << 1,
    MAC_FLAG_ACK_REQ = 1 << 4,
    MAC_FLAG_ACK_RSP = 1 << 5,
    MAC_FLAG_ACK_RQR = 1 << 6,

} mac_flag_t;

typedef struct __packed {
    u16 addr;
    u16 dest;
    u8  seqn;
    u8  flag;
    u8  size;
    u8  data[];

} nmac_pkt_t;

typedef struct __packed {
    u16 addr;
    u16 dest;
    u8  seqn;
    u8  flag;
    u8  size;
    u8  data[MAC_PKT_SIZE_MAX];

} nmac_static_pkt_t;

typedef struct __packed {
    s8 rssi;
    u8 lqi;
    u16 node;
    u16 peer;
    u16 dest;
    u16 size;
    u8 data[MAC_PKT_SIZE_MAX];

} nmac_recv_t;

typedef struct __packed {
    u16 addr;
    u8 seqn;

} nmac_peer_t;

typedef struct __packed {
    nmac_pkt_t *pkt;
    TaskHandle_t task;

} nmac_pend_t;


static bool send(u16 dest, u8 flag, u16 size, u8 data[]);
static void tx_task(void *param);
static void rx_task(void *param);
static bool handle_rx(u8 flag, u8 size, u8 data[], s8 rssi, u8 lqi);


static struct {
    mac_recv_t rx;

    TaskHandle_t task;
    StaticTask_t task_static;
    StackType_t task_stack[MAC_TX_TASK_STACK_SIZE];

    TaskHandle_t rx_task;
    StaticTask_t rx_task_static;
    StackType_t rx_task_stack[MAC_RX_TASK_STACK_SIZE];

    QueueHandle_t txq;
    StaticQueue_t txq_static;
    nmac_static_pkt_t txq_buf[NMAC_TXQ_SIZE];

    QueueHandle_t rxq;
    StaticQueue_t rxq_static;
    nmac_recv_t rxq_buf[NMAC_RXQ_SIZE];

    nmac_pend_t pend;
    nmac_peer_t peer[NMAC_PEER_MAX];

} nmac;

static u16 mac_addr = 0;
static u8 mac_seqn = 0;


static nmac_peer_t *peer_get_or_add(u16 addr)
{
    nmac_peer_t *pp = NULL;

    if (!addr) return NULL;

    for (u8 i = 0; i < NMAC_PEER_MAX; ++i) {
        if (nmac.peer[i].addr == addr) {
            return &nmac.peer[i];
        } else if (!pp && !nmac.peer[i].addr) { // TODO: Change this logic to support peer removal/expiry
            pp = &nmac.peer[i];
        }
    }

    if (pp) {
        pp->addr = addr;
        pp->seqn = 0;
        return pp;
    }

    return NULL;
}

bool nmac_init(u8 cell, u16 addr, bool sync_master, mac_recv_t rx)
{
    mac_addr = addr;
    nmac.rx = rx;

    nmac.txq = xQueueCreateStatic(NMAC_TXQ_SIZE, sizeof(nmac.txq_buf[0]), (u8 *)nmac.txq_buf, &nmac.txq_static);
    nmac.rxq = xQueueCreateStatic(NMAC_RXQ_SIZE, sizeof(nmac.rxq_buf[0]), (u8 *)nmac.rxq_buf, &nmac.rxq_static);
    nmac.pend = (nmac_pend_t){ NULL, NULL };

    nmac.task = xTaskCreateStatic(tx_task, "nmac:send", MAC_TX_TASK_STACK_SIZE, NULL, TASK_PRIO_HIGHEST - 1, nmac.task_stack, &nmac.task_static);
    nmac.rx_task = xTaskCreateStatic(rx_task, "nmac:recv", MAC_RX_TASK_STACK_SIZE, NULL, TASK_PRIO_HIGH, nmac.rx_task_stack, &nmac.rx_task_static);

    return nphy_init(cell, sync_master, handle_rx);
}

u16 nmac_get_addr(void)
{
    return mac_addr;
}

bool nmac_send(nmac_send_t type, u16 dest, u16 size, u8 data[])
{
    switch (type) {
        case NMAC_SEND_DGRM:
            return send(dest, 0, size, data);

        case NMAC_SEND_MESG:
            return send(dest, MAC_FLAG_ACK_REQ, size, data);

        case NMAC_SEND_TRXN:
            return send(dest, MAC_FLAG_ACK_REQ, size, data);

        case NMAC_SEND_STRM:
            return send(dest, MAC_FLAG_PKT_IMM, size, data);

        default:
            return false;
    }
}

static bool send_packet(nmac_static_pkt_t *pkt)
{
    assert(pkt->size <= MAC_PKT_SIZE_MAX);

    const u8 pkt_len = sizeof(nmac_pkt_t) + pkt->size;
    const bool imm = pkt->flag & MAC_FLAG_PKT_IMM;
    const bool needs_ack = pkt->flag & MAC_FLAG_ACK_REQ;
    const u8 nphy_flag = (u8)((imm ? PHY_PKT_FLAG_IMMEDIATE : 0) | PHY_PKT_FLAG_BLOCK);

    u8 retry = NMAC_PEND_RETRY;

    if (!(pkt->flag & MAC_FLAG_PKT_BLK)) {
        if (!xQueueSend(nmac.txq, pkt, portMAX_DELAY)) {
            nmac_debug("mac/tx: queue failed");
            return false;
        }

        // packet was queued
        return true;
    }

    if (needs_ack) {
        nmac.pend.pkt = (nmac_pkt_t *)pkt;
        nmac.pend.task = xTaskGetCurrentTaskHandle();
    }

    ///sclk_t sent;

_retry_tx:

    ///sent = sclk_time();

    if (!nphy_tx(nphy_flag, (u8 *)pkt, pkt_len)) {

        if (--retry) {
            nmac_warn("retry 0x%02X/%u [tx fail]", pkt->seqn, pkt->size);
            goto _retry_tx;

        } else {
            nmac_warn("drop 0x%02X/%u [tx fail]", pkt->seqn, pkt->size);

            // TODO: Better cleanup/return flow

            if (needs_ack) {
                nmac.pend = (nmac_pend_t){ NULL, NULL };
            }

            return false;
        }

    } else {
        nmac_debug_pkt(
                "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u time=%05lu \t %s",
                pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size, SCLK_MSEC(sent),
                pkt->flag & MAC_FLAG_ACK_REQ ? "TX-ACK-REQ" : (pkt->flag & MAC_FLAG_ACK_RSP ? "TX-ACK-RSP" : "TX")
        );
    }

    if (needs_ack) {
        const u32 tx_time = NMAC_PEND_TIME + nphy_delay(pkt->size + (u8)MAC_PKT_OVERHEAD) / 1000;
        //sclk_t elaps = sclk_time();

        if (!xTaskNotifyWait(MAC_NOTIFY_ACK, MAC_NOTIFY_ACK, NULL, pdMS_TO_TICKS(tx_time))) {
            //elaps = sclk_time() - elaps;

            if (((volatile u8)pkt->flag & MAC_FLAG_ACK_RSP)) {
                nmac_debug("race condition: packet acked successfully");

            } else {
                if (--retry) {

                    if (!(pkt->flag & MAC_FLAG_ACK_RQR)) {
                        pkt->flag |= MAC_FLAG_ACK_RQR;
                        //pkt->flag |= MAC_FLAG_PKT_IMM; // retries are urgent
                    }

                    nmac_debug(
                            "retry: seq=%03u len=%03u",// elaps=%lu",
                            pkt->seqn, pkt_len//, elaps
                    );

                    goto _retry_tx;
                } else {
                    nmac_warn("drop 0x%02X/%u", pkt->seqn, pkt->size);
                }
            }
        }

        nmac.pend = (nmac_pend_t){ NULL, NULL };
    }

    return true;
}

static bool send(u16 dest, u8 flag, u16 size, u8 data[])
{
    if (size > MAC_PKT_SIZE_MAX) {
        nmac_debug("(tx) packet too big, truncating");
        size = MAC_PKT_SIZE_MAX;
    }

    mac_seqn = (u8)((mac_seqn + 1) % UINT8_MAX);
    if (!mac_seqn) mac_seqn = 1;

    nmac_static_pkt_t pkt = {
            .addr = mac_addr,
            .dest = dest,
            .seqn = mac_seqn,
            .flag = flag,
            .size = (u8)size
    };

    if (size && data) memcpy(pkt.data, data, size);

    return send_packet(&pkt);
}

static void tx_task(void *param __unused)
{
    const QueueHandle_t txq = nmac.txq;
    nmac_static_pkt_t pkt;

    while (1) {
        if (xQueueReceive(txq, &pkt, portMAX_DELAY)) {
            assert(pkt.size <= MAC_PKT_SIZE_MAX && pkt.addr == mac_addr);
            pkt.flag |= MAC_FLAG_PKT_BLK;
            send_packet(&pkt);
        }
    }
}

static void rx_task(void *param __unused)
{
    const QueueHandle_t rxq = nmac.rxq;
    nmac_recv_t recv;

    while (1) {
        if (xQueueReceive(rxq, &recv, portMAX_DELAY)) {
            nmac.rx(recv.node, recv.peer, recv.dest, recv.size, recv.data, recv.rssi, recv.lqi);
        }
    }
}

static bool handle_rx(u8 flag, u8 size, u8 data[], s8 rssi, u8 lqi)
{
    nmac_pkt_t *const pkt = (nmac_pkt_t *)data;
    bool unblock = false;

    if (size != sizeof(nmac_pkt_t) + pkt->size) {
        nmac_debug("(rx) bad length: len=%u != size=%u + base=%u", size, pkt->size, sizeof(nmac_pkt_t));

    } else {
        nmac_debug_pkt(
                "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u time=%05lu \t %s",
                pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size, SCLK_MSEC(sclk_time()),
                pkt->flag & MAC_FLAG_ACK_REQ ? "RX-ACK-REQ" : (pkt->flag & MAC_FLAG_ACK_RSP ? "RX-ACK-RSP" : "RX")
        );

        if (pkt->dest != 0 && pkt->dest != mac_addr) {
            nmac_debug_v("(rx) drop: addr mismatch");
            
        } else {
            if (pkt->dest == mac_addr) {
                unblock = true;
            }

            nmac_peer_t *const peer = peer_get_or_add(pkt->addr);

            if (peer) {
                if (pkt->flag & MAC_FLAG_ACK_REQ) {
                    if (pkt->flag & MAC_FLAG_ACK_RQR) {
                        nmac_debug(
                                "rqr-ack-tx: seq=%03u pseq=%03u len=%03u",
                                pkt->seqn, peer->seqn, pkt->size + MAC_PKT_OVERHEAD
                        );
                    }

                    send(pkt->addr, MAC_FLAG_PKT_BLK | MAC_FLAG_PKT_IMM | MAC_FLAG_ACK_RSP, sizeof(pkt->seqn), &pkt->seqn);
                }

                if (pkt->flag & MAC_FLAG_ACK_RSP) {
                    const u8 seqn = *(u8 *)pkt->data;
                    nmac_pkt_t *const pend = nmac.pend.pkt;

                    if (pend && seqn == pend->seqn && (!pend->dest || pkt->addr == pend->dest)) {
                        if (!(pend->flag & MAC_FLAG_ACK_RSP)) {

                            pend->flag |= MAC_FLAG_ACK_RSP;

                            if (nmac.pend.task)
                                xTaskNotify(nmac.pend.task, MAC_NOTIFY_ACK, eSetBits);

                        } else {
                            itm_puts(0, "ack-dup\n");
                        }
                    }

                } else {
                    // Assumption: packets always arrive in non-monotonic increasing sequence order, retransmits included
                    //  This will not be the case if there is ever more than one TX queue
                    if (!(pkt->flag & MAC_FLAG_ACK_RQR) || (peer->seqn != pkt->seqn)) {
                        peer->seqn = pkt->seqn;

                        nmac_recv_t recv = {
                                .rssi = rssi, .lqi = lqi,
                                .node = mac_addr,
                                .peer = pkt->addr,
                                .dest = pkt->dest,
                                .size = pkt->size,
                        };

                        if (recv.size) memcpy(recv.data, pkt->data, recv.size);

                        if (!xQueueSend(nmac.rxq, &recv, 0)) {
                            nmac_debug("(rx) queue failed");
                        }
                    }

                }

            }
        }
    }

    return unblock;
}
