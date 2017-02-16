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
#include <itm.h>
#include <sclk.h>

#define NMAC_PEER_MAX       10

#define NMAC_PEND_MAX       1
#define NMAC_PEND_TIME      57
#define NMAC_PEND_RETRY     3

#define NMAC_TXQ_COUNT      1 // NMAC_PEND_MAX? When this is implemented it should be user-facing
#define NMAC_TXQ_SIZE       (NMAC_PEND_MAX * NMAC_TXQ_COUNT)


#define nmac_debug(format, ...) cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )
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
    nmac_pkt_t *pkt;
    xSemaphoreHandle mtx;
    StaticSemaphore_t mtx_static;
    xSemaphoreHandle sem;
    StaticSemaphore_t sem_static;

} nmac_pend_t;


static bool send(u16 dest, u8 flag, u16 size, u8 data[]);
static void tx_task(void *param);
static void handle_rx(u8 flag, u8 size, u8 data[], s8 rssi, u8 lqi);

typedef struct __packed {
    u16 addr;
    u8 seqn;

} nmac_peer_t;

#define MAC_TASK_STACK_SIZE  (TASK_STACK_SIZE_MEDIUM / sizeof(StackType_t))

static struct {
    mac_recv_t rx;

    xTaskHandle task;
    StaticTask_t task_static;
    StackType_t task_stack[MAC_TASK_STACK_SIZE];

    xQueueHandle txq;
    StaticQueue_t txq_static;
    nmac_static_pkt_t txq_buf[NMAC_TXQ_SIZE];

    xQueueHandle pendq;
    StaticQueue_t pendq_static;
    u8 pendq_buf[NMAC_PEND_MAX];

    nmac_pend_t pend[NMAC_PEND_MAX];
    nmac_peer_t peer[NMAC_PEER_MAX];

} nmac = {NULL};

static u16 mac_addr = 0;
static u8 mac_seqn = 0;

#define nmac_malloc     malloc
#define nmac_free       free

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
    nmac.pendq = xQueueCreateStatic(NMAC_PEND_MAX, sizeof(u8), nmac.pendq_buf, &nmac.pendq_static);

    for (u8 i = 0; i < NMAC_PEND_MAX; ++i) {
        nmac.pend[i].pkt = NULL;
        nmac.pend[i].mtx = xSemaphoreCreateBinaryStatic(&nmac.pend[i].mtx_static);
        nmac.pend[i].sem = xSemaphoreCreateBinaryStatic(&nmac.pend[i].sem_static);

        xSemaphoreGive(nmac.pend[i].mtx);

        if (!xQueueSend(nmac.pendq, &i, 0)) {
            nmac_debug("error: unable queue pend during setup");
            return false;
        }
    }

    //nmac.task = xTaskCreateStatic(tx_task, "nmac:send", MAC_TASK_STACK_SIZE, NULL, TASK_PRIO_HIGH - 1, nmac.task_stack, &nmac.task_static);

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
            return send(dest, MAC_FLAG_PKT_BLK, size, data);

        case NMAC_SEND_MESG:
            return send(dest, MAC_FLAG_PKT_BLK | MAC_FLAG_ACK_REQ, size, data);

        case NMAC_SEND_TRXN:
            return send(dest, MAC_FLAG_PKT_BLK | MAC_FLAG_ACK_REQ, size, data);

        case NMAC_SEND_STRM:
            return send(dest, MAC_FLAG_PKT_BLK | MAC_FLAG_PKT_IMM, size, data);

        default:
            return false;
    }
}

static bool send_packet(nmac_static_pkt_t *pkt)
{
    assert(pkt->size <= MAC_PKT_SIZE_MAX);

    const u8 pkt_len = sizeof(nmac_pkt_t) + pkt->size;
    const bool needs_ack = pkt->flag & MAC_FLAG_ACK_REQ;
    nmac_pend_t *pend = NULL;
    u8 pend_idx;
    u8 nphy_flag = 0;

    //nmac_debug_pkt(
    //        "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u time=%05lu \t TX-BEGIN",
    //        pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size, SCLK_MSEC(sclk_time())
    //);

    if (pkt->flag & MAC_FLAG_PKT_IMM) {
        nphy_flag |= PHY_PKT_FLAG_IMMEDIATE;
    }

    if (needs_ack) {
        if (!(pkt->flag & MAC_FLAG_PKT_BLK)) {
            if (pkt->flag & MAC_FLAG_ACK_RSP) {
                // Currently this does not happen
                if (!xQueueSendToFront(nmac.txq, pkt, pdMS_TO_TICKS(1000))) {
                    nmac_debug("mac/tx: queue immediate failed");
                    //nmac_free(pkt);
                    return false;
                }
            } else {
                if (!xQueueSend(nmac.txq, pkt, portMAX_DELAY)) {
                    nmac_debug("mac/tx: queue failed");
                    //nmac_free(pkt);
                    return false;
                }
            }

            // packet was queued
            return true;
        }

        if (!xQueueReceive(nmac.pendq, &pend_idx, pdMS_TO_TICKS(1000))) {
            nmac_debug("mac/tx: acquire pend failed");
            //nmac_free(pkt);
            return false;
        }

        pend = &nmac.pend[pend_idx];
        pend->pkt = (nmac_pkt_t *)pkt;

        if (!xSemaphoreTake(pend->mtx, pdMS_TO_TICKS(1000))) {
            nmac_debug("mac/tx: acquire pend mtx failed");
            //nmac_free(pkt);
            return false;
        }
    }

    //const sclk_t start = sclk_time();
    u8 retry = NMAC_PEND_RETRY;

    _retry_tx:
    nphy_tx(nphy_flag, (u8 *)pkt, pkt_len);
    nmac_debug_pkt(
            "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u time=%05lu \t %s",
            pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size, SCLK_MSEC(sclk_time()),
            pkt->flag & MAC_FLAG_ACK_REQ ? "TX-ACK-REQ" : (pkt->flag & MAC_FLAG_ACK_RSP ? "TX-ACK-RSP" : "TX")
    );


    if (needs_ack) {
        xSemaphoreGive(pend->mtx);

        /**
         * TODO: Dynamic timeout, current value is ridiculously long and wastes time.
         *       Should probably be something more like pkt_time*6 + <channel-block>.
         */

        if (!xSemaphoreTake(pend->sem, pdMS_TO_TICKS(NMAC_PEND_TIME))) {
            if (!xSemaphoreTake(pend->mtx, pdMS_TO_TICKS(1000))) {
                nmac_debug("(critical) unable to aquire mutex for pend (1)");
                assert(false);
            }

            if ((pkt->flag & MAC_FLAG_ACK_RSP)) {
                nmac_debug("race condition: packet acked successfully");

            } else {
                if (--retry) {

                    if (!(pkt->flag & MAC_FLAG_ACK_RQR)) {
                        pkt->flag |= MAC_FLAG_ACK_RQR;
                    }

                    //nmac_debug("retry: start=%lu now=%lu seq=%lu", SCLK_MSEC(start), SCLK_MSEC(sclk_time()), (u32)pkt->seqn);
                    goto _retry_tx;
                } else {
                    //nmac_debug("retry: start=%lu now=%lu seq=%lu <fail>", SCLK_MSEC(start), SCLK_MSEC(sclk_time()), (u32)pkt->seqn);
                    nmac_debug("<drop> %02X %u", pkt->seqn, pkt->size);
                }
            }

        } else {
            if (!xSemaphoreTake(pend->mtx, pdMS_TO_TICKS(1000))) {
                nmac_debug("(critical) unable to aquire mutex for pend (2)");
                assert(false);
            }
        }

        pend->pkt = NULL;

        if (!xQueueSend(nmac.pendq, &pend_idx, 0)) {
            nmac_debug("(critical) unable to return pend sem to resource queue");
            assert(false);
        }

        // clear or check sem?
        xSemaphoreGive(pend->mtx);
    }

    //nmac_free(pkt);
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
    const xQueueHandle txq = nmac.txq;
    nmac_static_pkt_t pkt;

    while (1) {
        if (xQueueReceive(txq, &pkt, portMAX_DELAY)) {
            assert(pkt.size <= MAC_PKT_SIZE_MAX && pkt.addr == mac_addr);
            pkt.flag |= MAC_FLAG_PKT_BLK;
            send_packet(&pkt);
        }
    }
}

static void handle_rx(u8 flag, u8 size, u8 data[], s8 rssi, u8 lqi)
{
    nmac_pkt_t *const pkt = (nmac_pkt_t *)data;

    if (size != sizeof(nmac_pkt_t) + pkt->size) {
        nmac_debug("(rx) bad length: len=%u != size=%u + sizeof(nmac_pkt_t)=%u", size, pkt->size, sizeof(nmac_pkt_t));

    } else {
        nmac_debug_pkt(
                "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u time=%05lu \t %s",
                pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size, SCLK_MSEC(sclk_time()),
                pkt->flag & MAC_FLAG_ACK_REQ ? "RX-ACK-REQ" : (pkt->flag & MAC_FLAG_ACK_RSP ? "RX-ACK-RSP" : "RX")
        );

        if (pkt->dest != 0 && pkt->dest != mac_addr) {
            nmac_debug_v("(rx) drop: addr mismatch");
            
        } else {
            nmac_peer_t *const peer = peer_get_or_add(pkt->addr);

            if (peer) {
                if (pkt->flag & MAC_FLAG_ACK_REQ) {
                    send(pkt->addr, MAC_FLAG_PKT_BLK | MAC_FLAG_PKT_IMM | MAC_FLAG_ACK_RSP, sizeof(pkt->seqn), &pkt->seqn);
                }

                if (pkt->flag & MAC_FLAG_ACK_RSP) {
                    const u8 seqn = *(u8 *)pkt->data;
                    u8 i;

                    for (i = 0; i < NMAC_PEND_MAX; ++i) {
                        if (nmac.pend[i].pkt) {
                            if (!xSemaphoreTake(nmac.pend[i].mtx, pdMS_TO_TICKS(1000))) {
                                nmac_debug("(critical) unable to aquire mutex for ack");
                                assert(false);
                            }

                            nmac_pkt_t *const pend = nmac.pend[i].pkt;

                            if (pend && seqn == pend->seqn && (!pend->dest || pkt->addr == pend->dest)) {
                                if (!(pend->flag & MAC_FLAG_ACK_RSP)) {
                                    pend->flag |= MAC_FLAG_ACK_RSP;

                                    //if ((pend->flag & MAC_FLAG_ACK_RQR)) {
                                    //    nmac_debug("acked rqr: now=%lu seq=%lu", SCLK_MSEC(sclk_time()), (u32)seqn);
                                    //}

                                    //nmac_debug("ack: now=%lu seq=%u", sync_timestamp(), seqn);
                                    xSemaphoreGive(nmac.pend[i].mtx);
                                    xSemaphoreGive(nmac.pend[i].sem);
                                } else {
                                    nmac_debug("(warning) ack already received: now=%lu seq=%lu", SCLK_MSEC(sclk_time()), (u32)seqn);
                                    xSemaphoreGive(nmac.pend[i].mtx);
                                }

                                break;
                            } else {
                                xSemaphoreGive(nmac.pend[i].mtx);
                            }
                        }
                    }

                    //if (i == NMAC_PEND_MAX) {
                    //    nmac_debug("(warning) ack too late");
                    //}

                } else {
                    // Assumption: packets always arrive in non-monotonic increasing sequence order, retransmits included
                    if (!(pkt->flag & MAC_FLAG_ACK_RQR) || (peer->seqn != pkt->seqn)) {
                        peer->seqn = pkt->seqn;
                        nmac.rx(mac_addr, pkt->addr, pkt->dest, pkt->size, pkt->data, rssi, lqi);
                    }

                }

            }
        }
    }
}
