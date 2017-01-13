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


#define NMAC_PEER_MAX       10

#define NMAC_PEND_MAX       3
#define NMAC_PEND_TIME      27

#define NMAC_TXQ_COUNT      1 // NMAC_PEND_MAX? When this is implemented it should be user-facing
#define NMAC_TXQ_SIZE       (NMAC_PEND_MAX * NMAC_TXQ_COUNT)




#define nmac_debug(format, ...) cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )
#define nmac_debug_pkt(format, ...) //cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )
#define nmac_debug_v(format, ...)

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

} mac_pkt_t;

typedef struct __packed {
    mac_pkt_t *pkt;
    xSemaphoreHandle sem;

} mac_txq_t;

extern u32 sync_timestamp(void);

static bool send(u16 dest, u8 flag, u8 size, u8 data[]);
static void tx_task(void *param);
static void handle_rx(u8 flags, u8 *buf, u8 len);

typedef struct __packed {
    u16 addr;

} nmac_peer_t;


static struct {
    mac_rx_t rx;
    xQueueHandle txq;
    xQueueHandle pendq;

    mac_txq_t *pend[NMAC_PEND_MAX];

    nmac_peer_t peer[NMAC_PEER_MAX];

} nmac = {NULL};

static u16 mac_addr = 0;
static u8 mac_seqn = 0;



s32 nmac_mem_count = 0;

static inline void *nmac_malloc(size_t size)
{
    void *ptr = malloc(size);
    if (ptr) ++nmac_mem_count;
    return ptr;
}

static inline void nmac_free(void *ptr)
{
    if (ptr) --nmac_mem_count;
    return free(ptr);
}


static nmac_peer_t *peer_get_or_add(u16 addr)
{
    nmac_peer_t *pp = NULL;

    if (!addr) return NULL;

    for (u8 i = 0; i < NMAC_PEER_MAX; ++i) {
        if (nmac.peer[i].addr == addr) {
            return &nmac.peer[i];
        } else if (!pp && !nmac.peer[i].addr) {
            pp = &nmac.peer[i];
        }
    }

    if (pp) {
        pp->addr = addr;
        //pp->seqn = 0;
        return pp;
    }

    return NULL;
}

bool nmac_init(u16 addr, bool sync_master, mac_rx_t rx)
{
    mac_addr = addr;
    nmac.rx = rx;

    nmac.txq = xQueueCreate(NMAC_TXQ_SIZE, sizeof(mac_txq_t)); assert(nmac.txq);
    nmac.pendq = xQueueCreate(NMAC_PEND_MAX, sizeof(xSemaphoreHandle)); assert(nmac.pendq);

    for (u8 i = 0; i < NMAC_PEND_MAX; ++i) {
        nmac.pend[i] = NULL;
        xSemaphoreHandle sem =xSemaphoreCreateBinary(); assert(sem);

        if (!xQueueSend(nmac.pendq, &sem, 0)) {
            nmac_debug("error: unable queue sem during setup");
            return false;
        }
    }

    if (!xTaskCreate(tx_task, "nmac:send", TASK_STACK_SIZE_DEFAULT, NULL, TASK_PRIO_HIGH, NULL)) {
        nmac_debug("error: unable to create send task");
        return false;
    }

    return nphy_init(handle_rx, sync_master);
}

bool nmac_send(nmac_send_t type, u16 dest, u8 size, u8 data[])
{
    switch (type) {
        case NMAC_SEND_DGRM:
            return send(dest, 0, size, data);

        case NMAC_SEND_MESG:
            return send(dest, MAC_FLAG_ACK_REQ, size, data);

        case NMAC_SEND_TRXN:
            return send(dest, MAC_FLAG_PKT_BLK | MAC_FLAG_ACK_REQ, size, data);

        case NMAC_SEND_STRM:
            return send(dest, MAC_FLAG_PKT_IMM, size, data);

        default:
            return false;
    }
}

static bool do_send(mac_txq_t *txqi)
{
    mac_pkt_t *const pkt = txqi->pkt;
    const u8 pkt_len = sizeof(mac_pkt_t) + pkt->size;
    const bool needs_ack = pkt->flag & MAC_FLAG_ACK_REQ;
    mac_txq_t **pend_txqi = NULL;
    u8 nphy_flag = 0;

    nmac_debug_pkt(
            "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u time=%05u \t TX-BEGIN",
            pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size, sync_timestamp()
    );

    if (pkt->flag & MAC_FLAG_PKT_IMM) {
        nphy_flag |= PHY_PKT_FLAG_IMMEDIATE;
    }

    if (needs_ack) {
        if (!(pkt->flag & MAC_FLAG_PKT_BLK)) {
            if (nphy_flag & PHY_PKT_FLAG_IMMEDIATE) {
                if (!xQueueSendToFront(nmac.txq, txqi, portMAX_DELAY)) {
                    nmac_debug("mac/tx: queue immediate failed");
                    nmac_free(pkt);
                    return false;
                }
            } else {
                if (!xQueueSend(nmac.txq, txqi, portMAX_DELAY)) {
                    nmac_debug("mac/tx: queue failed");
                    nmac_free(pkt);
                    return false;
                }
            }

            // packet was queued. what to return here?
            return true;
        }

        if (!xQueueReceive(nmac.pendq, &txqi->sem, portMAX_DELAY)) {
            nmac_debug("mac/tx: acquire pend sem failed");
            nmac_free(pkt);
            return false;
        }

        for (u8 i = 0; i < NMAC_PEND_MAX; ++i) {
            if (!nmac.pend[i]) {
                pend_txqi = &nmac.pend[i];
                nmac.pend[i] = txqi;
                break;
            }
        }

        assert(pend_txqi && *pend_txqi);
    }

    _retry_tx:
    nphy_tx(nphy_flag, (u8 *)pkt, pkt_len);
    nmac_debug_pkt(
            "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u time=%05u \t %s",
            pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size, sync_timestamp(),
            pkt->flag & MAC_FLAG_ACK_REQ ? "TX-ACK-REQ" : (pkt->flag & MAC_FLAG_ACK_RSP ? "TX-ACK-RSP" : "TX")
    );


    if (needs_ack) {
        assert(txqi->sem);

        if (!xSemaphoreTake(txqi->sem, pdMS_TO_TICKS(NMAC_PEND_TIME))) {
            if (!(pkt->flag & MAC_FLAG_ACK_RQR)) {
                pkt->flag |= MAC_FLAG_ACK_RQR;
            }

            goto _retry_tx;
        }

        if (!xQueueSend(nmac.pendq, &txqi->sem, 0)) {
            nmac_debug("mac/tx: unable to return pend sem to resource queue");
        }

        txqi->sem = NULL;
        *pend_txqi = NULL;
    }

    nmac_free(pkt);
    return true;
}

static bool send_packet(mac_pkt_t *pkt)
{
    mac_txq_t txqi = {.pkt = pkt, .sem = NULL };
    return do_send(&txqi);
}

static bool send(u16 dest, u8 flag, u8 size, u8 data[])
{
    const u8 pkt_len = sizeof(mac_pkt_t) + size;

    mac_pkt_t *const pkt = nmac_malloc(pkt_len); assert(pkt);

    mac_seqn = (u8)((mac_seqn + 1) % UINT8_MAX);
    if (!mac_seqn) mac_seqn = 1;

    pkt->addr = mac_addr;
    pkt->dest = dest;
    pkt->seqn = mac_seqn;
    pkt->flag = flag;
    pkt->size = size;

    if (size && data) memcpy(pkt->data, data, size);

    return send_packet(pkt);
}

static void tx_task(void *param __unused)
{
    const xQueueHandle txq = nmac.txq;
    mac_txq_t txqi;

    while (1) {
        if (xQueueReceive(txq, &txqi, portMAX_DELAY)) {
            txqi.pkt->flag |= MAC_FLAG_PKT_BLK;
            do_send(&txqi);
        }
    }
}

static void handle_rx(u8 flags, u8 *buf, u8 len)
{
    mac_pkt_t *const pkt = (mac_pkt_t *)buf;

    if (len != sizeof(mac_pkt_t) + pkt->size) {
        nmac_debug("(rx) bad length: len=%u != size=%u + sizeof(mac_pkt_t)=%u", len, pkt->size, sizeof(mac_pkt_t));

    } else {
        nmac_debug_pkt(
                "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u time=%05u \t %s",
                pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size, sync_timestamp(),
                pkt->flag & MAC_FLAG_ACK_REQ ? "RX-ACK-REQ" : (pkt->flag & MAC_FLAG_ACK_RSP ? "RX-ACK-RSP" : "RX")
        );

        if (pkt->dest != 0 && pkt->dest != mac_addr) {
            nmac_debug_v("(rx) drop: addr mismatch");
            
        } else {
            nmac_peer_t *const peer = peer_get_or_add(pkt->addr);

            if (peer) {
                if (pkt->flag & MAC_FLAG_ACK_REQ) {
                    send(pkt->addr, MAC_FLAG_PKT_IMM | MAC_FLAG_ACK_RSP, sizeof(pkt->seqn), &pkt->seqn);
                }

                if (pkt->flag & MAC_FLAG_ACK_RSP) {
                    for (u8 i = 0; i < NMAC_PEND_MAX; ++i) {
                        if (nmac.pend[i]) {
                            mac_pkt_t *const pend = nmac.pend[i]->pkt; assert(pend);
                            const u8 seqn = *(u8 *)pkt->data;

                            if (seqn == pend->seqn && (!pend->dest || pkt->addr == pend->dest)) {
                                if (!(pend->flag & MAC_FLAG_ACK_RSP)) {
                                    pend->flag |= MAC_FLAG_ACK_RSP;
                                    xSemaphoreGive(nmac.pend[i]->sem);
                                } else {
                                    nmac_debug("warning: ack already received");
                                }
                                break;
                            }
                        }
                    }

                } else {
                    nmac.rx(pkt->addr, pkt->dest, pkt->size, pkt->data);
                }

            }
        }
    }
}
