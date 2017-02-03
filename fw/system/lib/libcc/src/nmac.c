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
#define NMAC_PEND_TIME      47 // LBT min = 5ms. Channel blocking <= 25ms.

#define NMAC_TXQ_COUNT      1 // NMAC_PEND_MAX? When this is implemented it should be user-facing
#define NMAC_TXQ_SIZE       (NMAC_PEND_MAX * NMAC_TXQ_COUNT)


#define nmac_debug(format, ...) cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )
#define nmac_debug_pkt(format, ...) /*itm_printf(0, "<itm> " format "\r\n", ##__VA_ARGS__ )*/ /*cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )*/
#define nmac_debug_v(format, ...)

#undef assert
#define _sfy(x) #x
#define assert(x) if (!(x)) { itm_puts(0, "ASSERT FAIL: \" #x ""\" on line " _sfy( __LINE__ ) "\n"); asm("bkpt #0"); while (1) { asm("nop"); } }

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
    nmac_pkt_t *pkt;
    xSemaphoreHandle mtx;
    xSemaphoreHandle sem;

} nmac_pend_t;

typedef struct __packed {
    nmac_pkt_t *pkt;

} mac_txq_t;

static bool send(u16 dest, u8 flag, u16 size, u8 data[]);
static void tx_task(void *param);
static void handle_rx(u8 flag, u8 size, u8 data[], s8 rssi, u8 lqi);

typedef struct __packed {
    u16 addr;

} nmac_peer_t;


static struct {
    mac_recv_t rx;
    xQueueHandle txq;
    xQueueHandle pendq;

    nmac_pend_t *pend[NMAC_PEND_MAX];
    nmac_peer_t peer[NMAC_PEER_MAX];

} nmac = {NULL};

static u16 mac_addr = 0;
static u8 mac_seqn = 0;



s32 nmac_mem_count = 0;
s32 nmac_mem_size = 0;

static inline void *nmac_malloc(size_t size)
{
    void *ptr = malloc(size + sizeof(size_t));

    if (ptr) {
        *((size_t *)ptr) = size;
        ptr = (void *) &((size_t *)ptr)[1];
        ++nmac_mem_count;
        nmac_mem_size += size;
    }

    return ptr;
}

static inline void nmac_free(void *ptr)
{
    if (ptr) {
        if ((nmac_mem_size - ((size_t *)ptr)[-1]) < 0) {
            itm_puts(0, "mem size negative!\n");
            while (1) {
                asm("bkpt #0");
            }
        }

        nmac_mem_size -= ((size_t *)ptr)[-1];
        ptr = (void *) &((size_t *)ptr)[-1];
        --nmac_mem_count;
    }

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

bool nmac_init(u16 addr, bool sync_master, mac_recv_t rx)
{
    mac_addr = addr;
    nmac.rx = rx;

    nmac.txq = xQueueCreate(NMAC_TXQ_SIZE, sizeof(mac_txq_t)); assert(nmac.txq);
    nmac.pendq = xQueueCreate(NMAC_PEND_MAX, sizeof(nmac_pend_t)); assert(nmac.pendq);

    for (u8 i = 0; i < NMAC_PEND_MAX; ++i) {
        nmac.pend[i] = NULL;

        xSemaphoreHandle mtx = xSemaphoreCreateBinary(); assert(mtx);
        xSemaphoreHandle sem = xSemaphoreCreateBinary(); assert(sem);

        xSemaphoreGive(mtx);

        nmac_pend_t pend = {
                .pkt = NULL,
                .mtx = mtx,
                .sem = sem,
        };

        if (!xQueueSend(nmac.pendq, &pend, 0)) {
            nmac_debug("error: unable queue pend during setup");
            return false;
        }
    }

    if (!xTaskCreate(tx_task, "nmac:send", TASK_STACK_SIZE_DEFAULT, NULL, TASK_PRIO_HIGH, NULL)) {
        nmac_debug("error: unable to create send task");
        return false;
    }

    return nphy_init(handle_rx, sync_master);
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
            return send(dest, MAC_FLAG_PKT_BLK | MAC_FLAG_ACK_REQ, size, data);

        case NMAC_SEND_STRM:
            return send(dest, MAC_FLAG_PKT_IMM, size, data);

        default:
            return false;
    }
}

static bool do_send(mac_txq_t *txqi)
{
    assert(txqi);
    assert(txqi->pkt);
    assert(txqi->pkt->size <= MAC_PKT_SIZE_MAX);

    nmac_pkt_t *const pkt = txqi->pkt;
    const u8 pkt_len = sizeof(nmac_pkt_t) + pkt->size;
    const bool needs_ack = pkt->flag & MAC_FLAG_ACK_REQ;
    nmac_pend_t pend = {NULL};
    nmac_pend_t **pend_array_entry = NULL;
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
                if (!xQueueSendToFront(nmac.txq, txqi, pdMS_TO_TICKS(1000))) {
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

            // packet was queued
            return true;
        }

        if (!xQueueReceive(nmac.pendq, &pend, pdMS_TO_TICKS(1000))) {
            nmac_debug("mac/tx: acquire pend failed");
            nmac_free(pkt);
            return false;
        }

        pend.pkt = pkt;

        if (!xSemaphoreTake(pend.mtx, pdMS_TO_TICKS(1000))) {
            nmac_debug("mac/tx: acquire pend mtx failed");
            nmac_free(pkt);
            return false;
        }

        for (u8 i = 0; i < NMAC_PEND_MAX; ++i) {
            if (!nmac.pend[i]) {
                pend_array_entry = &nmac.pend[i];
                nmac.pend[i] = &pend;
                break;
            }
        }
    }

    const sclk_t start = sclk_time();

    _retry_tx:
    nphy_tx(nphy_flag, (u8 *)pkt, pkt_len);
    nmac_debug_pkt(
            "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u time=%05lu \t %s",
            pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size, SCLK_MSEC(sclk_time()),
            pkt->flag & MAC_FLAG_ACK_REQ ? "TX-ACK-REQ" : (pkt->flag & MAC_FLAG_ACK_RSP ? "TX-ACK-RSP" : "TX")
    );


    if (needs_ack) {
        xSemaphoreGive(pend.mtx);

        if (!xSemaphoreTake(pend.sem, pdMS_TO_TICKS(NMAC_PEND_TIME))) {
            if (!xSemaphoreTake(pend.mtx, pdMS_TO_TICKS(1000))) {
                nmac_debug("(critical) unable to aquire mutex for pend (1)");
                assert(false);
            }

            if ((pkt->flag & MAC_FLAG_ACK_RSP)) {
                nmac_debug("race condition: packet acked successfully");

            } else {

                if (!(pkt->flag & MAC_FLAG_ACK_RQR)) {
                    pkt->flag |= MAC_FLAG_ACK_RQR;
                }

                nmac_debug("retry: start=%lu now=%lu seq=%u", SCLK_MSEC(start), SCLK_MSEC(sclk_time()), (u32)pkt->seqn);
                goto _retry_tx;
            }

        } else {
            if (!xSemaphoreTake(pend.mtx, pdMS_TO_TICKS(1000))) {
                nmac_debug("(critical) unable to aquire mutex for pend (2)");
                assert(false);
            }
        }

        *pend_array_entry = NULL;
        pend.pkt = NULL;

        if (!xQueueSend(nmac.pendq, &pend, 0)) {
            nmac_debug("(critical) unable to return pend sem to resource queue");
            assert(false);
        }

        // clear or check sem?
        xSemaphoreGive(pend.mtx);
    }

    nmac_free(pkt);
    return true;
}

static bool send_packet(nmac_pkt_t *pkt)
{
    mac_txq_t txqi = {.pkt = pkt };
    return do_send(&txqi);
}

static bool send(u16 dest, u8 flag, u16 size, u8 data[])
{
    if (size > MAC_PKT_SIZE_MAX) {
        nmac_debug("(tx) packet too big, truncating");
        size = MAC_PKT_SIZE_MAX;
    }

    const u8 pkt_len = sizeof(nmac_pkt_t) + size;

    nmac_pkt_t *const pkt = nmac_malloc(pkt_len); assert(pkt);

    mac_seqn = (u8)((mac_seqn + 1) % UINT8_MAX);
    if (!mac_seqn) mac_seqn = 1;

    pkt->addr = mac_addr;
    pkt->dest = dest;
    pkt->seqn = mac_seqn;
    pkt->flag = flag;
    pkt->size = (u8)size;

    if (size && data) memcpy(pkt->data, data, size);

    return send_packet(pkt);
}

static void tx_task(void *param __unused)
{
    const xQueueHandle txq = nmac.txq;
    mac_txq_t txqi;

    while (1) {
        if (xQueueReceive(txq, &txqi, portMAX_DELAY)) {
            assert(txqi.pkt && txqi.pkt->size <= MAC_PKT_SIZE_MAX && txqi.pkt->addr == mac_addr);
            txqi.pkt->flag |= MAC_FLAG_PKT_BLK;
            do_send(&txqi);
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
                        if (nmac.pend[i]) {
                            if (!xSemaphoreTake(nmac.pend[i]->mtx, pdMS_TO_TICKS(1000))) {
                                nmac_debug("(critical) unable to aquire mutex for ack");
                                assert(false);
                            }

                            nmac_pkt_t *const pend = nmac.pend[i]->pkt; assert(pend);

                            if (seqn == pend->seqn && (!pend->dest || pkt->addr == pend->dest)) {
                                if (!(pend->flag & MAC_FLAG_ACK_RSP)) {
                                    pend->flag |= MAC_FLAG_ACK_RSP;

                                    if ((pend->flag & MAC_FLAG_ACK_RQR)) {
                                        nmac_debug("acked rqr: now=%lu seq=%u", SCLK_MSEC(sclk_time()), (u32)seqn);
                                    }

                                    //nmac_debug("ack: now=%lu seq=%u", sync_timestamp(), seqn);
                                    xSemaphoreGive(nmac.pend[i]->mtx);
                                    xSemaphoreGive(nmac.pend[i]->sem);
                                } else {
                                    nmac_debug("(warning) ack already received: now=%lu seq=%u", SCLK_MSEC(sclk_time()), (u32)seqn);
                                    xSemaphoreGive(nmac.pend[i]->mtx);
                                }

                                break;
                            } else {
                                xSemaphoreGive(nmac.pend[i]->mtx);
                            }
                        }
                    }

                    if (i == NMAC_PEND_MAX) {
                        nmac_debug("(warning) ack too late: now=%lu seq=%u", SCLK_MSEC(sclk_time()), (u32)seqn);
                    }
                } else {
                    nmac.rx(mac_addr, pkt->addr, pkt->dest, pkt->size, pkt->data, rssi, lqi);
                }

            }
        }
    }
}
