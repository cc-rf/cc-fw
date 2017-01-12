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


#define MAC_PENDING_ACK_MAX     1
#define MAC_ACK_TIMEOUT         50

#define nmac_debug(format, ...) cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )
#define nmac_debug_pkt(format, ...) //cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )
#define nmac_debug_v(format, ...)

typedef enum __packed {
    MAC_FLAG_ACK_REQ = 1 << 1,
    MAC_FLAG_ACK_RSP = 1 << 2,
    MAC_FLAG_ACK_RQR = 1 << 4,
    MAC_FLAG_PKT_BLK = 1 << 6,

} mac_flag_t;

typedef struct __packed {
    u16 addr;
    u16 dest;
    u8  seqn;
    u8  flag;
    u8  size;
    u8  data[];

} mac_pkt_t;

extern u32 sync_timestamp(void);

static void tx_task(void *param);
static void handle_rx(u8 flags, u8 *buf, u8 len);

static struct {
    mac_rx_t rx;
    xQueueHandle txq;

    mac_pkt_t *ack_pend;
    xSemaphoreHandle ack;

} nmac = {NULL};

static u16 mac_addr = 0;
static u8 mac_seqn = 0;

bool nmac_init(u16 addr, bool sync_master, mac_rx_t rx)
{
    mac_addr = addr;
    nmac.rx = rx;

    nmac.txq = xQueueCreate(3, sizeof(void *)); assert(nmac.txq);
    nmac.ack = xSemaphoreCreateBinary(); assert(nmac.ack);

    if (!xTaskCreate(tx_task, "nmac:send", TASK_STACK_SIZE_SMALL, NULL, TASK_PRIO_HIGH, NULL)) {
        cc_dbg("error: unable to create send task");
        return false;
    }

    return nphy_init(handle_rx, sync_master);
}

static bool send_packet(mac_pkt_t *pkt)
{
    const u8 pkt_len = sizeof(mac_pkt_t) + pkt->size;
    const bool needs_ack = pkt->flag & MAC_FLAG_ACK_REQ;
    u8 nphy_flag = 0;

    if (pkt->flag & MAC_FLAG_ACK_RSP) {
        nphy_flag |= PHY_PKT_FLAG_IMMEDIATE;
        // ... but hang on a (milli)sec
        //vTaskDelay(pdMS_TO_TICKS(3));
    }

    if (needs_ack) {
        if (!(pkt->flag & MAC_FLAG_PKT_BLK)) {
            // TODO: Error check
            if (nphy_flag & PHY_PKT_FLAG_IMMEDIATE) {
                xQueueSendToFront(nmac.txq, &pkt, portMAX_DELAY);
            } else {
                xQueueSend(nmac.txq, &pkt, portMAX_DELAY);
            }

            // packet was queued. what to return here?
            return true;
        }

        if (nmac.ack_pend) {
            cc_dbg("tx fail: no more pending packet slots!");
            return false;
        }

        nmac.ack_pend = pkt;
    }

    _retry_tx:
    nphy_tx(nphy_flag, (u8 *)pkt, pkt_len);
    nmac_debug_pkt(
            "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u time=%05u \t %s",
            pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size, sync_timestamp(),
            pkt->flag & MAC_FLAG_ACK_REQ ? "TX-ACK-REQ" : (pkt->flag & MAC_FLAG_ACK_RSP ? "TX-ACK-RSP" : "TX")
    );


    if (needs_ack) {

        if (!xSemaphoreTake(nmac.ack, pdMS_TO_TICKS(27))) {
            //nmac.ack_pend[pend_idx] = NULL;
            //nmac_debug("not acked: t=%lu", sync_timestamp());
            goto _retry_tx;
            return false;
        }

        nmac.ack_pend = NULL;
        return true;
    }

    return true;
}

static bool send(u16 dest, u8 flag, u8 size, u8 data[])
{
    const u8 pkt_len = sizeof(mac_pkt_t) + size;
    const bool needs_ack = flag & MAC_FLAG_ACK_REQ;
    u8 nphy_flag = 0;
    u8 pend_idx = 0;

    mac_pkt_t *const pkt = malloc(pkt_len); assert(pkt);

    pkt->addr = mac_addr;
    pkt->dest = dest;
    pkt->seqn = mac_seqn++;
    pkt->flag = flag;
    pkt->size = size;

    if (size && data) memcpy(pkt->data, data, size);

    return send_packet(pkt);
}

void nmac_tx(u16 dest, u8 size, u8 data[])
{
    send(dest, MAC_FLAG_ACK_REQ, size, data);
}

static void tx_task(void *param __unused)
{
    const xQueueHandle txq = nmac.txq;
    mac_pkt_t *pkt;

    while (1) {
        if (xQueueReceive(txq, &pkt, portMAX_DELAY)) {
            // TODO: Maybe copy packet to stack?
            pkt->flag |= MAC_FLAG_PKT_BLK;
            send_packet(pkt);
            free(pkt);
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

            if (pkt->flag & MAC_FLAG_ACK_REQ) {
                send(pkt->addr, MAC_FLAG_ACK_RSP, sizeof(pkt->seqn), &pkt->seqn);
            }

            if (pkt->flag & MAC_FLAG_ACK_RSP) {
                const mac_pkt_t *const pend = nmac.ack_pend;
                if (nmac.ack_pend && *(u8 *)pkt->data == pend->seqn && (!pend->dest || pkt->addr == pend->dest)) {
                    xSemaphoreGive(nmac.ack);
                }

            } else {
                // NOTE: RQR might be set here, and unfortunately sometimes the packet has already been handled
                nmac.rx(pkt->addr, pkt->dest, pkt->size, pkt->data);
            }
            
        }
    }
}
