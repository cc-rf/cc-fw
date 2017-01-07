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


#define MAC_PENDING_ACK_MAX     4
#define MAC_ACK_TIMEOUT         20

#define nmac_debug(format, ...) cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )
#define nmac_debug_pkt(format, ...) //cc_dbg_printf(format "\r\n", ##__VA_ARGS__ )
#define nmac_debug_v(format, ...)

typedef enum __packed {
    MAC_FLAG_ACK_REQ = 1 << 1,
    MAC_FLAG_ACK_RSP = 1 << 2,
    MAC_FLAG_ACK_RQR = 1 << 4,

} mac_flag_t;

typedef struct __packed {
    u16 addr;
    u16 dest;
    u8  seqn;
    u8  flag;
    u8  size;
    u8  data[];

} mac_pkt_t;


static void handle_rx(u8 flags, u8 *buf, u8 len);
static void ack_timeout(TimerHandle_t xTimer);

static struct {
    mac_rx_t rx;

} nmac = {NULL};

static u16 mac_addr = 0;
static u8 mac_seqn = 0;
static xTimerHandle ack_timer[MAC_PENDING_ACK_MAX] = {NULL};
//static xSemaphoreHandle ack_timer_mtx = NULL;
static xSemaphoreHandle ack_timer_sem = NULL;

bool nmac_init(u16 addr, bool sync_master, mac_rx_t rx)
{
    mac_addr = addr;
    nmac.rx = rx;

    ack_timer_sem = xSemaphoreCreateCounting(MAC_PENDING_ACK_MAX, MAC_PENDING_ACK_MAX); assert(ack_timer_sem);

    for (int i = 0; i < MAC_PENDING_ACK_MAX; ++i) {
        ack_timer[i] = xTimerCreate("nmac:acko", pdMS_TO_TICKS(MAC_ACK_TIMEOUT), pdFALSE, NULL, ack_timeout); assert(ack_timer[i]);
    }

    return nphy_init(handle_rx, sync_master);
}

static mac_pkt_t *send(u16 dest, u8 flag, u8 size, u8 data[])
{
    const u8 pkt_len = sizeof(mac_pkt_t) + size;
    const bool save = flag & MAC_FLAG_ACK_REQ;
    u8 nphy_flag = 0;

    mac_pkt_t *const pkt = save ? malloc(pkt_len) : alloca(pkt_len); assert(pkt);

    pkt->addr = mac_addr;
    pkt->dest = dest;
    pkt->seqn = mac_seqn++;
    pkt->flag = flag;
    pkt->size = size;

    if (size && data) memcpy(pkt->data, data, size);

    if (pkt->flag & MAC_FLAG_ACK_RSP) {
        nphy_flag |= PHY_PKT_FLAG_IMMEDIATE;
    }

    if (save) {
        if (!xSemaphoreTake(ack_timer_sem, pdMS_TO_TICKS(5000))) {
            nmac_debug("send: stalled waiting on ack timer semaphore!");
            free(pkt);
            return NULL;
        }

        u8 i;

        for (i = 0; i < MAC_PENDING_ACK_MAX; ++i) {
            TimerHandle_t *const tmr = ack_timer[i]; assert(tmr);

            if (!pvTimerGetTimerID(tmr)) {
                vTimerSetTimerID(tmr, pkt);
                if (!xTimerReset(tmr, pdMS_TO_TICKS(5000))) {
                    nmac_debug("send: stalled waiting on ack timer reset!");
                    free(pkt);
                    return NULL;
                }
                break;
            }
        }

        if (i >= MAC_PENDING_ACK_MAX) {
            nmac_debug("send: abort! no free timers! sem count=%lu", uxSemaphoreGetCount(ack_timer_sem));
            free(pkt);
            return NULL;
        }
    }

    nphy_tx(nphy_flag, (u8 *)pkt, pkt_len);
    nmac_debug_pkt(
            "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u \t %s",
            pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size,
            pkt->flag & MAC_FLAG_ACK_REQ ? "TX-ACK-REQ" : (pkt->flag & MAC_FLAG_ACK_RSP ? "TX-ACK-RSP" : "TX")
    );

    return save ? pkt : NULL;
}

void nmac_tx(u16 dest, u8 size, u8 data[])
{
    send(dest, MAC_FLAG_ACK_REQ, size, data);
}

static void ack_clear(mac_pkt_t *pkt)
{
    mac_pkt_t *spkt;
    u8 i;

    for (i = 0; i < MAC_PENDING_ACK_MAX; ++i) {
        TimerHandle_t *const tmr = ack_timer[i]; assert(tmr);
        spkt = pvTimerGetTimerID(tmr);
        
        if (*(u8 *)pkt->data == spkt->seqn && (!spkt->dest || pkt->addr == spkt->dest)) {

            if (!xTimerStop(tmr, pdMS_TO_TICKS(5000))) {
                nmac_debug("ack: stalled waiting on ack timer stop!");
                free(spkt);
                break;
            }

            vTimerSetTimerID(tmr, NULL);
            xSemaphoreGive(ack_timer_sem);

            nmac_debug_pkt(
                    "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u \t ACK",
                    spkt->addr, spkt->dest, spkt->seqn, spkt->flag, spkt->size
            );

            free(spkt);
            break;
        }
    }

    if (i == MAC_PENDING_ACK_MAX) {
        // seems to happen a lot...
        //nmac_debug("(rx) no ack to clear");
    }
}

static void handle_rx(u8 flags, u8 *buf, u8 len)
{
    mac_pkt_t *const pkt = (mac_pkt_t *)buf;

    if (len != sizeof(mac_pkt_t) + pkt->size) {
        nmac_debug("(rx) bad length: len=%u != size=%u + sizeof(mac_pkt_t)=%u", len, pkt->size, sizeof(mac_pkt_t));

    } else {
        nmac_debug_pkt(
                "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u \t %s",
                pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size,
                pkt->flag & MAC_FLAG_ACK_REQ ? "RX-ACK-REQ" : (pkt->flag & MAC_FLAG_ACK_RSP ? "RX-ACK-RSP" : "RX")
        );

        if (pkt->dest != 0 && pkt->dest != mac_addr) {
            nmac_debug_v("(rx) drop: addr mismatch");
            
        } else {

            if (pkt->flag & MAC_FLAG_ACK_REQ) {
                send(pkt->addr, MAC_FLAG_ACK_RSP, sizeof(pkt->seqn), &pkt->seqn);
            }

            if (pkt->flag & MAC_FLAG_ACK_RSP) {
                ack_clear(pkt);

            } else {
                nmac.rx(pkt->addr, pkt->dest, pkt->size, pkt->data);
            }
            
        }
    }
}

extern u32 sync_timestamp(void);

static void ack_timeout(TimerHandle_t xTimer)
{
    mac_pkt_t *const pkt = pvTimerGetTimerID(xTimer); assert(pkt);

    if (!(pkt->flag & MAC_FLAG_ACK_RQR)) {
        pkt->flag |= MAC_FLAG_ACK_RQR;

        nmac_debug_pkt(
                "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u \t RETRY \t T=%lu",
                pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size, sync_timestamp()
        );

        nphy_tx(0, (u8 *) pkt, sizeof(mac_pkt_t) + pkt->size);

        /*if (!xTimerReset(xTimer, pdMS_TO_TICKS(5000))) {
            nmac_debug("retry: stalled waiting on ack timer reset!");
            free(pkt);
        }*/

        vTimerSetTimerID(xTimer, NULL);
        xSemaphoreGive(ack_timer_sem);

    } /*else {
        nmac_debug_pkt(
                "pkt: addr=0x%04X dest=0x%04X seqn=0x%02X flag=0x%02X size=%03u \t TIMEOUT \t T=%lu",
                pkt->addr, pkt->dest, pkt->seqn, pkt->flag, pkt->size, sync_timestamp()
        );

        vTimerSetTimerID(xTimer, NULL);
        xSemaphoreGive(ack_timer_sem);
    }*/
}

