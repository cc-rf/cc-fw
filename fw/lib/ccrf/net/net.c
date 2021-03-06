#include <ccrf/net.h>
#include <ccrf/list.h>

#include <phy/phy.h>

#include "sys/trace.h"
#include "sys/local.h"
#include "sys/clock.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <fsl_rnga.h>
#include <kio/flsh.h>


#define net_trace_info          ccrf_trace_info
#define net_trace_warn          ccrf_trace_warn
#define net_trace_error         ccrf_trace_error
#define net_trace_debug         ccrf_trace_debug
#define net_trace_verbose(...)  //ccrf_trace_verbose


#define NET_PEER_MAX            64u
#define NET_PEER_EXPIRE_TIME    30u
#define NET_TIMER_INTERVAL      10u

#define NET_NOTIFY_TRXN_DONE    (1u<<18u)
#define NET_NOTIFY_PING_DONE    (1u<<20u)

#define NET_MAC_FLAG_NETLAYER   MAC_FLAG_0

#define NET_CORE_PEER_PORT          1
#define NET_CORE_PEER_TYPE          2
#define NET_CORE_PEER_LEAVE_TYPE    3
#define NET_CORE_PING_PORT          2
#define NET_CORE_PING_REQ_TYPE      1
#define NET_CORE_PING_RSP_TYPE      2


typedef struct __packed {
    net_info_t info;

} net_mesg_t;

typedef struct txni {
    net_path_t path;
    struct list_head *trxn;
    TaskHandle_t task;
    struct list_head list;

} net_txni_t;

typedef struct __packed {
    net_size_t size_rsp;
    bool strm;
    u8 data[];

} net_ping_req_mesg_t;

typedef struct __packed {
    pkt_meta_t meta;
    u8 data[];

} net_ping_rsp_mesg_t;


struct net {
    net_recv_t recv;
    net_evnt_t evnt;

    struct {
        net_time_t recv;
        net_time_t send;

    } last;
    
    phy_t phy;

    struct {
        mac_t mac;
        mac_addr_t addr;
        mac_recv_t recv;

    } mac;
    
    net_peer_list_t peer;
    net_time_t peer_bcast_last;

    TimerHandle_t timer;
    StaticTimer_t timer_static;

    struct list_head txni;

    net_stat_t stat;
    net_ping_t *ping;
    TaskHandle_t ping_task;
};


static void net_mesg_init(net_path_t path, mbuf_t *mbuf) __ccrf_code __nonnull_all;
static void net_trxn_resp(net_t net, net_addr_t addr, net_txni_t *txni, mbuf_t *mbuf) __ccrf_code;
static net_size_t net_send_base(net_t net, mac_send_t type, net_path_t path, mbuf_t *mbuf) __ccrf_code;
static inline net_size_t net_send_base_mesg(net_t net, net_path_t path, mbuf_t *mbuf) __ccrf_code;
static inline void net_send_base_dgrm(net_t net, net_path_t path, mbuf_t *mbuf) __ccrf_code;
static inline void net_send_base_strm(net_t net, net_path_t path, mbuf_t *mbuf) __ccrf_code;
static inline void net_send_base_bcst(net_t net, net_info_t info, mbuf_t *mbuf);
static void net_evnt_peer(net_t net, net_addr_t addr, net_event_peer_action_t action);
static void net_mac_recv(net_t net, mac_flag_t flag, mac_addr_t peer, mac_addr_t dest, mac_seqn_t seqn, mbuf_t *mbuf, pkt_meta_t meta) __ccrf_code;
static void net_core_recv(net_t net, net_addr_t addr, net_mesg_t *mesg, mbuf_t *mbuf, pkt_meta_t meta) __ccrf_code;
static void net_peer_bcast(net_t net);
static void net_peer_bcast_leave(net_t net, net_addr_t addr);
static net_peer_t *net_peer_get(net_t net, net_addr_t addr, bool add) __ccrf_code;
static net_peer_t *net_peer_update(net_t net, net_addr_t addr, pkt_meta_t meta) __ccrf_code;
static void net_peer_expire(net_t net, net_time_t time);
static bool net_peer_delete(net_t net, net_addr_t addr);
static void net_peer_remove(net_t net, net_peer_t *peer, net_event_peer_action_t action);
static void net_timer(TimerHandle_t timer);


static struct net nets[CCRF_CONFIG_RDIO_COUNT] __ccrf_data;


net_t net_init(net_config_t *config, bool *fail)
{
    net_t net = &nets[config->phy.rdid];
    *fail = false;

    memset(net, 0, sizeof(struct net));

    net->recv = config->net.recv;
    net->evnt = config->net.evnt;

    INIT_LIST_HEAD(&net->peer);

    net->mac.recv = config->mac.recv;

    mac_config_t mac_config = {
            .rdid = config->phy.rdid,
            .cell = config->phy.cell,
            .addr = config->mac.addr,
            .recv = (mac_recv_t) net_mac_recv,
            .recv_param = net
    };

    net->mac.mac = mac_init(&mac_config, fail);

    if (*fail) goto _fail;

    net->phy = mac_phy(net->mac.mac);

    net->mac.addr = mac_addr(net->mac.mac);

    net->timer = xTimerCreateStatic(
            "net.peer", pdSEC_TO_TICKS(NET_TIMER_INTERVAL),
            pdTRUE, net, net_timer, &net->timer_static
    );

    xTimerStart(net->timer, portMAX_DELAY);

    INIT_LIST_HEAD(&net->txni);

    goto _done;
    _fail:

    *fail = true;

    _done:
    return net;
}


void net_down(net_t net)
{
    // For now, just stop the broadcast timer and leave the network.

    xTimerStop(net->timer, 0);

    net_peer_bcast_leave(net, net->mac.addr);
}


net_time_t net_time(void)
{
    return CCRF_CLOCK_SEC(ccrf_clock());
}


mac_t net_mac(net_t net)
{
    return net->mac.mac;
}


net_addr_t net_addr(net_t net)
{
    return net->mac.addr;
}


net_addr_t net_addr_set(net_t net, net_addr_t orig, net_addr_t addr)
{
    net_addr_t set = net->mac.addr;

    if (addr && orig && orig == set) {
        set = (net_addr_t) mac_addr_set(net->mac.mac, orig, addr);

        if (set && set != net->mac.addr) {
            net->mac.addr = set;
            net_peer_delete(net, orig);
            net_peer_bcast_leave(net, orig);
        }
    }

    return set;
}


void net_stat(net_t net, net_stat_t *stat)
{
    *stat = net->stat;
}


void net_peers(net_t net, net_peer_list_t **peer)
{
    *peer = &net->peer;
}


void net_peers_wipe(net_t net)
{
    net_peer_expire(net, 0);
}


net_size_t net_peers_flat(net_t net, net_size_t extra, net_peer_info_t **list)
{
    net_size_t count = 0, next = 0, size_peers;
    net_peer_info_t *peers;
    net_peer_t *peer;

    *list = NULL;

    list_for_each_entry(peer, &net->peer, item) {
        count++;
    }

    if (!(size_peers = extra + count * sizeof(net_peer_info_t)))
        return count;

    *list = peers = pvPortMalloc(size_peers);

    if (extra) peers = (net_peer_info_t *) &((u8 *)peers)[extra];

    net_time_t now = net_time();

    list_for_each_entry(peer, &net->peer, item) {
        peers[next] = peer->info;
        peers[next++].last = now - peer->info.last;
    }

    return count;
}


static void net_mesg_init(net_path_t path, mbuf_t *mbuf)
{
    net_mesg_t mesg = { .info = path.info };

    if (!*mbuf)
        *mbuf = mbuf_alloc(sizeof(net_mesg_t), (u8 *) &mesg);
    else {
        mbuf_push(mbuf, sizeof(net_mesg_t), (u8 *) &mesg);
    }
}


void net_sync(net_t net, bool *resy)
{
    phy_sync(net->phy, resy);
}


net_size_t net_send(net_t net, net_path_t path, mbuf_t *mbuf)
{
    path.info.mode = 0;

    return net_send_base(net, MAC_SEND_DGRM, path, mbuf);
}


net_size_t net_mesg(net_t net, net_path_t path, mbuf_t *mbuf)
{
    path.info.mode = 0;

    return net_send_base(net, MAC_SEND_MESG, path, mbuf);
}


net_size_t net_resp(net_t net, net_path_t path, mbuf_t *mbuf)
{
    return net_mesg(net, path, mbuf);
}


static void stat_tx_add(net_t net, net_size_t size, mac_size_t rslt)
{
    if (rslt) {
        net->stat.tx.count++;
        net->stat.tx.bytes += size;
        net->last.send = net_time();
    } else {
        net->stat.tx.errors++;
    }
}


net_size_t net_send_base(net_t net, mac_send_t type, net_path_t path, mbuf_t *mbuf)
{
    if (*mbuf && (*mbuf)->used > NET_SEND_MAX) {
        net_trace_warn("send size too big: %u > %u", (*mbuf)->used, NET_SEND_MAX);
        return 0;
    }

    net_mesg_init(path, mbuf);

    if (((*mbuf)->used - sizeof(net_mesg_t)) > NET_SEND_MAX) {
        net_trace_warn("send size too big: %u > %u", (*mbuf)->used, NET_SEND_MAX);
        mbuf_popf(mbuf, sizeof(net_mesg_t), NULL);
        return 0;
    }

    if (type == MAC_SEND_MESG && path.addr == MAC_ADDR_BCST)
        type = MAC_SEND_DGRM;

    mac_size_t sent = mac_send(net->mac.mac, type, NET_MAC_FLAG_NETLAYER, path.addr, *mbuf, type != MAC_SEND_STRM);

    if (!(((net_mesg_t *) (*mbuf)->data)->info.mode & NET_MODE_FLAG_CORE))
        stat_tx_add(net, (net_size_t) (*mbuf)->used - sizeof(net_mesg_t), sent);

    if (type == MAC_SEND_MESG && sent) {
        // was acked, add to table
        net_peer_update(net, path.addr, mac_meta(net->mac.mac));
    }

    mbuf_popf(mbuf, sizeof(net_mesg_t), NULL);

    return sent;
}


static net_size_t net_send_base_mesg(net_t net, net_path_t path, mbuf_t *mbuf)
{
    return net_send_base(net, MAC_SEND_MESG, path, mbuf);
}


static void net_send_base_dgrm(net_t net, net_path_t path, mbuf_t *mbuf)
{
    net_send_base(net, MAC_SEND_DGRM, path, mbuf);
}


static void net_send_base_strm(net_t net, net_path_t path, mbuf_t *mbuf)
{
    net_send_base(net, MAC_SEND_STRM, path, mbuf);
}


static void net_send_base_bcst(net_t net, net_info_t info, mbuf_t *mbuf)
{
   net_path_t path = { .addr = MAC_ADDR_BCST, .info = info };

   net_send_base_dgrm(net, path, mbuf);
}


size_t net_trxn(net_t net, net_path_t path, mbuf_t *mbuf, net_time_t expiry, net_trxn_rslt_t *rslt)
{
    size_t sent = 0;

    INIT_LIST_HEAD(rslt);

    if (!expiry || expiry >= portMAX_DELAY) return sent;

    // TODO: Check that this is not the RX task. Will deadlock!

    path.info.mode = 0;

    net_txni_t txni = {
            .path = path,
            .trxn = rslt,
            .task = xTaskGetCurrentTaskHandle(),
            .list = LIST_HEAD_INIT(txni.list)
    };

    list_add_tail(&txni.list, &net->txni);

    if (!(sent = net_send_base_mesg(net, path, mbuf))) {
        net_trace_warn("trxn send fail");
        goto _fail;
    }

    u32 notify = 0;

    do {

        if (!xTaskNotifyWait(NET_NOTIFY_TRXN_DONE, NET_NOTIFY_TRXN_DONE, &notify, pdMS_TO_TICKS(expiry))) {
            notify = 0;
            break;
        }

    } while (!(notify & NET_NOTIFY_TRXN_DONE));


    goto _done;
    _fail:

    net_trxn_rslt_free(txni.trxn);

    _done:

    list_del(&txni.list);

    return sent;
}


void net_trxn_rslt_free(net_trxn_rslt_t *rslt)
{
    if (rslt) {
        net_trxn_t *trxn, *trxn_t;

        list_for_each_entry_safe(trxn, trxn_t, rslt, __list) {
            list_del(&trxn->__list);
            mbuf_free(&trxn->mbuf);
            vPortFree(trxn);
        }
    }
}


static void net_trxn_resp(net_t net, net_addr_t addr, net_txni_t *txni, mbuf_t *mbuf)
{
    // TODO: Is a race condition possible here?

    net_trxn_t *trxn = pvPortMalloc(sizeof(net_trxn_t));

    trxn->addr = addr;
    trxn->mbuf = *mbuf;
    *mbuf = NULL;

    list_add_tail(&trxn->__list, txni->trxn);

    if (txni->path.addr) {
        xTaskNotify(txni->task, NET_NOTIFY_TRXN_DONE, eSetBits);
    }
}


static void net_evnt_peer(net_t net, net_addr_t addr, net_event_peer_action_t action)
{
    net_event_peer_t event = {
            .addr = addr,
            .action = action
    };

    return net->evnt(net, NET_EVENT_PEER, &event);
}


bool net_ping(net_t net, net_addr_t addr, bool strm, net_size_t size, net_size_t size_resp, net_time_t timeout, net_ping_t *rslt)
{
    rslt->addr = addr;

    if (addr == NET_ADDR_BCST) return false;
    if ((size + sizeof(net_mesg_t) + sizeof(net_ping_req_mesg_t)) > NET_SEND_MAX) return false;
    if ((size_resp + sizeof(net_mesg_t) + sizeof(net_ping_req_mesg_t)) > NET_SEND_MAX) return false;
    if (!timeout || (timeout >= portMAX_DELAY)) return false;

    u32 notify;

    const net_path_t path = {
            .addr = addr,
            .info = {
                    .mode = NET_MODE_FLAG_CORE,
                    .port = NET_CORE_PING_PORT,
                    .type = NET_CORE_PING_REQ_TYPE
            }
    };

    net->ping = rslt;

    mbuf_t mbuf = mbuf_make(
            sizeof(net_mesg_t) + sizeof(net_ping_req_mesg_t) + size,
            sizeof(net_ping_req_mesg_t) + size,
            NULL
    );

    net_ping_req_mesg_t *ping_req = (net_ping_req_mesg_t *) mbuf->data;

    ping_req->size_rsp = size_resp;
    ping_req->strm = strm;

    if (size) memset(ping_req->data, 0xA5, size);

    net->ping_task = xTaskGetCurrentTaskHandle();

    net_time_t tx_time = rdio_util_get_tx_time(
            phy_rdio(mac_phy(net_mac(net))),
            sizeof(net_mesg_t) + sizeof(net_ping_req_mesg_t) + size + MAC_PKT_OVERHEAD + 2/*phy: cell, flag*/
    );

    rslt->rtt_usec = ccrf_clock() + tx_time;

    rslt->tx_count = net_send_base(net, strm ? MAC_SEND_STRM : MAC_SEND_DGRM, path, &mbuf);

    do {
        if (!xTaskNotifyWait(NET_NOTIFY_PING_DONE, NET_NOTIFY_PING_DONE, &notify, pdMS_TO_TICKS(timeout))) {

            net->ping = NULL;
            net->ping_task = NULL;

            notify = 0;

            rslt->rtt_usec = 0;

            break;
        }
        
    } while (!(notify & NET_NOTIFY_PING_DONE));
    
    mbuf_free(&mbuf);
    
    return notify != 0;
}


static void net_mac_recv(net_t net, mac_flag_t flag, mac_addr_t peer, mac_addr_t dest, mac_seqn_t seqn, mbuf_t *mbuf, pkt_meta_t meta)
{
    if (flag != NET_MAC_FLAG_NETLAYER) {
        // passthrough
        return net->mac.recv(net->mac.mac, flag, peer, dest, seqn, mbuf, meta);
    }

    net_mesg_t mesg;

    mbuf_popf(mbuf, sizeof(net_mesg_t), (u8 *) &mesg);

    if (!peer) {
        net_trace_warn("peer=0");
        net->stat.rx.errors++;
        return;
    }

    if (dest && (dest != net->mac.addr))
        return;

    if (!(mesg.info.mode & NET_MODE_FLAG_CORE)) {
        net_trace_verbose(
                "net: %04X: %04X->%04X %u:%u:%u #%u",
                net->mac.addr, peer, dest,
                mesg->info.mode, mesg->info.port, mesg->info.type,
                size
        );
    }

    if (mesg.info.mode & NET_MODE_FLAG_CORE)
        return net_core_recv(net, peer, &mesg, mbuf, meta);

    net_peer_update(net, peer, meta);

    net->stat.rx.count++;
    net->stat.rx.bytes += (*mbuf)->used;
    net->last.recv = net_time();

    net_txni_t *txni;

    list_for_each_entry(txni, &net->txni, list) {
        const bool match = net_path_info_match(&txni->path.info, &mesg.info);

        if (match)
            return net_trxn_resp(net, peer, txni, mbuf);
    }

    net_path_t path = {
            .addr = peer,
            .info = mesg.info
    };

    return net->recv(net, path, dest, seqn, mbuf, meta);
}


static void net_core_recv(net_t net, net_addr_t addr, net_mesg_t *mesg, mbuf_t *mbuf, pkt_meta_t meta)
{
    switch (mesg->info.port) {

        case NET_CORE_PEER_PORT:

            switch (mesg->info.type) {

                case NET_CORE_PEER_TYPE: {

                    net_bcst_t *bcst = (net_bcst_t *) (*mbuf)->data;
                    net_peer_t *peer;

                    if ((peer = net_peer_update(net, addr, meta))) {
                        bool new = !peer->info.bcst.date;

                        peer->info.bcst = *bcst;

                        if (!new) net_evnt_peer(net, peer->info.peer, NET_EVENT_PEER_UPD);
                    }
                }

                break;

                case NET_CORE_PEER_LEAVE_TYPE:
                    if ((*mbuf)->used == sizeof(net_addr_t)) {

                        net_addr_t left = *(net_addr_t *)(*mbuf)->data;

                        if (net_peer_delete(net, left)) {
                            net_evnt_peer(net, left, NET_EVENT_PEER_OUT);
                        }
                    }

                break;

            }

        break;

        case NET_CORE_PING_PORT:

            switch (mesg->info.type) {

                case NET_CORE_PING_REQ_TYPE: {

                    net_size_t size_rsp = ((net_ping_req_mesg_t *) (*mbuf)->data)->size_rsp;
                    bool strm = ((net_ping_req_mesg_t *) (*mbuf)->data)->strm;

                    mbuf_used(mbuf, sizeof(net_ping_rsp_mesg_t) + size_rsp);

                    net_ping_rsp_mesg_t *ping_rsp = (net_ping_rsp_mesg_t *) (*mbuf)->data;

                    ping_rsp->meta = meta;

                    if (size_rsp) memset(ping_rsp->data, 0x5A, size_rsp);

                    const net_path_t path = {
                            .addr = addr,
                            .info = {
                                    .mode = NET_MODE_FLAG_CORE,
                                    .port = NET_CORE_PING_PORT,
                                    .type = NET_CORE_PING_RSP_TYPE
                            }
                    };

                    net_send_base(net, strm ? MAC_SEND_STRM : MAC_SEND_DGRM, path, mbuf);

                    net_peer_update(net, addr, meta);
                }
                break;

                case NET_CORE_PING_RSP_TYPE: {

                    if (net->ping && addr == net->ping->addr) {
                        net_time_t rx_time = rdio_util_get_tx_time(
                                phy_rdio(mac_phy(net_mac(net))),
                                sizeof(net_mesg_t) + (*mbuf)->used + MAC_PKT_OVERHEAD + 2/*phy: cell, flag*/
                        );

                        net->ping->rtt_usec = (ccrf_clock() - rx_time) - net->ping->rtt_usec;
                        net->ping->meta_locl = meta;
                        net->ping->meta_peer = ((net_ping_rsp_mesg_t *) (*mbuf)->data)->meta;
                        net->ping = NULL;

                        if (net->ping_task) {
                            xTaskNotify(net->ping_task, NET_NOTIFY_PING_DONE, eSetBits);
                            net->ping_task = NULL;
                        }

                        net_peer_update(net, addr, meta);
                    }

                }
                break;
            }

        break;
    }
}


static void net_peer_bcast(net_t net)
{
    const static net_info_t bcst_info = {
            .mode = NET_MODE_FLAG_CORE,
            .port = NET_CORE_PEER_PORT,
            .type = NET_CORE_PEER_TYPE
    };

    net_time_t now = net_time();

    net_time_t bcst_diff = now - net->peer_bcast_last;

    if (bcst_diff < NET_TIMER_INTERVAL / 2) return;

    if ((now - net->last.recv) < 3) return;
    if ((now - net->last.send) < 3) return;

    net->peer_bcast_last = now;

    mbuf_t mbuf = mbuf_make(sizeof(net_bcst_t), sizeof(net_bcst_t), NULL);

    net_bcst_t *bcst = (net_bcst_t *) mbuf->data;

    bcst->vers = __flsh_version;
    bcst->date = __flsh_date;
    bcst->time = now;

    net_send_base_bcst(net, bcst_info, &mbuf);

    mbuf_free(&mbuf);
}


static void net_peer_bcast_leave(net_t net, net_addr_t addr)
{
    const static net_info_t info = {
        .mode = NET_MODE_FLAG_CORE,
        .port = NET_CORE_PEER_PORT,
        .type = NET_CORE_PEER_LEAVE_TYPE
    };

    mbuf_t mbuf = mbuf_alloc(sizeof(net_addr_t), (u8 *) &addr);

    net_send_base_bcst(net, info, &mbuf);

    mbuf_free(&mbuf);
}


static net_peer_t *net_peer_get(net_t net, net_addr_t addr, bool add)
{
    net_peer_t *peer, *temp;

    list_for_each_entry_safe(peer, temp, &net->peer, item) {
        if (peer->info.peer == addr) {
            return peer;
        }
    }

    if (!add) return NULL;

    peer = pvPortMalloc(sizeof(net_peer_t));

    memset(peer, 0, sizeof(*peer));

    peer->info.peer = addr;
    peer->info.last = net_time();

    list_add(&peer->item, &net->peer);

    net_evnt_peer(net, addr, NET_EVENT_PEER_SET);

    return peer;
}


static net_peer_t *net_peer_update(net_t net, net_addr_t addr, pkt_meta_t meta)
{
    net_peer_t *peer = net_peer_get(net, addr, true);
    peer->info.last = net_time();
    peer->info.meta = meta;
    return peer;
}


static void net_peer_expire(net_t net, net_time_t time)
{
    net_peer_t *peer, *temp;

    list_for_each_entry_safe(peer, temp, &net->peer, item) {

        if (peer->info.peer && (!time || (time - peer->info.last) >= NET_PEER_EXPIRE_TIME)) {
            net_peer_remove(net, peer, NET_EVENT_PEER_EXP);
        }
    }
}

static bool net_peer_delete(net_t net, net_addr_t addr)
{
    net_peer_t *peer, *temp;

    if (addr == NET_ADDR_BCST || addr == NET_ADDR_INVL || addr == net_addr(net)) return false;

    list_for_each_entry_safe(peer, temp, &net->peer, item) {

        if (peer->info.peer == addr) {
            net_peer_remove(net, peer, NET_EVENT_PEER_NONE);
            return true;
        }
    }

    return false;
}

static void net_peer_remove(net_t net, net_peer_t *peer, net_event_peer_action_t action)
{
    if (peer) {
        net_addr_t peer_addr = peer->info.peer;

        peer->info.peer = NET_ADDR_NONE;
        peer->info.last = NET_ADDR_NONE;

        list_del(&peer->item);
        vPortFree(peer);

        if (action != NET_EVENT_PEER_NONE) net_evnt_peer(net, peer_addr, action);
    }
}


static void net_timer(TimerHandle_t timer)
{
    static bool first = true;

    if (first) {
        first = false;
        xTimerStop(timer, 0);
        vTaskDelay(pdMS_TO_TICKS(RNGA_ReadEntropy(RNG) % (1000 * NET_TIMER_INTERVAL + 1)));
        xTimerReset(timer, portMAX_DELAY);
    }

    net_t net = pvTimerGetTimerID(timer);
    net_time_t now = net_time();

    net_peer_expire(net, now);
    net_peer_bcast(net);
}
