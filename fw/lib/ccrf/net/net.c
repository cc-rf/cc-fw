#include <ccrf/net.h>
#include <ccrf/list.h>

#include <phy/phy.h>

#include "sys/trace.h"
#include "sys/local.h"
#include "sys/clock.h"

#include <FreeRTOS.h>
#include <assert.h>
#include <task.h>
#include <queue.h>
#include <timers.h>


#define net_trace_info          ccrf_trace_info
#define net_trace_warn          ccrf_trace_warn
#define net_trace_error         ccrf_trace_error
#define net_trace_debug         ccrf_trace_debug
#define net_trace_verbose(...)  //ccrf_trace_verbose


#define NET_PEER_MAX            64u
#define NET_PEER_EXPIRE_TIME    600u
#define NET_TIMER_INTERVAL      10u

#define NET_NOTIFY_TRXN_DONE    (1u << 18u)

#define NET_MAC_FLAG_NETLAYER   MAC_FLAG_0

#define NET_CORE_PEER_PORT      1
#define NET_CORE_PEER_TYPE      2


typedef struct __packed {
    net_info_t info;
    u8 data[];

} net_mesg_t;

typedef struct txni {
    net_path_t path;
    struct list_head *trxn;
    TaskHandle_t task;
    struct list_head list;

} net_txni_t;


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
};


static net_mesg_t *net_mesg_init(net_t net, net_path_t path, net_size_t size, u8 data[]) __ccrf_code;
static void net_mesg_free(net_mesg_t **mesg) __ccrf_code;
static void net_trxn_resp(net_t net, net_addr_t addr, net_txni_t *txni, net_size_t size, u8 data[]) __ccrf_code;
static net_size_t net_send_base(net_t net, bool dgrm, net_path_t path, net_size_t size, u8 data[]) __ccrf_code;
static net_size_t net_send_base_mesg(net_t net, mac_addr_t dest, net_size_t size, net_mesg_t *mesg) __ccrf_code;
static net_size_t net_send_base_dgrm(net_t net, mac_addr_t dest, net_size_t size, net_mesg_t *mesg) __ccrf_code;
static net_size_t net_send_base_bcst(net_t net, net_size_t size, net_mesg_t *mesg) __ccrf_code;
static void net_evnt_peer(net_t net, net_addr_t addr, net_event_peer_action_t action);
static void net_mac_recv(net_t net, mac_flag_t flag, mac_addr_t peer, mac_addr_t dest, mac_size_t size, u8 data[], pkt_meta_t meta) __ccrf_code;
static void net_core_recv(net_t net, net_addr_t addr, net_size_t size, net_mesg_t *mesg) __ccrf_code;
static void net_peer_bcast(net_t net);
static net_peer_t *net_peer_get(net_t net, net_addr_t addr, bool add) __ccrf_code;
static void net_peer_update(net_t net, net_addr_t addr, pkt_meta_t meta) __ccrf_code;
static void net_peer_update_list(net_t net, net_addr_t addr, net_size_t count, net_peer_info_t *peers);
static void net_peer_expire(net_t net, net_time_t time);
static void net_timer(TimerHandle_t timer);


static struct net nets[CCRF_CONFIG_RDIO_COUNT] __ccrf_data;


net_t net_init(net_config_t *config)
{
    net_t net = &nets[config->phy.rdid];

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

    if (!(net->mac.mac = mac_init(&mac_config))) {
        goto _fail;
    }

    net->phy = mac_phy(net->mac.mac);

    net->mac.addr = mac_addr(net->mac.mac);

    net->timer = xTimerCreateStatic(
            "net.peer", pdSEC_TO_TICKS(NET_TIMER_INTERVAL),
            pdTRUE, net, net_timer, &net->timer_static
    );

    xTimerStart(net->timer, 0);

    INIT_LIST_HEAD(&net->txni);

    goto _done;
    _fail:

    if (net) {
        net = NULL;
    }

    _done:
    return net;
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


void net_stat(net_t net, net_stat_t *stat)
{
    *stat = net->stat;
}


void net_peers(net_t net, net_peer_list_t **peer)
{
    *peer = &net->peer;
}

net_size_t net_peers_flat(net_t net, net_size_t extra, bool all, net_peer_info_t **list)
{
    net_size_t count = 0, next = 0;
    net_peer_info_t *peers;
    net_peer_t *peer, *peeri;

    *list = NULL;

    list_for_each_entry(peer, &net->peer, item) {
        count++;

        if (all)
            list_for_each_entry(peeri, &peer->peer, item)
                count++;
    }

    *list = peers = pvPortMalloc(extra + count * sizeof(net_peer_info_t));

    if (extra) peers = (net_peer_info_t *) &((u8 *)peers)[extra];

    net_time_t now = net_time();

    list_for_each_entry(peer, &net->peer, item) {
        peers[next] = peer->info;
        peers[next++].last = now - peer->info.last;

        if (all)
            list_for_each_entry(peeri, &peer->peer, item) {
                peers[next] = peeri->info;
                peers[next++].last = peeri->info.last;
            }
    }

    return count;
}


static net_mesg_t *net_mesg_init(net_t net, net_path_t path, net_size_t size, u8 data[])
{
    net_mesg_t *mesg = pvPortMalloc(sizeof(*mesg) + size);

    mesg->info = path.info;
    if (size) memcpy(mesg->data, data, size);

    return mesg;
}


static void net_mesg_free(net_mesg_t **mesg)
{
    if (*mesg) {
        vPortFree(*mesg);
        *mesg = NULL;
    }
}


void net_sync(net_t net)
{
    phy_sync(net->phy, NULL);
}


net_size_t net_send(net_t net, net_path_t path, net_size_t size, u8 data[])
{
    return net_send_base(net, true, path, size, data);
}


net_size_t net_mesg(net_t net, net_path_t path, net_size_t size, u8 data[])
{
    if (!path.addr) {
        return net_send(net, path, size, data);
    }

    return net_send_base(net, false, path, size, data);
}


net_size_t net_resp(net_t net, net_path_t path, net_size_t size, u8 data[])
{
    return net_mesg(net, path, size, data);
}


net_size_t net_send_base(net_t net, bool dgrm, net_path_t path, net_size_t size, u8 data[])
{
    path.info.mode = 0;

    net_mesg_t *mesg = net_mesg_init(net, path, size, data);

    if (dgrm)
        size = net_send_base_dgrm(net, path.addr, size, mesg);
    else
        size = net_send_base_mesg(net, path.addr, size, mesg);

    net_mesg_free(&mesg);

    return size;
}


void net_trxn(net_t net, net_path_t path, net_size_t size, u8 data[], net_time_t expiry, net_trxn_rslt_t *rslt)
{
    if (!expiry || expiry >= portMAX_DELAY || !rslt) return;

    INIT_LIST_HEAD(rslt);

    // TODO: Check that this is not the RX task. Will deadlock!

    path.info.mode = 0;

    net_mesg_t *mesg = net_mesg_init(net, path, size, data);

    net_txni_t txni = {
            .path = path,
            .trxn = rslt,
            .task = xTaskGetCurrentTaskHandle(),
            .list = LIST_HEAD_INIT(txni.list)
    };

    list_add_tail(&txni.list, &net->txni);

    if (!net_send_base_mesg(net, path.addr, size, mesg)) {
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

    net_mesg_free(&mesg);
}


void net_trxn_rslt_free(net_trxn_rslt_t *rslt)
{
    if (rslt) {
        net_trxn_t *trxn, *trxn_t;

        list_for_each_entry_safe(trxn, trxn_t, rslt, __list) {
            list_del(&trxn->__list);
            vPortFree(trxn);
        }
    }
}


static void net_trxn_resp(net_t net, net_addr_t addr, net_txni_t *txni, net_size_t size, u8 data[])
{
    // TODO: Is a race condition possible here?

    net_trxn_t *trxn = pvPortMalloc(sizeof(*trxn) + size);
    trxn->addr = addr;
    trxn->size = size;

    if (size) {
        memcpy(trxn->data, data, size);
    }

    list_add_tail(&trxn->__list, txni->trxn);

    if (txni->path.addr) {
        xTaskNotify(txni->task, NET_NOTIFY_TRXN_DONE, eSetBits);
    }
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


static net_size_t net_send_base_mesg(net_t net, mac_addr_t dest, net_size_t size, net_mesg_t *mesg)
{
    if (dest == MAC_ADDR_BCST)
        return net_send_base_bcst(net, size, mesg);

    size += sizeof(net_mesg_t);
    mesg->info.mode &= (net_mode_t)~NET_MODE_FLAG_BCST;

    mac_size_t sent = mac_send(net->mac.mac, MAC_SEND_MESG, NET_MAC_FLAG_NETLAYER, dest, size, (u8 *) mesg, true);

    if (!(mesg->info.mode & NET_MODE_FLAG_CORE))
        stat_tx_add(net, size - sizeof(net_mesg_t), sent);

    if (sent) {
        // was acked, add to table
        net_peer_update(net, dest, mac_meta(net->mac.mac));
    }

    return sent;
}


static net_size_t net_send_base_dgrm(net_t net, mac_addr_t dest, net_size_t size, net_mesg_t *mesg)
{
    if (dest == MAC_ADDR_BCST)
        return net_send_base_bcst(net, size, mesg);

    size += sizeof(net_mesg_t);
    mesg->info.mode &= (net_mode_t)~NET_MODE_FLAG_BCST;

    mac_size_t sent = mac_send(net->mac.mac, MAC_SEND_DGRM, NET_MAC_FLAG_NETLAYER, dest, size, (u8 *) mesg, false);

    if (!(mesg->info.mode & NET_MODE_FLAG_CORE))
        stat_tx_add(net, size - sizeof(net_mesg_t), sent);

    return sent;
}


static net_size_t net_send_base_bcst(net_t net, net_size_t size, net_mesg_t *mesg)
{
    size += sizeof(net_mesg_t);
    mesg->info.mode |= NET_MODE_FLAG_BCST;

    mac_size_t sent = mac_send(net->mac.mac, MAC_SEND_DGRM, NET_MAC_FLAG_NETLAYER, MAC_ADDR_BCST, size, (u8 *) mesg, false);

    if (!(mesg->info.mode & NET_MODE_FLAG_CORE))
        stat_tx_add(net, size - sizeof(net_mesg_t), sent);

    return sent;
}


static void net_evnt_peer(net_t net, net_addr_t addr, net_event_peer_action_t action)
{
    net_event_peer_t event = {
            .addr = addr,
            .action = action
    };

    return net->evnt(net, NET_EVENT_PEER, &event);
}


static void net_mac_recv(net_t net, mac_flag_t flag, mac_addr_t peer, mac_addr_t dest, mac_size_t size, u8 data[], pkt_meta_t meta)
{
    if (flag != NET_MAC_FLAG_NETLAYER) {
        // passthrough
        return net->mac.recv(net->mac.mac, flag, peer, dest, size, data, meta);
    }

    net_mesg_t *mesg = (net_mesg_t *)data;

    if (!peer) {
        net_trace_warn("peer=0");
        net->stat.rx.errors++;
        return;
    }

    if (dest && (dest != net->mac.addr))
        return;

    if (!(mesg->info.mode & NET_MODE_FLAG_CORE)) {
        net_trace_verbose(
                "net: %04X->%04X %u:%u:%u",
                peer, net->mac.addr,
                mesg->info.mode, mesg->info.port, mesg->info.type
        );
    }

    if (!dest)
        mesg->info.mode |= NET_MODE_FLAG_BCST;

    net_peer_update(net, peer, meta);

    if (mesg->info.mode & NET_MODE_FLAG_CORE)
        return net_core_recv(net, peer, size - sizeof(net_mesg_t), mesg);

    net->stat.rx.count++;
    net->stat.rx.bytes += size - sizeof(net_mesg_t);
    net->last.recv = net_time();

    net_txni_t *txni;

    list_for_each_entry(txni, &net->txni, list) {
        const bool match = net_path_info_match(&txni->path.info, &mesg->info);

        if (match)
            return net_trxn_resp(net, peer, txni, size - sizeof(net_mesg_t), mesg->data);
    }

    net_path_t path = {
            .addr = peer,
            .info = mesg->info
    };

    return net->recv(net, path, dest, size - sizeof(net_mesg_t), mesg->data);
}


static void net_core_recv(net_t net, net_addr_t addr, net_size_t size, net_mesg_t *mesg)
{
    switch (mesg->info.port) {

        case NET_CORE_PEER_PORT:

            switch (mesg->info.type) {

                case NET_CORE_PEER_TYPE: {
                    net_size_t count = size / sizeof(net_peer_info_t);
                    if (count) net_peer_update_list(net, addr,  count, (net_peer_info_t *) mesg->data);
                    net_peer_bcast(net);
                }

            }
    }
}


static void net_peer_bcast(net_t net)
{
    net_time_t now = net_time();

    if ((now - net->peer_bcast_last) <= 5) return;
    if ((now - net->last.recv) < 5) return;
    if ((now - net->last.send) < 5) return;

    net->peer_bcast_last = now;

    net_mesg_t *send;
    net_peer_info_t *peers;
    net_size_t count = net_peers_flat(net, sizeof(net_mesg_t), false, &peers);

    send = (net_mesg_t *) peers;
    send->info.mode = NET_MODE_FLAG_CORE;
    send->info.port = NET_CORE_PEER_PORT;
    send->info.type = NET_CORE_PEER_TYPE;
    net_send_base_bcst(net, sizeof(net_mesg_t) + count * sizeof(net_peer_info_t), send);

    vPortFree(peers);
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

    peer->info.addr = net_addr(net);
    peer->info.peer = addr;
    peer->info.last = net_time();

    INIT_LIST_HEAD(&peer->peer);

    list_add(&peer->item, &net->peer);

    net_evnt_peer(net, addr, NET_EVENT_PEER_SET);

    return peer;
}


static void net_peer_update(net_t net, net_addr_t addr, pkt_meta_t meta)
{
    net_peer_t *peer = net_peer_get(net, addr, true);
    peer->info.last = net_time();
    peer->info.meta = meta;
}


static void net_peer_update_list(net_t net, net_addr_t addr, net_size_t count, net_peer_info_t *peers)
{
    net_peer_t *peer = net_peer_get(net, addr, true);
    net_time_t now = net_time();
    net_peer_t *peeri;
    bool found;

    peer->info.last = now;

    for (net_size_t pi = 0; pi < count; ++pi) {
        found = false;

        list_for_each_entry(peeri, &peer->peer, item) {
            if ((peeri->info.addr == peers[pi].addr) && (peeri->info.peer == peers[pi].peer)) {
                found = true;
                break;
            }
        }

        if (!found) {
            peeri = pvPortMalloc(sizeof(net_peer_t));

            INIT_LIST_HEAD(&peeri->peer);

            peeri->info = peers[pi];

            list_add(&peeri->item, &peer->peer);
        }

        peeri->info.last = now - peers[pi].last/*expected relative*/;
    }
}


static void net_peer_expire(net_t net, net_time_t time)
{
    net_peer_t *peer, *temp, *peeri, *tempi;

    list_for_each_entry_safe(peer, temp, &net->peer, item) {
        if (peer->info.addr && (!time || (time - peer->info.last) >= NET_PEER_EXPIRE_TIME)) {
            net_addr_t addr = peer->info.addr;

            peer->info.addr = NET_ADDR_NONE;
            peer->info.peer = NET_ADDR_NONE;
            peer->info.last = NET_ADDR_NONE;

            net_evnt_peer(net, addr, NET_EVENT_PEER_EXP);

            list_for_each_entry_safe(peeri, tempi, &peer->peer, item) {
                assert(list_empty(&peeri->peer));
                list_del(&peeri->item);
                vPortFree(peeri);
            }

            list_del(&peer->item);
            vPortFree(peer);
        }
    }
}


static void net_timer(TimerHandle_t timer)
{
    net_t net = pvTimerGetTimerID(timer);
    net_time_t now = net_time();
    net_peer_expire(net, now);
    net_peer_bcast(net);
}
