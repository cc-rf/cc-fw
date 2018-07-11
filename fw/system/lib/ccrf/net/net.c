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

#define NET_NOTIFY_TRXN_DONE    (1u << 18u)

#define NET_MAC_FLAG_NETLAYER   MAC_FLAG_0

//#define NET_PATH_CMD_PORT          ((net_port_t) 0xBEu)
//#define NET_PATH_CMD_TYPE_PING_REQ ((net_type_t) 0x01u)
//#define NET_PATH_CMD_TYPE_PING_RSP ((net_type_t) 0x02u)

typedef struct __packed {
    net_info_t info;
    u8 data[];

} net_mesg_t;

typedef struct __packed txni {
    net_path_t path;
    struct list_head *trxn;
    TaskHandle_t task;
    struct list_head list;

} net_txni_t;


struct __packed net {
    net_recv_t recv;
    net_evnt_t evnt;

    struct {
        mac_t mac;
        mac_addr_t addr;
        mac_recv_t recv;

    } mac;
    
    struct {
        mac_addr_t addr;
        bool boss;

    } boss;
    
    net_peer_t peer[NET_PEER_MAX];

    TimerHandle_t timer;
    StaticTimer_t timer_static;

    struct list_head txni;

    net_stat_t stat;
};


static net_mesg_t *net_mesg_init(net_t net, net_path_t path, net_size_t size, u8 data[]);
static void net_mesg_free(net_mesg_t **mesg);
static void net_trxn_resp(net_t net, net_addr_t addr, net_txni_t *txni, net_size_t size, u8 data[]);
static net_size_t net_send_base(net_t net, mac_addr_t dest, net_size_t size, net_mesg_t *mesg);
static net_size_t net_send_base_dgrm(net_t net, mac_addr_t dest, net_size_t size, net_mesg_t *mesg);
static net_size_t net_send_base_bcst(net_t net, net_size_t size, net_mesg_t *mesg);
static void net_evnt_peer(net_t net, net_addr_t addr, net_event_peer_action_t action);
static void net_mac_recv(mac_t mac, mac_flag_t flag, mac_addr_t peer, mac_addr_t dest, mac_size_t size, u8 data[], pkt_meta_t meta);
static void net_core_recv(net_t net, net_size_t size, net_mesg_t *mesg);
static void net_peer_update(net_t net, net_addr_t addr);
static void net_timer(TimerHandle_t timer);


static struct net nets[CCRF_CONFIG_RDIO_COUNT];


net_t net_init(net_config_t *config)
{
    net_t net = &nets[config->phy.rdid];

    memset(net, 0, sizeof(struct net));

    net->recv = config->net.recv;
    net->evnt = config->net.evnt;
    
    net->boss.boss = config->phy.boss;
    net->boss.addr = 0;

    for (net_addr_t peer = 0; peer < NET_PEER_MAX; ++peer) {
        net->peer[peer].addr = NET_ADDR_NONE;
        net->peer[peer].last = 0;
    }

    net->mac.recv = config->mac.recv;

    mac_config_t mac_config = {
            .rdid = config->phy.rdid,
            .cell = config->phy.cell,
            .addr = config->mac.addr,
            .boss = config->phy.boss,
            .sync = config->phy.sync,
            .recv = net_mac_recv
    };

    if (!(net->mac.mac = mac_init(&mac_config))) {
        goto _fail;
    }

    net->mac.addr = mac_addr(net->mac.mac);

    net->timer = xTimerCreateStatic(
            "net.peer", pdSEC_TO_TICKS(NET_PEER_EXPIRE_TIME),
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


net_time_t net_time(net_t net)
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


/*net_addr_t net_boss(net_t net)
{
    return net->boss.addr;
}*/


void net_stat(net_t net, net_stat_t *stat)
{
    *stat = net->stat;
}


size_t net_peers(net_t net, net_peer_t **peer)
{
    net_size_t count = 0;

    *peer = NULL;

    for (net_size_t pi = 0; pi < NET_PEER_MAX; ++pi)
        if (net->peer[pi].addr) ++count;

    if (count) {
        *peer = pvPortMalloc(sizeof(net_peer_t) * count);

        for (net_size_t pi = 0, ppi = 0; pi < NET_PEER_MAX; ++pi) {
            if (net->peer[pi].addr)
                (*peer)[ppi++] = net->peer[pi];
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
}


net_size_t net_send(net_t net, bool trxn_repl, net_path_t path, net_size_t size, u8 data[])
{
    if (trxn_repl && !path.addr) {
        net_trace_warn("unable to send trxn reply: dest not set");
        return 0;
    }

    /*if (!net->boss.boss && !phy_sync(mac_phy(net->mac.mac))) {
        net->boss.addr = 0;
        net_evnt_assoc(net, NET_NODE_NONE);
        net_trace_warn("unable to send: not synced");
        return 0;
    }*/

    path.info.mode = 0;
    
    net_mesg_t *mesg = net_mesg_init(net, path, size, data);

    if (trxn_repl)
        size = net_send_base(net, path.addr, size, mesg);
    else
        size = net_send_base_dgrm(net, path.addr, size, mesg);

    net_mesg_free(&mesg);

    return size;
}


void net_trxn(net_t net, net_path_t path, net_size_t size, u8 data[], net_time_t expiry, net_trxn_rslt_t *rslt)
{
    if (!expiry || expiry >= portMAX_DELAY || !rslt) return;

    INIT_LIST_HEAD(rslt);

    // TODO: Check that this is not the RX task. Will deadlock!

    /*if (!net->boss.boss && !phy_sync(mac_phy(net->mac.mac))) {
        net->boss.addr = 0;

        net_evnt_assoc(net, NET_NODE_NONE);

        net_trace_warn("unable to send trxn: not synced");
        return;
    }*/

    path.info.mode = 0;

    net_mesg_t *mesg = net_mesg_init(net, path, size, data);

    net_txni_t txni = {
            .path = path,
            .trxn = rslt,
            .task = xTaskGetCurrentTaskHandle(),
            .list = LIST_HEAD_INIT(txni.list)
    };

    list_add_tail(&txni.list, &net->txni);

    if (!net_send_base(net, path.addr, size, mesg)) {
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
    } else {
        net->stat.tx.errors++;
    }
}


static net_size_t net_send_base(net_t net, mac_addr_t dest, net_size_t size, net_mesg_t *mesg)
{
    if (dest == MAC_ADDR_BCST)
        return net_send_base_bcst(net, size, mesg);

    size += sizeof(net_mesg_t);
    mesg->info.mode &= (net_mode_t)~NET_MODE_FLAG_BCST;

    mac_size_t sent = mac_send(net->mac.mac, MAC_SEND_MESG, NET_MAC_FLAG_NETLAYER, dest, size, (u8 *) mesg, true);

    stat_tx_add(net, size - sizeof(net_mesg_t), sent);

    if (sent) {
        // was acked, add to table
        net_peer_update(net, dest);
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

    stat_tx_add(net, size - sizeof(net_mesg_t), sent);

    return sent;
}


static net_size_t net_send_base_bcst(net_t net, net_size_t size, net_mesg_t *mesg)
{
    size += sizeof(net_mesg_t);
    mesg->info.mode |= NET_MODE_FLAG_BCST;

    mac_size_t sent = mac_send(net->mac.mac, MAC_SEND_DGRM, NET_MAC_FLAG_NETLAYER, MAC_ADDR_BCST, size, (u8 *) mesg, false);

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


static void net_mac_recv(mac_t mac, mac_flag_t flag, mac_addr_t peer, mac_addr_t dest, mac_size_t size, u8 data[], pkt_meta_t meta)
{
    net_t net = &nets[rdio_id(phy_rdio(mac_phy(mac)))]; assert(net);

    if (flag != NET_MAC_FLAG_NETLAYER) {
        // passthrough
        return net->mac.recv(mac, flag, peer, dest, size, data, meta);
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

    net_peer_update(net, peer);

    if (mesg->info.mode & NET_MODE_FLAG_CORE)
        return net_core_recv(net, size - sizeof(net_mesg_t), mesg);

    net->stat.rx.count++;
    net->stat.rx.bytes += size - sizeof(net_mesg_t);

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

    return net->recv(net, path, size - sizeof(net_mesg_t), mesg->data);
}


static void net_core_recv(net_t net, net_size_t size, net_mesg_t *mesg)
{
    /*switch (path.info.port) {
        case NET_PATH_CMD_PORT:
            switch (path.info.type) {
                case NET_PATH_CMD_TYPE_PING_REQ:
                    path.info.type = NET_PATH_CMD_TYPE_PING_RSP;
                    mesg->info
                    net_send_base_dgrm(net, path.addr, 0, mesg);
            }
    }*/
}


static void net_peer_update(net_t net, net_addr_t addr)
{
    net_addr_t free = NET_PEER_MAX;

    for (net_addr_t pi = 0; pi < NET_PEER_MAX; ++pi) {
        net_peer_t *peer = &net->peer[pi];

        if (peer->addr == addr) {
            peer->last = net_time(net);
            return;
        }

        if (free > pi && !peer->addr) free = pi;
    }

    if (free < NET_PEER_MAX) {
        net->peer[free].addr = addr;
        net->peer[free].last = net_time(net);
        net_evnt_peer(net, addr, NET_EVENT_PEER_SET);
    }
}


static void net_timer(TimerHandle_t timer)
{
    net_t net = pvTimerGetTimerID(timer);
    net_time_t now = net_time(net);
    net_peer_t *peer;

    for (net_addr_t pi = 0; pi < NET_PEER_MAX; ++pi) {

        peer = &net->peer[pi];

        if (peer->addr && (now - peer->last) >= NET_PEER_EXPIRE_TIME) {
            net_addr_t addr = peer->addr;

            net_trace_debug("expire %04X (%lu -> %lu)", peer->addr, peer->last, now);

            peer->addr = 0;
            peer->last = 0;

            net_evnt_peer(net, addr, NET_EVENT_PEER_EXP);
        }
    }
}
