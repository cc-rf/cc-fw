#include <ccrf/net.h>
#include <ccrf/list.h>

#include <phy/phy.h>

#include "sys/trace.h"
#include "sys/local.h"

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
#define NET_PEER_BEACON_TIME    pdMS_TO_TICKS(10000)
#define NET_PEER_EXPIRE_TIME    (NET_PEER_BEACON_TIME * 3 + pdMS_TO_TICKS(1000))
#define NET_ASGN_BASE           32u

#define NET_NOTIFY_TRXN_DONE    (1u << 18u)

#define NET_MAC_FLAG_NETLAYER   MAC_FLAG_0

#define NET_BOSS_IO_PORT        ((net_port_t) 1u)
#define NET_BOSS_IO_TYPE_ASGN   ((net_type_t) 1u)

#define NET_EVENT_PORT          ((net_port_t) 2u)
#define NET_EVENT_TYPE_JOIN     ((net_type_t) 1u)
#define NET_EVENT_TYPE_HERE     ((net_type_t) 2u)


typedef struct __packed {
    net_node_t node;
    net_path_t dest;
    u8 data[];

} net_mesg_t;

typedef struct __packed txni {
    net_path_t path;
    struct list_head *trxn;
    TaskHandle_t task;
    struct list_head list;

} net_txni_t;


struct __packed net {
    net_node_t node;
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

    TimerHandle_t peer_timer;
    StaticTimer_t peer_timer_static;

    struct list_head txni;
};


static net_mesg_t *net_mesg_init(net_t net, net_path_t path, net_size_t size, u8 data[]);
static void net_mesg_free(net_mesg_t **mesg);
static mac_addr_t net_node_dest_mac(net_t net, net_node_t node);
static net_peer_t *net_peer_from_node(net_t net, net_node_t node);
static net_peer_t *net_peer_from_mac(net_t net, mac_addr_t addr);
static void net_trxn_resp(net_t net, net_node_t node, net_txni_t *txni, net_size_t size, u8 data[]);
static net_size_t net_send_base(net_t net, mac_addr_t dest, net_size_t size, net_mesg_t *mesg);
static net_size_t net_send_base_dgrm(net_t net, mac_addr_t dest, net_size_t size, net_mesg_t *mesg);
static net_size_t net_send_base_bcst(net_t net, net_size_t size, net_mesg_t *mesg);
static void net_evnt_assoc(net_t net, net_node_t node);
static void net_evnt_peer(net_t net, mac_addr_t addr, net_node_t node, net_event_peer_action_t action);
static void net_mac_recv(mac_t mac, mac_flag_t flag, mac_addr_t peer, mac_addr_t dest, mac_size_t size, u8 data[], pkt_meta_t meta);
static void net_core_recv_boss(net_t net, mac_addr_t addr, net_size_t size, net_mesg_t *mesg);
static void net_core_recv(net_t net, mac_addr_t addr, net_size_t size, net_mesg_t *mesg);
static void net_core_event_join(net_t net, mac_addr_t addr, net_mesg_t *mesg);
static void peer_timer(TimerHandle_t timer);


static struct net nets[CCRF_CONFIG_RDIO_COUNT];


net_t net_init(net_config_t *config)
{
    net_t net = &nets[config->phy.rdid];

    memset(net, 0, sizeof(struct net));

    net->node = config->phy.boss ? NET_NODE_BOSS : NET_NODE_NONE;
    net->recv = config->net.recv;
    net->evnt = config->net.evnt;
    
    net->boss.boss = config->phy.boss;
    net->boss.addr = 0;

    //INIT_LIST_HEAD(&net->peers);
    
    for (net_node_t peer = 0; peer < NET_PEER_MAX; ++peer) {
        net->peer[peer].addr.base = 0;
        net->peer[peer].addr.node = NET_ASGN_BASE + peer;
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

    net->peer_timer = xTimerCreateStatic(
            "net.peer", net->boss.boss ? NET_PEER_EXPIRE_TIME : NET_PEER_BEACON_TIME,
            pdTRUE, net, peer_timer, &net->peer_timer_static
    );

    xTimerStart(net->peer_timer, 0);

    INIT_LIST_HEAD(&net->txni);

    goto _done;
    _fail:

    if (net) {
        net = NULL;
    }

    _done:
    return net;
}


mac_t net_mac(net_t net)
{
    return net->mac.mac;
}


net_node_t net_node(net_t net)
{
    return net->node;
}


net_peer_t *net_peers(net_t net)
{
    // TODO: Maybe later only return assigned peers
    return net->peer;
}


static net_mesg_t *net_mesg_init(net_t net, net_path_t path, net_size_t size, u8 data[])
{
    net_mesg_t *mesg = pvPortMalloc(sizeof(*mesg) + size);

    mesg->node = net->node;
    mesg->dest = path;
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


static mac_addr_t net_node_dest_mac(net_t net, net_node_t node)
{
    if (!node)
        return MAC_ADDR_BCST;

    if (node == NET_NODE_BOSS)
        return net->boss.addr;
    else
        return net_peer_from_node(net, node)->addr.base;
}


static net_peer_t *net_peer_from_node(net_t net, net_node_t node)
{
    if (node < NET_ASGN_BASE || node >= (NET_ASGN_BASE + NET_PEER_MAX))
        return NULL;
    
    return &net->peer[node - NET_ASGN_BASE];
}


static net_peer_t *net_peer_from_mac(net_t net, mac_addr_t addr)
{
    net_node_t peer, free = NET_PEER_MAX;

    for (peer = 0; peer < NET_PEER_MAX; ++peer) {
        if (net->peer[peer].addr.base == addr) {
            break;
        } else if ((free == NET_PEER_MAX) && !net->peer[peer].addr.base) {
            free = peer;
        }
    }

    if (peer == NET_PEER_MAX) {
        if (free >= NET_PEER_MAX) return NULL;
        peer = free;
    }

    return &net->peer[peer];
}


void net_sync(net_t net)
{
    if (!net->boss.boss && !net->node) {
        net_mesg_t req = {
                .node = NET_NODE_NONE,
                .dest = {
                        .node = NET_NODE_BOSS,
                        .info = {
                                .mode = NET_MODE_FLAG_CORE,
                                .port = NET_BOSS_IO_PORT,
                                .type = NET_BOSS_IO_TYPE_ASGN
                        }
                }
        };

        if (net_send_base_bcst(net, 0, &req)) {
            net_trace_verbose("join req sent");
        } else {
            net_trace_warn("join req not sent");
        }
    }
}


net_size_t net_send(net_t net, bool trxn_repl, net_path_t path, net_size_t size, u8 data[])
{
    if (!net->node) {
        net_trace_warn("unable to send: not joined");
        return 0;
    }

    if (trxn_repl && !path.node) {
        net_trace_warn("unable to send trxn reply: node not set");
        return 0;
    }

    if (!net->boss.boss && !phy_sync(mac_phy(net->mac.mac))) {
        net->node = 0;
        net->boss.addr = 0;

        net_evnt_assoc(net, NET_NODE_NONE);

        net_trace_warn("unable to send: not joined (sync lost)");
        return 0;
    }

    path.info.mode = 0;
    
    net_mesg_t *mesg = net_mesg_init(net, path, size, data);
    mac_addr_t addr = net_node_dest_mac(net, path.node);


    if (trxn_repl)
        size = net_send_base(net, addr, size, mesg);
    else
        size = net_send_base_dgrm(net, addr, size, mesg);

    net_mesg_free(&mesg);

    return size;
}


void net_trxn(net_t net, net_path_t path, net_size_t size, u8 data[], net_time_t expiry, net_trxn_rslt_t *rslt)
{
    // TODO: Check that this is NOT receive queue task. Will deadlock.
    if (!expiry || expiry >= portMAX_DELAY || !rslt) return;

    INIT_LIST_HEAD(rslt);

    if (!net->node) {
        net_trace_warn("unable to send trxn: not joined");
        return;
    }

    if (!net->boss.boss && !phy_sync(mac_phy(net->mac.mac))) {
        net->node = 0;
        net->boss.addr = 0;

        net_evnt_assoc(net, NET_NODE_NONE);

        net_trace_warn("unable to send: not joined (sync lost)");
        return;
    }

    path.info.mode = 0;

    net_mesg_t *mesg = net_mesg_init(net, path, size, data);
    mac_addr_t addr = net_node_dest_mac(net, path.node);


    net_txni_t txni = {
            .path = path,
            .trxn = rslt,
            .task = xTaskGetCurrentTaskHandle(),
            .list = LIST_HEAD_INIT(txni.list)
    };

    list_add_tail(&txni.list, &net->txni);

    if (!net_send_base(net, addr, size, mesg)) {
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


static void net_trxn_resp(net_t net, net_node_t node, net_txni_t *txni, net_size_t size, u8 data[])
{
    // TODO: Is a race condition possible here?

    net_trxn_t *trxn = pvPortMalloc(sizeof(*trxn) + size);
    trxn->node = node;
    trxn->size = size;

    if (size) {
        memcpy(trxn->data, data, size);
    }

    list_add_tail(&trxn->__list, txni->trxn);

    if (txni->path.node) {
        xTaskNotify(txni->task, NET_NOTIFY_TRXN_DONE, eSetBits);
    }
}



static net_size_t net_send_base(net_t net, mac_addr_t dest, net_size_t size, net_mesg_t *mesg)
{
    if (dest == MAC_ADDR_BCST) {
        return net_send_base_bcst(net, size, mesg);
    } else {
        size += sizeof(net_mesg_t);
        mesg->dest.info.mode &= (net_mode_t)~NET_MODE_FLAG_BCST;
        return (net_size_t) mac_send(net->mac.mac, MAC_SEND_MESG, NET_MAC_FLAG_NETLAYER, dest, size, (u8 *) mesg, false/*true?*/);    }
}


static net_size_t net_send_base_dgrm(net_t net, mac_addr_t dest, net_size_t size, net_mesg_t *mesg)
{
    if (dest == MAC_ADDR_BCST) {
        return net_send_base(net, dest, size, mesg);
    } else {
        size += sizeof(net_mesg_t);
        mesg->dest.info.mode &= (net_mode_t)~NET_MODE_FLAG_BCST;
        return (net_size_t) mac_send(net->mac.mac, MAC_SEND_DGRM, NET_MAC_FLAG_NETLAYER, dest, size, (u8 *) mesg, false);
    }
}


static net_size_t net_send_base_bcst(net_t net, net_size_t size, net_mesg_t *mesg)
{
    size += sizeof(net_mesg_t);
    mesg->dest.info.mode |= NET_MODE_FLAG_BCST;
    return (net_size_t) mac_send(net->mac.mac, MAC_SEND_DGRM, NET_MAC_FLAG_NETLAYER, MAC_ADDR_BCST, size, (u8 *) mesg, false);
}


static void net_evnt_assoc(net_t net, net_node_t node)
{
    net_event_assoc_t event = {
            .node = node
    };

    return net->evnt(net, NET_EVENT_ASSOC, &event);
}


static void net_evnt_peer(net_t net, mac_addr_t addr, net_node_t node, net_event_peer_action_t action)
{
    net_event_peer_t event = {
            .addr = addr,
            .node = node,
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
        return;
    }

    if (!(mesg->dest.info.mode & NET_MODE_FLAG_CORE)) {
        net_trace_verbose(
                "net: %04X.%02X->%04X.%02X %u:%u:%u",
                peer, mesg->node, dest, mesg->dest.node,
                mesg->dest.info.mode, mesg->dest.info.port, mesg->dest.info.type
        );
    }

    if (dest && (dest != net->mac.addr))
        return;

    if (!dest)
        mesg->dest.info.mode |= NET_MODE_FLAG_BCST;

    if (mesg->dest.node && net->node && (mesg->dest.node != net->node)) {
        // special case: reassingment from boss (which is a unicast dgram)
        if (mesg->node != NET_NODE_BOSS ||
            !(mesg->dest.info.mode & NET_MODE_FLAG_CORE) ||
            mesg->dest.info.port != NET_BOSS_IO_PORT ||
            mesg->dest.info.type != NET_BOSS_IO_TYPE_ASGN) {

            return;
        }
    }

    if (mesg->dest.info.mode & NET_MODE_FLAG_CORE) {
        if (net->boss.boss) {
            return net_core_recv_boss(net, peer, size - sizeof(net_mesg_t), mesg);
        } else if (mesg->node) {
            return net_core_recv(net, peer, size - sizeof(net_mesg_t), mesg);
        }
    } else if (mesg->node) {

        if (mesg->node != NET_NODE_BOSS) {
            net_peer_t *net_peer = net_peer_from_node(net, mesg->node);

            if (!net_peer || peer != net_peer->addr.base) {
                net_core_event_join(net, peer, mesg);

                net_peer = net_peer_from_node(net, mesg->node);

                if (!net_peer || peer != net_peer->addr.base) {
                    net_trace_warn("unable to recv: no peer entry for %04X.%02X", peer, mesg->node);
                    return;
                }
            }
        }

        net_txni_t *txni;

        list_for_each_entry(txni, &net->txni, list) {
            const bool match = net_path_info_match_port_type(&txni->path.info, &mesg->dest.info);

            if (match && (!txni->path.node || (txni->path.node == mesg->node)))
                return net_trxn_resp(net, mesg->node, txni, size - sizeof(net_mesg_t), mesg->data);
        }

        net_path_t addr = {
                .node = mesg->node,
                .info = mesg->dest.info
        };

        return net->recv(net, addr, size - sizeof(net_mesg_t), mesg->data);
    }
}


static void net_core_recv_boss(net_t net, mac_addr_t addr, net_size_t size, net_mesg_t *mesg)
{
    switch (mesg->dest.info.port) {

        case NET_BOSS_IO_PORT:
            switch (mesg->dest.info.type) {

                case NET_BOSS_IO_TYPE_ASGN: {

                    if (mesg->node) // doesn't make sense here
                        return;

                    // join request

                    net_peer_t *peer = net_peer_from_mac(net, addr);
                    if (!peer) return;

                    if (!peer->addr.base) {
                        net_trace_verbose("new %04X.%02X", addr, peer->addr.node);
                    }

                    net_mesg_t resp = {
                            .node = net->node,
                            .dest = {
                                    .node = peer->addr.node,
                                    .info = {
                                            .mode = NET_MODE_FLAG_CORE,
                                            .port = NET_BOSS_IO_PORT,
                                            .type = NET_BOSS_IO_TYPE_ASGN
                                    }
                            }
                    };

                    if (net_send_base_dgrm(net, addr, 0, &resp)) {
                        peer->addr.base = addr;
                        peer->last = xTaskGetTickCount();

                        net_evnt_peer(net, peer->addr.base, peer->addr.node, NET_EVENT_PEER_SET);

                        net_trace_debug("join %04X.%02X", addr, peer->addr.node);

                    } else {
                        net_evnt_peer(net, addr, peer->addr.node, NET_EVENT_PEER_REM);

                        net_trace_warn("join %04X.%02X fail", addr, peer->addr.node);
                    }

                    return;
                }

                default:
                    return;
            }

            break;


        case NET_EVENT_PORT:
            switch (mesg->dest.info.type) {

                case NET_EVENT_TYPE_JOIN:
                case NET_EVENT_TYPE_HERE:
                    return net_core_event_join(net, addr, mesg);

                default:
                    return;
            }

            break;


        default:
            return;
    }
}


static void net_core_recv(net_t net, mac_addr_t addr, net_size_t size, net_mesg_t *mesg)
{

    switch (mesg->dest.info.port) {

        case NET_BOSS_IO_PORT:
            switch (mesg->dest.info.type) {

                case NET_BOSS_IO_TYPE_ASGN:
                    if (!mesg->dest.node) return;

                    if (net->node) {
                        net_trace_debug("joining: %04X.[%02X->%02X]", net->mac.addr, net->node, mesg->dest.node);
                    } else {
                        net_trace_debug("joining: %04X.%02X", net->mac.addr, mesg->dest.node);
                    }

                    net->node = mesg->dest.node;

                    if (!net->boss.addr) {
                        net->boss.addr = addr;
                        net_trace_verbose("boss found: %04X", addr);
                    }

                    net_mesg_t announce = {
                            .node = net->node,
                            .dest = {
                                    .node = NET_NODE_NONE,
                                    .info = {
                                            .mode = NET_MODE_FLAG_CORE,
                                            .port = NET_EVENT_PORT,
                                            .type = NET_EVENT_TYPE_JOIN
                                    }
                            }
                    };

                    if (net_send_base_bcst(net, 0, &announce)) {
                        net_evnt_assoc(net, net->node);
                        net_trace_debug("join as %04X.%02X", net->mac.addr, net->node);

                    } else {
                        net->node = 0;
                        net_evnt_assoc(net, 0);
                        net_trace_warn("join as %04X.%02X fail", net->mac.addr, net->node);
                    }

                    break;

                default:
                    return;
            }

            break;

        case NET_EVENT_PORT:
            switch (mesg->dest.info.type) {

                case NET_EVENT_TYPE_JOIN:
                case NET_EVENT_TYPE_HERE:
                    return net_core_event_join(net, addr, mesg);

                default:
                    return;
            }

            break;

        default:
            return;
    }
}


static void net_core_event_join(net_t net, mac_addr_t addr, net_mesg_t *mesg)
{
    net_trace_verbose("announce from %04X.%02X", addr, mesg->node);

    if (mesg->node == net->node) {
        net_trace_debug(
                "announce %04X.%02X but this node is %04X.%02X: ignore",
                addr, mesg->node,
                net->mac.addr, net->node
        );

        return;
    }

    net_peer_t *peer = net_peer_from_mac(net, addr);
    if (!peer) return;

    if (peer->addr.base == addr) {
        if (peer->addr.node != mesg->node) {
            net_evnt_peer(net, addr, peer->addr.node, NET_EVENT_PEER_REM);

            peer->addr.base = 0;
            peer = NULL;

            net_trace_debug("mismatch %04X.[%02X != %02X]", addr, mesg->node, peer->addr.node);
        }
    } else {
        peer = NULL;
    }

    if (!peer) {
        peer = net_peer_from_node(net, mesg->node);
        if (!peer) return;

        if (peer->addr.base && peer->addr.base != addr) {
            if (!net->boss.boss) {
                if (mesg->dest.info.port == NET_EVENT_PORT && mesg->dest.info.type == NET_EVENT_TYPE_JOIN) {

                    net_evnt_peer(net, peer->addr.base, peer->addr.node, NET_EVENT_PEER_REM);

                    peer->addr.base = addr;
                    peer->last = xTaskGetTickCount();

                    net_evnt_peer(net, peer->addr.base, peer->addr.node, NET_EVENT_PEER_SET);

                    net_trace_verbose(
                            "update %04X.[%02X->%02X]: owned by %04X (overwrite)",
                            addr, peer->addr.node, mesg->node, peer->addr.base
                    );


                } else {
                    // TODO: What if first join was missed and old node id is stale??
                    
                    net_trace_verbose(
                            "update %04X.[%02X->%02X]: owned by %04X (skip)",
                            addr, peer->addr.node, mesg->node, peer->addr.base
                    );
                }
                
                return;
            }

            net_node_t node_prev = peer->addr.node;

            // NOTE: This assert fails sometimes. TODO: Debug.
            if (node_prev == mesg->node) {
                net_trace_debug(
                        "node_prev=%02X mesg->node=%02X peer_addr_base=%04X addr=%04X",
                        node_prev, mesg->node, peer->addr.base, addr
                );
            }

            assert(node_prev == mesg->node);

            peer = net_peer_from_mac(net, addr);
            if (!peer) {
                net_trace_debug("reassign: peer pointer null!");
            }

            net_evnt_peer(net, addr, node_prev, NET_EVENT_PEER_REM);

            net_trace_debug(
                    "reassign %04X.[%02X->%02X]: %02X was taken", addr, mesg->node, peer->addr.node, node_prev
            );

            net_mesg_t asgn = {
                    .node = net->node,
                    .dest = {
                            .node = peer->addr.node,
                            .info = {
                                    .mode = NET_MODE_FLAG_CORE,
                                    .port = NET_BOSS_IO_PORT,
                                    .type = NET_BOSS_IO_TYPE_ASGN
                            }
                    }
            };

            net_evnt_peer(net, peer->addr.base, peer->addr.node, NET_EVENT_PEER_REM);

            if (net_send_base_dgrm(net, addr, 0, &asgn)) {
                peer->addr.base = addr;
                peer->last = xTaskGetTickCount();

                net_evnt_peer(net, addr, peer->addr.node, NET_EVENT_PEER_SET);

                net_trace_verbose("reassign %04X.[%02X->%02X]", addr, node_prev, peer->addr.node);
            } else {
                net_trace_verbose("reassign %04X.[%02X->%02X] fail", addr, node_prev, peer->addr.node);
            }

            return;

        } else {
            // TODO: When boss, need to do anything else here? Hmm.
            //      Gets here on rx after expiry.
            net_trace_debug("add %04X.%02X", addr, peer->addr.node);
        }

        peer->addr.base = addr;

        net_evnt_peer(net, addr, peer->addr.node, NET_EVENT_PEER_SET);
    }

    peer->last = xTaskGetTickCount();
}

static void peer_timer(TimerHandle_t timer)
{
    net_t net = pvTimerGetTimerID(timer);
    TickType_t now = xTaskGetTickCount();
    net_peer_t *peer;

    for (net_node_t pi = 0; pi < NET_PEER_MAX; ++pi) {

        peer = &net->peer[pi];

        if (peer->addr.base && (now - peer->last) >= NET_PEER_EXPIRE_TIME) {
            net_evnt_peer(net, peer->addr.base, peer->addr.node, NET_EVENT_PEER_REM);

            net_trace_debug("expire %04X.%02X (%lu -> %lu)", peer->addr.base, peer->addr.node, peer->last, now);

            peer->addr.base = 0;
            peer->last = 0;
        }
    }

    if (!net->boss.boss && net->node) {
        if (!phy_sync(mac_phy(net->mac.mac))) {
            net->node = NET_NODE_NONE;
            net->boss.addr = 0;

            net_evnt_assoc(net, 0x00);

            net_trace_debug("leave %04X.%02X", net->mac.addr, net->node);
        } else {
            net_trace_verbose("announce %04X.%02X", net->mac.addr, net->node);

            net_mesg_t announce = {
                    .node = net->node,
                    .dest = {
                            .node = NET_NODE_NONE,
                            .info = {
                                    .mode = NET_MODE_FLAG_CORE,
                                    .port = NET_EVENT_PORT,
                                    .type = NET_EVENT_TYPE_HERE
                            }
                    }
            };

            net_send_base_bcst(net, 0, &announce);
        }
    }
}
