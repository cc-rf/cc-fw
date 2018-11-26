#pragma once

#include <ccrf/ccrf.h>
#include <ccrf/mac.h>
#include <ccrf/list.h>


//#define NET_ADDR_NONE       ((net_addr_t) 0x00)

#define NET_MODE_BITS       2u
#define NET_PORT_BITS       10u
#define NET_TYPE_BITS       4u

#define NET_MODE_MASK           ((net_mode_t)((1u << NET_MODE_BITS) - 1u))
#define NET_PORT_MASK           ((net_port_t)((1u << NET_PORT_BITS) - 1u))
#define NET_TYPE_MASK           ((net_type_t)((1u << NET_TYPE_BITS) - 1u))

#define NET_MODE_FLAG_BCST      ((net_mode_t) (1u << 1u))
#define NET_MODE_FLAG_CORE      ((net_mode_t) (1u << 0u))

#define NET_ADDR_NONE           ((net_addr_t) 0u)
#define NET_ADDR_BCST           NET_ADDR_NONE
#define NET_ADDR_MASK           ((net_addr_t) 0xFFFFu)
#define NET_ADDR_BITS           16
#define NET_ADDR_INVL           NET_ADDR_MASK

#define NET_SEND_MAX            MAC_SEND_MAX


typedef struct net *net_t;

typedef u16 net_size_t;

typedef u8 net_mode_t;
typedef u16 net_port_t;
typedef u8 net_type_t;
typedef u32 net_time_t;
typedef mac_addr_t net_addr_t;

typedef struct __packed {
    net_mode_t mode : NET_MODE_BITS;
    net_port_t port : NET_PORT_BITS;
    net_type_t type : NET_TYPE_BITS;

} net_info_t;

typedef struct __packed {
    net_addr_t addr;
    net_info_t info;

} net_path_t;

typedef struct list_head net_peer_list_t;

typedef struct __packed {
    net_time_t vers;
    net_time_t date;
    net_time_t time;

} net_bcst_t;

typedef struct __packed {
    net_addr_t peer;
    pkt_meta_t meta;
    net_time_t last;
    net_bcst_t bcst;

} net_peer_info_t;

typedef struct {
    net_peer_info_t info;
    net_peer_list_t item;

} net_peer_t;

typedef struct {
    struct list_head __list;
    net_addr_t addr;
    mbuf_t mbuf;
    u8 data[];

} net_trxn_t;

typedef struct list_head net_trxn_rslt_t;

typedef enum __packed {
    NET_EVENT_PEER

} net_event_t;

typedef enum __packed {
    NET_EVENT_PEER_SET,
    NET_EVENT_PEER_EXP,
    NET_EVENT_PEER_OUT,
    NET_EVENT_PEER_UPD,

} net_event_peer_action_t;

typedef struct __packed {
    net_addr_t addr;
    net_event_peer_action_t action;

} net_event_peer_t;

typedef void (* net_recv_t)(net_t net, net_path_t path, net_addr_t dest, mbuf_t *mbuf);
typedef void (* net_evnt_t)(net_t net, net_event_t event, void *info);

typedef struct {

    struct {
        rdio_id_t rdid;
        phy_cell_t cell;

    } phy;

    struct {
        mac_addr_t addr;
        mac_recv_t recv; // param is mac instance

    } mac;

    struct {
        net_recv_t recv;
        net_evnt_t evnt;

    } net;

} net_config_t;

typedef struct {
    u32 count;
    u32 bytes;
    u32 errors;

} net_stat_item_t;

typedef struct {
    net_stat_item_t rx;
    net_stat_item_t tx;

} net_stat_t;

typedef struct {
    net_addr_t addr;
    net_size_t tx_count;
    pkt_meta_t meta_locl;
    pkt_meta_t meta_peer;
    u64 rtt_usec;

} net_ping_t;


net_t net_init(net_config_t *config, bool *fail) __nonnull_all;
void net_down(net_t net) __nonnull_all;

net_time_t net_time(void) __ccrf_code;
mac_t net_mac(net_t net) __ccrf_code __nonnull_all;
net_addr_t net_addr(net_t net) __ccrf_code __nonnull_all;
net_addr_t net_addr_set(net_t net, net_addr_t orig, net_addr_t addr) __nonnull_all;
void net_stat(net_t net, net_stat_t *stat) __nonnull_all;
void net_peers(net_t net, net_peer_list_t **peer) __ccrf_code __nonnull_all;
void net_peers_wipe(net_t net) __nonnull_all;
net_size_t net_peers_flat(net_t net, net_size_t extra, net_peer_info_t **list) __ccrf_code __nonnull_all;
void net_sync(net_t net) __ccrf_code __nonnull_all;
net_size_t net_send(net_t net, net_path_t path, mbuf_t *mbuf) __ccrf_code __nonnull((1));

net_size_t net_mesg(net_t net, net_path_t path, mbuf_t *mbuf) __ccrf_code __nonnull((1));
net_size_t net_resp(net_t net, net_path_t path, mbuf_t *mbuf) __ccrf_code __nonnull((1));
size_t net_trxn(net_t net, net_path_t path, mbuf_t *mbuf, net_time_t expiry, net_trxn_rslt_t *rslt) __ccrf_code  __nonnull((1,5));
void net_trxn_rslt_free(net_trxn_rslt_t *rslt) __ccrf_code;

bool net_ping(net_t net, net_addr_t addr, bool strm, net_size_t size, net_size_t size_resp, net_time_t timeout,
              net_ping_t *rslt) __nonnull_all;


static inline __nonnull_all bool net_path_info_match(const net_info_t *const info_a, const net_info_t *const info_b)
{
    return info_a->port == info_b->port &&
           info_a->type == info_b->type &&
           (info_a->mode & NET_MODE_FLAG_CORE) == (info_b->mode & NET_MODE_FLAG_CORE);
}

static inline __nonnull_all bool net_path_match(const net_path_t *const path_a, const net_path_t *const path_b)
{
    return path_a->addr == path_b->addr && net_path_info_match(&path_a->info, &path_b->info);
}
