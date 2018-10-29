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
    net_addr_t addr;
    net_addr_t peer;
    net_time_t last;
    pkt_meta_t meta;

} net_peer_info_t;

typedef struct {
    net_peer_info_t info;
    net_peer_list_t peer;
    net_peer_list_t item;

} net_peer_t;

typedef struct {
    struct list_head __list;
    net_addr_t addr;
    net_size_t size;
    u8 data[];

} net_trxn_t;

typedef struct list_head net_trxn_rslt_t;

typedef enum __packed {
    NET_EVENT_PEER

} net_event_t;

typedef enum __packed {
    NET_EVENT_PEER_SET,
    NET_EVENT_PEER_EXP

} net_event_peer_action_t;

typedef struct __packed {
    net_addr_t addr;
    net_event_peer_action_t action;

} net_event_peer_t;

typedef void (* net_recv_t)(net_t net, net_path_t path, net_addr_t dest, size_t size, u8 data[]);
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


net_t net_init(net_config_t *config);

net_time_t net_time(net_t net);
mac_t net_mac(net_t net);
net_addr_t net_addr(net_t net);
void net_stat(net_t net, net_stat_t *stat);
void net_peers(net_t net, net_peer_list_t **peer) __ccrf_code;
net_size_t net_peers_flat(net_t net, net_size_t extra, bool all, net_peer_info_t **list) __ccrf_code;
void net_sync(net_t net) __ccrf_code;

net_size_t net_send(net_t net, net_path_t path, net_size_t size, u8 data[]) __ccrf_code;
net_size_t net_mesg(net_t net, net_path_t path, net_size_t size, u8 data[]) __ccrf_code;
net_size_t net_resp(net_t net, net_path_t path, net_size_t size, u8 data[]) __ccrf_code;
void net_trxn(net_t net, net_path_t path, net_size_t size, u8 data[], net_time_t expiry, net_trxn_rslt_t *rslt) __ccrf_code;
void net_trxn_rslt_free(net_trxn_rslt_t *rslt) __ccrf_code;


static inline bool net_path_info_match(const net_info_t *const info_a, const net_info_t *const info_b)
{
    return info_a->port == info_b->port &&
           info_a->type == info_b->type &&
           (info_a->mode & NET_MODE_FLAG_CORE) == (info_b->mode & NET_MODE_FLAG_CORE);
}

static inline bool net_path_match(const net_path_t *const path_a, const net_path_t *const path_b)
{
    return path_a->addr == path_b->addr && net_path_info_match(&path_a->info, &path_b->info);
}
