#pragma once

#include <ccrf/ccrf.h>
#include <ccrf/phy.h>
#include <usr/mbuf.h>


#define MAC_PKT_OVERHEAD    8
#define MAC_PKT_SIZE_MAX    ((mac_size_t)(PHY_FRAME_SIZE_MAX - MAC_PKT_OVERHEAD))

#define MAC_ADDR_BCST       ((mac_addr_t) 0x00)

#define MAC_FLAG_0          ((mac_flag_t) 0x20)
#define MAC_FLAG_1          ((mac_flag_t) 0x40)
#define MAC_FLAG_2          ((mac_flag_t) 0x80)
#define MAC_FLAG_MASK       ((mac_flag_t) (MAC_FLAG_0 | MAC_FLAG_1 | MAC_FLAG_2))

#define MAC_SEND_MAX        ((mac_size_t) 0xFFFA)

typedef struct mac *mac_t;

typedef u16 mac_addr_t;
typedef u16 mac_size_t;
typedef u8 mac_flag_t;
typedef u8 mac_seqn_t;

typedef void (* mac_recv_t)(void *param, mac_flag_t flag, mac_addr_t peer, mac_addr_t dest, mac_seqn_t seqn, mbuf_t *mbuf, pkt_meta_t meta);

typedef enum __packed {
    MAC_SEND_DGRM,
    MAC_SEND_MESG,
    MAC_SEND_TRXN,
    MAC_SEND_STRM

} mac_send_t;

typedef struct {
    rdio_id_t rdid;
    phy_cell_t cell;
    mac_addr_t addr;
    mac_recv_t recv;
    void *recv_param;

} mac_config_t;

typedef struct __packed {
    u32 count;
    u32 bytes;
    u32 errors;

} mac_stat_item_t;

typedef struct __packed {
    mac_stat_item_t rx;
    mac_stat_item_t tx;

} mac_stat_t;

mac_t mac_init(mac_config_t *config, bool *fail) __nonnull_all;

phy_t mac_phy(mac_t mac) __ccrf_code __nonnull_all;
mac_addr_t mac_addr(mac_t mac) __ccrf_code __nonnull_all;
mac_addr_t mac_addr_set(mac_t mac, mac_addr_t orig, mac_addr_t addr) __nonnull_all;
void mac_stat(mac_t mac, mac_stat_t *stat) __nonnull_all;
u32 mac_task_rx_stack_usage(mac_t mac) __nonnull_all;
pkt_meta_t mac_meta(mac_t mac) __ccrf_code __nonnull_all;

mac_size_t mac_send(mac_t mac, mac_send_t type, mac_flag_t flags, mac_addr_t dest, mbuf_t mbuf, bool wait) __ccrf_code __nonnull((1));
