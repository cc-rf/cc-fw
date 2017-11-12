#pragma once

#include <ccrf/ccrf.h>
#include <ccrf/phy.h>

#define MAC_PKT_OVERHEAD    8
#define MAC_PKT_SIZE_MAX    ((mac_size_t)(PHY_FRAME_SIZE_MAX - MAC_PKT_OVERHEAD))


typedef struct mac *mac_t;

typedef u16 mac_addr_t;
typedef u16 mac_size_t;

typedef void (* mac_recv_t)(mac_t mac, mac_addr_t peer, mac_addr_t dest, mac_size_t size, u8 data[], pkt_meta_t meta);

typedef enum __packed {
    MAC_SEND_DGRM,
    MAC_SEND_MESG,
    MAC_SEND_TRXN,
    MAC_SEND_STRM

} mac_send_t;

typedef struct __packed {
    rdio_id_t rdid;
    bool boss;
    mac_addr_t addr;
    phy_cell_t cell;
    mac_recv_t recv;
    phy_sync_t sync;

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

mac_t mac_init(mac_config_t *config);

phy_t mac_phy(mac_t mac);
mac_addr_t mac_addr(mac_t mac);
void mac_stat(mac_t mac, mac_stat_t *stat);

mac_size_t mac_send(mac_t mac, mac_send_t type, mac_addr_t dest, mac_size_t size, u8 data[], bool wait);
