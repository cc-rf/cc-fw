#pragma once

#include <ccrf/ccrf.h>
#include <ccrf/phy.h>

#define MAC_PKT_OVERHEAD    8
#define MAC_PKT_SIZE_MAX    ((mac_size_t)(PHY_FRAME_SIZE_MAX - MAC_PKT_OVERHEAD))

#define MAC_ADDR_BCST       ((mac_addr_t) 0u)

#define MAC_FLAG_0          ((mac_flag_t) 0x20u)
#define MAC_FLAG_1          ((mac_flag_t) 0x40u)
#define MAC_FLAG_2          ((mac_flag_t) 0x80u)
#define MAC_FLAG_MASK       ((mac_flag_t) (MAC_FLAG_0 | MAC_FLAG_1 | MAC_FLAG_2))


typedef struct mac *mac_t;

typedef u16 mac_addr_t;
typedef u16 mac_size_t;
typedef u8 mac_flag_t;

typedef void (* mac_recv_t)(mac_t mac, mac_flag_t flag, mac_addr_t peer, mac_addr_t dest, mac_size_t size, u8 data[], pkt_meta_t meta);

typedef enum __packed {
    MAC_SEND_DGRM,
    MAC_SEND_MESG,
    MAC_SEND_TRXN,
    MAC_SEND_STRM

} mac_send_t;

typedef struct __packed {
    rdio_id_t rdid;
    phy_cell_t cell;
    mac_addr_t addr;
    mac_recv_t recv;

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
pkt_meta_t mac_meta(mac_t mac);

mac_size_t mac_send(mac_t mac, mac_send_t type, mac_flag_t flags, mac_addr_t dest, mac_size_t size, u8 data[], bool wait);
