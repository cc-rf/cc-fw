#pragma once

#include <ccrf/phy.h>
#include "rdio/rdio.h"
#include "phy/chan.h"


#define PHY_PKT_FLAG_IMMEDIATE  ((u8) 0x02)  // Send without tx-gap delay (or CCA).
#define PHY_PKT_FLAG_BLOCK      ((u8) 0x01)  // Return when packet is sent or fails (Local).
#define PHY_PKT_FLAG_USER_MASK  ((u8) 0xFC)
#define PHY_PKT_FLAG_USER_0     ((u8) 0x04)
#define PHY_PKT_FLAG_USER_1     ((u8) 0x08)
#define PHY_PKT_FLAG_USER_2     ((u8) 0x10)
#define PHY_PKT_FLAG_USER_3     ((u8) 0x20)
#define PHY_PKT_FLAG_USER_4     ((u8) 0x40)
#define PHY_PKT_FLAG_USER_5     ((u8) 0x80)


typedef void (* phy_recv_t)(void *param, u8 flag, u8 size, u8 data[], pkt_meta_t meta);

typedef struct __packed {
    rdio_id_t rdid;
    phy_cell_t cell;
    phy_sync_t sync;
    phy_recv_t recv;
    void *recv_param;

} phy_config_t;


phy_t phy_init(phy_config_t *config);

rdio_t phy_rdio(phy_t phy);

u32 phy_delay(u8 size);

bool phy_send(phy_t phy, u8 flag, u8 *data, u8 size);
