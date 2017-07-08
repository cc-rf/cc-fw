#pragma once

#include "rdio/rdio.h"
#include "phy/chan.h"


#define PHY_FRAME_SIZE_MAX  123

#define PHY_PKT_FLAG_IMMEDIATE  (0x04)  // send without tx-gap delay and put at front of queue
#define PHY_PKT_FLAG_BLOCK      (0x08)  // [local] return when packet is sent or fails
#define PHY_PKT_FLAG_USER_MASK  (0x0F)
#define PHY_PKT_FLAG_USER_SHIFT (0x04)


typedef struct phy *phy_t;

typedef u8 phy_cell_t;

typedef void (* phy_sync_t)(chan_id_t chan);
typedef bool (* phy_recv_t)(void *param, u8 flag, u8 size, u8 data[], s8 rssi, u8 lqi);

typedef struct __packed {
    rdio_id_t rdid;
    bool boss;
    phy_cell_t cell;
    phy_sync_t sync;
    phy_recv_t recv;
    void *recv_param;


} phy_config_t;


phy_t phy_init(phy_config_t *config);

rdio_t phy_rdio(phy_t phy);
bool phy_boss(phy_t phy);
phy_cell_t phy_cell(phy_t phy);

u32 phy_delay(u8 size);

bool phy_send(phy_t phy, u8 flag, u8 *data, u8 size);
