#pragma once

#include <ccrf/ccrf.h>


#define PHY_FRAME_SIZE_MAX  123


typedef u8 phy_cell_t;

typedef struct __packed {
    s8 rssi;
    u8 lqi;

} pkt_meta_t;

typedef void (* phy_sync_t)(chan_id_t chan);
