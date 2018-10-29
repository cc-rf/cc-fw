#pragma once

#include <ccrf/ccrf.h>

/**
 * Packet overhead is 2 bytes, need 1 byte for size in FIFO,
 * and room for 2 status bytes on receive.
 */
#define PHY_FRAME_SIZE_MAX  123
#define PHY_CHAN_COUNT      25u
#define PHY_PWR_MAX         60

typedef struct phy *phy_t;

typedef u8 phy_cell_t;

typedef struct __packed {
    s8 rssi;
    u8 lqi;

} pkt_meta_t;

typedef struct __packed {
    u32 count;
    u32 bytes;
    u32 errors;

} phy_stat_item_t;

typedef struct __packed {
    phy_stat_item_t rx;
    phy_stat_item_t tx;

} phy_stat_t;

void phy_stat(phy_t phy, phy_stat_t *stat) __nonnull_all;
u32 phy_task_stack_usage(phy_t phy) __nonnull_all;
void phy_sync(phy_t phy, bool *resy) __nonnull((1));

chan_id_t phy_chan_real(phy_t phy, u32 *freq) __nonnull((1));
chan_id_t phy_chan(phy_t phy) __nonnull_all;
u32 phy_freq(phy_t phy, chan_id_t chan) __nonnull_all;
void phy_hops(phy_t phy, chan_id_t chan[]) __nonnull_all;

bool phy_boss(phy_t phy) __nonnull_all;
phy_cell_t phy_cell(phy_t phy) __nonnull_all;
bool phy_hgm(phy_t phy) __nonnull_all;
u8 phy_pwr(phy_t phy) __nonnull_all;

bool phy_diag_boss(phy_t phy, bool boss, bool nosync) __nonnull_all;
bool phy_diag_chan(phy_t phy, chan_id_t chan, u32 *freq) __nonnull((1));
bool phy_diag_cw(phy_t phy, bool cw) __nonnull_all;
bool phy_diag_pwr(phy_t phy, u8 pwr) __nonnull_all;
bool phy_diag_hgm(phy_t phy, bool hgm) __nonnull_all;
