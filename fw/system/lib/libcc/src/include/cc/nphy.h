#pragma once

#include <cc/common.h>

#define PHY_FRAME_SIZE_MAX  123

#define PHY_PKT_FLAG_IMMEDIATE  4   // send without tx-gap delay and put at front of queue

typedef void (* nphy_hook_t)(u32 chan);
typedef void (* nphy_rx_t)(u8 flag, u8 size, u8 data[], s8 rssi, u8 lqi);

bool nphy_init(u8 cell, bool sync_master, nphy_rx_t rx);
void nphy_hook_sync(nphy_hook_t hook);

void nphy_tx(u8 flag, u8 *buf, u8 len);
