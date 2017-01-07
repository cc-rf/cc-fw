#pragma once

#include <cc/common.h>

#define PHY_PKT_FLAG_IMMEDIATE  4   // send without tx-gap delay and put at front of queue

typedef void (* nphy_rx_t)(u8 flag, u8 *buf, u8 len);

bool nphy_init(nphy_rx_t rx, bool sync_master);
void nphy_tx(u8 flag, u8 *buf, u8 len);

bool nphy_recv(u8 **buf, u8 *len, u32 timeout);
void nphy_free_buf(u8 *buf);
