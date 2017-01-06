#pragma once

#include <cc/common.h>

typedef void (* nphy_rx_t)(u8 flag, u8 *buf, u8 len);

bool nphy_init(nphy_rx_t rx, bool sync_master);
void nphy_tx(u8 flag, u8 *buf, u8 len);
