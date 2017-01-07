#pragma once

#include <cc/common.h>

typedef void (* mac_rx_t)(u16 addr, u16 dest, u8 size, u8 data[]);

bool nmac_init(u16 addr, bool sync_master, mac_rx_t rx);

void nmac_tx(u16 dest, u8 size, u8 data[]);
