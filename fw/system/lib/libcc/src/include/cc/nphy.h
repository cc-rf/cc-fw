#pragma once

#include <cc/common.h>

bool nphy_init(void);
cc_pkt_t *nphy_rx(void);
void nphy_tx(cc_pkt_t *pkt);
