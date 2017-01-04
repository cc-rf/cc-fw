#pragma once

#include <cc/common.h>

typedef void (* nphy_rx_t)(cc_pkt_t *pkt);

bool nphy_init(nphy_rx_t rx);
void nphy_tx(cc_pkt_t *pkt);
