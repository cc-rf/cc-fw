#pragma once

#include <cc/common.h>


typedef struct {
    void (* signal)(cc_dev_t dev, u8 ms1, void *param);
    void (* rx)(cc_dev_t dev, u8 *buf, u8 len, s8 rssi, u8 lqi);

} phy_cfg_t;

bool phy_init(cc_dev_t dev, phy_cfg_t *cfg);
void phy_rx_enable(cc_dev_t dev);
void phy_rx_disable(cc_dev_t dev);
bool phy_rx_enabled(cc_dev_t dev);
void phy_tx(cc_dev_t dev, bool cca, u8 *buf, u32 len);