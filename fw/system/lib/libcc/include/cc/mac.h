#pragma once

#include <cc/type.h>
#include <cc/chan.h>


typedef struct {
    void (* rx)(cc_dev_t dev, u8 *buf, u8 len);

} mac_cfg_t;


bool mac_init(mac_cfg_t *cfg);
void mac_tx_begin(cc_dev_t dev, chan_t chan);
void mac_tx_end(cc_dev_t dev);
void mac_tx(cc_dev_t dev, bool cca, u8 *buf, u32 len);
void mac_set_mod_cfg_0(cc_dev_t dev);
void mac_set_mod_cfg_1(cc_dev_t dev);
void mac_set_rx_channel(cc_dev_t dev, u8 channel);
void mac_rx_enable(void);
void mac_rx_disable(void);