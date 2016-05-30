#pragma once

#include <cc/type.h>
#include <cc/chan.h>


typedef struct {
    void (* rx)(cc_dev_t dev, u8 *buf, u8 len);

} mac_cfg_t;


bool mac_init(mac_cfg_t *cfg);
void mac_tx_begin(chan_t chan);
void mac_tx_end(void);
void mac_tx(bool cca, u8 *buf, u32 len);
void mac_set_rx_channel(cc_dev_t dev, u8 channel);
void mac_rx_enable(void);
void mac_rx_disable(void);