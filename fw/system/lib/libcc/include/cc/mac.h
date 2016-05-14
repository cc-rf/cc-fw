#pragma once

#include <cc/type.h>

typedef struct {
    void (* rx)(cc_dev_t dev, u8 *buf, u8 len);

} mac_cfg_t;


bool mac_init(mac_cfg_t *cfg);
void mac_task(void);
void mac_tx_begin(void);
void mac_tx_end(void);
void mac_tx(u8 *buf, u32 len);
void mac_rx_enable(void);
void mac_rx_disable(void);