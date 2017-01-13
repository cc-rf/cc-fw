#pragma once

#include <cc/common.h>

typedef void (* mac_rx_t)(u16 addr, u16 dest, u8 size, u8 data[]);

typedef enum __packed {
    NMAC_SEND_DGRM,
    NMAC_SEND_MESG,
    NMAC_SEND_TRXN,
    NMAC_SEND_STRM

} nmac_send_t;

bool nmac_init(u16 addr, bool sync_master, mac_rx_t rx);

bool nmac_send(nmac_send_t type, u16 dest, u8 size, u8 data[]);
