#pragma once

#include <cc/common.h>
#include <cc/nphy.h>

#define MAC_PKT_OVERHEAD    7
#define MAC_PKT_SIZE_MAX    (PHY_FRAME_SIZE_MAX - MAC_PKT_OVERHEAD)

typedef void (* mac_recv_t)(u16 node, u16 peer, u16 dest, u16 size, u8 data[], s8 rssi, u8 lqi);

typedef enum __packed {
    NMAC_SEND_DGRM,
    NMAC_SEND_MESG,
    NMAC_SEND_TRXN,
    NMAC_SEND_STRM

} nmac_send_t;

bool nmac_init(u16 addr, bool sync_master, mac_recv_t rx);
u16 nmac_get_addr(void);
bool nmac_send(nmac_send_t type, u16 dest, u16 size, u8 data[]);
