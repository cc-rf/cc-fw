#pragma once

#include <ccrf/net.h>
#include <usr/serf.h>
#include "cloudchaser.h"
#include "fabi.h"


#define CCIO_PORT               0x01E0
#define CCIO_LED                0x0B
#define CCIO_UART               0x0D

#if CLOUDCHASER_FCC_MODE
#if CLOUDCHASER_UART_MODE
        #error UART mode and FCC mode cannot coexist.
    #endif
    #define SERF_USB_PORT       1
    #define UART_USB_PORT       2
#else
#if CLOUDCHASER_UART_MODE
#define SERF_USB_PORT       1
        #define UART_USB_PORT       0
#else
#define SERF_USB_PORT       0
#define UART_USB_PORT       1
#endif
#endif


#define CODE_ID_ECHO            0
#define CODE_ID_STATUS          1
#define CODE_ID_MAC_SEND        2
#define CODE_ID_MAC_RECV        3
#define CODE_ID_SEND            4
#define CODE_ID_MESG_SENT       5
#define CODE_ID_RECV            6
#define CODE_ID_TRXN            7
#define CODE_ID_RESP            8
#define CODE_ID_EVNT            9
#define CODE_ID_PEER            10
#define CODE_ID_RESET           17
#define CODE_ID_UART            26
#define CODE_ID_LED             27
#define CODE_ID_RAINBOW         29

#define RESET_MAGIC             0xD1E00D1E

#define CODE_SEND_FLAG_WAIT     1

#define CC_MAC_FLAG             0


typedef struct __packed {
    u32 version;
    u64 serial;
    u32 uptime;
    u16 macid;
    u8 cell;
    u8 rdid;
    u32 phy_task_stack_usage;
    u32 mac_task_stack_usage_tx;
    u32 mac_task_stack_usage_rx;
    u32 heap_free;
    u32 heap_usage;

    struct __packed {
        phy_stat_t phy;
        mac_stat_t mac;
        net_stat_t net;

    } stat;

} code_status_t;

typedef struct __packed {
    u32 magic;

} code_reset_t;

typedef struct __packed {
    u16 addr;
    u16 peer;
    u16 dest;
    u16 size;
    pkt_meta_t meta;
    u8 data[];

} code_mac_recv_t;

typedef struct __packed {
    u8 type;
    u8 flag;
    u16 addr;
    u16 dest;
    u16 size;
    u8 data[];

} code_mac_send_t;

typedef struct __packed {
    net_addr_t addr;
    net_port_t port;
    net_type_t type;
    u8 mesg;
    u8 data[];

} code_send_t;

typedef struct __packed {
    net_addr_t addr;
    net_addr_t dest;
    net_port_t port;
    net_type_t type;
    u8 data[];

} code_recv_t;

typedef struct __packed {
    net_addr_t addr;
    net_port_t port;
    net_type_t type;
    net_time_t wait;
    u8 data[];

} code_trxn_t;

typedef struct __packed {
    u16 addr;
    u32 stat;

} code_mac_send_stat_t;

typedef struct __packed {
    net_addr_t addr;
    net_port_t port;
    net_type_t type;
    u8 data[];

} code_trxn_stat_t;

typedef struct __packed {
    net_event_t event;
    net_addr_t addr;
    net_event_peer_action_t action;

} code_evnt_peer_t;

typedef struct __packed {
    net_addr_t addr;
    net_time_t time;
    net_peer_info_t peer[];

} code_peer_t;

typedef struct __packed {
    net_addr_t addr;
    fabi_msg_t msg;
    fabi_rgb_t data[];

} code_led_t;

extern code_status_t status;


void ccio_init(net_t nets[], mac_t macs[]) __nonnull_all;
void ccio_recv(u8 port, serf_t *frame, size_t size) __nonnull_all;

void write_code_mac_recv(u16 addr, u16 peer, u16 dest, size_t size, u8 data[], pkt_meta_t meta) __nonnull_all;
void write_code_recv(net_path_t path, net_addr_t dest, size_t size, u8 data[]) __nonnull_all;
void write_code_evnt(net_size_t size, u8 data[]) __nonnull_all;
void write_code_status(u8 port, code_status_t *code_status) __nonnull_all;
void write_code_peer(u8 port, size_t size, code_peer_t *code_peer) __nonnull_all;
void write_code_mac_send_stat(u8 port, code_mac_send_stat_t *code_send_stat) __nonnull_all;
void write_code_trxn_stat(u8 port, net_size_t size, code_trxn_stat_t *code_trxn_stat) __nonnull_all;
void write_code_mesg_sent(u8 port, net_size_t size) __nonnull_all;
void write_code_uart(size_t size, u8 *data) __nonnull_all;
