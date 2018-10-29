#include "cloudchaser.h"
#include "led.h"
#include "console.h"
#include "rf_uart.h"

#include <board.h>
#include <virtual_com.h>

#include <usr/serf.h>
#include <kio/sclk.h>
#include <kio/itm.h>
#include <kio/uid.h>

#include <ccrf/mac.h>
#include <ccrf/net.h>

#include <fabi.h>


#define CLOUDCHASER_RDIO_COUNT  1


#define USB_IN_DATA_MAX     (8192 + 64)

#define CODE_ID_ECHO            0
#define CODE_ID_STATUS          1
#define CODE_ID_MAC_SEND        2
#define CODE_ID_MAC_RECV        3
#define CODE_ID_SEND            4
#define CODE_ID_MESG            5
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

#define CC_MAC_FLAG             0

#define RF_UART_ID              0
#define RF_UART_BAUD            115200


typedef struct __packed {
    u32 version;
    u64 serial;
    u32 uptime;
    u16 macid;
    u8 cell;
    u8 rdid;

    struct __packed {
        phy_stat_t phy;
        mac_stat_t mac;
        net_stat_t net;

    } stat;

    // TODO: Maybe add heap stats

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


extern bool pflag_set(void);
extern bool uflag1_set(void);
extern bool uflag2_set(void);

static void net_recv(net_t net, net_path_t path, net_addr_t dest, size_t size, u8 data[]);
static void net_evnt(net_t net, net_event_t event, void *info);

static void mac_recv(mac_t mac, mac_flag_t flag, mac_addr_t peer, mac_addr_t dest, mac_size_t size, u8 *data, pkt_meta_t meta);
static void sync_hook(chan_id_t chan);

static void frame_recv(u8 port, serf_t *frame, size_t size);

static void handle_code_mac_send(u8 port, size_t size, u8 *data);
static void handle_code_send(u8 port, size_t size, u8 *data);
static void handle_code_mesg(u8 port, size_t size, u8 *data);
static void handle_code_trxn(u8 port, size_t size, u8 *data);
static void handle_code_resp(u8 port, size_t size, u8 *data);
static void handle_code_reset(u8 port, size_t size, u8 *data);
static void handle_code_status(u8 port, size_t size, u8 *data);
static void handle_code_peer(u8 port, size_t size, u8 *data);
static void handle_code_echo(u8 port, size_t size, u8 *data);
static void handle_code_uart(size_t size, u8 *data);
static void handle_code_rainbow(size_t size, u8 *data);
static void handle_code_led(size_t size, u8 *data);

static void write_code_mac_recv(u16 addr, u16 peer, u16 dest, size_t size, u8 data[], pkt_meta_t meta);
static void write_code_recv(net_path_t path, net_addr_t dest, size_t size, u8 data[]);
static void write_code_evnt(net_size_t size, u8 data[]);
static void write_code_status(u8 port, code_status_t *code_status);
static void write_code_peer(u8 port, size_t size, code_peer_t *code_peer);
static void write_code_mac_send_stat(u8 port, code_mac_send_stat_t *code_send_stat);
static void write_code_trxn_stat(u8 port, net_size_t size, code_trxn_stat_t *code_trxn_stat);
static void write_code_mesg_sent(u8 port, net_size_t size);
static void write_code_uart(size_t size, u8 *data);


static net_t nets[CLOUDCHASER_RDIO_COUNT];
static mac_t macs[CLOUDCHASER_RDIO_COUNT];

static code_status_t status;

static size_t usb_in_size[USB_CDC_INSTANCE_COUNT] = {0};
static u8 usb_in_data[USB_CDC_INSTANCE_COUNT][USB_IN_DATA_MAX];

static const net_path_t rf_uart_path = {
        .addr = NET_ADDR_BCST,
        .info = {.mode = 0, .port = CCIO_PORT, .type = CCIO_UART}
};

static const led_rgb_t rainbow_colors[][2] = {
        {{  0,   0,   0,   0}, {  0,   0,   0,   0}},

        {{100,   0,   0, 255}, { 25,   5,   0, 255}},
        {{100, 100,   0, 100}, { 50,   3,   0, 100}},
        {{  0, 100,   0,  50}, {100,   0,   0,  50}},
        {{  0, 100, 200, 255}, {100, 100,   0, 255}},
        {{ 30,   0, 200, 100}, {  0, 100,   0, 100}},
        {{ 70,   0, 180,  50}, {  0, 100, 200,  50}},
        {{ 40,   0, 100, 200}, { 30,   0, 200, 200}},
        {{ 20,   0,  50,  40}, { 70,   0, 180,  40}},

        {{100,   0,   0, 255}, { 25,   5,   0, 255}},
        {{100, 100,   0, 100}, { 50,   3,   0, 100}},
        {{  0, 100,   0,  50}, {100,   0,   0,  50}},
        {{  0, 100, 200, 255}, {100, 100,   0, 255}},
        {{ 30,   0, 200, 100}, {  0, 100,   0, 100}},
        {{ 70,   0, 180,  50}, {  0, 100, 200,  50}},
        {{ 40,   0, 100, 200}, { 30,   0, 200, 200}},
        {{ 20,   0,  50,  40}, { 70,   0, 180,  40}},

        {{100,   0,   0, 255}, { 25,   5,   0, 255}},
        {{100, 100,   0, 100}, { 50,   3,   0, 100}},
        {{  0, 100,   0,  50}, {100,   0,   0,  50}},
        {{  0, 100, 200, 255}, {100, 100,   0, 255}},
        {{ 30,   0, 200, 100}, {  0, 100,   0, 100}},
        {{ 70,   0, 180,  50}, {  0, 100, 200,  50}},
        {{ 40,   0, 100, 200}, { 30,   0, 200, 200}},
        {{ 20,   0,  50,  40}, { 70,   0, 180,  40}},

        {{100,   0,   0, 255}, { 25,   5,   0, 255}},
        {{100, 100,   0, 100}, { 50,   3,   0, 100}},
        {{  0, 100,   0,  50}, {100,   0,   0,  50}},
        {{  0, 100, 200, 255}, {100, 100,   0, 255}},
        {{ 30,   0, 200, 100}, {  0, 100,   0, 100}},
        {{ 70,   0, 180,  50}, {  0, 100, 200,  50}},
        {{ 40,   0, 100, 200}, { 30,   0, 200, 200}},
        {{ 20,   0,  50,  40}, { 70,   0, 180,  40}},

        {{  0,   0,   0,   0}, {  0,   0,   0,   0}},
};

static volatile bool sync_blink = true;

void rainbow(void)
{
    const TickType_t delay = 0;
    const u16 resolution = 1000;
    sync_blink = false;
    led_run_program(resolution, delay, rainbow_colors, ARRAY_SIZE(rainbow_colors));
    sync_blink = true;
}


void cloudchaser_main(void)
{
    memset(&status, 0, sizeof(status));

    status.version = 1;
    status.serial = uid();
    status.macid = uid_short();
    status.rdid = 0;

    net_config_t net_config = {
            .phy = {
                    .rdid = 0,
                    .cell = 0,
            },

            .mac = {
                    .addr = status.macid,
                    .recv = mac_recv
            },

            .net = {
                    .recv = net_recv,
                    .evnt = net_evnt
            }
    };

    if (uflag1_set())       net_config.phy.cell = 0xA1;
    else if (uflag2_set())  net_config.phy.cell = 0xA2;
    else                    net_config.phy.cell = 0xA0;

    status.cell = net_config.phy.cell;

    const u32 serial_hi = (u32)(status.serial >> 32);
    const u32 serial_lo = (u32)status.serial;

    printf(
            "\r\nCloud Chaser %08lX%08lX@%02X.%04X\r\n\r\n",
            serial_hi, serial_lo, net_config.phy.cell, status.macid
    );

    rainbow();

    led_off(LED_BLUE_0);
    led_off(LED_BLUE_1);

    if (!(nets[0] = net_init(&net_config))) {
        return;
    }

    macs[0] = net_mac(nets[0]);

    #if CLOUDCHASER_RDIO_COUNT > 1

        net_config.phy.rdid = 1;
        net_config.mac.addr += 1;

        if (!(nets[1] = net_init(&net_config))) {
            return;
        }

        macs[1] = net_mac(nets[1]);

    #endif

    #if CONSOLE_ENABLED

        console_init(macs[0]);

    #endif

    rf_uart_config_t rf_uart_config = {
            .net = nets[0],
            .path = rf_uart_path,
            .recv = write_code_uart,
            .id = RF_UART_ID,
            .baud = RF_UART_BAUD
    };

    rf_uart_init(&rf_uart_config);

    //fabi_init();

    phy_t phy = mac_phy(macs[0]);

    while (1) {
        net_sync(nets[0]);
        sync_hook(phy_chan(phy));
    }
}


static void net_recv(net_t net __unused, net_path_t path, net_addr_t dest, size_t size, u8 data[])
{
    switch (path.info.port) {
        case CCIO_PORT:
            switch (path.info.type) {
                case CCIO_LED: {
                    fabi_msg_t *msg = (fabi_msg_t *)data;
                    return fabi_write(msg->mask, (fabi_rgb_t *)msg->data, size - sizeof(msg->mask));
                }
                case CCIO_UART:
                    rf_uart_write(size, data);
                    write_code_uart(size, data);
                    break;
            }
            break;
    }

    return write_code_recv(path, dest, size, data);
}


static void net_evnt(net_t net __unused, net_event_t event, void *info)
{
    switch (event) {
        case NET_EVENT_PEER: {
            net_event_peer_t *peer = info;

            code_evnt_peer_t code_evnt_peer = {
                    .event = event,
                    .addr = peer->addr,
                    .action = peer->action
            };

            return write_code_evnt(sizeof(code_evnt_peer), (u8 *) &code_evnt_peer);
        }
    }
}


static void mac_recv(mac_t mac, mac_flag_t flag __unused, mac_addr_t peer, mac_addr_t dest, mac_size_t size, u8 *data,
                     pkt_meta_t meta)
{
    return write_code_mac_recv(mac_addr(mac), peer, dest, size, data, meta);
}


static void sync_hook(chan_id_t chan)
{
    static net_stat_t stat_prev = {0};
    static net_stat_t stat;

    if (sync_blink) {

        #if PHY_CHAN_COUNT == 50
        // Assumption that makes this equivalent: channel time is halved
        led_set(LED_0, (chan == 11*2) ? LED_ON : LED_OFF);
        led_set(LED_2, (chan == 13*2) ? LED_ON : LED_OFF);
        led_set(LED_1, (chan == 15*2) ? LED_ON : LED_OFF);
        led_set(LED_3, (chan == 17*2) ? LED_ON : LED_OFF);
        #else
        led_set(LED_0, (chan == 11) ? 2 : LED_OFF);
        led_set(LED_2, (chan == 13) ? 2: LED_OFF);
        led_set(LED_1, (chan == 15) ? 2 : LED_OFF);
        led_set(LED_3, (chan == 17) ? 2 : LED_OFF);
        #endif

        net_stat(nets[0], &stat);

        if (stat.rx.count != stat_prev.rx.count) {
            led_set(LED_RGB0_GREEN, 5);
        } else {
            led_set(LED_RGB0_GREEN, 0);
        }

        if (stat.rx.errors != stat_prev.rx.errors) {
            led_set(LED_RGB0_RED, 5);
        } else {
            led_set(LED_RGB0_RED, 0);
        }

        if (stat.tx.count != stat_prev.tx.count) {
            led_set(LED_RGB1_BLUE, 8);
        } else {
            led_set(LED_RGB1_BLUE, 0);
        }

        if (stat.tx.errors != stat_prev.tx.errors) {
            led_set(LED_RGB1_RED, 8);
        } else {
            led_set(LED_RGB1_RED, 0);
        }
    }

    stat_prev = stat;
}


void usb_recv(u8 port, size_t size, u8 *data)
{
    //itm_printf(0, "usb[%i] rx: size=%lu\r\n", port, size);

    if (!size || !data) return;

    if ((size + usb_in_size[port]) > USB_IN_DATA_MAX) {
        // TODO: Also guarantee that size parameter does not exceed limit
        itm_printf(0, "usb: input overflow, trashing %u bytes\n", usb_in_size[port]);
        usb_in_size[port] = 0;
    }

    memcpy(&usb_in_data[port][usb_in_size[port]], data, size);
    usb_in_size[port] += size;

    if (port == SERF_USB_PORT) {
        serf_t *frame = pvPortMalloc(sizeof(serf_t) + usb_in_size[port] + 1);
        size_t frame_size = serf_decode(usb_in_data[port], &usb_in_size[port], frame, usb_in_size[port] + 1);

        if (frame_size) {
            frame_recv(port, frame, frame_size);
        }

        vPortFree(frame);

    } else if (port == UART_USB_PORT) {
        rf_uart_write(usb_in_size[port], usb_in_data[port]);
        rf_uart_send((net_size_t) usb_in_size[port], usb_in_data[port]);
        usb_in_size[port] = 0;

    } else if (port == CONSOLE_USB_PORT) {
        const static char nl[] = "\r\n\0";
        static volatile u8 esc = 0;
        static volatile u8 escb = 0;
        size_t scan_size = usb_in_size[port] - size;

        for (size_t i = scan_size; i < usb_in_size[port]; ++i) {
            if (usb_in_data[port][i] == '\x1B' || usb_in_data[port][i] == '\x08' || usb_in_data[port][i] == '\x7E') {
                if (usb_in_data[port][i] == '\x1B') esc = 1;
                --usb_in_size[port];
                if (i >= usb_in_size[port]) continue;
                memcpy(&usb_in_data[port][i], &usb_in_data[port][i+1], usb_in_size[port] - i);
                --i;
                continue;
            }

            if (esc && (usb_in_data[port][i] == '[')) {
                ++escb;
                --usb_in_size[port];
                if (i >= usb_in_size[port]) continue;
                memcpy(&usb_in_data[port][i], &usb_in_data[port][i+1], usb_in_size[port] - i);
                --i;
                continue;
            } else {
                if (esc && !escb) esc = 0;
            }

            if (escb && esc) {
                if (!--escb) esc = 0;
                --usb_in_size[port];
                if (i >= usb_in_size[port]) continue;
                memcpy(&usb_in_data[port][i], &usb_in_data[port][i+1], usb_in_size[port] - i);
                --i;
                continue;
            }

            if (usb_in_data[port][i] == '\r' || usb_in_data[port][i] == '\n') {
                usb_write_raw(port, (u8 *) nl, 2);
                usb_in_data[port][i] = '\0';
                console_input((char *) usb_in_data[port]);
                usb_in_size[port] -= i + 1;

                if (usb_in_size[port])
                    memcpy(usb_in_data[port], &usb_in_data[port][i+1], usb_in_size[port]);
            } else {
                usb_write_raw(port, &usb_in_data[port][i], 1);
            }
        }
    }
}


static void frame_recv(u8 port, serf_t *frame, size_t size)
{
    if ((frame->code & SERF_CODE_PROTO_M) != SERF_CODE_PROTO_VAL) {
        printf("(frame) invalid proto bits: size=%u code=0x%02x\r\n", size, frame->code);
        return;
    }

    size -= sizeof(serf_t);
    frame->code &= SERF_CODE_M;

    switch (frame->code) {
        case CODE_ID_ECHO:
            return handle_code_echo(port, size, frame->data);

        case CODE_ID_STATUS:
            return handle_code_status(port, size, frame->data);

        case CODE_ID_MAC_SEND:
            return handle_code_mac_send(port, size, frame->data);

        case CODE_ID_SEND:
            return handle_code_send(port, size, frame->data);

        case CODE_ID_MESG:
            return handle_code_mesg(port, size, frame->data);

        case CODE_ID_TRXN:
            return handle_code_trxn(port, size, frame->data);

        case CODE_ID_RESP:
            return handle_code_resp(port, size, frame->data);

        case CODE_ID_PEER:
            return handle_code_peer(port, size, frame->data);

        case CODE_ID_RESET:
            return handle_code_reset(port, size, frame->data);

        case CODE_ID_UART:
            return handle_code_uart(size, frame->data);

        case CODE_ID_RAINBOW:
            return handle_code_rainbow(size, frame->data);

        case CODE_ID_LED:
            return handle_code_led(size, frame->data);

        default:
            printf("(frame) unknown code: size=%u code=0x%02x\r\n", size, frame->code);
            break;
    }
}


static void handle_code_mac_send(u8 port, size_t size, u8 *data)
{
    assert(size >= sizeof(code_mac_send_t)); assert(data);

    code_mac_send_t *const code_send = (code_mac_send_t *)data;

    if (code_send->size != (size - sizeof(code_mac_send_t))) {
        printf("(send) error: size mismatch: %u != %u\r\n", code_send->size, size - sizeof(code_mac_send_t));
        return;
    }

    for (u8 i = 0; i < CLOUDCHASER_RDIO_COUNT; ++i) {
        if (!code_send->addr || code_send->addr == mac_addr(macs[i])) {
            const bool wait = (code_send->flag & CODE_SEND_FLAG_WAIT) != 0;

            const mac_size_t result = mac_send(
                    macs[i],
                    (mac_send_t) code_send->type,
                    CC_MAC_FLAG,
                    code_send->dest,
                    code_send->size,
                    code_send->data,
                    wait
            );

            if (result) {
                //LED_C_TOGGLE();
                //LED_D_TOGGLE();
            }

            if (wait) {
                code_mac_send_stat_t code_send_stat = {
                        .addr =  mac_addr(macs[i]),
                        .stat = (u32) result
                };

                write_code_mac_send_stat(port, &code_send_stat);
            }

            break;
        }
    }
}


static void handle_code_send(u8 port __unused, size_t size, u8 *data)
{
    assert(size >= sizeof(code_send_t)); assert(data);

    code_send_t *const code_send = (code_send_t *)data;

    net_path_t path = {
            .addr = code_send->addr,
            .info = {
                    .mode = 0,
                    .port = code_send->port,
                    .type = code_send->type
            }
    };

    net_send(nets[0], path, (net_size_t)size - sizeof(code_send_t), code_send->data);
}


static void handle_code_mesg(u8 port, size_t size, u8 *data)
{
    assert(size >= sizeof(code_send_t)); assert(data);

    code_send_t *const code_send = (code_send_t *)data;

    net_path_t path = {
            .addr = code_send->addr,
            .info = {
                    .port = code_send->port,
                    .type = code_send->type
            }
    };

    net_size_t sent = net_mesg(nets[0], path, (net_size_t)size - sizeof(code_send_t), code_send->data);

    write_code_mesg_sent(port, sent);
}


static void handle_code_trxn(u8 port, size_t size, u8 *data)
{
    assert(size >= sizeof(code_trxn_t)); assert(data);

    code_trxn_t *const code_trxn = (code_trxn_t *)data;

    if (!code_trxn->wait || code_trxn->wait > INT32_MAX) {
        printf("(trxn) error: bad wait time %lu\r\n", code_trxn->wait);
        return;
    }

    net_path_t path = {
            .addr = code_trxn->addr,
            .info = {
                    .port = code_trxn->port,
                    .type = code_trxn->type
            }
    };

    net_trxn_rslt_t rslt;

    net_trxn(
            nets[0], path, (net_size_t)size - sizeof(code_trxn_t),
            code_trxn->data, code_trxn->wait, &rslt
    );

    if (!list_empty(&rslt)) {
        net_trxn_t *trxn;
        code_trxn_stat_t *trxn_stat;

        list_for_each_entry(trxn, &rslt, __list) {
            trxn_stat = pvPortMalloc(trxn->size + sizeof(code_trxn_stat_t));

            trxn_stat->addr = trxn->addr;
            trxn_stat->port = path.info.port;
            trxn_stat->type = path.info.type;

            if (trxn->size) memcpy(trxn_stat->data, trxn->data, trxn->size);

            write_code_trxn_stat(port, trxn->size, trxn_stat);

            if (list_is_last(&trxn->__list, &rslt)) {
                trxn_stat->addr = NET_ADDR_NONE;
                write_code_trxn_stat(port, 0, trxn_stat);
            }

            vPortFree(trxn_stat);
        }
    } else {
        code_trxn_stat_t trxn_stat = {
                .addr = NET_ADDR_NONE,
                .port = path.info.port,
                .type = path.info.type
        };

        write_code_trxn_stat(port, 0, &trxn_stat);
    }

    net_trxn_rslt_free(&rslt);
}


static void handle_code_resp(u8 port __unused, size_t size, u8 *data)
{
    assert(size >= sizeof(code_send_t)); assert(data);

    code_send_t *const code_send = (code_send_t *)data;

    net_path_t path = {
            .addr = code_send->addr,
            .info = {
                    .port = code_send->port,
                    .type = code_send->type
            }
    };

    net_resp(nets[0], path, (net_size_t)size - sizeof(code_send_t), code_send->data);
}


static void handle_code_reset(u8 port __unused, size_t size, u8 *data)
{
    assert(size == sizeof(code_reset_t)); assert(data);

    code_reset_t *const code_reset = (code_reset_t *)data;

    if (code_reset->magic == RESET_MAGIC) {
        printf("<reset>\r\n");
        vTaskDelay(pdMS_TO_TICKS(500));
        NVIC_SystemReset();
    } else {
        printf("reset: malformed magic code\r\n");
    }
}


static void handle_code_status(u8 port, size_t size, u8 *data)
{
    (void)size;
    (void)data;

    status.uptime = SCLK_MSEC(sclk_time());

    phy_stat(mac_phy(macs[0]), &status.stat.phy);
    mac_stat(macs[0], &status.stat.mac);
    net_stat(nets[0], &status.stat.net);

    write_code_status(port, &status);
}


static void handle_code_peer(u8 port, size_t size, u8 *data)
{
    (void)size;
    (void)data;

    net_t net = nets[0];

    net_peer_info_t *peers;
    net_size_t count = net_peers_flat(net, sizeof(code_peer_t), true, &peers);

    code_peer_t *code_peer = (code_peer_t *)peers;

    code_peer->addr = net_addr(net);
    code_peer->time = net_time(net);

    write_code_peer(port, count * sizeof(net_peer_info_t), code_peer);

    vPortFree(code_peer);
}


static void handle_code_echo(u8 port __unused, size_t size, u8 *data)
{
    if (data[size - 1] != '\n')
        printf("(remote) %s\r\n", data);
    else
        printf("(remote) %s", data);
}


static void handle_code_uart(size_t size, u8 *data)
{
    rf_uart_write(size, data);
}


static void handle_code_rainbow(size_t size __unused, u8 *data __unused)
{
    rainbow();
}


static void handle_code_led(size_t size, u8 *data)
{
    code_led_t *code_led = (code_led_t *)data;

    if (code_led->addr == NET_ADDR_MASK || code_led->addr == net_addr(nets[0])) {
        return fabi_write(code_led->msg.mask, code_led->data, size - sizeof(code_led_t));
    } else {
        net_path_t path = {
                .addr = code_led->addr,
                .info = {
                        .port = CCIO_PORT,
                        .type = CCIO_LED
                }
        };

        net_size_t net_size = (net_size_t)(size - sizeof(code_led_t) + sizeof(fabi_msg_t));

        net_send(nets[0], path, net_size, (u8 *)&code_led->msg);
    }
}


static void write_code_mac_recv(u16 addr, u16 peer, u16 dest, size_t size, u8 data[], pkt_meta_t meta)
{
    if (usb_attached(SERF_USB_PORT)) {
        code_mac_recv_t *code_recv = pvPortMalloc(sizeof(code_mac_recv_t) + size);

        code_recv->addr = addr;
        code_recv->peer = peer;
        code_recv->dest = dest;
        code_recv->size = (u16) size;
        code_recv->meta = meta;

        if (size) memcpy(code_recv->data, data, size);
        size += sizeof(code_mac_recv_t);

        u8 *frame;
        size = serf_encode(CODE_ID_MAC_RECV, (u8 *) code_recv, size, &frame);

        vPortFree(code_recv);

        if (frame) {
            usb_write_direct(SERF_USB_PORT, frame, size);
        }
    }
}


static void write_code_recv(net_path_t path, net_addr_t dest, size_t size, u8 data[])
{
    if (usb_attached(SERF_USB_PORT)) {
        code_recv_t *code_recv = pvPortMalloc(sizeof(code_recv_t) + size);

        code_recv->addr = path.addr;
        code_recv->dest = dest;
        code_recv->port = path.info.port;
        code_recv->type = path.info.type;

        if (size) memcpy(code_recv->data, data, size);
        size += sizeof(code_recv_t);

        u8 *frame;
        size = serf_encode(CODE_ID_RECV, (u8 *) code_recv, size, &frame);

        vPortFree(code_recv);

        if (frame) {
            usb_write_direct(SERF_USB_PORT, frame, size);
        }
    }
}


static void write_code_evnt(net_size_t size, u8 data[])
{
    if (usb_attached(SERF_USB_PORT)) {
        u8 *frame;
        const size_t fsize = serf_encode(CODE_ID_EVNT, data, size, &frame);
        if (frame) usb_write_direct(SERF_USB_PORT, frame, fsize);
    }
}


static void write_code_status(u8 port, code_status_t *code_status)
{
    if (usb_attached(port)) {
        u8 *frame;
        const size_t size = serf_encode(CODE_ID_STATUS, (u8 *) code_status, sizeof(code_status_t), &frame);
        if (frame) usb_write_direct(port, frame, size);
    }
}


static void write_code_peer(u8 port, size_t size, code_peer_t *code_peer)
{
    u8 *frame;
    const size_t fsize = serf_encode(CODE_ID_PEER, (u8 *)code_peer, size + sizeof(code_peer_t), &frame);
    if (frame) usb_write_direct(port, frame, fsize);
}


static void write_code_mac_send_stat(u8 port, code_mac_send_stat_t *code_send_stat)
{
    if (usb_attached(port)) {
        u8 *frame;
        const size_t size = serf_encode(CODE_ID_MAC_SEND, (u8 *) code_send_stat, sizeof(code_mac_send_stat_t), &frame);
        if (frame) usb_write_direct(port, frame, size);
    }
}


static void write_code_trxn_stat(u8 port, net_size_t size, code_trxn_stat_t *code_trxn_stat)
{
    if (usb_attached(port)) {
        u8 *frame;
        const size_t fsize = serf_encode(CODE_ID_TRXN, (u8 *) code_trxn_stat, size + sizeof(code_trxn_stat_t), &frame);
        if (frame) usb_write_direct(port, frame, fsize);
    }
}


static void write_code_mesg_sent(u8 port, net_size_t size)
{
    if (usb_attached(port)) {
        u8 *frame;
        const size_t fsize = serf_encode(CODE_ID_MESG_SENT, (u8 *) &size, sizeof(net_size_t), &frame);
        if (frame) usb_write_direct(port, frame, fsize);
    }
}


static void write_code_uart(size_t size, u8 *data)
{
    if (usb_attached(SERF_USB_PORT)) {
        u8 *frame;
        const size_t fsize = serf_encode(CODE_ID_UART, data, size, &frame);
        if (frame) usb_write_direct(SERF_USB_PORT, frame, fsize);
    }

    if (usb_attached(UART_USB_PORT))
        usb_write_raw(UART_USB_PORT, data, size);
}
