#include "cloudchaser.h"
#include <board/led.h>
#include <board/trace.h>
#include "console.h"
#include "rf_uart.h"
#include "ccio.h"

#include <board/board.h>
#include <virtual_com.h>

#include <usr/serf.h>
#include <kio/sclk.h>
#include <kio/itm.h>
#include <kio/uid.h>

#include <ccrf/mac.h>
#include <ccrf/net.h>

#if FABI
#include <fabi.h>
#endif


typedef struct {
    size_t used;
    size_t size;
    u8 *data;

} usb_read_t;


extern bool pflag_set(void);
extern bool uflag1_set(void);
extern bool uflag2_set(void);

static void net_recv(net_t net, net_path_t path, net_addr_t dest, size_t size, u8 data[]);
static void net_evnt(net_t net, net_event_t event, void *info);

static void mac_recv(mac_t mac, mac_flag_t flag, mac_addr_t peer, mac_addr_t dest, mac_size_t size, u8 *data, pkt_meta_t meta);
static void sync_hook(chan_id_t chan) __fast_code;


net_t nets[CLOUDCHASER_RDIO_COUNT];
mac_t macs[CLOUDCHASER_RDIO_COUNT];

code_status_t status;

net_addr_t ccrf_addr_flsh __section(".user") = NET_ADDR_NONE;
phy_cell_t ccrf_cell_flsh __section(".user") = 0xA1;

static usb_read_t usb_read[USB_CDC_INSTANCE_COUNT] = {{0}};

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

    if (ccrf_addr_flsh != NET_ADDR_NONE) {
        status.macid = ccrf_addr_flsh;
    }

    net_config_t net_config = {
            .phy = {
                    .rdid = 0,
                    .cell = ccrf_cell_flsh,
            },

            .mac = {
                    .addr = status.macid,
                    .recv = (mac_recv_t) mac_recv
            },

            .net = {
                    .recv = net_recv,
                    .evnt = net_evnt
            }
    };

    status.cell = net_config.phy.cell;

    const u32 serial_hi = (u32)(status.serial >> 32);
    const u32 serial_lo = (u32)status.serial;

    printf(
            "\r\nCloud Chaser %08lX%08lX@%02X:%04X\r\n\r\n",
            serial_hi, serial_lo, net_config.phy.cell, status.macid
    );

    rainbow();

    led_off(LED_BLUE_0);
    led_off(LED_BLUE_1);

    bool fail;

    nets[0] = net_init(&net_config, &fail);

    if (fail) return;

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

    #else

        rf_uart_config_t rf_uart_config = {
                .net = nets[0],
                .path = rf_uart_path,
                .recv = write_code_uart,
                .id = RF_UART_ID,
                .baud = RF_UART_BAUD
        };

        rf_uart_init(&rf_uart_config);

    #endif

    #if FABI
    fabi_init();
    #endif

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
                #if FABI
                case CCIO_LED: {
                    fabi_msg_t *msg = (fabi_msg_t *)data;
                    return fabi_write(msg->mask, (fabi_rgb_t *)msg->data, size - sizeof(msg->mask));
                }
                #endif
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
    static net_stat_t stat_prev = {{0}};
    static net_stat_t stat;

    if (sync_blink) {

        #if PHY_CHAN_COUNT == 50
        // Assumption that makes this equivalent: channel time is halved
        led_set(LED_0, (chan == 11*2) ? LED_ON : LED_OFF);
        led_set(LED_2, (chan == 13*2) ? LED_ON : LED_OFF);
        led_set(LED_1, (chan == 15*2) ? LED_ON : LED_OFF);
        led_set(LED_3, (chan == 17*2) ? LED_ON : LED_OFF);
        #else
        led_set(LED_0, (u8) ((chan == 11) ? 2 : LED_OFF));
        led_set(LED_2, (u8) ((chan == 13) ? 2 : LED_OFF));
        led_set(LED_1, (u8) ((chan == 15) ? 2 : LED_OFF));
        led_set(LED_3, (u8) ((chan == 17) ? 2 : LED_OFF));
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
    if (port >= USB_CDC_INSTANCE_COUNT)
        return;

    usb_read_t *read = &usb_read[port];

    if ((read->used + size) > read->size) {
        read->size = (read->used + size) * 2;
        u8 *new = pvPortMalloc(read->size);
        if (read->used) memcpy(new, read->data, read->used);
        memcpy(&new[read->used], data, size);
        read->used += size;
        if (read->data) vPortFree(read->data);
        read->data = new;
    } else {
        memcpy(&read->data[read->used], data, size);
        read->used += size;
    }

    switch (port) {
        default:
            read->used = 0;
            break;

        case SERF_USB_PORT: {
            size_t frame_size = serf_decode(read->data, &read->used, (serf_t *) read->data);
            if (frame_size) ccio_recv(port, (serf_t *) read->data, frame_size);
            break;
        }

        case UART_USB_PORT:
            rf_uart_write(read->used, read->data);
            rf_uart_send((net_size_t) read->used, read->data);
            read->used = 0;
            break;

        case CONSOLE_USB_PORT: {
            const static char nl[] = "\r\n\0";
            static volatile u8 esc = 0;
            static volatile u8 escb = 0;
            size_t scan_size = read->used - size;

            for (size_t i = scan_size; i < read->used; ++i) {
                if (read->data[i] == '\x1B' || read->data[i] == '\x08' || read->data[i] == '\x7E') {
                    if (read->data[i] == '\x1B') esc = 1;
                    --read->used;
                    if (i >= read->used) continue;
                    memcpy(&read->data[i], &read->data[i+1], read->used - i);
                    --i;
                    continue;
                }

                if (esc && (read->data[i] == '[')) {
                    ++escb;
                    --read->used;
                    if (i >= read->used) continue;
                    memcpy(&read->data[i], &read->data[i+1], read->used - i);
                    --i;
                    continue;
                } else {
                    if (esc && !escb) esc = 0;
                }

                if (escb && esc) {
                    if (!--escb) esc = 0;
                    --read->used;
                    if (i >= read->used) continue;
                    memcpy(&read->data[i], &read->data[i+1], read->used - i);
                    --i;
                    continue;
                }

                if (read->data[i] == '\r' || read->data[i] == '\n') {
                    usb_write_raw(port, (u8 *) nl, 2);
                    read->data[i] = '\0';
                    console_input((char *) read->data);
                    read->used -= i + 1;

                    if (read->used)
                        memcpy(read->data, &read->data[i+1], read->used);
                } else {
                    usb_write_raw(port, &read->data[i], 1);
                }
            }

            break;
        }
    }

    if ((read->size - read->used) > 4096) {
        if (read->used) {
            u8 *new = pvPortMalloc(read->used);
            memcpy(new, read->data, read->used);
            vPortFree(read->data);
            read->size = read->used;
            read->data = new;
        } else {
            read->size = 0;
            vPortFree(read->data);
            read->data = NULL;
        }
    }
}
