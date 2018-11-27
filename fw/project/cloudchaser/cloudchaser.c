#include "cloudchaser.h"
#include <board/led.h>
#include <board/trace.h>
#include "console.h"
#include "rf_uart.h"
#include "ccio.h"

#include <board/board.h>
#include <virtual_com.h>

#include <usr/serf.h>
#include <usr/cobs.h>
#include <kio/sclk.h>
#include <kio/itm.h>
#include <kio/uid.h>

#include <ccrf/mac.h>
#include <ccrf/net.h>

#if FABI
#include <fabi.h>
#endif


extern bool pflag_set(void);
extern bool uflag1_set(void);
extern bool uflag2_set(void);

static void net_recv(net_t net, net_path_t path, net_addr_t dest, mbuf_t *mbuf);
static void net_evnt(net_t net, net_event_t event, void *info);

static void mac_recv(mac_t mac, mac_flag_t flag, mac_addr_t peer, mac_addr_t dest, mbuf_t *mbuf, pkt_meta_t meta);
static void sync_hook(chan_id_t chan) __fast_code;


net_t nets[CLOUDCHASER_RDIO_COUNT];
mac_t macs[CLOUDCHASER_RDIO_COUNT];

code_status_t status;

net_addr_t ccrf_addr_flsh __section(".user") = NET_ADDR_NONE;
phy_cell_t ccrf_cell_flsh __section(".user") = 0xA1;

static mbuf_t usb_read[USB_CDC_INSTANCE_COUNT] = {NULL};

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


static const led_rgb_t sync_colors[][2] = {
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,  20}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   5}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,  20}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   5}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
        {{   0,   0,   0,   0}, {   0,   0,   0,   0}},
};

static volatile bool sync_blink = true;


void rainbow(void)
{
    const static TickType_t delay = 0;
    const static u16 resolution = 1000;
    sync_blink = false;
    led_run_program(resolution, 0.0f, delay, rainbow_colors, ARRAY_SIZE(rainbow_colors));
    sync_blink = true;
}


static void __unused rainbow_step(u16 start, u16 end)
{
    const TickType_t delay = 100;
    const u16 resolution = 10;

    led_step_program(resolution, 0.05f, delay, rainbow_colors, start, end);
}


static void __unused sync_step(u16 chan)
{
    const static TickType_t delay = pdMS_TO_TICKS(0);
    const static u16 resolution = PHY_CHAN_COUNT / (ARRAY_SIZE(sync_colors) - 1);

    if (chan == 0 || (++chan != PHY_CHAN_COUNT)) {

        if (chan == 3) led_set(LED_1, LED_ON);
        else led_set(LED_1, LED_OFF);

        if (chan == 7) led_set(LED_0, LED_ON);
        else led_set(LED_0, LED_OFF);

        led_step_program(resolution, 0.0f, delay, sync_colors, chan, (u16) 1 + chan);
    }
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
            "\r\nCloud Chaser %08lx.%08lx %08lX%08lX:%02X:%04X\r\n\r\n",
            __flsh_version, __flsh_date,
            serial_hi, serial_lo, net_config.phy.cell, status.macid
    );

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

    rainbow();

    phy_t phy = mac_phy(macs[0]);

    while (1) {
        net_sync(nets[0]);
        sync_hook(phy_chan(phy));
    }
}


void usb_descriptor_serial_update(size_t size, char *serial)
{
    char serial_str[size];

    u64 serno = uid();
    const u32 serial_hi = (u32)(serno >> 32);
    const u32 serial_lo = (u32)serno;
    net_addr_t addr = ccrf_addr_flsh != NET_ADDR_NONE ? ccrf_addr_flsh : uid_short();

    sprintf(serial_str, "%08lx.%08lx %08lX%08lX:%02X:%04X",
            __flsh_version,
            __flsh_date,
            serial_hi,
            serial_lo,
            ccrf_cell_flsh,
            addr
    );

    size = strlen(serial_str);

    for (size_t c = 0; c < size; ++c) {
        *serial++ = serial_str[c];
        *serial++ = 0x00;
    }
}


static void net_recv(net_t net, net_path_t path, net_addr_t dest, mbuf_t *mbuf)
{
    switch (path.info.port) {
        case CCIO_PORT:
            return ccio_net_recv(net, path, dest, mbuf);
    }

    write_code_recv(path, dest, mbuf);
}


static void net_evnt(net_t net __unused, net_event_t event, void *info)
{
    switch (event) {
        case NET_EVENT_PEER: {
            net_event_peer_t *peer = info;

            mbuf_t mbuf = mbuf_new(sizeof(code_evnt_peer_t));

            code_evnt_peer_t *code_evnt_peer = (code_evnt_peer_t *) mbuf->data;

            code_evnt_peer->event = event;
            code_evnt_peer->addr = peer->addr;
            code_evnt_peer->action = peer->action;

            write_code_evnt(&mbuf);

            mbuf_free(&mbuf);
        }
    }
}


static void mac_recv(mac_t mac, mac_flag_t flag __unused, mac_addr_t peer, mac_addr_t dest, mbuf_t *mbuf, pkt_meta_t meta)
{
    write_code_mac_recv(mac_addr(mac), peer, dest, mbuf, meta);
}


static void sync_hook(chan_id_t chan)
{
    static net_stat_t stat_prev = {{0}};
    static net_stat_t stat;

    if (sync_blink) {

        #if PHY_CHAN_COUNT == 50
        // Assumption that makes this equivalent: channel time is halved
        led_set(LED_0, (u8) ((chan == 11*2) ? 2 : LED_OFF));
        led_set(LED_2, (u8) ((chan == 13*2) ? 2 : LED_OFF));
        led_set(LED_1, (u8) ((chan == 15*2) ? 2 : LED_OFF));
        led_set(LED_3, (u8) ((chan == 17*2) ? 2 : LED_OFF));
        #else

        //sync_step(chan);

        led_set(LED_2, (u8) ((chan == 0) ? 10 : LED_OFF));

        //led_set(LED_0, (u8) ((chan == 0) ? 10 : LED_OFF));
        //led_set(LED_2, (u8) ((chan == 3) ? 10 : LED_OFF));
        //led_set(LED_1, (u8) ((chan == 5) ? 10 : LED_OFF));
        //led_set(LED_3, (u8) ((chan == 7) ? 10 : LED_OFF));


        /*if (chan > 1) {
            const u16 step = ARRAY_SIZE(rainbow_colors) / PHY_CHAN_COUNT;

            rainbow_step((chan - 1) * step, chan * step);
        }*/

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


void usb_recv(u8 port, mbuf_t *mbuf)
{
    if (!mbuf) {
        board_trace("cc-usb: mbuf == NULL !!")
        return;
    }

    if (port >= USB_CDC_INSTANCE_COUNT)
        return;

    mbuf_t *pmbuf = &usb_read[port];

    if (*pmbuf)
        mbuf_extd(pmbuf, mbuf);
    else
        *pmbuf = mbuf_copy(*mbuf);

    switch (port) {
        default:
            mbuf_done(pmbuf);
            mbuf_free(pmbuf);
            break;

        case SERF_USB_PORT: {
            size_t decoded_size;
            size_t rem_size;

            if ((*mbuf)->used == 2 && !(*mbuf)->data[0] && !(*mbuf)->data[1]) {
                board_trace("serf-usb: flush");

                if ((*pmbuf)->size > 0x200)
                    mbuf_free(pmbuf);
                else
                    mbuf_done(pmbuf);

                break;
            }

            _dec_next:

            if ((decoded_size = serf_decode(*pmbuf, &rem_size))) {

                if (rem_size) {

                    serf_t *serf = (serf_t *) (*pmbuf)->data;

                    if (serf->code == 0xFF) {

                        u32 new_size = *(u32 *)serf->data;

                        new_size = cobs_encode_size_max(new_size) + 2/*trailing zeroes for size frame and then data frame(or not if it continues)*/;

                        if (new_size < 200000) {

                            if (new_size > (*pmbuf)->size) {
                                mbuf_fits(pmbuf, new_size);
                            } else if ((new_size >= (*pmbuf)->used) && ((*pmbuf)->size - new_size) > 0x200) {
                                mbuf_size(pmbuf, new_size);
                            }
                        }

                    } else {

                        mbuf_used(mbuf, decoded_size);
                        memcpy((*mbuf)->data, (*pmbuf)->data, decoded_size);
                        ccio_recv(port, mbuf);
                    }

                    mbuf_popf(pmbuf, (*pmbuf)->used - rem_size, NULL);

                    if (rem_size)
                        goto _dec_next;

                } else {
                    mbuf_used(pmbuf, decoded_size);
                    ccio_recv(port, pmbuf);
                    mbuf_done(pmbuf);
                }
            } else {
                // corner case here would be leading zeroes if the first byte(s) are zero?
            }

            break;
        }

        case UART_USB_PORT:
            rf_uart_write(*pmbuf);
            rf_uart_send(pmbuf);
            mbuf_done(pmbuf);
            break;

        #if CONSOLE_ENABLED

        case CONSOLE_USB_PORT: {
            const static char nl[] = "\r\n\0";
            static volatile u8 esc = 0;
            static volatile u8 escb = 0;
            size_t scan_size = (*pmbuf)->used - size;

            for (size_t i = scan_size; i < (*pmbuf)->used; ++i) {
                if ((*pmbuf)->data[i] == '\x1B' || (*pmbuf)->data[i] == '\x08' || (*pmbuf)->data[i] == '\x7E') {
                    if ((*pmbuf)->data[i] == '\x1B') esc = 1;
                    --(*pmbuf)->used;
                    if (i >= (*pmbuf)->used) continue;
                    memcpy(&(*pmbuf)->data[i], &(*pmbuf)->data[i+1], (*pmbuf)->used - i);
                    --i;
                    continue;
                }

                if (esc && ((*pmbuf)->data[i] == '[')) {
                    ++escb;
                    --(*pmbuf)->used;
                    if (i >= (*pmbuf)->used) continue;
                    memcpy(&(*pmbuf)->data[i], &(*pmbuf)->data[i+1], (*pmbuf)->used - i);
                    --i;
                    continue;
                } else {
                    if (esc && !escb) esc = 0;
                }

                if (escb && esc) {
                    if (!--escb) esc = 0;
                    --(*pmbuf)->used;
                    if (i >= (*pmbuf)->used) continue;
                    memcpy(&(*pmbuf)->data[i], &(*pmbuf)->data[i+1], (*pmbuf)->used - i);
                    --i;
                    continue;
                }

                if ((*pmbuf)->data[i] == '\r' || (*pmbuf)->data[i] == '\n') {
                    usb_write_raw(port, (u8 *) nl, 2);
                    (*pmbuf)->data[i] = '\0';
                    console_input((char *) (*pmbuf)->data);
                    (*pmbuf)->used -= i + 1;

                    if ((*pmbuf)->used)
                        memcpy((*pmbuf)->data, &(*pmbuf)->data[i+1], (*pmbuf)->used);
                } else {
                    usb_write_raw(port, &(*pmbuf)->data[i], 1);
                }
            }

            mbuf_done(pmbuf);

            break;
        }
        #endif

    }
}
