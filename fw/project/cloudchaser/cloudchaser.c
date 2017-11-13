#include "cloudchaser.h"
#include "led.h"
#include "console.h"

#include <board.h>
#include <virtual_com.h>

#include <usr/serf.h>
#include <kio/sclk.h>
#include <kio/itm.h>
#include <kio/uid.h>
#include <kio/uart.h>

#include <ccrf/mac.h>


#define CLOUDCHASER_RDIO_COUNT  1


#define USB_IN_DATA_MAX     (8192 + 64)

#define CODE_ID_ECHO        0
#define CODE_ID_STATUS      1
#define CODE_ID_SEND        2
#define CODE_ID_RECV        3
#define CODE_ID_RESET       9
#define CODE_ID_UART        26

#define RESET_MAGIC         0xD1E00D1E

#define CODE_SEND_FLAG_WAIT 1

#define SERF_USB_PORT       0

typedef struct __packed {
    u32 version;
    u64 serial;
    u32 uptime;
    u16 node;
    mac_stat_t stat;

    // TODO: Add error stats, maybe fragment stats, and heap stats

} code_status_t;

typedef struct __packed {
    u32 magic;

} code_reset_t;

typedef struct __packed {
    u16 node;
    u16 peer;
    u16 dest;
    u16 size;
    pkt_meta_t meta;
    u8 data[];

} code_recv_t;

typedef struct __packed {
    u8 type;
    u8 flag;
    u16 node;
    u16 dest;
    u16 size;
    u8 data[];

} code_send_t;

typedef struct __packed {
    u16 node;
    u32 stat;

} code_send_stat_t;

typedef struct __packed {
    u8 code;
    u8 data[];

} code_uart_t;

typedef struct __packed {
    u8 type; // == CODE_ID_UART
    u8 code;
    u8 data[];

} uart_pkt_t;


extern bool pflag_set(void);
extern bool uflag1_set(void);
extern bool uflag2_set(void);

static void uart_relay_run(void);

static void handle_rx(mac_t mac, mac_addr_t peer, mac_addr_t dest, mac_size_t size, u8 data[], pkt_meta_t meta);
static void sync_hook(chan_id_t chan);

static void frame_recv(u8 port, serf_t *frame, size_t size);

static void handle_code_send(u8 port, size_t size, u8 *data);
static void handle_code_reset(u8 port, size_t size, u8 *data);
static void handle_code_status(u8 port, size_t size, u8 *data);
static void handle_code_echo(u8 port, size_t size, u8 *data);
static void handle_code_uart(size_t size, u8 *data);

static void write_code_recv(u16 node, u16 peer, u16 dest, size_t size, u8 data[], pkt_meta_t meta);
static void write_code_status(u8 port, code_status_t *code_status);
static void write_code_send_stat(u8 port, code_send_stat_t *code_send_stat);

static void write_code_uart(code_uart_t *code_uart, size_t size);


static mac_t macs[CLOUDCHASER_RDIO_COUNT];

static code_status_t status;

static uart_t uart = NULL;
static size_t usb_in_size[USB_CDC_INSTANCE_COUNT] = {0};
static u8 usb_in_data[USB_CDC_INSTANCE_COUNT][USB_IN_DATA_MAX];


void cloudchaser_main(void)
{
    status = (code_status_t){
            .version = 0,
            .serial = uid(),
            .uptime = 0,
            .node = uid_short(),
            .stat = {{0}}
    };

    mac_config_t mac_config = {
            .rdid = 0,
            .boss = false,
            .addr = status.node,
            .cell = 0,
            .recv = handle_rx,
            .sync = NULL
    };

    mac_config.sync = sync_hook;

    #if BOARD_REVISION == 2
        mac_config.boss = /*status.node == 0x4BDB;*/ pflag_set();
    #else
        mac_config.boss = pflag_set();
    #endif

    if (uflag1_set())       mac_config.cell = 0xA1;
    else if (uflag2_set())  mac_config.cell = 0xA2;
    else                    mac_config.cell = 0xA0;

    printf(
            "\r\nCloud Chaser %08lX%08lX@%02X.%04X\r\n\r\n",
            (u32)(status.serial >> 32), (u32)status.serial, mac_config.cell, status.node
    );

    led_on(LED_RGB0_GREEN);
    led_on(LED_RGB0_RED);
    led_on(LED_RGB1_GREEN);
    led_on(LED_RGB1_RED);
    led_on(LED_BLUE_0);
    led_on(LED_BLUE_1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    led_off(LED_RGB0_GREEN);
    led_off(LED_RGB0_RED);
    led_off(LED_RGB1_GREEN);
    led_off(LED_RGB1_RED);
    led_off(LED_BLUE_0);
    led_off(LED_BLUE_1);

    if (!(macs[0] = mac_init(&mac_config))) {
        return;
    }

    #if CLOUDCHASER_RDIO_COUNT > 1

        mac_config.rdid = 1;
        mac_config.sync = NULL;
        mac_config.addr += 1;

        if (!(macs[1] = mac_init(&mac_config))) {
            return;
        }

    #endif

    #if CONSOLE_ENABLED

        console_init(macs[0]);

    #endif

    if (uflag1_set()) {
        uart_relay_run();
    }

    if (uflag2_set() && mac_config.boss) {
        printf("meter: auto-tx enabled\r\n");
        #define TXLEN 45

        while (1) {
            const char to_send[TXLEN] = { [ 0 ... (TXLEN-1) ] = '\xA5' };
            mac_send(macs[0], MAC_SEND_STRM, 0x0000, TXLEN, (u8 *)to_send, false);
            vTaskDelay(pdMS_TO_TICKS(23));
        }
    }
}


static void uart_relay_run(void)
{
    serf_t *frame = alloca(UINT8_MAX);
    size_t frame_size;
    size_t uart_pkt_size;
    uart_pkt_t *uart_pkt = alloca(UINT8_MAX - sizeof(uart_pkt_t));

    itm_puts(0, "uart: relay enabled\r\n");

    uart = uart_init(0, 96000);

    while (1) {
        frame_size = uart_read_frame(uart, frame, UINT8_MAX);

        if (frame_size) {
            if ((frame->code & SERF_CODE_PROTO_M) != SERF_CODE_PROTO_VAL) {
                printf("uart: (frame) invalid proto bits: size=%u code=0x%02x\r\n", frame_size, frame->code);
                continue;
            }

            frame->code &= SERF_CODE_M;

            frame_size -= sizeof(serf_t);
            uart_pkt_size = sizeof(uart_pkt_t) + frame_size; assert(uart_pkt_size < UINT8_MAX);
            uart_pkt->type = CODE_ID_UART;
            uart_pkt->code = frame->code;
            memcpy(uart_pkt->data, frame->data, frame_size);

            if (mac_send(macs[0], MAC_SEND_STRM, 0x0000, (mac_size_t) uart_pkt_size, (u8 *) uart_pkt, false)) {
                led_toggle(LED_RGB0_BLUE);
                led_off(LED_RGB0_RED);
            } else {
                led_on(LED_RGB0_RED);
            }
        } else {
            itm_puts(0, "uart: empty frame\n");
        }
    }
}


static void handle_rx(mac_t mac, mac_addr_t peer, mac_addr_t dest, mac_size_t size, u8 data[], pkt_meta_t meta)
{
    //LED_A_TOGGLE();
    //LED_B_TOGGLE();

    if (uflag1_set() && size >= sizeof(uart_pkt_t) && uart) {
        uart_pkt_t *const uart_pkt = (uart_pkt_t *) data;

        if (uart_pkt->type == CODE_ID_UART) {
            led_toggle(LED_RGB1_GREEN);
            code_uart_t *code_uart = (code_uart_t *) &uart_pkt->code;
            size = size - sizeof(uart_pkt_t);
            write_code_uart(code_uart, size);
            return;
        }
    }

    if (uflag2_set()) {
        /*if (rssi >= -34) {  // 60 dB down
            LED_A_ON();
            LED_B_ON();
            LED_C_ON();
        } else if (rssi >= -44) { // 70 dB down
            LED_A_ON();
            LED_B_ON();
            LED_C_OFF();
        } else if (rssi >= -74) { // 100 dB down
            LED_A_ON();
            LED_B_OFF();
            LED_C_OFF();
        } else if (rssi >= -89) { // 115 dB down
            LED_A_OFF();
            LED_B_OFF();
            LED_C_ON();
        } else {
            LED_A_OFF();
            LED_B_OFF();
            LED_C_OFF();
        }*/

        if (!phy_boss(mac_phy(mac))) {
            mac_send(mac, MAC_SEND_STRM, 0x0000, size, data, false);
            return;
        }

        return;
    }

    write_code_recv(mac_addr(mac), peer, dest, size, data, meta);
}


static void sync_hook(chan_id_t chan)
{
    if (uflag2_set()) {
        //if (chan == 11 || chan == 13 || chan == 15 || chan == 17) LED_D_ON();
        //else LED_D_OFF();
    } else {
        //led_set(LED_0, (chan == 0) || (chan == 2) || (chan == 4) ? LED_ON : LED_OFF);
        //led_set(LED_1, (chan == 1) || (chan == 1) || (chan == 3) ? LED_ON : LED_OFF);
        led_set(LED_0, (chan == 11) ? LED_ON : LED_OFF);
        led_set(LED_2, (chan == 13) ? LED_ON : LED_OFF);
        led_set(LED_1, (chan == 15) ? LED_ON : LED_OFF);
        led_set(LED_3, (chan == 17) ? LED_ON : LED_OFF);
    }
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
        assert(frame);
        size_t frame_size = serf_decode(usb_in_data[port], &usb_in_size[port], frame, usb_in_size[port] + 1);

        if (frame_size) {
            frame_recv(port, frame, frame_size);
        }

        vPortFree(frame);

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

        case CODE_ID_SEND:
            return handle_code_send(port, size, frame->data);

        case CODE_ID_RESET:
            return handle_code_reset(port, size, frame->data);

        case CODE_ID_UART:
            return handle_code_uart(size, frame->data);

        default:
            printf("(frame) unknown code: size=%u code=0x%02x\r\n", size, frame->code);
            break;
    }
}


static void handle_code_send(u8 port, size_t size, u8 *data)
{
    assert(size >= sizeof(code_send_t)); assert(data);

    code_send_t *const code_send = (code_send_t *)data;

    if (code_send->size != (size - sizeof(code_send_t))) {
        printf("(send) error: size mismatch: %u != %u\r\n", code_send->size, size - sizeof(code_send_t));
        return;
    }

    for (u8 i = 0; i < CLOUDCHASER_RDIO_COUNT; ++i) {
        if (!code_send->node || code_send->node == mac_addr(macs[i])) {
            const bool wait = (code_send->flag & CODE_SEND_FLAG_WAIT) != 0;

            const mac_size_t result = mac_send(
                    macs[i],
                    (mac_send_t) code_send->type,
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
                code_send_stat_t code_send_stat = {
                        .node =  mac_addr(macs[i]),
                        .stat = (u32) result
                };

                write_code_send_stat(port, &code_send_stat);
            }

            break;
        }
    }
}


static void handle_code_reset(u8 port, size_t size, u8 *data)
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
    mac_stat(macs[0], &status.stat);

    write_code_status(port, &status);
}


static void handle_code_echo(u8 port, size_t size, u8 *data)
{
    if (data[size - 1] != '\n')
        printf("(remote) %s\r\n", data);
    else
        printf("(remote) %s", data);
}


static void handle_code_uart(size_t size, u8 *data)
{
    code_uart_t *code_uart = (code_uart_t *) data;

    if (size >= sizeof(code_uart_t)) {
        const size_t uart_pkt_size = sizeof(uart_pkt_t) + size - sizeof(code_uart_t);
        uart_pkt_t *uart_pkt = alloca(uart_pkt_size);
        uart_pkt->type = CODE_ID_UART;
        uart_pkt->code = code_uart->code;
        memcpy(uart_pkt->data, code_uart->data, size - sizeof(code_uart_t));

        //itm_printf(0, "uart: rf relay %lu byte(s), packet size = %lu\r\n", size - sizeof(code_uart_t), uart_pkt_size);

        if (mac_send(macs[0], MAC_SEND_DGRM, 0x0000, (mac_size_t) uart_pkt_size, (u8 *) uart_pkt, false)) {
            led_toggle(LED_RGB0_BLUE);
            led_off(LED_RGB0_RED);
        } else {
            led_on(LED_RGB0_RED);
        }
    }
}


static void write_code_recv(u16 node, u16 peer, u16 dest, size_t size, u8 data[], pkt_meta_t meta)
{
    code_recv_t *code_recv = pvPortMalloc(sizeof(code_recv_t) + size); assert(code_recv);

    code_recv->node = node;
    code_recv->peer = peer;
    code_recv->dest = dest;
    code_recv->size = (u16)size;
    code_recv->meta = meta;

    memcpy(code_recv->data, data, size);
    size += sizeof(code_recv_t);

    u8 *frame;
    size = serf_encode(CODE_ID_RECV, (u8 *)code_recv, size, &frame);

    vPortFree(code_recv);

    if (frame) {
        usb_write_direct(SERF_USB_PORT, frame, size);
    }
}


static void write_code_status(u8 port, code_status_t *code_status)
{
    u8 *frame;
    const size_t size = serf_encode(CODE_ID_STATUS, (u8 *)code_status, sizeof(code_status_t), &frame);
    if (frame) usb_write_direct(port, frame, size);
}


static void write_code_send_stat(u8 port, code_send_stat_t *code_send_stat)
{
    u8 *frame;
    const size_t size = serf_encode(CODE_ID_SEND, (u8 *)code_send_stat, sizeof(code_send_stat_t), &frame);
    if (frame) usb_write_direct(port, frame, size);
}


static void write_code_uart(code_uart_t *code_uart, size_t size)
{
    u8 *frame;
    size_t frame_size;

    if (uart) {
        frame_size = serf_encode(code_uart->code, code_uart->data, size, &frame);

        if (frame) {
            uart_write(uart, frame, frame_size);
            vPortFree(frame);
        }
    }

    if (usb_attached(SERF_USB_PORT)) {
        frame_size = serf_encode(CODE_ID_UART, &code_uart->code, size + sizeof(code_uart->code), &frame);

        if (frame) {
            usb_write_direct(SERF_USB_PORT, frame, frame_size);
        }
    }
}
