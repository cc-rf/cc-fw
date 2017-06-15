#include "cloudchaser.h"

#include <board.h>
#include <virtual_com.h>

#include <usr/serf.h>
#include <kio/sclk.h>
#include <kio/itm.h>
#include <kio/uid.h>
#include <kio/uart.h>

#include <cc/nmac.h>

#define USB_IN_DATA_MAX     (MAC_PKT_SIZE_MAX + 64)

#define CODE_ID_ECHO        0
#define CODE_ID_STATUS      1
#define CODE_ID_SEND        2
#define CODE_ID_RECV        3
#define CODE_ID_RESET       9

#define RESET_MAGIC         0xD1E00D1E


typedef struct __packed {
    u32 version;
    u64 serial;
    u32 uptime;
    u16 node;
    u32 recv_count;
    u32 recv_bytes;
    u32 send_count;
    u32 send_bytes;

} code_status_t;

typedef struct __packed {
    u32 magic;

} code_reset_t;

typedef struct __packed {
    u16 node;
    u16 peer;
    u16 dest;
    u16 size;
    s8 rssi;
    u8 lqi;
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
    u8 type; // == 0x2a
    u8 data[];

} uart_pkt_t;


extern bool pflag_set(void);
extern bool uflag1_set(void);
extern bool uflag2_set(void);

static void uart_relay_run(void);

static void handle_rx(u16 node, u16 peer, u16 dest, u16 size, u8 data[], s8 rssi, u8 lqi);
static void sync_hook(u32 chan);

static void frame_recv(u8 port, serf_t *frame, size_t size);

static void handle_code_send(u8 port, size_t size, u8 *data);
static void handle_code_reset(u8 port, size_t size, u8 *data);
static void handle_code_status(u8 port, size_t size, u8 *data);
static void handle_code_echo(u8 port, size_t size, u8 *data);

static void write_code_recv(u16 node, u16 peer, u16 dest, size_t size, u8 data[], s8 rssi, u8 lqi);
static void write_code_status(u8 port, code_status_t *code_status);


static code_status_t status;

static uart_t uart = NULL;
static size_t usb_in_size[USB_CDC_INSTANCE_COUNT] = {0};
static u8 usb_in_data[USB_CDC_INSTANCE_COUNT][USB_IN_DATA_MAX];

static bool boss;
static u8 cell;


void cloudchaser_main(void)
{
    status = (code_status_t){
            .version = 0,
            .serial = uid(),
            .uptime = 0,
            .node = uid_short(),
            .recv_count = 0,
            .recv_bytes = 0,
            .send_count = 0,
            .send_bytes = 0
    };

    if (uflag1_set()) {
        LED_A_TOGGLE();
        LED_C_TOGGLE();

    } else {
        nphy_hook_sync(sync_hook);
    }


    boss = pflag_set();

    if (uflag1_set())       cell = 0xA1;
    else if (uflag2_set())  cell = 0xA2;
    else                    cell = 0xA0;

    printf(
            "\r\nCloud Chaser %08lX%08lX@%02X.%04X\r\n\r\n",
            (u32)(status.serial >> 32), (u32)status.serial, cell, status.node
    );

    if (nmac_init(cell, status.node, boss, handle_rx)) {

        if (uflag1_set()) {
            uart_relay_run();
        }

        if (uflag2_set() && boss) {
            printf("meter: auto-tx enabled\r\n");
            #define TXLEN 45

            while (1) {
                const char to_send[TXLEN] = { [ 0 ... (TXLEN-1) ] = '\xA5' };
                nmac_send(NMAC_SEND_STRM, 0x0000, TXLEN, (u8 *)to_send);
                vTaskDelay(pdMS_TO_TICKS(23));
            }
        }

        /*while (1) {
            itm_puts(0, "<itm> usb: send periodic\n");
            const char to_send[] = "Hello Peer\r\n";
            usb_write((u8 *)to_send, strlen(to_send));
            vTaskDelay(pdMS_TO_TICKS(5000));
        }*/

        //goto _end;
        while(1) vTaskDelay(portMAX_DELAY);
    }
}


static void uart_relay_run(void)
{
    struct __packed {
        uart_pkt_t pkt;
        u8 input[1];

    } message = {
            .pkt.type = 0x2a
    };

    itm_puts(0, "uart: relay enabled\r\n");

    uart = uart_init(0, 115200);

    while (1) {
        uart_read(uart, (u8 *)message.input, 1);
        //itm_printf(0, "uart: send 0x%02X\r\n", message.input[0]);
        if (nmac_send(NMAC_SEND_DGRM, 0x0000, sizeof(message), (u8 *)&message)) {
            if (!uflag2_set()) {
                LED_C_TOGGLE();
                LED_D_TOGGLE();
            }
        }
    }
}


static void handle_rx(u16 node, u16 peer, u16 dest, u16 size, u8 data[], s8 rssi, u8 lqi)
{
    ++status.recv_count;
    status.recv_bytes += size;

    //LED_A_TOGGLE();
    //LED_B_TOGGLE();

    if (uflag1_set() && size > sizeof(uart_pkt_t) && uart) {
        uart_pkt_t *const uart_pkt = (uart_pkt_t *)data;

        if (uart_pkt->type == 0x2a) {
            if (!uflag2_set()) {
                LED_A_TOGGLE();
                LED_B_TOGGLE();
            }

            size -= sizeof(uart_pkt_t);
            //itm_printf(0, "uart: rf rx %u byte(s)\r\n", size);
            uart_write(uart, uart_pkt->data, size);

            // TODO: Find a suitable code for passthrough data
            /*if (usb_attached(0)) {
                u8 *buf = malloc(size); assert(buf);
                usb_write_direct(0, buf, size);
            }*/

            return;
        }
    }

    if (uflag2_set()) {
        if (rssi >= -34) {  // 60 dB down
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
        }

        if (!boss) {
            nmac_send(NMAC_SEND_STRM, 0x0000, size, data);
            return;
        }

        return;
    }

    write_code_recv(node, peer, dest, size, data, rssi, lqi);
}


static void sync_hook(u32 chan)
{
    if (uflag2_set()) {
        if (chan == 11 || chan == 13 || chan == 15 || chan == 17) LED_D_ON();
        else LED_D_OFF();
    } else {
        if (chan == 11) LED_A_ON();
        else LED_A_OFF();
        if (chan == 13) LED_B_ON();
        else LED_B_OFF();
        if (chan == 15) LED_C_ON();
        else LED_C_OFF();
        if (chan == 17) LED_D_ON();
        else LED_D_OFF();
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

    serf_t *frame = alloca(sizeof(serf_t) + USB_IN_DATA_MAX);
    size_t frame_size = serf_decode(usb_in_data[port], &usb_in_size[port], frame, USB_IN_DATA_MAX);

    if (frame_size) {
        frame_recv(port, frame, frame_size);
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

    if (code_send->size > MAC_PKT_SIZE_MAX) {
        printf("(send) warning: truncating size from %u to %u\r\n", size, MAC_PKT_SIZE_MAX);
        code_send->size = MAC_PKT_SIZE_MAX;
    }

    if (!code_send->node || code_send->node == nmac_get_addr()) {
        const bool result = nmac_send(
                (nmac_send_t) code_send->type,
                code_send->dest,
                code_send->size,
                code_send->data
        );

        if (result) {
            ++status.send_count;
            status.send_bytes += code_send->size;

            //LED_C_TOGGLE();
            //LED_D_TOGGLE();
        }
    }

    /*itm_printf(0, "(send) type=0x%02X dest=0x%04X size=0x%02X result=%u\r\n",
           (nmac_send_t)code_send->type,
           code_send->dest,
           pkt_size,
           result
    );*/
}


static void handle_code_reset(u8 port, size_t size, u8 *data)
{
    assert(size == sizeof(code_reset_t)); assert(data);

    code_reset_t *const code_reset = (code_reset_t *)data;

    if (code_reset->magic == RESET_MAGIC) {
        printf("<reset>\r\n");
        vTaskDelay(pdMS_TO_TICKS(2317));
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

    write_code_status(port, &status);
}


static void handle_code_echo(u8 port, size_t size, u8 *data)
{
    //itm_printf(0, "<itm> echo: \"%s\"\n", data);
    if (data[size - 1] != '\n')
        printf("(remote) %s\r\n", data);
    else
        printf("(remote) %s", data);
}


static void write_code_recv(u16 node, u16 peer, u16 dest, size_t size, u8 data[], s8 rssi, u8 lqi)
{
    //itm_printf(0, "<itm> recv: node=0x%04X peer=0x%04X dest=0x%04X size=%lu\n", node, peer, dest, size);
    code_recv_t *code_recv = malloc(sizeof(code_recv_t) + size); assert(code_recv);

    code_recv->node = node;
    code_recv->peer = peer;
    code_recv->dest = dest;
    code_recv->size = (u16)size;
    code_recv->rssi = rssi;
    code_recv->lqi = lqi;

    memcpy(code_recv->data, data, size);
    size += sizeof(code_recv_t);

    //itm_printf(0, "<itm> recv: raw frame data size=%lu\n", size);
    u8 *frame;
    size = serf_encode(CODE_ID_RECV, (u8 *)code_recv, size, &frame);
    free(code_recv);

    if (frame) {
        //itm_printf(0, "<itm> recv: frame size=%lu\n", size);
        usb_write_direct(0, frame, size);
        //itm_printf(0, "<itm> recv: frame size=%lu <wrote>\n", size);
        //free(frame);
    }
}


static void write_code_status(u8 port, code_status_t *code_status)
{
    u8 *frame;
    const size_t size = serf_encode(CODE_ID_STATUS, (u8 *)code_status, sizeof(code_status_t), &frame);
    if (frame) usb_write_direct(port, frame, size);
}
