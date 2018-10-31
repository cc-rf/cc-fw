#include "ccio.h"
#include "fabi.h"
#include "virtual_com.h"
#include "rf_uart.h"
#include <kio/sclk.h>

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


void ccio_recv(u8 port, serf_t *frame, size_t size)
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

    status.phy_task_stack_usage = phy_task_stack_usage(mac_phy(macs[0]));
    status.mac_task_stack_usage_tx = mac_task_tx_stack_usage(macs[0]);
    status.mac_task_stack_usage_rx = mac_task_rx_stack_usage(macs[0]);

    status.heap_free = xPortGetFreeHeapSize();
    status.heap_usage = configTOTAL_HEAP_SIZE - xPortGetMinimumEverFreeHeapSize();

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
    code_peer->time = net_time();

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


void write_code_mac_recv(u16 addr, u16 peer, u16 dest, size_t size, u8 data[], pkt_meta_t meta)
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


void write_code_recv(net_path_t path, net_addr_t dest, size_t size, u8 data[])
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


void write_code_evnt(net_size_t size, u8 data[])
{
    if (usb_attached(SERF_USB_PORT)) {
        u8 *frame;
        const size_t fsize = serf_encode(CODE_ID_EVNT, data, size, &frame);
        if (frame) usb_write_direct(SERF_USB_PORT, frame, fsize);
    }
}


void write_code_status(u8 port, code_status_t *code_status)
{
    if (usb_attached(port)) {
        u8 *frame;
        const size_t size = serf_encode(CODE_ID_STATUS, (u8 *) code_status, sizeof(code_status_t), &frame);
        if (frame) usb_write_direct(port, frame, size);
    }
}


void write_code_peer(u8 port, size_t size, code_peer_t *code_peer)
{
    u8 *frame;
    const size_t fsize = serf_encode(CODE_ID_PEER, (u8 *)code_peer, size + sizeof(code_peer_t), &frame);
    if (frame) usb_write_direct(port, frame, fsize);
}


void write_code_mac_send_stat(u8 port, code_mac_send_stat_t *code_send_stat)
{
    if (usb_attached(port)) {
        u8 *frame;
        const size_t size = serf_encode(CODE_ID_MAC_SEND, (u8 *) code_send_stat, sizeof(code_mac_send_stat_t), &frame);
        if (frame) usb_write_direct(port, frame, size);
    }
}


void write_code_trxn_stat(u8 port, net_size_t size, code_trxn_stat_t *code_trxn_stat)
{
    if (usb_attached(port)) {
        u8 *frame;
        const size_t fsize = serf_encode(CODE_ID_TRXN, (u8 *) code_trxn_stat, size + sizeof(code_trxn_stat_t), &frame);
        if (frame) usb_write_direct(port, frame, fsize);
    }
}


void write_code_mesg_sent(u8 port, net_size_t size)
{
    if (usb_attached(port)) {
        u8 *frame;
        const size_t fsize = serf_encode(CODE_ID_MESG_SENT, (u8 *) &size, sizeof(net_size_t), &frame);
        if (frame) usb_write_direct(port, frame, fsize);
    }
}


void write_code_uart(size_t size, u8 *data)
{
    if (usb_attached(SERF_USB_PORT)) {
        u8 *frame;
        const size_t fsize = serf_encode(CODE_ID_UART, data, size, &frame);
        if (frame) usb_write_direct(SERF_USB_PORT, frame, fsize);
    }

    if (usb_attached(UART_USB_PORT))
        usb_write_raw(UART_USB_PORT, data, size);
}
