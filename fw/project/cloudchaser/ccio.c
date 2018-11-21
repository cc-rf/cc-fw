#include "ccio.h"
#include <board/trace.h>
#include "virtual_com.h"
#include "rf_uart.h"
#include <kio/sclk.h>
#include <kio/flsh.h>
#include <umm_malloc_cfg.h>


static void handle_code_echo(u8 port, mbuf_t *mbuf);
static void handle_code_status(u8 port, mbuf_t *mbuf);
static void handle_code_config(u8 port, mbuf_t *mbuf);
static void handle_code_mac_send(u8 port, mbuf_t *mbuf);
static void handle_code_send(u8 port, mbuf_t *mbuf);
static void handle_code_trxn(u8 port, mbuf_t *mbuf);
static void handle_code_resp(u8 port, mbuf_t *mbuf);
static void handle_code_reset(u8 port, mbuf_t *mbuf);
static void handle_code_flash(u8 port, mbuf_t *mbuf);
static void handle_code_peer(u8 port, mbuf_t *mbuf);
static void handle_code_uart(mbuf_t *mbuf);
static void handle_code_rainbow(mbuf_t *mbuf);

#if FABI
static mbuf_t handle_code_led(mbuf_t mbuf);
#endif

static void write_code_usb(u8 port, u8 code, mbuf_t *mbuf);


static bool code_data_check(mbuf_t *mbuf, size_t size)
{
    if ((*mbuf)->used < size) {
        board_trace_f("ccio: data too small for given command code (%u < %u)", (*mbuf)->used, size);
        return false;
    }

    return true;
}


void ccio_recv(u8 port, mbuf_t *mbuf)
{
    serf_t frame;
    
    if ((*mbuf)->used < sizeof(serf_t)) {
        printf("(frame) too small: size=%u < %u\r\n", (*mbuf)->used, sizeof(serf_t));
        return;
    }
    
    mbuf_popf(mbuf, sizeof(serf_t), (u8 *) &frame);

    if ((frame.code & SERF_CODE_PROTO_M) != SERF_CODE_PROTO_VAL) {
        printf("(frame) invalid proto bits: size=%u code=0x%02x\r\n", (*mbuf)->used, frame.code);
        return;
    }

    frame.code &= SERF_CODE_M;

    switch (frame.code) {
        case CODE_ID_ECHO:
            return handle_code_echo(port, mbuf);

        case CODE_ID_STATUS:
            return handle_code_status(port, mbuf);

        case CODE_ID_CONFIG:
            return handle_code_config(port, mbuf);

        case CODE_ID_MAC_SEND:
            return handle_code_mac_send(port, mbuf);

        case CODE_ID_SEND:
            return handle_code_send(port, mbuf);

        case CODE_ID_TRXN:
            return handle_code_trxn(port, mbuf);

        case CODE_ID_RESP:
            return handle_code_resp(port, mbuf);

        case CODE_ID_PEER:
            return handle_code_peer(port, mbuf);

        case CODE_ID_RESET:
            return handle_code_reset(port, mbuf);

        case CODE_ID_FLASH:
            return handle_code_flash(port, mbuf);

        case CODE_ID_UART:
            return handle_code_uart(mbuf);

        case CODE_ID_RAINBOW:
            return handle_code_rainbow(mbuf);
        #if FABI
        case CODE_ID_LED:
            return handle_code_led(mbuf);
        #endif

        default:
            printf("(frame) unknown code: size=%u code=0x%02x\r\n", (*mbuf)->used, frame.code);
            break;
    }
}


static void handle_code_echo(u8 port __unused, mbuf_t *mbuf)
{
    write_code_usb(port, CODE_ID_ECHO, mbuf);
}


static void handle_code_status(u8 port, mbuf_t *mbuf)
{
    mbuf_used(mbuf, sizeof(status));

    code_status_t *stat = (code_status_t *) (*mbuf)->data;

    status.version = __flsh_version;
    status.date = __flsh_date;

    status.macid = mac_addr(macs[0]);
    status.cell = phy_cell(mac_phy(macs[0]));

    status.uptime = SCLK_MSEC(sclk_time());

    phy_stat(mac_phy(macs[0]), &status.stat.phy);
    mac_stat(macs[0], &status.stat.mac);
    net_stat(nets[0], &status.stat.net);

    status.phy_task_stack_usage = phy_task_stack_usage(mac_phy(macs[0]));
    status.mac_task_stack_usage_rx = mac_task_rx_stack_usage(macs[0]);

    size_t free = umm_free_heap_size();

    status.heap_free = free;
    status.heap_usage = configTOTAL_HEAP_SIZE - status.heap_free;

    phy_chan_all(mac_phy(macs[0]), status.chan);

    *stat = status;

    write_code_usb(port, CODE_ID_STATUS, mbuf);
}


static void handle_code_config(u8 port, mbuf_t *mbuf)
{
    if (!code_data_check(mbuf, sizeof(code_config_t))) return;

    code_config_t *code_config = (code_config_t *) (*mbuf)->data;

    code_config_rsp_t rsp = {
            .rslt = CONFIG_RSLT_ERR
    };

    switch (code_config->id) {
        case CONFIG_ID_ADDR:
            {
                net_addr_t addr = net_addr(nets[0]);

                rsp.rslt = net_addr_set(nets[0], code_config->addr.orig, code_config->addr.addr);

                if (rsp.rslt && rsp.rslt != addr) {
                    ccrf_addr_flsh = (net_addr_t) rsp.rslt;
                    flsh_user_cmit();
                }
            }
            break;

        case CONFIG_ID_CELL:
            if (code_config->cell.addr == net_addr(nets[0]))
            {
                phy_t phy = mac_phy(macs[0]);
                phy_cell_t cell = phy_cell(phy);


                rsp.rslt = phy_cell_set(phy, code_config->cell.orig, code_config->cell.cell);

                if (rsp.rslt && rsp.rslt != cell) {
                    ccrf_cell_flsh = (phy_cell_t) rsp.rslt;
                    flsh_user_cmit();
                    net_peers_wipe(nets[0]);
                }
            }
            break;
    }
    
    mbuf_used(mbuf, sizeof(code_config_rsp_t));
    
    *(code_config_rsp_t *)(*mbuf)->data = rsp;

    write_code_usb(port, CODE_ID_CONFIG_RSP, mbuf);
}


static void handle_code_mac_send(u8 port, mbuf_t *mbuf)
{
    if (!code_data_check(mbuf, sizeof(code_mac_send_t))) return;

    const code_mac_send_t code_send;

    mbuf_popf(mbuf, sizeof(code_mac_send_t), (u8 *) &code_send);

    if (code_send.size != (*mbuf)->used) {
        printf("(send) error: size mismatch: %u != %u\r\n", code_send.size, (*mbuf)->used);
        return;
    }

    for (u8 i = 0; i < CLOUDCHASER_RDIO_COUNT; ++i) {
        if (!code_send.addr || code_send.addr == mac_addr(macs[i])) {
            const bool wait = (code_send.flag & CODE_MAC_SEND_FLAG_WAIT) != 0;

            const mac_size_t result = mac_send(
                    macs[i],
                    (mac_send_t) code_send.type,
                    CC_MAC_FLAG,
                    code_send.dest,
                    *mbuf,
                    wait
            );

            if (result) {
                //LED_C_TOGGLE();
                //LED_D_TOGGLE();
            }

            if (wait) {
                mbuf_used(mbuf, sizeof(code_mac_send_stat_t));
                code_mac_send_stat_t *code_send_stat = (code_mac_send_stat_t *) (*mbuf)->data;

                code_send_stat->addr =  mac_addr(macs[i]);
                code_send_stat->stat = (u32) result;

                write_code_usb(port, CODE_ID_MAC_SEND, mbuf);
            }

            break;
        }
    }
}


static void handle_code_send(u8 port __unused, mbuf_t *mbuf)
{
    if (!code_data_check(mbuf, sizeof(code_send_t))) return;

    code_send_t code_send;

    mbuf_popf(mbuf, sizeof(code_send_t), (u8 *) &code_send);

    net_path_t path = {
            .addr = code_send.addr,
            .info = {
                    .mode = 0,
                    .port = code_send.port,
                    .type = code_send.type
            }
    };

    net_size_t sent;

    if (code_send.flag & CODE_SEND_FLAG_MESG)
         sent = net_mesg(nets[0], path, mbuf);
    else
        sent = net_send(nets[0], path, mbuf);

    if (code_send.flag & CODE_SEND_FLAG_RSLT) {
        mbuf_used(mbuf, sizeof(net_size_t));
        *(net_size_t *)(*mbuf)->data = sent;
        write_code_usb(port, CODE_ID_SEND_DONE, mbuf);
    }

}


static void handle_code_trxn(u8 port, mbuf_t *mbuf)
{
    if (!code_data_check(mbuf, sizeof(code_trxn_t))) return;

    code_trxn_t code_trxn;

    mbuf_popf(mbuf, sizeof(code_trxn_t), (u8 *) &code_trxn);
    
    if (!code_trxn.wait || code_trxn.wait > INT32_MAX) {
        printf("(trxn) error: bad wait time %lu\r\n", code_trxn.wait);
        return;
    }

    net_path_t path = {
            .addr = code_trxn.addr,
            .info = {
                    .port = code_trxn.port,
                    .type = code_trxn.type
            }
    };

    net_trxn_rslt_t rslt;

    net_trxn(nets[0], path, mbuf, code_trxn.wait, &rslt);
    
    mbuf_used(mbuf, sizeof(code_trxn_stat_t));

    code_trxn_stat_t *trxn_stat = (code_trxn_stat_t *) (**mbuf).data;

    trxn_stat->port = path.info.port;
    trxn_stat->type = path.info.type;

    if (!list_empty(&rslt)) {
        net_trxn_t *trxn;

        list_for_each_entry(trxn, &rslt, __list) {
            trxn_stat->addr = trxn->addr;

            mbuf_used(mbuf, sizeof(code_trxn_stat_t));
            mbuf_extd(mbuf, &trxn->mbuf);

            write_code_usb(port, CODE_ID_TRXN, mbuf);
            
            if (list_is_last(&trxn->__list, &rslt)) {
                break;
            }
        }
    }

    trxn_stat->addr = NET_ADDR_NONE;

    write_code_usb(port, CODE_ID_TRXN, mbuf);

    net_trxn_rslt_free(&rslt);
}


static void handle_code_resp(u8 port __unused, mbuf_t *mbuf)
{
    if (!code_data_check(mbuf, sizeof(code_send_t))) return;

    code_send_t code_send;
    
    mbuf_popf(mbuf, sizeof(code_send_t), (u8 *) &code_send);

    net_path_t path = {
            .addr = code_send.addr,
            .info = {
                    .port = code_send.port,
                    .type = code_send.type
            }
    };

    if (code_send.flag & CODE_SEND_FLAG_MESG)
        net_resp(nets[0], path, mbuf);
    else
        net_send(nets[0], path, mbuf);
}


static void handle_code_reset(u8 port __unused, mbuf_t *mbuf)
{
    if (!code_data_check(mbuf, sizeof(code_reset_t))) return;

    code_reset_t *const code_reset = (code_reset_t *) (*mbuf)->data;

    if (code_reset->magic == RESET_MAGIC) {
        printf("<reset>\r\n");
        vTaskDelay(pdMS_TO_TICKS(500));
        NVIC_SystemReset();
    } else {
        printf("reset: malformed magic code\r\n");
    }
}


static void handle_code_flash(u8 port, mbuf_t *mbuf)
{
    if (!code_data_check(mbuf, sizeof(code_flash_t))) return;

    code_flash_t *const code_flash = (code_flash_t *) (*mbuf)->data;

    size_t total =
            code_flash->size.header +
            code_flash->size.user_rom +
            code_flash->size.fast_code +
            code_flash->size.text +
            code_flash->size.data;

    if (!code_data_check(mbuf, sizeof(code_flash_t) + total)) return;

    struct mbuf mbuf_flash = mbuf_view(
            *mbuf,
            sizeof(code_flash_t),
            total
    );

    struct mbuf mbuf_flash_header = mbuf_view(
            &mbuf_flash,
            0,
            code_flash->size.header
    );

    struct mbuf mbuf_flash_user = mbuf_view(
            &mbuf_flash,
            code_flash->size.header,
            code_flash->size.user_rom
    );

    struct mbuf mbuf_flash_code = mbuf_view(
            &mbuf_flash,
            code_flash->size.header + code_flash->size.user_rom,
            code_flash->size.fast_code
    );

    struct mbuf mbuf_flash_text = mbuf_view(
            &mbuf_flash,
            code_flash->size.header + code_flash->size.user_rom + code_flash->size.fast_code,
            code_flash->size.text
    );

    struct mbuf mbuf_flash_data = mbuf_view(
            &mbuf_flash,
            code_flash->size.header + code_flash->size.user_rom + code_flash->size.fast_code + code_flash->size.text,
            code_flash->size.data
    );

    u32 sanity = ((user_flash_t *) mbuf_flash_user.data)->sanity;

    board_trace_r("flash: init update... ");

    status_t status;

    status = flsh_updt_init(
            mbuf_flash_header.used,
            mbuf_flash_user.used,
            mbuf_flash_code.used,
            mbuf_flash_text.used,
            mbuf_flash_data.used
    );

    if (!status) {
        board_trace("done.");

        board_trace_r("flash: update part 1... ");

        if (!(status = flsh_updt_part_1(&mbuf_flash_header, &mbuf_flash_user))) {
            board_trace("done.");
        }
    }

    if (!status) {
        board_trace_r("flash: update part 2... ");

        if (!(status = flsh_updt_part_2(&mbuf_flash_code, &mbuf_flash_text, &mbuf_flash_data))) {
            board_trace("done.");

            status = flsh_updt_done(sanity);
        }
    }

    mbuf_used(mbuf, sizeof(code_flash_stat_t));

    ((code_flash_stat_t *) (*mbuf)->data)->status = status;

    write_code_usb(port, CODE_ID_FLASH_STAT, mbuf);

    if (!status) {
        board_trace_r("flash: update finished, reboot!\r\n\r\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        NVIC_SystemReset();
    }
}


static void handle_code_peer(u8 port, mbuf_t *mbuf)
{
    net_t net = nets[0];

    net_peer_info_t *peers;
    net_size_t count = net_peers_flat(net, sizeof(code_peer_t), true, &peers);

    code_peer_t *code_peer = (code_peer_t *)peers;

    code_peer->addr = net_addr(net);
    code_peer->time = net_time();

    mbuf_conv(mbuf, sizeof(code_peer_t) + count * sizeof(net_peer_info_t), (u8 **) &code_peer);

    write_code_usb(port, CODE_ID_PEER, mbuf);
}


static void handle_code_uart(mbuf_t *mbuf)
{
    rf_uart_write(*mbuf);
}


static void handle_code_rainbow(mbuf_t *mbuf)
{
    if (!code_data_check(mbuf, sizeof(code_rbow_t))) return;

    net_t net = nets[0];

    code_rbow_t *code_rbow = (code_rbow_t *) (**mbuf).data;

    net_path_t path = {
            .addr = code_rbow->addr,
            .info = { .port = CCIO_PORT, .type = CCIO_RBOW }
    };

    if (path.addr == NET_ADDR_INVL || path.addr == net_addr(net)) {

        rainbow();

    } else {

        mbuf_done(mbuf);
        net_send(net, path, mbuf);
    }

    if (path.addr == NET_ADDR_BCST)
        rainbow();
}


#if FABI

static void handle_code_led(mbuf_t *mbuf)
{
    if (!code_data_check(mbuf, sizeof(code_led_t))) return;

    code_led_t *code_led = (code_led_t *) (*mbuf)->data;

    if (code_led->addr == NET_ADDR_MASK || code_led->addr == net_addr(nets[0])) {
        return fabi_write(code_led->msg.mask, code_led->data, (*mbuf)->used - sizeof(code_led_t));
    } else {
        net_path_t path = {
                .addr = code_led->addr,
                .info = {
                        .port = CCIO_PORT,
                        .type = CCIO_LED
                }
        };

        net_size_t net_size = (net_size_t)((*mbuf)->used - sizeof(code_led_t) + sizeof(fabi_msg_t));

        net_send(nets[0], path, net_size, (u8 *)&code_led->msg);
    }
}

#endif


static void write_code_usb(u8 port, u8 code, mbuf_t *mbuf)
{
    if (usb_attached(port)) {
        mbuf_t frame = serf_encode(code, mbuf);
        usb_write_direct(port, &frame);
    }
}


void write_code_mac_recv(u16 addr, u16 peer, u16 dest, mbuf_t *mbuf, pkt_meta_t meta)
{
    if (usb_attached(SERF_USB_PORT)) {
        code_mac_recv_t code_mac_recv = {
                .addr = addr,
                .peer = peer,
                .dest = dest,
                .size = (net_size_t) (*mbuf)->used,
                .meta = meta
        };

        mbuf_push(mbuf, sizeof(code_mac_recv_t), (u8 *) &code_mac_recv);

        write_code_usb(SERF_USB_PORT, CODE_ID_MAC_RECV, mbuf);
    }
}


void write_code_recv(net_path_t path, net_addr_t dest, mbuf_t *mbuf)
{
    if (usb_attached(SERF_USB_PORT)) {

        code_recv_t code_recv = {
                .addr = path.addr,
                .dest = dest,
                .port = path.info.port,
                .type = path.info.type
        };

        mbuf_push(mbuf, sizeof(code_recv_t), (u8 *) &code_recv);

        write_code_usb(SERF_USB_PORT, CODE_ID_RECV, mbuf);
    }
}


void write_code_evnt(mbuf_t *mbuf)
{
    return write_code_usb(SERF_USB_PORT, CODE_ID_EVNT, mbuf);
}


void write_code_uart(mbuf_t *mbuf)
{
    if (usb_attached(UART_USB_PORT)) {
        usb_write_raw(UART_USB_PORT, (*mbuf)->data, (*mbuf)->used);
    }

    return write_code_usb(SERF_USB_PORT, CODE_ID_UART, mbuf);
}
