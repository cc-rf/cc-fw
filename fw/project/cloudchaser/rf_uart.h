#pragma once

#include <usr/type.h>
#include <usr/mbuf.h>
#include <ccrf/net.h>
#include <kio/uart.h>


typedef void (* rf_uart_recv_t)(mbuf_t *mbuf);

typedef struct __packed {
    net_t net;
    net_path_t path;
    rf_uart_recv_t recv;

    uart_id_t id;
    baud_t baud;

} rf_uart_config_t;


void rf_uart_init(rf_uart_config_t *config);
void rf_uart_write(mbuf_t mbuf);
size_t rf_uart_send(mbuf_t *mbuf);
