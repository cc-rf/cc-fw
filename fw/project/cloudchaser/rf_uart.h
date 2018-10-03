#pragma once

#include <usr/type.h>
#include <ccrf/net.h>
#include <kio/uart.h>


typedef void (* rf_uart_recv_t)(size_t size, u8 *data);

typedef struct __packed {
    net_t net;
    net_path_t path;
    rf_uart_recv_t recv;

    uart_id_t id;
    baud_t baud;

} rf_uart_config_t;


void rf_uart_init(rf_uart_config_t *config);
void rf_uart_write(size_t size, u8 *data);
void rf_uart_send(net_size_t size, u8 *data);
