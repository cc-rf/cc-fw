#include "rf_uart.h"

#include <kio/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <kio/itm.h>


#define RF_UART_TASK_STACK_SIZE     TASK_STACK_SIZE_MEDIUM
#define RF_UART_BUF_SIZE            1024u


struct rf_uart {
    uart_t uart;
    net_t net;
    net_path_t path;
    rf_uart_recv_t recv;

    TaskHandle_t task;
    StaticTask_t task_static;
    StackType_t task_stack[RF_UART_TASK_STACK_SIZE];

    u8 buf[RF_UART_BUF_SIZE];
};


static void rf_uart_task(void *p);


static struct rf_uart rf_uart;


void rf_uart_init(rf_uart_config_t *config)
{
    rf_uart.uart = uart_init(config->id, config->baud);

    rf_uart.net = config->net;
    rf_uart.path = config->path;
    rf_uart.recv = config->recv;

    rf_uart.task = xTaskCreateStatic(
            (TaskFunction_t) rf_uart_task, "rf_uart", RF_UART_TASK_STACK_SIZE,
            NULL, TASK_PRIO_HIGHEST, rf_uart.task_stack, &rf_uart.task_static
    );
}


void rf_uart_write(size_t size, u8 *data)
{
    return uart_write(rf_uart.uart, size, data);
}


void rf_uart_send(net_size_t size, u8 *data)
{
    net_send(rf_uart.net, rf_uart.path, size, data);
}


static void rf_uart_task(void *p __unused)
{
    net_size_t size;

    while (1) {
        size = (net_size_t) uart_read(rf_uart.uart, RF_UART_BUF_SIZE, rf_uart.buf);
        //rf_uart.buf[size] = 0;
        //board_trace_f("uart-recv %lu [%s]\n", size, rf_uart.buf);

        if (size) {
            rf_uart_send(size, rf_uart.buf);
            if (rf_uart.recv) rf_uart.recv(size, rf_uart.buf);
        }
    }
}
