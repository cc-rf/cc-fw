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
            NULL, TASK_PRIO_HIGH - 1, rf_uart.task_stack, &rf_uart.task_static
    );
}


void rf_uart_write(mbuf_t mbuf)
{
    uart_write(rf_uart.uart, mbuf->used, mbuf->data);
}


size_t rf_uart_send(mbuf_t *mbuf)
{
    return net_send(rf_uart.net, rf_uart.path, mbuf);
}


static void rf_uart_task(void *p __unused)
{
    mbuf_t mbuf = NULL;

    while (1) {
        if (!mbuf) mbuf = mbuf_alloc(RF_UART_BUF_SIZE, NULL);
        else if (!mbuf_good(mbuf)) {
            mbuf_free(&mbuf);
            continue;
        }

        mbuf_used(&mbuf, uart_read(rf_uart.uart, RF_UART_BUF_SIZE, mbuf->data));

        if (mbuf->used) {
            rf_uart_send(&mbuf);
            if (rf_uart.recv) rf_uart.recv(&mbuf);
            mbuf_done(&mbuf);
        }
    }
}
