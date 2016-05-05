#include <uart.h>
#include <string.h>

#ifdef FREERTOS
#include <fsl_uart_freertos.h>
#else
#error No support for non-FREERTOS UART yet
#include <fsl_uart.h>
#endif

#define TRACE_UART              UART2
#define TRACE_UART_CLKSRC       BUS_CLK
#define TRACE_UART_RX_TX_IRQn   UART2_RX_TX_IRQn
#define TRACE_UART_IRQ_PRIO     5
#define TRACE_UART_TASK_PRIO    (configMAX_PRIORITIES - 1)


static uart_rtos_handle_t uart_rtos;
static uart_handle_t uart;

static u8 buf_bg[32];

static struct rtos_uart_config uart_cfg = {
        .base = TRACE_UART,
        .baudrate = 115200,
        .parity = kUART_ParityDisabled,
        .stopbits = kUART_OneStopBit,
        .buffer = buf_bg,
        .buffer_size = sizeof(buf_bg),
};


void uart_init(void)
{
    NVIC_SetPriority(TRACE_UART_RX_TX_IRQn, TRACE_UART_IRQ_PRIO);
    uart_cfg.srcclk = CLOCK_GetFreq(TRACE_UART_CLKSRC);
    UART_RTOS_Init(&uart_rtos, &uart, &uart_cfg);
    TRACE_UART->C2 &= ~UART_C2_RE_MASK;
}

void uart_write(const u8 *const buf, const size_t len)
{
    UART_RTOS_Send(&uart_rtos, buf, len);
}

void uart_puts(const char *const str)
{
    uart_write((u8 *)str, strlen(str));
}