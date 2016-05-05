#include <uart.h>
#include <string.h>

#include <fsl_uart.h>
#include <itm.h>

#define TRACE_UART              UART0
#define TRACE_UART_CLKSRC       BUS_CLK

static uart_config_t uart_cfg;

void uart_init(void)
{
    UART_GetDefaultConfig(&uart_cfg);
    uart_cfg.baudRate_Bps = 115200;
    //uart_cfg.enableRx = true;
    //uart_cfg.enableTx = true;
    //uart_cfg.parityMode = kUART_ParityDisabled;
#if defined(FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT) && FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT
    //uart_cfg.stopBitCount = kUART_OneStopBit;
#endif
    u32 clock_speed = CLOCK_GetFreq(TRACE_UART_CLKSRC);
    UART_Init(TRACE_UART, &uart_cfg, clock_speed);
    UART_EnableTx(TRACE_UART, true);
    UART_EnableRx(TRACE_UART, true);
}

void uart_write(const u8 *buf, size_t len)
{
    UART_WriteBlocking(TRACE_UART, buf, len);

    //while (len--) UART_WriteBlocking(TRACE_UART, buf++, 1);
}

void uart_read(u8 *buf, size_t len)
{
    UART_ReadBlocking(TRACE_UART, buf, len);
}

void uart_puts(const char *str)
{
    uart_write((u8 *)str, strlen(str)+1);
}

size_t uart_gets(char *str, size_t len)
{
    size_t count = 0;

    /*while (len-- && !UART_ReadBlocking(TRACE_UART, (u8*)str++, 1) && str[count])
        ++count;*/
    while (len--) {
        if (UART_ReadBlocking(TRACE_UART, (u8*)str, 1) != kStatus_Success) {
            break;
        }

        if (!*str++) break;

        count++;
    }

    return count;
}