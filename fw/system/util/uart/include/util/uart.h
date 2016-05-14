#pragma once

#include <usr/type.h>

typedef u8 uart_id_t;
typedef u32 baud_t;

typedef struct uart *uart_t;

uart_t uart_init(const uart_id_t id, const baud_t baud);
void uart_free(uart_t const uart);

void uart_write(uart_t const uart, const u8 *buf, size_t len);

void uart_read(uart_t const uart, u8 *buf, size_t len);
size_t uart_read_frame(uart_t const uart, u8 **buf);

void uart_puts(uart_t const uart, const char *str);
void uart_putch(uart_t const uart, const char ch);

size_t uart_gets(uart_t const uart, char *str, size_t len);
void uart_getch(uart_t const uart, char *ch);