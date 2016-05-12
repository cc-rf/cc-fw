#pragma once

#include <type.h>


void uart_init(void);
void uart_write(const u8 *buf, size_t len);
void uart_read(u8 *buf, size_t len);
size_t uart_read_frame(u8 **buf);
void uart_puts(const char *str);
void uart_putch(const char ch);
size_t uart_gets(char *str, size_t len);
void uart_getch(char *ch);