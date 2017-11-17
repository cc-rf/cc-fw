#pragma once

#include <usr/type.h>
#include <ccrf/mac.h>

#define CONSOLE_ENABLED     1
#define CONSOLE_USB_PORT    0

void console_init(mac_t mac);
void console_input(char *data);
