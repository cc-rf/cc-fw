#pragma once

#include "cloudchaser.h"

#include <usr/type.h>
#include <ccrf/mac.h>


#if CLOUDCHASER_FCC_MODE
    #define CONSOLE_ENABLED     1
    #define CONSOLE_USB_PORT    0
#else
    #ifndef CONSOLE_ENABLED
        #define CONSOLE_ENABLED     0
    #endif

    #define CONSOLE_USB_PORT    2
#endif


void console_init(mac_t mac);
void console_input(char *data);
