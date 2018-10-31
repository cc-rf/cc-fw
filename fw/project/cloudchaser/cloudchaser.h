#pragma once

#include <usr/type.h>
#include <ccrf/net.h>


#define CLOUDCHASER_RDIO_COUNT  1
#define CLOUDCHASER_FCC_MODE    0
#define CLOUDCHASER_UART_MODE   0

#define RF_UART_ID              0
#define RF_UART_BAUD            115200


extern net_t nets[CLOUDCHASER_RDIO_COUNT];
extern mac_t macs[CLOUDCHASER_RDIO_COUNT];


void cloudchaser_main(void);
void rainbow(void);
