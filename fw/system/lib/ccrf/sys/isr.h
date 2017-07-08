#pragma once

#include "rdio/rdio.h"


typedef enum __packed {
    CCRF_ISR_SRC_GPIO3,
    CCRF_ISR_SRC_GPIO2,
    CCRF_ISR_SRC_GPIO0,

} ccrf_isr_src_t;

typedef enum __packed {
    CCRF_ISR_EDGE_FALLING,
    CCRF_ISR_EDGE_RISING,
    CCRF_ISR_EDGE_BOTH,
    CCRF_ISR_EDGE_HIGH,
    CCRF_ISR_EDGE_LOW

} ccrf_isr_edge_t;

typedef void (* ccrf_isr_t)(void *param);


void ccrf_isr_configure(rdio_t rdio, ccrf_isr_src_t src, ccrf_isr_edge_t edge, ccrf_isr_t isr, void *param);
