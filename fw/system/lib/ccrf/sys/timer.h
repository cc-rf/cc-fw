#pragma once

#include "rdio/rdio.h"


typedef struct ccrf_timer *ccrf_timer_t;

typedef void (* ccrf_timer_handler_t)(ccrf_timer_t timer, void *param);


ccrf_timer_t ccrf_timer_init(u32 period, ccrf_timer_handler_t handler, void *param);

void ccrf_timer_start(ccrf_timer_t timer);
void ccrf_timer_stop(ccrf_timer_t timer);
void ccrf_timer_restart(ccrf_timer_t timer);
u32 ccrf_timer_remaining(ccrf_timer_t timer);
