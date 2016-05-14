#pragma once

#include <cc/common.h>

typedef u32 tmr_tick_t;
typedef u64 tmr_nsec_t;

typedef struct tmr *tmr_t;

typedef void (* tmr_handler_t)(tmr_t tmr, void *param);

typedef struct {
    tmr_tick_t period;
    tmr_handler_t handler;
    void *param;

} tmr_cfg_t;


tmr_t tmr_alloc(const tmr_cfg_t *cfg);
void tmr_free(tmr_t tmr);

tmr_tick_t tmr_nsec_tick(tmr_nsec_t nsec);
tmr_nsec_t tmr_tick_nsec(tmr_tick_t tick);

void tmr_set_period(tmr_t tmr, tmr_tick_t period);
tmr_tick_t tmr_get_period(tmr_t tmr);
tmr_tick_t tmr_get_current(tmr_t tmr);
tmr_tick_t tmr_get_elapsed(tmr_t tmr);
bool tmr_started(tmr_t tmr);

void tmr_start(tmr_t tmr);
void tmr_stop(tmr_t tmr);
