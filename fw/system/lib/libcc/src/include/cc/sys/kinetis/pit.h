#pragma once

#include <cc/common.h>


typedef u32 pit_tick_t;
typedef u64 pit_nsec_t;
typedef u32 pit_prio_t;

typedef struct pit *pit_t;

typedef void (* pit_handler_t)(pit_t pit, void *param);

typedef struct {
    pit_tick_t period;
    pit_handler_t handler;
    void *param;

} pit_cfg_t;


pit_t pit_alloc(const pit_cfg_t *cfg);
pit_t pit_chain(pit_t pit, pit_cfg_t *cfg);
void pit_free(pit_t pit);

pit_tick_t pit_nsec_tick(pit_nsec_t nsec);
pit_nsec_t pit_tick_nsec(pit_tick_t tick);

pit_prio_t pit_get_prio(pit_t pit);
void pit_set_prio(pit_t pit, pit_prio_t prio);
void pit_set_period(pit_t pit, pit_tick_t period);
pit_tick_t pit_get_period(pit_t pit);
pit_tick_t pit_get_current(pit_t pit);
pit_tick_t pit_get_elapsed(pit_t pit);
bool pit_started(pit_t pit);

void pit_start(pit_t pit);
void pit_stop(pit_t pit);
void pit_restart(pit_t pit);

void pit_ltt_init(void);
pit_nsec_t pit_ltt_current(void);
