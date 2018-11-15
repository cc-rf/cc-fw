#pragma once

#include <usr/type.h>
#include <fsl_pit.h>


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

typedef struct pit {
    pit_chnl_t chnl;
    pit_handler_t handler;
    void *param;
    bool used;

} *pit_t;


void pit_init(void);
pit_t pit_alloc(const pit_cfg_t *cfg) __nonnull_all;
pit_t pit_chain(pit_t pit, pit_cfg_t *cfg) __nonnull_all;
void pit_free(pit_t pit);

pit_tick_t pit_nsec_tick(pit_nsec_t nsec) __fast_code;
pit_nsec_t pit_tick_nsec(pit_tick_t tick) __fast_code;

pit_prio_t pit_get_prio(pit_t pit) __nonnull_all;
void pit_set_prio(pit_t pit, pit_prio_t prio) __nonnull_all;
void pit_set_period(pit_t pit, pit_tick_t period) __fast_code __nonnull_all;
pit_tick_t pit_get_period(pit_t pit) __fast_code __nonnull_all;
static inline pit_tick_t pit_get_current(pit_t pit) __fast_code __nonnull_all;
bool pit_started(pit_t pit) __fast_code __nonnull_all;

void pit_start(pit_t pit) __fast_code __nonnull_all;
void pit_stop(pit_t pit) __fast_code __nonnull_all;
void pit_restart(pit_t pit) __fast_code __nonnull_all;

void pit_ltt_init(void);
pit_nsec_t pit_ltt_current(void);


static inline pit_tick_t pit_get_current(pit_t pit)
{
    // NOTE: Documentation says not to read this value if the timer is disabled
    return PIT_GetCurrentTimerCount(PIT, pit->chnl);
}

