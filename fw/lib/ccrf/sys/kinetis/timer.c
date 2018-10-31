#include "sys/timer.h"

#include <kio/pit.h>


ccrf_timer_t ccrf_timer_init(u32 period, ccrf_timer_handler_t handler, void *param)
{
    pit_init();
    
    const pit_cfg_t config = {
            .period = pit_nsec_tick(period * 1000),
            .handler = (pit_handler_t) handler,
            .param = param
    };
    
    pit_t pit = pit_alloc(&config);
    return (ccrf_timer_t) pit;
}

void ccrf_timer_start(ccrf_timer_t timer)
{
    pit_start((pit_t) timer);
}

void ccrf_timer_stop(ccrf_timer_t timer)
{
    pit_stop((pit_t) timer);
}

void ccrf_timer_restart(ccrf_timer_t timer)
{
    pit_restart((pit_t) timer);
}

u32 ccrf_timer_remaining(ccrf_timer_t timer)
{
    return (u32) pit_tick_nsec(pit_get_current((pit_t) timer)) / 1000;
}
