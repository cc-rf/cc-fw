#include <cc/interface/kinetis/pit.h>
#include <cc/tmr.h>


inline tmr_t tmr_alloc(const tmr_cfg_t *cfg)
{
    return (tmr_t)pit_alloc((const pit_cfg_t *)cfg);
}

inline void tmr_free(tmr_t tmr)
{
    return pit_free((pit_t)tmr);
}

inline tmr_tick_t tmr_nsec_tick(tmr_nsec_t nsec)
{
    return pit_nsec_tick(nsec);
}

inline tmr_nsec_t tmr_tick_nsec(tmr_tick_t tick)
{
    return pit_tick_nsec(tick);
}

inline void tmr_set_period(tmr_t tmr, tmr_tick_t period)
{
    return pit_set_period((pit_t)tmr, period);
}

inline tmr_tick_t tmr_get_period(tmr_t tmr)
{
    return pit_get_period((pit_t)tmr);
}

inline tmr_tick_t tmr_get_current(tmr_t tmr)
{
    return pit_get_current((pit_t)tmr);
}

inline tmr_tick_t tmr_get_elapsed(tmr_t tmr)
{
    return pit_get_elapsed((pit_t)tmr);
}

inline bool tmr_started(tmr_t tmr)
{
    return pit_started((pit_t)tmr);
}

inline void tmr_start(tmr_t tmr)
{
    return pit_start((pit_t)tmr);
}

inline void tmr_stop(tmr_t tmr)
{
    return pit_stop((pit_t)tmr);
}