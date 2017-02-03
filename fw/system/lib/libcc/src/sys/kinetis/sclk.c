#include <sclk.h>
#include "kinetis.h"
#include <cc/sys/kinetis/pit.h>

static pit_t sclk_pit[3];

void sclk_init(void)
{
    sclk_pit[0] = pit_alloc(&(pit_cfg_t){
            .period = pit_nsec_tick(1000)
    });

    sclk_pit[1] = pit_chain(sclk_pit[0], &(pit_cfg_t){
            .period = UINT32_MAX
    });

    sclk_pit[2] = pit_chain(sclk_pit[1], &(pit_cfg_t){
            .period = UINT32_MAX
    });

    pit_start(sclk_pit[2]);
    pit_start(sclk_pit[1]);
    pit_start(sclk_pit[0]);

    //pit_ltt_init();
}

sclk_t sclk_time(void)
{
    return UINT64_MAX -
            (((u64)pit_get_current(sclk_pit[2]) << 32u) |
                    (u64)pit_get_current(sclk_pit[1])
            );

    //return pit_ltt_current() / 1000u;
}
