#include <kio/sclk.h>
#include <kio/pit.h>


#define SCLK_PERIOD_0_NSEC    1000
#define SCLK_PERIOD_1_COUNT   (1000000 * 10)


static void sclk_pit_cycle(pit_t pit, void *param);

static bool sclk_initialized = false;
static pit_t sclk_pit[2] __fast_data;

static volatile pit_nsec_t sclk_high_value __fast_data;


void sclk_init(void)
{
    if (!sclk_initialized) {
        sclk_initialized = true;

        sclk_high_value = 0;

        pit_init();

        sclk_pit[0] = pit_alloc(&(pit_cfg_t){
                .period = pit_nsec_tick(SCLK_PERIOD_0_NSEC)
        });

        sclk_pit[1] = pit_chain(sclk_pit[0], &(pit_cfg_t){
                .period = SCLK_PERIOD_1_COUNT,
                .handler = sclk_pit_cycle
        });

        pit_start(sclk_pit[1]);
        pit_start(sclk_pit[0]);
    }
}


static void sclk_pit_cycle(pit_t pit, void *param)
{
    const pit_nsec_t sclk_high_value_new = sclk_high_value + SCLK_PERIOD_1_COUNT;
    sclk_high_value = sclk_high_value_new;
}


sclk_t sclk_time(void)
{
    return sclk_high_value + SCLK_PERIOD_1_COUNT - pit_get_current(sclk_pit[1]);
}
