#include "kinetis.h"
#include "sys/clock.h"

#include <kio/sclk.h>


bool ccrf_clock_init(void)
{
    sclk_init();
    return true;
}

ccrf_clock_t ccrf_clock(void)
{
    return sclk_time();
}
