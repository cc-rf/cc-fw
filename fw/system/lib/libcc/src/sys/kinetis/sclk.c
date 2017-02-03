#include <sclk.h>
#include "kinetis.h"
#include <cc/sys/kinetis/pit.h>


void sclk_init(void)
{
    pit_ltt_init();
}

sclk_t sclk_time(void)
{
    return pit_ltt_current() / 1000u;
}
