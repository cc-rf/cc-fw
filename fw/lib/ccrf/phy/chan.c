#include "phy/chan.h"

#include <fsl_rnga.h>
#include <stdlib.h>
#include <sys/trace.h>


static inline void chan_calibrate(chan_group_t *group, chan_info_t *chan);
static inline void chan_set(chan_group_t *group, chan_info_t *chan);


void chan_group_init(chan_group_t *group, chan_id_t hop_table[])
{
    ccrf_assert(group); ccrf_assert(group->size); ccrf_assert(group->rdio);

    const freq_t step = group->freq.bw;

    freq_t freq = CHAN_FREQ_ROUND(group->freq.base + step / 2);

    /**
     * Degraded sensitivity in RX at multiples of XOSC/2 and in TX at multiples of XOSC.
     */
    #define BAD_FREQ_GAP        100000
    #define BAD_FREQ_ADJ        1000
    #define IS_TOO_CLOSE(freq)  (((freq) % (CC_XOSC_FREQ / 2)) < BAD_FREQ_GAP) || (((CC_XOSC_FREQ / 2) - ((freq) % (CC_XOSC_FREQ / 2))) < BAD_FREQ_GAP)

    for (chan_id_t c = 0; c < group->size; ++c, freq = CHAN_FREQ_ROUND(freq+step)) {
        group->chan[c].id = c;
        group->chan[c].rssi = group->chan[c].rssi_prev = (rssi_t) 0xFF;
        group->chan[c].cal.valid = false;

        while (IS_TOO_CLOSE(freq)) {
            //ccrf_trace_debug("chan: adjust freq %lu -> %lu", freq, freq + BAD_FREQ_GAP / 10);
            freq += BAD_FREQ_ADJ;
        }

        freq = rdio_util_map_freq(group->rdio, freq, &group->chan[c].cal.reg.freq);
        group->chan[c].freq = freq;
        if (hop_table) hop_table[c] = c;
    }
}


void chan_table_reorder(chan_group_t *group, u_int seed, chan_id_t hop_table[])
{
    chan_id_t chan_temp;
    chan_id_t rnd;

    srand(seed);

    for (chan_id_t c = 0; c < group->size; ++c) {
        do {
            rnd = (chan_id_t)(rand() % group->size);

        } while (rnd == c);

        chan_temp = hop_table[c];
        hop_table[c] = hop_table[rnd];
        hop_table[rnd] = chan_temp;
    }
}


void chan_table_reset(chan_group_t *group, chan_id_t hop_table[])
{
    for (chan_id_t c = 0; c < group->size; ++c) {
        hop_table[c] = c;
    }
}


static inline void chan_calibrate(chan_group_t *group, chan_info_t *chan)
{
    /* TODO: Check/wait for idle first? */
    rdio_util_set_freq_reg(group->rdio, chan->cal.reg.freq);
    rdio_strobe_cal(group->rdio);

    while ((rdio_strobe_noop(group->rdio) & RDIO_STATUS_MASK) != RDIO_STATUS_IDLE);

    chan->cal.reg.chp = rdio_reg_get(group->rdio, CC1200_FS_CHP, NULL);
    chan->cal.reg.vco[0] = rdio_reg_get(group->rdio, CC1200_FS_VCO4, NULL);
    chan->cal.reg.vco[1] = rdio_reg_get(group->rdio, CC1200_FS_VCO2, NULL);

    chan->cal.valid = true;
}

void chan_group_calibrate(chan_group_t *group)
{
    for (chan_id_t c = 0; c < group->size; ++c)
        chan_calibrate(group, &group->chan[c]);
}

static inline void chan_set(chan_group_t *group, chan_info_t *chan)
{
    rdio_util_set_freq_reg(group->rdio, chan->cal.reg.freq);

    if (chan->cal.valid) {
        /*if (group->cur) {
            if (chan->cal.reg.chp != group->cur->cal.reg.chp)    rdio_reg_set(group->rdio, CC1200_FS_CHP, chan->cal.reg.chp);
            if (chan->cal.reg.vco[0] != group->cur->cal.reg.vco[0]) rdio_reg_set(group->rdio, CC1200_FS_VCO4, chan->cal.reg.vco[0]);
            if (chan->cal.reg.vco[1] != group->cur->cal.reg.vco[1]) rdio_reg_set(group->rdio, CC1200_FS_VCO2, chan->cal.reg.vco[1]);
        } else*/ {
            rdio_reg_set(group->rdio, CC1200_FS_CHP, chan->cal.reg.chp);
            rdio_reg_set(group->rdio, CC1200_FS_VCO4, chan->cal.reg.vco[0]);
            rdio_reg_set(group->rdio, CC1200_FS_VCO2, chan->cal.reg.vco[1]);
        }
    }

    group->cur = chan;
}

void chan_select(chan_group_t *group, chan_id_t id)
{
    chan_set(group, &group->chan[id]);
}

void chan_recalibrate(chan_group_t *group, chan_id_t id)
{
    chan_calibrate(group, &group->chan[id]);
}
