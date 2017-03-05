#include <cc/common.h>
#include <cc/chan.h>
#include <cc/io.h>

#include <assert.h>
#include <fsl_rnga.h>
#include <stdlib.h>

static void chan_calibrate(chan_grp_t *grp, chan_inf_t *chan);
static void chan_set(chan_grp_t *grp, chan_inf_t *chan);

void chan_grp_init(chan_grp_t *grp, chan_t hop_table[])
{
    assert(grp); assert(grp->size); assert(CC_DEV_VALID(grp->dev));

    const u32 step = grp->freq.bw;
    u32 freq = grp->freq.base + step / 2;

    /**
     * Degraded sensitivity in RX at multiples of XOSC/2 and in TX at multiples of XOSC.
     */

    #define IS_TOO_CLOSE(freq)  ((freq % (CC_XOSC_FREQ / 2)) < 1000000) || (((CC_XOSC_FREQ / 2) - (freq % (CC_XOSC_FREQ / 2))) < 1000000)

    for (chan_t c = 0; c < grp->size; ++c, freq += step) {
        grp->chan[c].id = c;
        grp->chan[c].cal.valid = false;

        while (IS_TOO_CLOSE(freq)) {
            //cc_dbg("chan: adjust freq %lu -> %lu", freq, freq + step);
            freq += step;
        }

        grp->chan[c].freq = cc_map_freq(grp->dev, freq, &grp->chan[c].cal.reg.freq);
        if (hop_table) hop_table[c] = c;
    }
}

void chan_table_reorder(chan_grp_t *grp, u_int seed, chan_t hop_table[])
{
    chan_t chan_temp;
    chan_t rnd;

    srand(seed);

    for (chan_t c = 0; c < grp->size; ++c) {
        do {
            rnd = (chan_t)(rand() % grp->size);

        } while (rnd == c);

        chan_temp = hop_table[c];
        hop_table[c] = hop_table[rnd];
        hop_table[rnd] = chan_temp;
    }
}

static void chan_calibrate(chan_grp_t *grp, chan_inf_t *chan)
{
    assert(grp); assert(chan);

    /* TODO: Check/wait for idle first? */
    cc_set_freq_reg(grp->dev, chan->cal.reg.freq);
    cc_strobe(grp->dev, CC1200_SCAL);

    /* TODO: Look into using RFEND_CFG0.CAL_END_WAKE_UP_EN */
    while ((cc_strobe(grp->dev, CC1200_SNOP) & CC1200_STATUS_STATE_M) != CC1200_STATE_IDLE);

    chan->cal.reg.chp = cc_get(grp->dev, CC1200_FS_CHP);
    chan->cal.reg.vco[0] = cc_get(grp->dev, CC1200_FS_VCO4);
    chan->cal.reg.vco[1] = cc_get(grp->dev, CC1200_FS_VCO2);

    chan->cal.valid = true;
}

void chan_grp_calibrate(chan_grp_t *grp)
{
    assert(grp);
    for (chan_t c = 0; c < grp->size; ++c)
        chan_calibrate(grp, &grp->chan[c]);
}

static void chan_set(chan_grp_t *grp, chan_inf_t *chan)
{
    assert(grp); assert(chan);
    cc_set_freq_reg(grp->dev, chan->cal.reg.freq);

    if (chan->cal.valid) {
        if (grp->cur) {
            if (chan->cal.reg.chp != grp->cur->cal.reg.chp)    cc_set(grp->dev, CC1200_FS_CHP, chan->cal.reg.chp);
            if (chan->cal.reg.vco[0] != grp->cur->cal.reg.vco[0]) cc_set(grp->dev, CC1200_FS_VCO4, chan->cal.reg.vco[0]);
            if (chan->cal.reg.vco[1] != grp->cur->cal.reg.vco[1]) cc_set(grp->dev, CC1200_FS_VCO2, chan->cal.reg.vco[1]);
        } else {
            cc_set(grp->dev, CC1200_FS_CHP, chan->cal.reg.chp);
            cc_set(grp->dev, CC1200_FS_VCO4, chan->cal.reg.vco[0]);
            cc_set(grp->dev, CC1200_FS_VCO2, chan->cal.reg.vco[1]);
        }
    }

    grp->cur = chan;
}

void chan_select(chan_grp_t *grp, chan_t id)
{
    assert(grp);
    assert(id < grp->size);
    chan_set(grp, &grp->chan[id]);
}
