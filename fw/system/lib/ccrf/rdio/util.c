#include "rdio/util.h"
#include "sys/trace.h"

#define CC_RCOSC_FREQ   (CC_XOSC_FREQ/1000u)


u8 rdio_util_get_lo_div(rdio_t rdio)
{
    #if !CC_MULTIBAND
        return CC1200_FS_CFG_FSD_BANDSELECT_LO_04_820_960 << 1;
    #else
        const u8 fs_cfg = rdio_reg_get(rdio, CC1200_FS_CFG);
        return (fs_cfg & CC1200_FS_CFG_FSD_BANDSELECT_M) << 1;
    #endif
}

u8 rdio_util_set_lo_div(rdio_t rdio, u8 lo_div)
{
    #if !CC_MULTIBAND
        return rdio_util_get_lo_div(rdio);
    #else
        lo_div = ((lo_div >> 1) & CC1200_FS_CFG_FSD_BANDSELECT_M);
        cc_update(rdio, CC1200_FS_CFG, CC1200_FS_CFG_FSD_BANDSELECT_M, lo_div);
        return lo_div;
    #endif
}

static s16 rdio_util_get_raw_freq_off_est(rdio_t rdio)
{
    return (s16)rdio_reg_get16(rdio, CC1200_FREQOFF_EST1, NULL);
}

s32 rdio_util_get_freq_off_est(rdio_t rdio)
{
    s64 freq_off;

    if (!(freq_off = rdio_util_get_raw_freq_off_est(rdio))) return 0;

    const u32 div = rdio_util_get_lo_div(rdio) << 18;

    freq_off *= CC_XOSC_FREQ;
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    freq_off < 0 ? (freq_off -= (div >> 1)) : (freq_off += (div >> 1));
    freq_off /= div;
    return (s32)freq_off;
}

static s16 rdio_util_get_raw_freq_off(rdio_t rdio)
{
    return (s16)rdio_reg_get16(rdio, CC1200_FREQOFF1, NULL);
}

s32 rdio_util_get_freq_off(rdio_t rdio)
{
    s64 freq_off;

    if (!(freq_off = rdio_util_get_raw_freq_off(rdio))) return 0;

    const u32 div = rdio_util_get_lo_div(rdio) << 18;

    freq_off *= CC_XOSC_FREQ;
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    freq_off < 0 ? (freq_off -= (div >> 1)) : (freq_off += (div >> 1));
    freq_off /= div;
    return (s32)freq_off;
}

s32 rdio_util_set_freq_off(rdio_t rdio, s32 freq_off)
{
    s64 fq = (s64)freq_off;
    const u32 div = rdio_util_get_lo_div(rdio) << 18;

    fq *= div;
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    fq += CC_XOSC_FREQ >> 1;
    fq /= CC_XOSC_FREQ;

    const s16 fq16 = (s16)fq;
    rdio_reg_set16(rdio, CC1200_FREQOFF1, (u16) fq16);

    /* TODO: Determine whether a read-back is needed first.
     * Otherwise, just convert back in case of rounding errors
     * in order to indicate the actual configured offset. */
    fq = fq16;
    fq *= CC_XOSC_FREQ;
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    fq < 0 ? (fq -= (div >> 1)) : (fq += (div >> 1));
    fq /= div;

    return (s32)fq;
}

u32 rdio_util_get_freq(rdio_t rdio)
{
    u32 div = rdio_util_get_lo_div(rdio) << 16;
    s16 raw_freq_off = rdio_util_get_raw_freq_off(rdio);

    /*union {
        freq_reg_t reg;
        u64 val;
    } fq = { .reg = cc_get_freq_reg() };*/
    u64 val = rdio_util_get_freq_reg(rdio).val;

    if (raw_freq_off) {
        div <<= 2;
        val = (val << 2) + raw_freq_off;
    }

    /* TODO: Possibly find a way to use the map function. */
    val *= CC_XOSC_FREQ;
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    val += div >> 1;
    val /= div;

    return (u32)val;
}

freq_reg_t rdio_util_get_freq_reg(rdio_t rdio)
{
    freq_reg_t fr;
    rdio_reg_read(rdio, CC1200_FREQ2, (u8 *)&fr, sizeof(fr));
    return (freq_reg_t){ .reg = {fr.reg[2], fr.reg[1], fr.reg[0]} };
}

u32 rdio_util_set_freq(rdio_t rdio, u32 freq)
{
    freq_reg_t reg;
    freq = rdio_util_map_freq(rdio, freq, &reg);
    rdio_util_set_freq_reg(rdio, reg);
    return freq;
}

void rdio_util_set_freq_reg(rdio_t rdio, freq_reg_t fr)
{
    /* TODO: Look at code difference when modifying reg instead. */
    const freq_reg_t sreg = { .reg = {fr.reg[2], fr.reg[1], fr.reg[0]} };
    rdio_reg_write(rdio, CC1200_FREQ2, (u8 *)&sreg, sizeof(sreg));
}

u32 rdio_util_map_freq(rdio_t rdio, u32 freq, freq_reg_t *reg)
{
    const u32 div = rdio_util_get_lo_div(rdio) << 16;

    union {
        freq_reg_t reg;
        u64 val;
    } fq = { .val = freq };

    /* Convert to register format */
    fq.val *= div;
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    fq.val += CC_XOSC_FREQ >> 1;
    fq.val /= CC_XOSC_FREQ;

    if (reg) *reg = fq.reg;

    /* Convert back */
    fq.val &= 0xFFFFFF;
    fq.val *= CC_XOSC_FREQ;
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    fq.val += div >> 1;
    fq.val /= div;

    return (u32)fq.val;
}

u32 rdio_util_get_freq_dev(rdio_t rdio)
{
    u8 dev_m, dev_e;
    u64 f;
    u32 div = 1u << 22;

    dev_m = rdio_reg_get(rdio, CC1200_DEVIATION_M, NULL);
    dev_e = rdio_reg_get(rdio, CC1200_MODCFG_DEV_E, NULL);
    dev_e = (dev_e & CC1200_MODCFG_DEV_E_DEV_E_M) >> CC1200_MODCFG_DEV_E_DEV_E_S;

    f = dev_m;

    if (dev_e) {
        f += 256;
        f *= CC_XOSC_FREQ << dev_e;
    } else {
        div >>= 1;
        f *= CC_XOSC_FREQ;
    }

    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    f += div >> 1;
    f /= div;

    return (u32)f;
}

u32 rdio_util_set_freq_dev(rdio_t rdio, u32 freq_dev)
{
    u8 reg, dev_e = 0;
    u64 div;
    u64 fdr;

    div = CC_XOSC_FREQ;
    fdr = ((freq_dev << 21) + (div >> 1)) / div;

    while (fdr > UINT8_MAX && dev_e < (CC1200_MODCFG_DEV_E_DEV_E_M>>CC1200_MODCFG_DEV_E_DEV_E_S)) {
        ++dev_e;
        div = (u64)CC_XOSC_FREQ << dev_e; /* TODO: Maybe just subtract dev_e from 22 below. */
        /* NOTE: This rounds .5 to the next integer away from zero. However,
         * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
        fdr = ((freq_dev << 22) + (div >> 1)) / div - 256;
    }

    if (fdr > UINT8_MAX) fdr = UINT8_MAX;

    reg = (u8)fdr;
    rdio_reg_set(rdio, CC1200_DEVIATION_M, reg);
    ccrf_trace_verbose("DEVIATION_M = 0x%02X", reg);
    reg = rdio_reg_get(rdio, CC1200_MODCFG_DEV_E, NULL);
    reg |= dev_e << CC1200_MODCFG_DEV_E_DEV_E_S;
    rdio_reg_set(rdio, CC1200_MODCFG_DEV_E, reg);
    ccrf_trace_verbose("MODCFG_DEV_E = 0x%02X", reg);

    /* TODO: Determine whether a read-back is needed first.
     * Otherwise, just convert back in case of rounding errors
     * in order to indicate the actual configured deviation. */
    div = 1u << 22;

    if (dev_e) {
        fdr += 256;
        fdr *= CC_XOSC_FREQ << dev_e;
    } else {
        div >>= 1;
        fdr *= CC_XOSC_FREQ;
    }

    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    fdr += div >> 1;
    fdr /= div;

    return (u32)fdr;
}

u32 rdio_util_get_symbol_rate(rdio_t rdio)
{
    union {
        u8 reg[sizeof(u64)];
        u64 val;
    } sr = { .val = 0 };

    u8 sr_e;
    u64 div;

    /* TODO: Potentially read in order and then byte swap. */
    sr.reg[2] = rdio_reg_get(rdio, CC1200_SYMBOL_RATE2, NULL);
    sr.reg[1] = rdio_reg_get(rdio, CC1200_SYMBOL_RATE1, NULL);
    sr.reg[0] = rdio_reg_get(rdio, CC1200_SYMBOL_RATE0, NULL);

    sr_e = (sr.reg[2] & CC1200_SYMBOL_RATE2_SRATE_E_M) >> CC1200_SYMBOL_RATE2_SRATE_E_S;
    sr.reg[2] &= ~CC1200_SYMBOL_RATE2_SRATE_E_M;

    if (sr_e) {
        div = UINT64_C(1) << (39 - sr_e);
        sr.val += 1u << 20;
    } else {
        div = UINT64_C(1) << 38;
    }

    /* NOTE: Storage format is in ksps but might eventually use sps and mul * 1k here. */
    sr.val *= CC_XOSC_FREQ;
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    sr.val += div >> 1;
    sr.val /= div;

    return (u32)sr.val;
}

u32 rdio_util_set_symbol_rate(rdio_t rdio, u32 symbol_rate)
{
    union {
        u8 reg[sizeof(u64)];
        u64 val;
    } sr;

    u8 sr_e = 0;
    u64 div = CC_XOSC_FREQ;

    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    sr.val = (((u64)symbol_rate << 38) + (div >> 1)) / div;

    while (sr.val >= (UINT64_C(1)<<20) && sr_e < (CC1200_SYMBOL_RATE2_SRATE_E_M>>CC1200_SYMBOL_RATE2_SRATE_E_S)) {
        ++sr_e;
        /* NOTE: This rounds .5 to the next integer away from zero. However,
         * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
        sr.val = (((u64)symbol_rate << (39 - sr_e)) + (div >> 1)) / div - (UINT64_C(1)<<20);
    }

    sr.reg[2] |= sr_e << CC1200_SYMBOL_RATE2_SRATE_E_S;

    /* TODO: Potentially byte swap and then write in order. */
    rdio_reg_set(rdio, CC1200_SYMBOL_RATE2, sr.reg[2]);
    rdio_reg_set(rdio, CC1200_SYMBOL_RATE1, sr.reg[1]);
    rdio_reg_set(rdio, CC1200_SYMBOL_RATE0, sr.reg[0]);
    ccrf_trace_verbose("SYMBOL_RATE = [ 2: 0x%02X, 1: 0x%02X, 0: 0x%02X ]", sr.reg[2], sr.reg[1], sr.reg[0]);

    /* TODO: Determine whether a read-back is needed first.
     * Otherwise, just convert back in case of rounding errors
     * in order to indicate the actual configured symbol rate. */
    sr.reg[2] &= ~CC1200_SYMBOL_RATE2_SRATE_E_M;

    if (sr_e) {
        div = UINT64_C(1) << (39 - sr_e);
        sr.val += 1u << 20;
    } else {
        div = UINT64_C(1) << 38;
    }

    /* NOTE: Storage format is in ksps but might eventually use sps and mul * 1k here. */
    sr.val *= CC_XOSC_FREQ;
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    sr.val += div >> 1;
    sr.val /= div;

    return (u32)sr.val;
}

u32 rdio_util_get_rx_filt_bw(rdio_t rdio)
{
    const u8 cbw = rdio_reg_get(rdio, CC1200_CHAN_BW, NULL);
    const u8 adf = CC1200_CHAN_BW_ADC_CIC_DECFACT_BASE << ((cbw & CC1200_CHAN_BW_ADC_CIC_DECFACT_M) >> CC1200_CHAN_BW_ADC_CIC_DECFACT_S);
    const u8 bdf = (cbw & CC1200_CHAN_BW_BB_CIC_DECFACT_M) >> CC1200_CHAN_BW_BB_CIC_DECFACT_S;

    const u32 div = adf * bdf * 2u;

    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    return (CC_XOSC_FREQ + (div >> 1)) / div;
}

u32 rdio_util_set_rx_filt_bw(rdio_t rdio, u32 rx_filt_bw)
{
    #define DEC_MIN ((CC1200_CHAN_BW_ADC_CIC_DECFACT_BASE << CC1200_CHAN_BW_ADC_CIC_DECFACT_MIN) \
                        * CC1200_CHAN_BW_BB_CIC_DECFACT_MIN * 2)
    #define DEC_MAX ((CC1200_CHAN_BW_ADC_CIC_DECFACT_BASE << CC1200_CHAN_BW_ADC_CIC_DECFACT_MAX) \
                        * CC1200_CHAN_BW_BB_CIC_DECFACT_MAX * 2)
    #define RX_FILT_BW_MIN ((CC_XOSC_FREQ + (DEC_MAX >> 1)) / DEC_MAX)
    #define RX_FILT_BW_MAX ((CC_XOSC_FREQ + (DEC_MIN >> 1)) / DEC_MIN)

    if (rx_filt_bw > RX_FILT_BW_MAX) rx_filt_bw = RX_FILT_BW_MAX;
    else if (rx_filt_bw < RX_FILT_BW_MIN) rx_filt_bw = RX_FILT_BW_MIN;

    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    u16 adf_bdf = (u16)((CC_XOSC_FREQ + rx_filt_bw) / (rx_filt_bw >> 1)); // Dividing by rx_filt_bw * 2

    u8 adf, bdf;
    u8 adf_best = CC1200_CHAN_BW_ADC_CIC_DECFACT_BASE << CC1200_CHAN_BW_ADC_CIC_DECFACT_MIN;
    u8 bdf_best = CC1200_CHAN_BW_BB_CIC_DECFACT_MIN;
    u8 adfs = CC1200_CHAN_BW_ADC_CIC_DECFACT_MAX;
    u16 adf_bdf_diff_best = (DEC_MAX / 2) + 1;

    do {
        if ((adf = CC1200_CHAN_BW_ADC_CIC_DECFACT_BASE << adfs) > adf_bdf) continue;

        /* NOTE: This rounds .5 to the next integer away from zero. However,
         * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
        if ((bdf = (u8)((adf_bdf + (adf >> 1)) / adf)) > CC1200_CHAN_BW_BB_CIC_DECFACT_MAX) {
            adf_best = adf;
            bdf_best = CC1200_CHAN_BW_BB_CIC_DECFACT_MIN;
            break;
        }

        if (bdf < CC1200_CHAN_BW_BB_CIC_DECFACT_MIN) continue;

        const u16 adf_bdf_cur = adf * bdf;
        const u16 adf_bdf_diff = (adf_bdf_cur > adf_bdf) ? (adf_bdf_cur - adf_bdf) : (adf_bdf - adf_bdf_cur);

        if (adf_bdf_diff < adf_bdf_diff_best) {
            adf_best = adf;
            bdf_best = bdf;
            if (!adf_bdf_diff) break;
        }
    } while (adfs-- > CC1200_CHAN_BW_ADC_CIC_DECFACT_MIN);

    const u8 cbw = ((adf_best << CC1200_CHAN_BW_ADC_CIC_DECFACT_S) & CC1200_CHAN_BW_ADC_CIC_DECFACT_M)
                 | ((bdf_best << CC1200_CHAN_BW_BB_CIC_DECFACT_S) & CC1200_CHAN_BW_BB_CIC_DECFACT_M);

    rdio_reg_set(rdio, CC1200_CHAN_BW, cbw);
    ccrf_trace_verbose("CHAN_BW = 0x%02X", cbw);

    const u32 div = adf_best * bdf_best * 2u;

    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    return (CC_XOSC_FREQ + (div >> 1)) / div;
}

u64 rdio_util_get_wor_event0(rdio_t rdio)
{
    const u8 wr = (rdio_reg_get(rdio, CC1200_WOR_CFG1, NULL) & CC1200_WOR_CFG1_WOR_RES_M) >> CC1200_WOR_CFG1_WOR_RES_S;
    const u16 ev = rdio_reg_get16(rdio, CC1200_WOR_EVENT0_MSB, NULL);
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    u64 ns = ((1000000000ul * ev * (UINT64_C(1) << (wr * 5))) + (CC_RCOSC_FREQ >> 1)) / CC_RCOSC_FREQ;
    return ns;
}

u64 rdio_util_set_wor_event0(rdio_t rdio, u64 ns)
{
    u8 wr;
    u64 ev = 1;

    for (wr = CC1200_WOR_CFG1_WOR_RES_MIN; wr <= CC1200_WOR_CFG1_WOR_RES_MAX; ++wr) {
        ev = ((ns * CC_RCOSC_FREQ) >> (5 * wr)) / 1000000000ul;
        if (ev <= UINT16_MAX) break;
        else ev = 1;
    }

    rdio_reg_update(rdio, CC1200_WOR_CFG1, CC1200_WOR_CFG1_WOR_RES_M, wr << CC1200_WOR_CFG1_WOR_RES_S);
    rdio_reg_set16(rdio, CC1200_WOR_EVENT0_MSB, (u16)ev);

    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    ns = ((1000000000ul * ev * (UINT64_C(1) << (wr * 5))) + (CC_RCOSC_FREQ >> 1)) / CC_RCOSC_FREQ;
    return ns;
}

u64 rdio_util_get_rx_timeout(rdio_t rdio)
{
    const u8 rt = (rdio_reg_get(rdio, CC1200_RFEND_CFG1, NULL) & CC1200_RFEND_CFG1_RX_TIME_M) >> CC1200_RFEND_CFG1_RX_TIME_S;

    if (rt == CC1200_RFEND_CFG1_RX_TIME_FOREVER) return 0;

    const u8 wr = (rdio_reg_get(rdio, CC1200_WOR_CFG1, NULL) & CC1200_WOR_CFG1_WOR_RES_M) >> CC1200_WOR_CFG1_WOR_RES_S;
    u16 ev = rdio_reg_get16(rdio, CC1200_WOR_EVENT0_MSB, NULL) >> (rt + 3);
    /*u64 ns = ev >> (rt + 3);
    if (!ns) ns = 1;*/
    if (!ev) ev = 1;
    return ((UINT64_C(1000000000) * 1250ul * ev) << (4 * wr)) / CC_XOSC_FREQ; /* TODO: Is rounding needed? */
}

u64 rdio_util_set_rx_timeout(rdio_t rdio, u64 ns)
{
    u8 wr = CC1200_WOR_CFG1_WOR_RES_MIN;
    u64 ev = 0;
    u8 rt = 0;

    if (ns) {
        do {
            ev = ((ns * CC_XOSC_FREQ) << (/*RX_TIME(=0) +*/ 3)) / ((1250ul << (4ul * wr)) * UINT64_C(1000000000));
            if (ev <= UINT16_MAX) break;
            else ev = 1;
        } while (++wr <= CC1200_WOR_CFG1_WOR_RES_MAX);

    } else {
        rt = CC1200_RFEND_CFG1_RX_TIME_FOREVER;
    }

    rdio_reg_update(rdio, CC1200_WOR_CFG1, CC1200_WOR_CFG1_WOR_RES_M, wr << CC1200_WOR_CFG1_WOR_RES_S);
    rdio_reg_set16(rdio, CC1200_WOR_EVENT0_MSB, (u16)ev);
    rdio_reg_update(rdio, CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RX_TIME_M, rt << CC1200_RFEND_CFG1_RX_TIME_S);

    if (!ns) return 0;

    if (ev < (1<<3))
        ev = (1<<3);

    ev >>= 3;

    return ((1000000000ul * 1250ul * ev) << (4 * wr)) / CC_XOSC_FREQ; /* TODO: Is rounding needed? */
}

s16 rdio_util_get_rssi(rdio_t rdio)
{
    return (s16)(s8)rdio_reg_get(rdio, CC1200_RSSI1, NULL);
}

u32 rdio_util_get_tx_time(rdio_t rdio, u32 len)
{
    // TODO: Make this smarter and base it off of preamble, sync word, crc and others.
    //       The current assumption is: preamble/6, sync/4, len/1, data/len, crc/2,
    //         symbol rate = 250ksps * modmul/1 (=1 for 2-ary, 0.25 for DSSS, 2 for 4-ary)
    const static u32 overhead = 6/*preamble*/ + 4/*sync*/ + 1/*len*/ + 2/*crc*/;
    const static u32 symbol_rate = 250000 * 1/*modmul*/;

    // Note for later: with 4-ary modulation the preamble is still sent as 2-ary.


    return 200/*I/O overhead*/ + ((1000000 * 8 * (len + overhead) /*+ (symbol_rate >> 1)*/) / symbol_rate);
}
