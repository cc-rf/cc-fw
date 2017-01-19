#include <cc/freq.h>
#include <cc/io.h>
#include <cc/cc1200.h>

#define CC_RCOSC_FREQ   (CC_XOSC_FREQ/1000u)


u8 cc_get_lo_div(cc_dev_t dev)
{
    #if !CC_MULTIBAND
        (void)dev;
        return CC1200_FS_CFG_FSD_BANDSELECT_LO_04_820_960 << 1;
    #else
        const u8 fs_cfg = cc_get(dev, CC1200_FS_CFG);
        return (fs_cfg & CC1200_FS_CFG_FSD_BANDSELECT_M) << 1;
    #endif
}

u8 cc_set_lo_div(cc_dev_t dev, u8 lo_div)
{
    #if !CC_MULTIBAND
        return cc_get_lo_div(dev);
    #else
        lo_div = ((lo_div >> 1) & CC1200_FS_CFG_FSD_BANDSELECT_M);
        cc_update(dev, CC1200_FS_CFG, CC1200_FS_CFG_FSD_BANDSELECT_M, lo_div);
        return lo_div;
    #endif
}

static s16 cc_get_raw_freq_off_est(cc_dev_t dev)
{
    return (s16)cc_get16(dev, CC1200_FREQOFF_EST1);
}

s32 cc_get_freq_off_est(cc_dev_t dev)
{
    s64 freq_off;

    if (!(freq_off = cc_get_raw_freq_off_est(dev))) return 0;

    const u32 div = cc_get_lo_div(dev) << 18;

    freq_off *= CC_XOSC_FREQ;
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    freq_off < 0 ? (freq_off -= (div >> 1)) : (freq_off += (div >> 1));
    freq_off /= div;
    return (s32)freq_off;
}

static s16 cc_get_raw_freq_off(cc_dev_t dev)
{
    return (s16)cc_get16(dev, CC1200_FREQOFF1);
}

s32 cc_get_freq_off(cc_dev_t dev)
{
    s64 freq_off;

    if (!(freq_off = cc_get_raw_freq_off(dev))) return 0;

    const u32 div = cc_get_lo_div(dev) << 18;

    freq_off *= CC_XOSC_FREQ;
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    freq_off < 0 ? (freq_off -= (div >> 1)) : (freq_off += (div >> 1));
    freq_off /= div;
    return (s32)freq_off;
}

s32 cc_set_freq_off(cc_dev_t dev, s32 freq_off)
{
    s64 fq = (s64)freq_off;
    const u32 div = cc_get_lo_div(dev) << 18;

    fq *= div;
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    fq += CC_XOSC_FREQ >> 1;
    fq /= CC_XOSC_FREQ;

    const s16 fq16 = (s16)fq;
    cc_set16(dev, CC1200_FREQOFF1, fq16);

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

u32 cc_get_freq(cc_dev_t dev)
{
    u32 div = cc_get_lo_div(dev) << 16;
    s16 raw_freq_off = cc_get_raw_freq_off(dev);

    /*union {
        freq_reg_t reg;
        u64 val;
    } fq = { .reg = cc_get_freq_reg() };*/
    u64 val = cc_get_freq_reg(dev).val;

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

freq_reg_t cc_get_freq_reg(cc_dev_t dev)
{
    freq_reg_t fr;
    cc_read(dev, CC1200_FREQ2, (u8 *)&fr, sizeof(fr));
    return (freq_reg_t){ .reg = {fr.reg[2], fr.reg[1], fr.reg[0]} };
}

u32 cc_set_freq(cc_dev_t dev, u32 freq)
{
    freq_reg_t reg;
    freq = cc_map_freq(dev, freq, &reg);
    cc_set_freq_reg(dev, reg);
    return freq;
}

void cc_set_freq_reg(cc_dev_t dev, freq_reg_t fr)
{
    /* TODO: Look at code difference when modifying reg instead. */
    const freq_reg_t sreg = { .reg = {fr.reg[2], fr.reg[1], fr.reg[0]} };
    cc_write(dev, CC1200_FREQ2, (u8 *)&sreg, sizeof(sreg));
}

u32 cc_map_freq(cc_dev_t dev, u32 freq, freq_reg_t *reg)
{
    const u32 div = cc_get_lo_div(dev) << 16;

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

u32 cc_get_freq_dev(cc_dev_t dev)
{
    u8 dev_m, dev_e;
    u64 f;
    u32 div = 1u << 22;

    dev_m = cc_get(dev, CC1200_DEVIATION_M);
    dev_e = cc_get(dev, CC1200_MODCFG_DEV_E);
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

u32 cc_set_freq_dev(cc_dev_t dev, u32 freq_dev)
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
    cc_set(dev, CC1200_DEVIATION_M, reg);
    cc_dbg("[%u] DEVIATION_M = 0x%02X", dev, reg);
    reg = cc_get(dev, CC1200_MODCFG_DEV_E);
    reg |= dev_e << CC1200_MODCFG_DEV_E_DEV_E_S;
    cc_set(dev, CC1200_MODCFG_DEV_E, reg);
    cc_dbg("[%u] MODCFG_DEV_E = 0x%02X", dev, reg);

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

u32 cc_get_symbol_rate(cc_dev_t dev)
{
    union {
        u8 reg[sizeof(u64)];
        u64 val;
    } sr = { .val = 0 };

    u8 sr_e;
    u64 div;

    /* TODO: Potentially read in order and then byte swap. */
    sr.reg[2] = cc_get(dev, CC1200_SYMBOL_RATE2);
    sr.reg[1] = cc_get(dev, CC1200_SYMBOL_RATE1);
    sr.reg[0] = cc_get(dev, CC1200_SYMBOL_RATE0);

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

u32 cc_set_symbol_rate(cc_dev_t dev, u32 symbol_rate)
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
    cc_set(dev, CC1200_SYMBOL_RATE2, sr.reg[2]);
    cc_set(dev, CC1200_SYMBOL_RATE1, sr.reg[1]);
    cc_set(dev, CC1200_SYMBOL_RATE0, sr.reg[0]);
    cc_dbg("[%u] SYMBOL_RATE = [ 2: 0x%02X, 1: 0x%02X, 0: 0x%02X ]", dev, sr.reg[2], sr.reg[1], sr.reg[0]);

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

u32 cc_get_rx_filt_bw(cc_dev_t dev)
{
    const u8 cbw = cc_get(dev, CC1200_CHAN_BW);
    const u8 adf = CC1200_CHAN_BW_ADC_CIC_DECFACT_BASE << ((cbw & CC1200_CHAN_BW_ADC_CIC_DECFACT_M) >> CC1200_CHAN_BW_ADC_CIC_DECFACT_S);
    const u8 bdf = (cbw & CC1200_CHAN_BW_BB_CIC_DECFACT_M) >> CC1200_CHAN_BW_BB_CIC_DECFACT_S;

    const u32 div = adf * bdf * 2u;

    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    return (CC_XOSC_FREQ + (div >> 1)) / div;
}

u32 cc_set_rx_filt_bw(cc_dev_t dev, u32 rx_filt_bw)
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

    cc_set(dev, CC1200_CHAN_BW, cbw);
    cc_dbg("[%u] CHAN_BW = 0x%02X", dev, cbw);

    const u32 div = adf_best * bdf_best * 2u;

    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    return (CC_XOSC_FREQ + (div >> 1)) / div;
}

u64 cc_get_wor_event0(cc_dev_t dev)
{
    const u8 wr = (cc_get(dev, CC1200_WOR_CFG1) & CC1200_WOR_CFG1_WOR_RES_M) >> CC1200_WOR_CFG1_WOR_RES_S;
    const u16 ev = cc_get16(dev, CC1200_WOR_EVENT0_MSB);
    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    u64 ns = ((1000000000ul * ev * (UINT64_C(1) << (wr * 5))) + (CC_RCOSC_FREQ >> 1)) / CC_RCOSC_FREQ;
    return ns;
}

u64 cc_set_wor_event0(cc_dev_t dev, u64 ns)
{
    u8 wr;
    u64 ev = 1;

    for (wr = CC1200_WOR_CFG1_WOR_RES_MIN; wr <= CC1200_WOR_CFG1_WOR_RES_MAX; ++wr) {
        ev = ((ns * CC_RCOSC_FREQ) >> (5 * wr)) / 1000000000ul;
        if (ev <= UINT16_MAX) break;
        else ev = 1;
    }

    cc_update(dev, CC1200_WOR_CFG1, CC1200_WOR_CFG1_WOR_RES_M, wr << CC1200_WOR_CFG1_WOR_RES_S);
    cc_set16(dev, CC1200_WOR_EVENT0_MSB, (u16)ev);

    /* NOTE: This rounds .5 to the next integer away from zero. However,
     * IEEE754 arithmetic rounds .5 towards the nearest even integer. */
    ns = ((1000000000ul * ev * (UINT64_C(1) << (wr * 5))) + (CC_RCOSC_FREQ >> 1)) / CC_RCOSC_FREQ;
    return ns;
}

u64 cc_get_rx_timeout(cc_dev_t dev)
{
    const u8 rt = (cc_get(dev, CC1200_RFEND_CFG1) & CC1200_RFEND_CFG1_RX_TIME_M) >> CC1200_RFEND_CFG1_RX_TIME_S;

    if (rt == CC1200_RFEND_CFG1_RX_TIME_FOREVER) return 0;

    const u8 wr = (cc_get(dev, CC1200_WOR_CFG1) & CC1200_WOR_CFG1_WOR_RES_M) >> CC1200_WOR_CFG1_WOR_RES_S;
    u16 ev = cc_get16(dev, CC1200_WOR_EVENT0_MSB) >> (rt + 3);
    /*u64 ns = ev >> (rt + 3);
    if (!ns) ns = 1;*/
    if (!ev) ev = 1;
    return ((UINT64_C(1000000000) * 1250ul * ev) << (4 * wr)) / CC_XOSC_FREQ; /* TODO: Is rounding needed? */
}

u64 cc_set_rx_timeout(cc_dev_t dev, u64 ns)
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

    cc_update(dev, CC1200_WOR_CFG1, CC1200_WOR_CFG1_WOR_RES_M, wr << CC1200_WOR_CFG1_WOR_RES_S);
    cc_set16(dev, CC1200_WOR_EVENT0_MSB, (u16)ev);
    cc_update(dev, CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RX_TIME_M, rt << CC1200_RFEND_CFG1_RX_TIME_S);

    if (!ns) return 0;

    if (ev < (1<<3))
        ev = (1<<3);

    ev >>= 3;

    return ((1000000000ul * 1250ul * ev) << (4 * wr)) / CC_XOSC_FREQ; /* TODO: Is rounding needed? */
}

void cc_set_mod_cfg_0(cc_dev_t dev)
{
    cc_strobe(dev, CC1200_SIDLE);

    //cc_set_freq_dev(dev, 20000);
    cc_set(dev, CC1200_DEVIATION_M, 0x06);
    cc_set(dev, CC1200_MODCFG_DEV_E, 0x0B);

    cc_set_symbol_rate(dev, 50000);

    //cc_set_rx_filt_bw(dev, 105000);
    cc_set(dev, CC1200_CHAN_BW, 0x84);

    cc_set(dev, CC1200_DCFILT_CFG, 0x56);
    cc_set(dev, CC1200_AGC_REF, /*0x2E*/0x4E/*TI says 3E*/);
    cc_set(dev, CC1200_AGC_CS_THR, /*0x01*/0xEC);
    cc_set(dev, CC1200_IFAMP, 0x05);
    cc_set(dev, CC1200_PA_CFG0, 0x53);

    cc_set(dev, CC1200_SYNC_CFG1, 0x09 | CC1200_SYNC_CFG1_SYNC_MODE_16);
    cc_set(dev, CC1200_PKT_CFG1, CC1200_PKT_CFG1_CRC_CFG_ON_INIT_1D0F |
                                 CC1200_PKT_CFG1_ADDR_CHECK_CFG_OFF |
                                 CC1200_PKT_CFG1_APPEND_STATUS);
    cc_set(dev, CC1200_PREAMBLE_CFG1, 0x4 << 2);

    // FB2PLL off
    //cc_set(dev, CC1200_FREQOFF_CFG, 0x20);

    //cc_update(dev, CC1200_MODCFG_DEV_E, CC1200_MODCFG_DEV_E_MODEM_MODE_M, CC1200_MODCFG_DEV_E_MODEM_MODE_NORMAL);
    //cc_dbg("[%u] MODCFG_DEV_E[NORMAL] = 0x%02X", dev, cc_get(dev, CC1200_MODCFG_DEV_E));
}

void cc_set_mod_cfg_1(cc_dev_t dev)
{
    cc_strobe(dev, CC1200_SIDLE);

    //cc_set_freq_dev(dev, 180000);
    cc_set(dev, CC1200_DEVIATION_M, 0x27); // 180 kHz
    //cc_set(dev, CC1200_DEVIATION_M, 0x89); // 120 kHz
    //cc_set(dev, CC1200_DEVIATION_M, 0x48); // 200 kHz
    cc_set(dev, CC1200_MODCFG_DEV_E, 0x8E); // 180/200 kHz DSSS
    //cc_set(dev, CC1200_MODCFG_DEV_E, 0x2E); // 180/200 kHz NORMAL, 4-GFSK
    //cc_set(dev, CC1200_MODCFG_DEV_E, 0xAD); // 90 kHz DSSS, 4-GFSK
    //cc_set(dev, CC1200_MODCFG_DEV_E, 0x8D); // 120 kHz

    cc_set_symbol_rate(dev, 200000);

    //cc_set_rx_filt_bw(dev, 512000);
    cc_set(dev, CC1200_CHAN_BW, 0x03);

    cc_set(dev, CC1200_DCFILT_CFG, 0x56);
    cc_set(dev, CC1200_AGC_REF, /*0x27*/0x54/*TI says 0x45*/);
    cc_set(dev, CC1200_AGC_CS_THR, /*0x01*/0xEC);
    cc_set(dev, CC1200_IFAMP, 0x05);
    cc_set(dev, CC1200_PA_CFG0, 0x51);

    cc_set(dev, CC1200_SYNC_CFG1, 0xA5);
    cc_set(dev, CC1200_PKT_CFG1, CC1200_PKT_CFG1_CRC_CFG_ON_INIT_1D0F |
                                 CC1200_PKT_CFG1_ADDR_CHECK_CFG_OFF |
                                 CC1200_PKT_CFG1_APPEND_STATUS |
                                 CC1200_PKT_CFG1_WHITE_DATA);
    cc_set(dev, CC1200_PREAMBLE_CFG1, 0x2 << 2);

    // FB2PLL
    //cc_set(dev, CC1200_FREQOFF_CFG, 0x30);

    //cc_update(dev, CC1200_MODCFG_DEV_E, CC1200_MODCFG_DEV_E_MODEM_MODE_M, CC1200_MODCFG_DEV_E_MODEM_MODE_DSSS_PN);
    //cc_dbg("[%u] MODCFG_DEV_E[DSSS] = 0x%02X", dev, cc_get(dev, CC1200_MODCFG_DEV_E));

}

s16 cc_get_rssi(cc_dev_t dev)
{
    //u8 rssi1 = cc_get(dev, CC1200_RSSI1);
    //s16 rssi = rssi1;
    return (s16)(s8)cc_get(dev, CC1200_RSSI1);// - (s16)107/*99*/;
}

u32 cc_get_tx_time(cc_dev_t dev, u32 len)
{
    // TODO: Make this smarter and base it off of preamble, sync word, crc and others.
    //       The current assumption is: preamble/2, sync/4, len/1, data/len, crc/2,
    //         symbol rate divider = 1 (/2 for 4-ary modulation, /4 for DSSS, 4-ary+DSSS not possible)
    //         symbol rate = 250ksps
    const static u32 overhead = 2/*preamble*/ + 4/*sync*/ + 1/*len*/ + 2/*crc*/;
    const static u32 divider = 1;
    const static u32 symbol_rate = 250000;


    const u32 div = (symbol_rate + (4 * divider)) / (8 * divider);

    // round up to next nearest millisecond
    return (1000*(len + overhead) + div) / div;
}
