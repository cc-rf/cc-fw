#pragma once

#include <cc/common.h>
#include <cc/io.h>

// TODO: implement a more robust config management system

struct cc_cfg_reg {
    u16 addr;
    u8 data;
};

static inline bool cc_cfg_regs(cc_dev_t dev, const struct cc_cfg_reg regs[], u32 len)
{
    for (u32 i = 0; i < len; i++) {
        cc_set(dev, regs[i].addr, regs[i].data);

        if (regs[i].addr == CC1200_RNDGEN) continue;

        u8 data = cc_get(dev, regs[i].addr);

        if (regs[i].addr == CC1200_SERIAL_STATUS) {
            // Mask out clock signals etc.
            data &= regs[i].data;
        }

        if (data != regs[i].data) {
            cc_dbg("[%u] validation error: addr=0x%04X data=0x%02X data[read-back]=0x%02X", dev, regs[i].addr, regs[i].data, data);
            return false;
        }
    }

    return true;
}

/**
 * Standard medium-rate long range configuration.
 *
 * Deviation        62.5 kHz
 * Modulation       2-GFSK
 * Symbol Rate      250 ksps
 *
 * Notes:
 *
 * AGC_REF could be wrong. w/o 1190 was 0x40, with was 0x35. HGM greyed out, choosing 0x34.
 * FREQOFF_CFG slightly different, FOC has always been on
 * SYNC_THR changed to 8 from C based on SmartRF values.
 * Why does SmartRF not set DVGA_GAIN to -18 as is recommended? Should we use the "reserved" value?
 * Recommended AGC_REF is really high. 0x35
 * AGC_CFG2: changed back to normal mode from optimized linearity mode, and then back again because it may be good
 * recently changed agc offset level in nphy.
 * AGC_CFG1 uupdate. Note that it changes the AGC RSSI step from 3/10 dB to 6/16 dB.
 * SYNC_CFG0: removing unnecessary limitation bit (because s.rate <= 2*rx filter bw) improved things hugely, although avg lqi dropped.
 * AGC_REF appears ideal at 0x34 which should account for inability to configure HGM in SmartRF.
 * IQIC: trying SmartRF-suggested disable. Appears to be the appropriate choice.
 * IFAMP: Against SmartRF suggestion but in line with requirements, change to max (1500kHz). Improvement at low RSSI very notable.
 * DEVIATION_M/MODCFG_DEV_E: Raised deviation from 62.5 to 125. LQI improvement for low rssi, no change for high rssi. lqi does not max out.
 * FREQOFF_CFG: Change ki factor to stop FOC after sync word. Limit now bw/8.
 * AGC_REF: Back to 0x35 (from 0x34) to keep w/SmartRF, but now entirely unsure of actual recommended value. Using rule of thumb and going with 2D.
 * DEVIATION_M/MODCFG_DEV_E: Back to 62.5. Change from GFSK to FSK seems to improve in low rssi cases.
 * SYNC_CFG1: SmartRF changed this up to B when going GFSk->FSK, but going down to 5 seems to work okay too.
 * Testing smaller RX BW: CHAN_BW to 416, IQIC 0x4B->0xCB(enable). SYNC_CFG0:0x13(No more auto clear, but set limitation bit), IF_MIX_CFG:0x18->0x0C IFAMP:0x0D->0x05(then back)
 * Going back to bigger: CHAN_BW to 555, IQIC:0x4B, AGC_REF:0x2D->0x3D->2E->49->31->46->2E->33(somewhat unjustified) AGC_CFG2:0x00->0x20->00 SYNC_CFG0:0x03(no lim), IF_MIX_CFG:0x18(SmartRF sayx 0x1C) IFAMP:0x0D
 * DEVIATION_M/DEV_E: 0x9A/0x04->0x9A/0x25. 4-FSK dev(outer) 125kHz. Great results but a little lossy. lqi's zero??
 *      Next: 0x1F/0x26. dev-> 175kHz (250*.7).
 *      Next: 0x0A/0x26. dev-> 162.5kHz (250*.65). <-- Winner! For when high-speed mode is "enabled."
 * SYNC_CFG1: 0xA5->0xAC. Forgot higher meant more accepting
 * DEVIATION_M/DEV_E: 0x9A/0x04->0x9A/0x05 (Back to 125 kHz). Seems to have fixed some issues with back&forth packets, but lqi has degraded (from 7 to 11 no matter rssi)
 * FREQOFF_CFG:0x24->0x26 (ki factor -> 2, do foc during packet). Can't tell if it helps so restoring. Something up in the low rssi recv now...
 * DEVIATION_M/DEV_E: 0x9A/0x05->0x33/0x05 (Back to 93.75 kHz). Trying to make sure I didn't mess up the low signal strength receive capability. Didn't seem to have an effect, restoring.
 * DEVIATION_M/DEV_E: 0x9A/0x05->0x9A/0x0D (FSK->GFSK). This is good, keeping. Almost entirely fixes low rssi concern.
 * SYNC_CFG1: 0xAC->0xA5. More strict sync word qual. Meant to improve PER above sensitivity limit. https://e2e.ti.com/support/wireless_connectivity/proprietary_sub_1_ghz_simpliciti/f/156/p/374447/1338299
 * IF_MIX_CFG: 0x18->0x1C: f_IF only needs to be greater than HALF the RX filt BW when IQIC is not enabled (https://e2e.ti.com/support/wireless_connectivity/proprietary_sub_1_ghz_simpliciti/f/156/p/374447/1317974#1317974)
 *              (Restored: does not help)
 * IFAMP: 0x0D->0x04. Single side BW must be > f_IF + (RXBW/2) so 555.5. Restoring: seems to still not be useful.
 * IF_MIX_CFG: 0x18->0x1C: Again because showed (a bit) improvement at very low signals, although changing IFAMP still did not.
 * DEVIATION_M/DEV_E: 0x9A/0x0D->0xFF/0x0D (156 kHz). Trying to improve lqi.
 *
 * TODO: Research more about DC offset removal (DCFILT), Low-IF and image correction. Also look at DCFILT auto vs. fixed compensation.
 * TODO: Revisit FB2PLL (FREQOFF_CFG)
 * TODO: Look at MDMCFG1 collision detect (|=0x08) and CS sync search gate (|=0x80).
 */

static const struct cc_cfg_reg CC_CFG_DEFAULT[] = {
        {CC1200_SYNC3,             0x5A},
        {CC1200_SYNC2,             0x0F},
        {CC1200_SYNC1,             0xBE},
        {CC1200_SYNC0,             0x66},
        {CC1200_SYNC_CFG1,         0xA5},
        {CC1200_SYNC_CFG0,         0x03},
        {CC1200_DEVIATION_M,       0xFF},
        {CC1200_MODCFG_DEV_E,      0x0D},
        {CC1200_DCFILT_CFG,        0x5D},
        {CC1200_PREAMBLE_CFG1,     0x18},
        {CC1200_PREAMBLE_CFG0,     0x8A},
        {CC1200_IQIC,              0x4B},
        {CC1200_CHAN_BW,           0x03},
        {CC1200_MDMCFG1,           0x42},
        {CC1200_MDMCFG0,           0x05},
        {CC1200_SYMBOL_RATE2,      0xB9},
        {CC1200_SYMBOL_RATE1,      0x99},
        {CC1200_SYMBOL_RATE0,      0x9A},
        {CC1200_AGC_REF,           0x33},
        {CC1200_AGC_CS_THR,        (u8)-117},
        {CC1200_AGC_CFG2,          0x00},
        {CC1200_AGC_CFG1,          0x51},
        {CC1200_AGC_CFG0,          0x87},
        {CC1200_FIFO_CFG,          0x00},
        {CC1200_FS_CFG,            0x12},
        {CC1200_PKT_CFG2,          0x00},
        {CC1200_PKT_CFG0,          0x20},
        {CC1200_PA_CFG1,           0x7F}, // w/pa: 0x55 == 17dBm 0x77 == 26+dBm other: 0x63 == 0dBm 0x43 == min
        {CC1200_PA_CFG0,           0x51},
        {CC1200_PKT_LEN,           0xFF},
        {CC1200_IF_MIX_CFG,        0x1C},
        {CC1200_FREQOFF_CFG,       0x24},
        {CC1200_TOC_CFG,           0x03},
        {CC1200_MDMCFG2,           0x02},
        {CC1200_FREQ2,             0x5C},
        {CC1200_FREQ1,             0x0F},
        {CC1200_FREQ0,             0x5C},
        {CC1200_IF_ADC1,           0xEE},
        {CC1200_IF_ADC0,           0x10},
        {CC1200_FS_DIG1,           0x07},
        {CC1200_FS_DIG0,           0xAF},
        {CC1200_FS_CAL3,           0x40},
        {CC1200_FS_CAL1,           0x40},
        {CC1200_FS_CAL0,           0x0E},
        {CC1200_FS_DIVTWO,         0x03},
        {CC1200_FS_DSM0,           0x33},
        {CC1200_FS_DVC0,           0x17},
        {CC1200_FS_PFD,            0x00},
        {CC1200_FS_PRE,            0x6E},
        {CC1200_FS_REG_DIV_CML,    0x1C},
        {CC1200_FS_SPARE,          0xAC},
        {CC1200_FS_VCO0,           0xB5},
        {CC1200_IFAMP,             0x0D},
        {CC1200_XOSC5,             0x0E},
        {CC1200_XOSC1,             0x03},
};
