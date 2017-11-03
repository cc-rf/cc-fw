#pragma once

#include "rdio/rdio.h"


/**
 * Standard medium-rate long range configuration.
 *
 * Deviation        50 kHz
 * Modulation       2-GFSK
 * Symbol Rate      200 ksps
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
 * AGC_CFG1 update. Note that it changes the AGC RSSI step from 3/10 dB to 6/16 dB.
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
 * FS_DIG0:0xAF->0xA0(FS Loop BW in RX/TX from 500kHz to 200kHz). Latest potential SmartRF value. Barely makes a difference but does appear to be good.
 * TOC_CFG: Leaving as default, SmartRF maybe doesn't use.
 * AGC_CFG1: 0x51->0x00. Minimum windows sizes. Benefit of this and next two are unclear so far.
 * AGC_CFG0: 0x87->0x40. RSSI_VALID_CNT from 2 to 1 sample. Hysteresis level 2(or was it 7?)->4dB. ASK_DECAY: don't care.
 * SYNC_CFG1: 0xA5->0xA9. Make SYNC_THR slightly more tolerant.
 * PREAMBLE_CFG0: 0x8A->0x8F. Make maximally tolerant of low-quality preambles.
 * PREAMBLE_CFG1: 0x18->0x20. Preamble size 4->6 bytes. Needed to support large packets, but more especially highly variable packet sizes.
 * SYNC_CFG1: 0xA9->0xAA. Tuning this more won't change much it seems.
 * PREAMBLE_CFG1: 0x20->0x28->0x2C. Preamble size 6->8->12 bytes. Was still seeing loss when packet size varies, but this takes care of it (does not seem to be costly despite the big overhead increase)
 * PA_CFG1: 0x7F->0x77: Latter was the highest recommended by TI, and output power can decrease if the PA's input level is too high.
 * FREQOFF_CFG:0x24->0x26 (ki factor -> 2, do foc during packet). Trying again because it has been noticed that longer packets are more often dropped.
 * PREAMBLE_CFG1: 0x2C->0x18. Preamble size 12->4 bytes. Finding it hard to believe it needs to be so long...
 * DEVIATION_M/DEV_E: 0xFF/0x0D->0x9A/0x0C->0x48/0x0D->0x48/0x0C. Deviation 156kHz->62.5kHz->100kHz->50kHz. Feeling like ~MSK is the correct choice. 62.5 magically solved the periodic random drop issues! Trying 100 gave drops after ~7min, but then 50 was good.
 * DCFILT_CFG: 0x5D->0x1D. Enable. Need to explore these values further.
 * DEVIATION_M/DEV_E: 0x48/0x0C->0x48/0x04. 2-GFSK->2-FSK. Trying with new lower deviation. New LQI is only +1/2 (to ~9/8) on 60dB attenuator. Will keep and evaluate.
 * DCFILT_CFG: 0x1D->0x5D. Disable. Still unlcear on benefit.
 * TEMP: Testing smaller RX BW (again): CHAN_BW to 416 (0x03->0x04), IQIC 0x4B->0xCB(enable). SYNC_CFG0:0x03->0x13(No more auto clear, but set limitation bit), IF_MIX_CFG:0x1C->0x1C(Why was it this already?!) IFAMP:0x0D->0x09(ssbw=1MHz, could be 600kHz(0x05) but that is < 615kHz (f_IF + RXBW/2))
 * TEMP: ^ Results so far good, next to explore alternate decimation factor with same rx bw. (CHAN_BW->0x81, dec fact 48). SSBW can go to 600kHz (IFAMP->0x05), IF_MIX_CFG:0x1C->0x01(f_IF becomes 208.33kHz)
 * TEMP: ^ End result: do not keep IQIC, DO keep RX config limitation (improves LQI by ~5), go back to 555kHz RXBW, Fix wrong IF_MIX_CFG (was set for 400kHz rxbw)
 * IF_MIX_CFG:0x08. f_IF->555.5kHz. IFAMP:0x09. SSBW->1MHz.
 * IF_MIX_CFG:0x04. f_IF->833.3kHz. IFAMP:0x0D. SSBW->1.5MHz. IQIC:0x4B->0x80. Enable, no coefficients, low sample counts. Overall better LQI, sensitivty est. down 3dB. Sensitivity improved 5dB at the cost of -8 lqi (~97/92dBm) when using this config with f_IF back to 555.5kHz (same at -51dBm, much loss)
 * DEVIATION_M/DEV_E: 0x48/0x04->0x89/0x04->0x9A/0x04->0x48/0x05->0x48/0x0D->0x48/0x0C->0x48/0x04. Deviation 50kHz->60kHz->62.5kHz->100kHz->GFSK->50kHz->FSK. Seems to be generally the best choice.
 * AGC_CFG2: 0x00->0x20->0x00. AGC mode back to normal (instead of optimized linearity mode). Needs more testing before kept.
 * PREAMBLE_CFG1: Saw varied performance/sensitivity at higher values and more accurate RSSI measurement at 5 but magically 4 seems to always be best, although the rssi measurement is down by exactly 10dB.
 * DEVIATION_M/DEV_E: 0x48/0x04->0xCB/0x04->0x68/0x05->0x89/0x05->0x48/0x05->0x48/0x0D. Deviation 50kHz->70kHz->110kHz->120kHz->100kHz->GFSK.
 * AGC_CFG3: 0xB1->0x31. Freeze AGC gain and RSSI -> Freeze AGC but keep measuring RSSI. Makes RSSI measurement much more accurate without having to alter preamble length!
 * AGC_CFG2: 0x00->0x20. AGC mode -> back to normal. Might be making more sense now.
 * FREQOFF_CFG: 0x26->0x34->0x2C. Enable FB2PLL with medium loop gain at rxbw/8 max -> high loop gain.
 * IF_MIX_CFG:0x08. f_IF->555.5kHz. IFAMP:0x09. SSBW->1MHz. Now performs better thanks to FB2PLL.
 * AGC_CFG2: 0x20->0x00. Yet more differences, going to leave it here FOREVER.
 * AGC_CFG1: 0x51->0x0C. RSSI step 3dB, AGC win size 16 samples, settle 64 samples
 * AGC_CFG0: 0x87->0x0B->0x07->0x0B. Hysteresis 2dB, RSSI avg 5 samples -> RSSI avg 2 samples -> back to 5.
 * TEMP: CHAN_BW: 0x03->0x04. Should be able to go to 416kHz now, FB2PLL adds a bit to RXBW and with 10pmm crystal min rxbw is 436.6kHz (rate+2*dev+4*ppm*freq). Result: great, but lqi drops by 3.
 * TEMP: DEVIATION_M/DEV_E: 0x48/0x0D->0x27/0x0D->0x48/0x0D. Deviation 100kHz->90kHz. Reduces min rxbw to exactly 416kHz. Putting back, prefer phased shift and performance/lqi clearly better at -100dB.
 * TEMP: IQIC: 0x80->0x00->0xD8. SmartRF disables IQIC when reducing RXBW but its default for other configs has it on. Became useful to have off after reducing IF freq to 416kHz. These two changes together bring a ~+15dB RSSI level indication increase that seems maybe inaccurate (update: 100dB of atten reads as -85dBm, 90dB reads as -95, clearly signal was just too low).
 * TEMP: IF_MIX_CFG: 0x08->0x18->0x1C. -f_IF -> f_IF, 555kHz->416kHz. IFAMP: 0x09->0x05->0x09 (SSBW 1MHz->600kHz->back). 600kHz clearly was causing issues.
 * ^ Overall okay results but not as good across power range as with larger rxbw.
 * IQIC: 0x80->0x00->0xD8. SmartRF cannot decide it seems. 0xD8 better than 0x80 but 3dB down.
 * IF_MIX_CFG: 0x08->0x18. Invert f_IF.
 * SYNC_CFG1: 0x13->0x33. Re-enable auto clear (SmartRF).
 * AGC_CFG0: 0x0B->0x4F. Hysteresis 2dB->4dB. RSSI valid cnt 5->9. The RSSI count further improved measurement accuracy.
 * AGC_CFG1: 0x0C->0x12->0x08->0x1D->0x1E->0x27->0x1E. AGC settle/64->settle/40. AGC win/32->win/16,settle/24->64/80->64/96->128/127->64/80. 64/80 improves RSSI measurement accuracy.
 * PREAMBLE_CFG1: 0x18->0x20. Preamble size 4->6 bytes. To help with synchronization and support greter packet-to-packet size variation.
 * DEVIATION_M/DEV_E: 0x48/0x0D->0x9A/0x0C. Deviation 100kHz->62.5kHz. I forgot that this is MSK when the rate is 250kbps (NOT 200!).
 * ---
 * --- 2017-08-05: Tuning for V2 and crystal @38.4MHz.
 * ---
 * DEVIATION_M/DEV_E: 0x9A/0x0C->0xAB/0x0C. Same deviation but value given by SmartRF for new xtal.
 * IQIC: 0x80->0x58. Back to SmartRF recommended, which is disabled. Brought LQI from 23 to 5 and no more drops for simple xfers!
 * FS_DIG1/0: 0x07/0xA0->0x04/0x55. From SmartRF.
 * FS_CAL3: 0x40->0x00 (default). SmartRF.
 * TOC_CFG: Default->0x03. SmartRF (Timing Offset Correction). LQI 5->4.
 * FREQOFF_CFG: 0x2C->0x0C. Disable Frequency Offset Correction, has little impact.
 * IQIC,DEVIATION_M: Change based on xtal freq to SmartRF values.
 * Changing rate to 200ksps, deviation to 50kHz. Makes for a solid high-quality signal, works @4GFSK with dev=100 nicely (when rxbw=800).
 * ^ SYNC_CFG0: Trying to unset RX_CONFIG_LIMITATION brings LQI from 1->4.
 * PREAMBLE_CFG1: 0x20->0x18->0x28->0x30->0x28. Preamble 6->4->8->24->8 bytes.
 * PREAMBLE_CFG1: 0x28->0x18. Preamble 8->4 bytes.
 *
 * TODO: Research more about DC offset removal (DCFILT), Low-IF and image correction. Also look at DCFILT auto vs. fixed compensation.
 * TODO: Revisit FB2PLL (FREQOFF_CFG)
 * TODO: Look at MDMCFG1 collision detect (|=0x08) and CS sync search gate (|=0x80).
 *
 * 2017-02-25: TI DN005 Gives some insight on deviation selection and DC filter impact. http://www.ti.com/lit/an/swra122c/swra122c.pdf
 */
static const rdio_reg_config_t RDIO_REG_CONFIG_DEFAULT[] = {
        {CC1200_SYNC3,             0x5A},
        {CC1200_SYNC2,             0x0F},
        {CC1200_SYNC1,             0xBE},
        {CC1200_SYNC0,             0x66},
        {CC1200_SYNC_CFG1,         0xAA},
        {CC1200_SYNC_CFG0,         0x33},
        #if defined(BOARD_CLOUDCHASER) && BOARD_REVISION == 2
        {CC1200_IQIC,              0x58},
        {CC1200_DEVIATION_M,       0x55},
        #else
        {CC1200_IQIC,              0xD8},
        {CC1200_DEVIATION_M,       0x47},
        #endif
        {CC1200_MODCFG_DEV_E,      0x0C},
        {CC1200_DCFILT_CFG,        0x4B},
        {CC1200_PREAMBLE_CFG1,     0x18},
        {CC1200_PREAMBLE_CFG0,     0x8F},
        {CC1200_CHAN_BW,           0x03},
        {CC1200_MDMCFG1,           0x42},
        {CC1200_MDMCFG0,           0x05},
        #if defined(BOARD_CLOUDCHASER) && BOARD_REVISION == 2
        {CC1200_SYMBOL_RATE2,      0xB5},
        {CC1200_SYMBOL_RATE1,      0x55},
        {CC1200_SYMBOL_RATE0,      0x55},
        #else
        {CC1200_SYMBOL_RATE2,      0xB4},
        {CC1200_SYMBOL_RATE1,      0x7A},
        {CC1200_SYMBOL_RATE0,      0xE1},
        #endif
        {CC1200_AGC_REF,           0x35},
        {CC1200_AGC_CFG3,          0x31},
        {CC1200_AGC_CFG2,          0x00},
        {CC1200_AGC_CFG1,          0x27},
        {CC1200_AGC_CFG0,          0x4F},
        {CC1200_FIFO_CFG,          0x00},
        {CC1200_FS_CFG,            0x12},
        {CC1200_PA_CFG1,           0x77}, // w/pa: 0x55 == 17dBm 0x5A == 20dBm 0x77 == 26+dBm other: 0x63 == 0dBm 0x43 == min
        {CC1200_PA_CFG0,           0x51},
        {CC1200_IF_MIX_CFG,        0x18},
        {CC1200_FREQOFF_CFG,       0x0C},
        {CC1200_TOC_CFG,           0x03},
        {CC1200_MDMCFG2,           0x02},
        {CC1200_FREQ2,             0x5C},
        {CC1200_FREQ1,             0x0F},
        {CC1200_FREQ0,             0x5C},
        {CC1200_IF_ADC1,           0xEE},
        {CC1200_IF_ADC0,           0x10},
        {CC1200_FS_DIG1,           0x04},
        {CC1200_FS_DIG0,           0x55},
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
        {CC1200_IFAMP,             0x09},
        {CC1200_XOSC5,             0x0E},
        {CC1200_XOSC1,             0x03},
};


/**
 * Standard medium-fast medium range configuration.
 *
 * Deviation        150 kHz
 * Modulation       4-GFSK
 * Symbol Rate      250 ksps
 *
 * Notes:
 *
 * 2017-05-20: Copied above config using old notes to revive a 4GFSK setup.
 * DEVIATION_M/DEV_E: 0xEB/0x2D->0x0A/0x2E->0x0A/0x26. dev 150kHz->162.5kHz, 4-GFSK->FSK
 * DEVIATION_M/DEV_E: ->0x9A/0x25. dev ->125kHz.
 *
 * Currently this still sucks.
 */
static const rdio_reg_config_t RDIO_REG_CONFIG_DEFAULT_1[] = {
        {CC1200_SYNC3,             0x5A},
        {CC1200_SYNC2,             0x0F},
        {CC1200_SYNC1,             0xBE},
        {CC1200_SYNC0,             0x66},
        {CC1200_SYNC_CFG1,         0xAA},
        {CC1200_SYNC_CFG0,         0x33},
        {CC1200_DEVIATION_M,       0x9A},
        {CC1200_MODCFG_DEV_E,      0x25},
        {CC1200_DCFILT_CFG,        0x5D},
        {CC1200_PREAMBLE_CFG1,     0x20},
        {CC1200_PREAMBLE_CFG0,     0x8F},
        {CC1200_IQIC,              0x80},
        {CC1200_CHAN_BW,           0x03},
        {CC1200_MDMCFG1,           0x42},
        {CC1200_MDMCFG0,           0x05},
        {CC1200_SYMBOL_RATE2,      0xB9},
        {CC1200_SYMBOL_RATE1,      0x99},
        {CC1200_SYMBOL_RATE0,      0x9A},
        {CC1200_AGC_REF,           0x35},
        {CC1200_AGC_CFG3,          0x31},
        {CC1200_AGC_CFG2,          0x00},
        {CC1200_AGC_CFG1,          0x27},
        {CC1200_AGC_CFG0,          0x4F},
        {CC1200_FIFO_CFG,          0x00},
        {CC1200_FS_CFG,            0x12},
        {CC1200_PA_CFG1,           0x77}, // w/pa: 0x55 == 17dBm 0x5A == 20dBm 0x77 == 26+dBm other: 0x63 == 0dBm 0x43 == min
        {CC1200_PA_CFG0,           0x51},
        {CC1200_IF_MIX_CFG,        0x18},
        {CC1200_FREQOFF_CFG,       0x2C},
        {CC1200_MDMCFG2,           0x02},
        {CC1200_FREQ2,             0x5C},
        {CC1200_FREQ1,             0x0F},
        {CC1200_FREQ0,             0x5C},
        {CC1200_IF_ADC1,           0xEE},
        {CC1200_IF_ADC0,           0x10},
        {CC1200_FS_DIG1,           0x07},
        {CC1200_FS_DIG0,           0xA0},
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
        {CC1200_IFAMP,             0x09},
        {CC1200_XOSC5,             0x0E},
        {CC1200_XOSC1,             0x03},
};




/**
 * New max rate configuration.
 *
 * Deviation        600 kHz
 * Modulation       4-GFSK
 * Symbol Rate      600 ksps
 *
 * Notes:
 *
 * SYNC_CFG1: 0xA8->0xAA. Increasing sync word match tolerance 0x8->0xA.
 * SYNC_CFG0: 0x03->0x13. RX config limitation bit.
 * CHAN_BW: 0x01->0x41->0x01. RX bandwidth 1.6MHz->833kHz->1.6MHz.
 * DEVIATION_M/DEV_E: 0x47/0x2F->0x9A/0x2E->0x48/0x2F. dev 399kHz->250kHz->400kHz.
 * PREAMBLE_CFG1: 0x00->0x20. Preamble length 3->6 bytes.
 * AGC_REF: 0x33->0x38. SmartRF recommended after changing chan bw.
 * SYNC_CFG0: 0x13->0x03. Does not work when set, but not needed anyway.
 * DEVIATION_M/DEV_E: 0x48/0x2F->0x9A/0x2F. dev 400kHz->500kHz.
 * AGC_CFG2: 0x60->0x00. Default mode.
 * IF_MIX_CFG: 0x00. Calling out default of Zero-IF mode.
 * SYMBOL_RATE: 0xC9/0x99/0x9A->0xCE/0xB8/0x52. 500ksps->600ksps.
 * DEVIATION_M/DEV_E: 0x9A/0x2F->0xEC/0x2F. dev 500kHz->600kHz.
 * PA_CFG1: 0x77->0x47. PA power +10dBm -> -14dBm. Should produce -1dBm with HGM off.
 * PA_CFG0: Default->0x50. SmartRF.
 * AGC_CFG2: 0x00->0x60. SmartRF (Zero-IF).
 * AGC_REF: 0x33->0x39.
 * PREAMBLE_CFG1: 0x20->0x18->0x28->0x30->0x28. Preamble 6->4->8->24->8 bytes.
 *
 */
static const rdio_reg_config_t RDIO_REG_CONFIG_DEFAULT_MAXRATE[] = {
        {CC1200_SYNC3,             0x5A},
        {CC1200_SYNC2,             0x0F},
        {CC1200_SYNC1,             0xBE},
        {CC1200_SYNC0,             0x66},
        {CC1200_PA_CFG1,           0x47},

        {CC1200_SYNC_CFG1,         0xAA},
        {CC1200_SYNC_CFG0,         0x03},
        #if defined(BOARD_CLOUDCHASER) && BOARD_REVISION == 2
        {CC1200_DEVIATION_M,       0xFF},
        #else
        {CC1200_DEVIATION_M,       0xEC},
        #endif
        {CC1200_MODCFG_DEV_E,      0x2F},
        {CC1200_DCFILT_CFG,        0x1E},
        {CC1200_PREAMBLE_CFG1,     0x28},
        {CC1200_PREAMBLE_CFG0,     0x8A},
        {CC1200_IQIC,              0x00},
        {CC1200_CHAN_BW,           0x01},
        {CC1200_MDMCFG1,           0x42},
        {CC1200_MDMCFG0,           0x05},
        #if defined(BOARD_CLOUDCHASER) && BOARD_REVISION == 2
        {CC1200_SYMBOL_RATE2,      0xD0},
        {CC1200_SYMBOL_RATE1,      0x00},
        {CC1200_SYMBOL_RATE0,      0x00},
        #else
        {CC1200_SYMBOL_RATE2,      0xCE},
        {CC1200_SYMBOL_RATE1,      0xB8},
        {CC1200_SYMBOL_RATE0,      0x52},
        #endif
        {CC1200_AGC_REF,           0x39},
        {CC1200_AGC_CFG2,          0x60},
        {CC1200_AGC_CFG1,          0x12},
        {CC1200_AGC_CFG0,          0x84},
        {CC1200_FIFO_CFG,          0x00},
        {CC1200_FS_CFG,            0x12},
        {CC1200_IF_MIX_CFG,        0x00},
        {CC1200_PA_CFG0,           0x50},
        {CC1200_FREQOFF_CFG,       0x23},
        {CC1200_MDMCFG2,           0x00},
        {CC1200_FREQ2,             0x5C},
        {CC1200_FREQ1,             0x0F},
        {CC1200_FREQ0,             0x5C},
        {CC1200_IF_ADC1,           0xEE},
        {CC1200_IF_ADC0,           0x10},
        {CC1200_FS_DIG1,           0x04},
        {CC1200_FS_DIG0,           0xA3},
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
