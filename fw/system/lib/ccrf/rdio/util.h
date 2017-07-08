/**
 * Frequency configuration functionality.
 */
#pragma once

#include "rdio/rdio.h"

//typedef u32 freq_t;
//typedef s32 foff_t;

typedef union __packed {
    u8 reg[3];
    u32 val : 24;
} freq_reg_t;

/**
 * Get the frequency band configuration as indicated by the local oscillator
 * (LO) divider. When multiband capability is disabled, this isn't queried in
 * internal code; instead, a constant is used since the band does not vary.
 *
 * In single-band mode, only 900MHz is supported and the return value is:
 *  CC1200_FS_CFG_FSD_BANDSELECT_LO_04_820_960 << 1 (4).
 *
 * @return LO divider value.
 */
u8 rdio_util_get_lo_div(rdio_t rdio);

/**
 * Set the frequency band configuration.
 *
 * @return actual value of frequency band configuration.
 */
u8 rdio_util_set_lo_div(rdio_t rdio, u8 lo_div);

/**
 * Get the frequency offset estimate, calculated by the radio during
 * calibration. Usually zero.
 *
 * @return frequency offset estimate (Hz) indicated by CC1200_FREQOFF_EST.
 */
s32 rdio_util_get_freq_off_est(rdio_t rdio);

/**
 * Get the configured frequency offset used to determine the corrected FS
 * frequency.
 *
 * @return frequency offset (Hz) indicated by CC1200_FREQOFF.
 */
s32 rdio_util_get_freq_off(rdio_t rdio);

/**
 * Set the configured frequency offset used to determine the corrected FS
 * frequency. Can also be set automatically from estimate using the SAFC
 * strobe.
 *
 * @param freq_off frequency offset in Hz.
 *
 * @return actual value of configured frequency offset.
 */
s32 rdio_util_set_freq_off(rdio_t rdio, s32 freq_off);

/**
 * Get the radio operating frequency.
 *
 * @return current frequency in Hz.
 */
u32 rdio_util_get_freq(rdio_t rdio);
freq_reg_t rdio_util_get_freq_reg(rdio_t rdio);

/**
 * Set the radio operating frequency.
 *
 * @param freq desired frequency in Hz.
 *
 * @return actual value set in the radio (Hz).
 */
u32 rdio_util_set_freq(rdio_t rdio, u32 freq);
void rdio_util_set_freq_reg(rdio_t rdio, freq_reg_t reg);

/**
 * Map a desired frequency to an actual setting.
 *
 * @param freq desired frequency in Hz.
 * @param reg optional destination for corresponding register values.
 *
 * @return nearest actual frequency in Hz.
 */
u32 rdio_util_map_freq(rdio_t rdio, u32 freq, freq_reg_t *reg);

/**
 * Get the frequency deviation, the basis for channel TX bandwidth.
 *
 * @return current frequency deviation in Hz.
 */
u32 rdio_util_get_freq_dev(rdio_t rdio);

/**
 * Set the frequency deviation.
 *
 * @param freq_dev frequency deviation in Hz.
 *
 * @return actual value set in the radio (Hz).
 */
u32 rdio_util_set_freq_dev(rdio_t rdio, u32 freq_dev);

/**
 * Get the current symbol rate. Bit rate is calculated by considering
 * the symbol rate, modulation, FEC, DSSS, and manchester configurations.
 * In the simplest cases, this maps to bit rate 1:1.
 *
 * @return current symbol rate in ksps.
 */
u32 rdio_util_get_symbol_rate(rdio_t rdio);

/**
 * Set the symbol rate.
 *
 * @param symbol_rate symbol rate in ksps, typical maximum is 500.
 *
 * @return actual value set in the radio (ksps).
 */
u32 rdio_util_set_symbol_rate(rdio_t rdio, u32 symbol_rate);

/**
 * Get the RX filter bandwidth.
 *
 * @return current RX filter bandwidth (Hz).
 */
u32 rdio_util_get_rx_filt_bw(rdio_t rdio);

/**
 * Set the RX filter bandwidth.
 *
 * @param symbol_rate RX filter bandwidth in Hz. Acceptable range is 9470-1666667 when XOSC is 40 MHz.
 *
 * @return actual value set in the radio (Hz).
 */
u32 rdio_util_set_rx_filt_bw(rdio_t rdio, u32 rx_filt_bw);

/**
 * Get WOR EVENT0 time
 *
 * @return EVENT0 time (ns).
 */
u64 rdio_util_get_wor_event0(rdio_t rdio);

/**
 * Set WOR EVENT0 time
 *
 * @param ns EVENT0 time (ns).
 *
 * @return configured EVENT0 time (ns).
 */
u64 rdio_util_set_wor_event0(rdio_t rdio, u64 ns);

/**
 * Get RX termination timeout
 *
 * @return RX termination timeout (ns).
 */
u64 rdio_util_get_rx_timeout(rdio_t rdio);

/**
 * Set RX termination timeout
 *
 * @param ns RX termination timeout (ns).
 *
 * @return configured RX termination timeout (ns).
 */
u64 rdio_util_set_rx_timeout(rdio_t rdio, u64 ns);

s16 rdio_util_get_rssi(rdio_t rdio);

u32 rdio_util_get_tx_time(rdio_t rdio, u32 len);
