/**
 * Common include file for internal library headers.
 */
#pragma once

#include <cc/type.h>
#include <cc/interface/interface.h>


#define CC_XOSC_FREQ            40000000u       /* External oscillator frequency in Hz. */
#define CC_MULTIBAND            0               /* Enable multiband support. Otherwise fixed at 820-960MHz. */
#define CC_PA_SUPPORT           1               /* Enable functions for using power amplifier */

#ifndef CC_DEV_MIN
#define CC_DEV_MIN              0
#endif

#ifndef CC_DEV_MAX
#define CC_DEV_MAX              (CC_NUM_DEVICES-1)
#endif

#ifndef CC_DEV_VALID
#define CC_DEV_VALID(DEV)       (((DEV) >= CC_DEV_MIN) && ((DEV) <= CC_DEV_MAX))
#endif


#define CC_DEBUG                1
#define CC_DEBUG_VERBOSE        0

#if defined(CC_DEBUG) && CC_DEBUG
#include <stdio.h>

#define cc_dbg_printf(format, ...) printf( format, ##__VA_ARGS__ )

#define cc_dbg(format, ...) cc_dbg_printf("%s: " format "\r\n", __func__, ##__VA_ARGS__ )

#if defined(CC_DEBUG_VERBOSE) && CC_DEBUG_VERBOSE
#define cc_dbg_printf_v(format, ...) cc_dbg_printf( format, ##__VA_ARGS__ )
#define cc_dbg_v(format, ...) cc_dbg( format, ##__VA_ARGS__ )
#else
#define cc_dbg_printf_v(format, ...)
#define cc_dbg_v(format, ...)
#endif

#else

#define cc_dbg_printf(format, ...)
#define cc_dbg(format, ...)
#define cc_dbg_v(format, ...)
#define cc_dbg_printf_v(format, ...)

#endif

/**
 * Operational modes.
 *
 * Calibration
 *   - XOSC differential mode (accuracy etc)
 *   - RC calibration manual vs auto
 *
 * Generic
 *   - Whitening
 *   - Manchester
 *   - Channel config (exclude low qual due to XOSC/2 multiples)
 *   - DC offset removal
 *
 * Receive
 *   - Scanning/hopping, or fixed.
 *   - WOR, eWOR for coordination or not.
 *   - duty cycle or sniff enablement.
 *   - Settings
 *     - FB2PLL
 *     - Filter BW
 *     - RX termination (PQT, CS, timeout)
 *     - Gain?
 *     - RX off mode
 *     - Preamble, sync, packet, CS/CCA, PQT, CRC, FEC, RSSI/LQI info config
 *     - FIFO management
 *     - AGC
 *     - IQIC settings (sample time etc)
 *     - Collision handling
 *
 * Transmit
 *   - Scanning/hopping or fixed
 *   - PA ramping
 *   - CCA
 *   - FIFO
 *   - Off mode
 *   - Packet/preamble/sync
 *   -
*/