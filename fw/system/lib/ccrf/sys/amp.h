#pragma once

#include "rdio/rdio.h"


typedef enum __packed {
    CCRF_AMP_HGM,
    CCRF_AMP_LNA,
    CCRF_AMP_PA

} ccrf_amp_t;

void ccrf_amp_init(rdio_t rdio);
void ccrf_amp_ctrl(rdio_t rdio, ccrf_amp_t amp, bool enable);

static inline void ccrf_amp_mode_rx(rdio_t rdio, bool enable)
{
    ccrf_amp_ctrl(rdio, CCRF_AMP_LNA, enable);
    ccrf_amp_ctrl(rdio, CCRF_AMP_PA, !enable);
}

static inline void ccrf_amp_mode_tx(rdio_t rdio, bool enable)
{
    ccrf_amp_mode_rx(rdio, !enable);
}