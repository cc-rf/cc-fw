#pragma once

#include <cc/type.h>
#include <cc/freq.h>

typedef u16 chan_t;

typedef struct __packed {
    bool valid;

    struct __packed {
        freq_reg_t freq;
        u8 vco[2];
        u8 chp;
    } reg;

} chan_cal_t;

typedef struct __packed {
    chan_t id;
    u32 freq;
    chan_cal_t cal;

} chan_inf_t;

typedef struct __packed {
    cc_dev_t dev;

    struct {
        u32 base;
        u32 bw;
    } freq;

    chan_t size;
    chan_inf_t *cur;
    chan_inf_t chan[];

} chan_grp_t;

void chan_grp_init(chan_grp_t *grp, chan_t hop_table[]);
void chan_grp_calibrate(chan_grp_t *grp);

void chan_table_reorder(chan_grp_t *grp, u_int seed, chan_t hop_table[]);

void chan_select(chan_grp_t *grp, chan_t id);
