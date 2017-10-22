#pragma once

#include "rdio/rdio.h"
#include "rdio/util.h"


#define CHAN_FREQ_PRECISION 1000
#define CHAN_FREQ_ROUND(freq) ((((freq) + (CHAN_FREQ_PRECISION>>1)) / CHAN_FREQ_PRECISION) * CHAN_FREQ_PRECISION)


typedef u16 chan_id_t;

typedef struct __packed {
    bool valid;

    struct __packed {
        freq_reg_t freq;
        u8 vco[2];
        u8 chp;
    } reg;

} chan_cal_t;

typedef struct __packed {
    chan_id_t id;
    u32 freq;
    chan_cal_t cal;

} chan_info_t;

typedef struct __packed {
    rdio_t rdio;

    struct {
        u32 base;
        u32 bw;
    } freq;

    chan_id_t size;
    chan_info_t *cur;
    chan_info_t chan[];

} chan_group_t;


void chan_group_init(chan_group_t *group, chan_id_t hop_table[]);
void chan_group_calibrate(chan_group_t *group);

void chan_table_reorder(chan_group_t *group, u_int seed, chan_id_t hop_table[]);

void chan_select(chan_group_t *group, chan_id_t id);
void chan_recalibrate(chan_group_t *group, chan_id_t id);
