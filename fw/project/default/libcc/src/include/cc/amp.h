#pragma once

#include <cc/common.h>

typedef enum {
    AMP_HGM,
    AMP_LNA,
    AMP_PA
} amp_t;

void amp_init(cc_dev_t dev);
void amp_ctrl(cc_dev_t dev, amp_t amp, bool enable);

#if !CC_PA_SUPPORT

inline void amp_init(cc_dev_t dev) {}
inline void amp_ctrl(cc_dev_t dev, amp_t amp, bool enable) {}

#endif