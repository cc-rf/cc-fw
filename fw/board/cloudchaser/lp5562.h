#pragma once

#include <usr/type.h>

#define LP5562_NUM_DEVICES          2

#define LP5562_ADDR                 ((u8)(0x30 /*<< 1*/))

#define LP5562_CURRENT_RED          0x07
#define LP5562_CURRENT_GREEN        0x06
#define LP5562_CURRENT_BLUE         0x05
#define LP5562_CURRENT_WHITE        0x0F

#define LP5562_PWM_RED              0x04
#define LP5562_PWM_GREEN            0x03
#define LP5562_PWM_BLUE             0x02
#define LP5562_PWM_WHITE            0x0E

typedef struct lp5562 *lp5562_t;


lp5562_t lp5562_init(u8 addr);
void lp5562_reset(lp5562_t lp);
void lp5562_set_current(lp5562_t lp, u8 chan, u8 current);
void lp5562_set_pwm(lp5562_t lp, u8 chan, u8 pwm);
u8 lp5562_get_current(lp5562_t lp, u8 chan);
