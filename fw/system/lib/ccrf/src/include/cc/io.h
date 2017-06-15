#pragma once

#include <cc/common.h>
#include <cc/cc1200.h>

u8 cc_strobe(cc_dev_t dev, u8 cmd);

void cc_read(cc_dev_t dev, u16 addr, u8 *buf, u32 len);
void cc_write(cc_dev_t dev, u16 addr, u8 *buf, u32 len);

u8 cc_get(cc_dev_t dev, u16 addr);
u16 cc_get16(cc_dev_t dev, u16 addr);

void cc_set(cc_dev_t dev, u16 addr, u8 val);
void cc_set16(cc_dev_t dev, u16 addr, u16 val);

void cc_update(cc_dev_t dev, u16 addr, u8 mask, u8 val);
void cc_update16(cc_dev_t dev, u16 addr, u16 mask, u16 val);

void cc_fifo_read(cc_dev_t dev, u8 *buf, u32 len);
void cc_fifo_write(cc_dev_t dev, u8 *buf, u32 len);