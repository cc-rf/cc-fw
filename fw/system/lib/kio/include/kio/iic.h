#pragma once

#include <usr/type.h>
#include <fsl_common.h>


typedef struct iic *iic_t;

typedef enum __packed {
    IIC_WRITE,
    IIC_READ,

} iic_rw_t;


iic_t iic_init(u8 bus, u32 baud);
status_t iic_io(iic_t iic, iic_rw_t rw, u8 addr, u8 cmd, void *data, size_t size);
