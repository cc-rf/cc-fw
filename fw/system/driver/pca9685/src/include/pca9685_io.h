#pragma once
#include <usr/type.h>

#define PCA9685_MODE1               0x00
#define PCA9685_MODE2               0x01
#define PCA9685_SUBADDR1            0x02
#define PCA9685_SUBADDR2            0x03
#define PCA9685_SUBADDR3            0x04
#define PCA9685_ALLCALLADDR         0x05

#define PCA9685_CHAN_ON_BASE_L      0x06
#define PCA9685_CHAN_ON_BASE_H      0x07
#define PCA9685_CHAN_OFF_BASE_L     0x08
#define PCA9685_CHAN_OFF_BASE_H     0x09

#define PCA9685_CHAN_ON_L(chan)     ((u8)(PCA9685_CHAN_ON_BASE_L  + (4u * (chan))))
#define PCA9685_CHAN_ON_H(chan)     ((u8)(PCA9685_CHAN_ON_BASE_H  + (4u * (chan))))
#define PCA9685_CHAN_OFF_L(chan)    ((u8)(PCA9685_CHAN_OFF_BASE_L + (4u * (chan))))
#define PCA9685_CHAN_OFF_H(chan)    ((u8)(PCA9685_CHAN_OFF_BASE_H + (4u * (chan))))

#define PCA9685_ALL_ON_L            0xFA
#define PCA9685_ALL_ON_H            0xFB
#define PCA9685_ALL_OFF_L           0xFC
#define PCA9685_ALL_OFF_H           0xFD


#define PCA9685_PRESCALE     0xFE

#define PCA9685_NUMREGS      0xFF
#define PCA9685_MAXCHAN      0x10


typedef enum __packed {
    PCA9685_WRITE,
    PCA9685_READ

} pca9685_rw_t;

typedef void *pca9685_io_handle_t;


pca9685_io_handle_t pca9685_io_init(u8 bus);
void pca9685_io(pca9685_io_handle_t handle, pca9685_rw_t rw, u8 addr, u8 cmd, size_t size, void *data);

