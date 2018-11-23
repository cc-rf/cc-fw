#pragma once

#include "rdio/rdio.h"


#define CC_RSSI_OFFSET      (s8)(-81 - 3)


static const rdio_reg_config_t PHY_RDIO_REG_CONFIG_DEFAULT[] = {

        {CC1200_IOCFG3, CC1200_IOCFG_GPIO_CFG_HW0},
        {CC1200_IOCFG2, CC1200_IOCFG_GPIO_CFG_HW0},
        {CC1200_IOCFG1, CC1200_IOCFG_GPIO_CFG_HIGHZ},
        {CC1200_IOCFG0, CC1200_IOCFG_GPIO_CFG_HW0},

        {CC1200_RFEND_CFG1, CC1200_RFEND_CFG1_RXOFF_MODE_RX | (CC1200_RFEND_CFG1_RX_TIME_FOREVER << CC1200_RFEND_CFG1_RX_TIME_S)},
        {CC1200_RFEND_CFG0, CC1200_RFEND_CFG0_TXOFF_MODE_RX},

        {CC1200_PKT_CFG2,   CC1200_PKT_CFG2_CCA_MODE_RSSI_THR_NOT_RX},
        {CC1200_PKT_CFG1,   CC1200_PKT_CFG1_CRC_CFG_ON_INIT_1D0F | CC1200_PKT_CFG1_ADDR_CHECK_CFG_ON_NO_BCAST | CC1200_PKT_CFG1_APPEND_STATUS | CC1200_PKT_CFG1_WHITE_DATA},
        {CC1200_PKT_CFG0,   CC1200_PKT_CFG0_LENGTH_CONFIG_VARIABLE},
        {CC1200_PKT_LEN,    255}, // CRC Autoflush limits this to 127 in variable packet length mode, and then there needs to be room for the two status bytes
        {CC1200_DEV_ADDR,   0x00},

        {CC1200_RNDGEN,     CC1200_RNDGEN_EN}, // Needed for random backoff for LBT CCA: https://e2e.ti.com/support/wireless_connectivity/f/156/t/370230
        {CC1200_FIFO_CFG,   CC1200_FIFO_CFG_CRC_AUTOFLUSH},

        {CC1200_AGC_GAIN_ADJUST,    (u8)CC_RSSI_OFFSET},
        {CC1200_AGC_CS_THR,         (u8)(-97 - CC_RSSI_OFFSET)},

        {CC1200_SETTLING_CFG, 0x3}, // Defaults except never auto calibrate

};
