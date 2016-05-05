#pragma once

#include <cc/common.h>

#define __CC1200_FLGDEF(x)      ((u8)x)
#define __CC1200_REGDEF(x)      ((u16)x)

#define CC1200_ACCESS_BURST     __CC1200_FLGDEF(0x40)
#define CC1200_ACCESS_SINGLE    __CC1200_FLGDEF(0x00)
#define CC1200_ACCESS_READ      __CC1200_FLGDEF(0x80)
#define CC1200_ACCESS_WRITE     __CC1200_FLGDEF(0x00)

#define CC1200_IOCFG_REG_FROM_PIN(P)  ((u16[]){0, 1, 3})[P]

#define CC1200_IOCFG3                  __CC1200_REGDEF(0x0000)
#define CC1200_IOCFG2                  __CC1200_REGDEF(0x0001)
#define CC1200_IOCFG1                  __CC1200_REGDEF(0x0002)
#define CC1200_IOCFG0                  __CC1200_REGDEF(0x0003)
#define CC1200_SYNC3                   __CC1200_REGDEF(0x0004)
#define CC1200_SYNC2                   __CC1200_REGDEF(0x0005)
#define CC1200_SYNC1                   __CC1200_REGDEF(0x0006)
#define CC1200_SYNC0                   __CC1200_REGDEF(0x0007)
#define CC1200_SYNC_CFG1               __CC1200_REGDEF(0x0008)
#define CC1200_SYNC_CFG0               __CC1200_REGDEF(0x0009)
#define CC1200_DEVIATION_M             __CC1200_REGDEF(0x000A)
#define CC1200_MODCFG_DEV_E            __CC1200_REGDEF(0x000B)
#define CC1200_DCFILT_CFG              __CC1200_REGDEF(0x000C)
#define CC1200_PREAMBLE_CFG1           __CC1200_REGDEF(0x000D)
#define CC1200_PREAMBLE_CFG0           __CC1200_REGDEF(0x000E)
#define CC1200_IQIC                    __CC1200_REGDEF(0x000F)
#define CC1200_CHAN_BW                 __CC1200_REGDEF(0x0010)
#define CC1200_MDMCFG1                 __CC1200_REGDEF(0x0011)
#define CC1200_MDMCFG0                 __CC1200_REGDEF(0x0012)
#define CC1200_SYMBOL_RATE2            __CC1200_REGDEF(0x0013)
#define CC1200_SYMBOL_RATE1            __CC1200_REGDEF(0x0014)
#define CC1200_SYMBOL_RATE0            __CC1200_REGDEF(0x0015)
#define CC1200_AGC_REF                 __CC1200_REGDEF(0x0016)
#define CC1200_AGC_CS_THR              __CC1200_REGDEF(0x0017)
#define CC1200_AGC_GAIN_ADJUST         __CC1200_REGDEF(0x0018)
#define CC1200_AGC_CFG3                __CC1200_REGDEF(0x0019)
#define CC1200_AGC_CFG2                __CC1200_REGDEF(0x001A)
#define CC1200_AGC_CFG1                __CC1200_REGDEF(0x001B)
#define CC1200_AGC_CFG0                __CC1200_REGDEF(0x001C)
#define CC1200_FIFO_CFG                __CC1200_REGDEF(0x001D)
#define CC1200_DEV_ADDR                __CC1200_REGDEF(0x001E)
#define CC1200_SETTLING_CFG            __CC1200_REGDEF(0x001F)
#define CC1200_FS_CFG                  __CC1200_REGDEF(0x0020)
#define CC1200_WOR_CFG1                __CC1200_REGDEF(0x0021)
#define CC1200_WOR_CFG0                __CC1200_REGDEF(0x0022)
#define CC1200_WOR_EVENT0_MSB          __CC1200_REGDEF(0x0023)
#define CC1200_WOR_EVENT0_LSB          __CC1200_REGDEF(0x0024)
#define CC1200_RXDCM_TIME              __CC1200_REGDEF(0x0025)
#define CC1200_PKT_CFG2                __CC1200_REGDEF(0x0026)
#define CC1200_PKT_CFG1                __CC1200_REGDEF(0x0027)
#define CC1200_PKT_CFG0                __CC1200_REGDEF(0x0028)
#define CC1200_RFEND_CFG1              __CC1200_REGDEF(0x0029)
#define CC1200_RFEND_CFG0              __CC1200_REGDEF(0x002A)
#define CC1200_PA_CFG1                 __CC1200_REGDEF(0x002B)
#define CC1200_PA_CFG0                 __CC1200_REGDEF(0x002C)
#define CC1200_ASK_CFG                 __CC1200_REGDEF(0x002D)
#define CC1200_PKT_LEN                 __CC1200_REGDEF(0x002E)

#define CC1200_IF_MIX_CFG              __CC1200_REGDEF(0x2F00)
#define CC1200_FREQOFF_CFG             __CC1200_REGDEF(0x2F01)
#define CC1200_TOC_CFG                 __CC1200_REGDEF(0x2F02)
#define CC1200_MARC_SPARE              __CC1200_REGDEF(0x2F03)
#define CC1200_ECG_CFG                 __CC1200_REGDEF(0x2F04)
#define CC1200_MDMCFG2                 __CC1200_REGDEF(0x2F05)
#define CC1200_EXT_CTRL                __CC1200_REGDEF(0x2F06)
#define CC1200_RCCAL_FINE              __CC1200_REGDEF(0x2F07)
#define CC1200_RCCAL_COARSE            __CC1200_REGDEF(0x2F08)
#define CC1200_RCCAL_OFFSET            __CC1200_REGDEF(0x2F09)
#define CC1200_FREQOFF1                __CC1200_REGDEF(0x2F0A)
#define CC1200_FREQOFF0                __CC1200_REGDEF(0x2F0B)
#define CC1200_FREQ2                   __CC1200_REGDEF(0x2F0C)
#define CC1200_FREQ1                   __CC1200_REGDEF(0x2F0D)
#define CC1200_FREQ0                   __CC1200_REGDEF(0x2F0E)
#define CC1200_IF_ADC2                 __CC1200_REGDEF(0x2F0F)
#define CC1200_IF_ADC1                 __CC1200_REGDEF(0x2F10)
#define CC1200_IF_ADC0                 __CC1200_REGDEF(0x2F11)
#define CC1200_FS_DIG1                 __CC1200_REGDEF(0x2F12)
#define CC1200_FS_DIG0                 __CC1200_REGDEF(0x2F13)
#define CC1200_FS_CAL3                 __CC1200_REGDEF(0x2F14)
#define CC1200_FS_CAL2                 __CC1200_REGDEF(0x2F15)
#define CC1200_FS_CAL1                 __CC1200_REGDEF(0x2F16)
#define CC1200_FS_CAL0                 __CC1200_REGDEF(0x2F17)
#define CC1200_FS_CHP                  __CC1200_REGDEF(0x2F18)
#define CC1200_FS_DIVTWO               __CC1200_REGDEF(0x2F19)
#define CC1200_FS_DSM1                 __CC1200_REGDEF(0x2F1A)
#define CC1200_FS_DSM0                 __CC1200_REGDEF(0x2F1B)
#define CC1200_FS_DVC1                 __CC1200_REGDEF(0x2F1C)
#define CC1200_FS_DVC0                 __CC1200_REGDEF(0x2F1D)
#define CC1200_FS_LBI                  __CC1200_REGDEF(0x2F1E)
#define CC1200_FS_PFD                  __CC1200_REGDEF(0x2F1F)
#define CC1200_FS_PRE                  __CC1200_REGDEF(0x2F20)
#define CC1200_FS_REG_DIV_CML          __CC1200_REGDEF(0x2F21)
#define CC1200_FS_SPARE                __CC1200_REGDEF(0x2F22)
#define CC1200_FS_VCO4                 __CC1200_REGDEF(0x2F23)
#define CC1200_FS_VCO3                 __CC1200_REGDEF(0x2F24)
#define CC1200_FS_VCO2                 __CC1200_REGDEF(0x2F25)
#define CC1200_FS_VCO1                 __CC1200_REGDEF(0x2F26)
#define CC1200_FS_VCO0                 __CC1200_REGDEF(0x2F27)
#define CC1200_GBIAS6                  __CC1200_REGDEF(0x2F28)
#define CC1200_GBIAS5                  __CC1200_REGDEF(0x2F29)
#define CC1200_GBIAS4                  __CC1200_REGDEF(0x2F2A)
#define CC1200_GBIAS3                  __CC1200_REGDEF(0x2F2B)
#define CC1200_GBIAS2                  __CC1200_REGDEF(0x2F2C)
#define CC1200_GBIAS1                  __CC1200_REGDEF(0x2F2D)
#define CC1200_GBIAS0                  __CC1200_REGDEF(0x2F2E)
#define CC1200_IFAMP                   __CC1200_REGDEF(0x2F2F)
#define CC1200_LNA                     __CC1200_REGDEF(0x2F30)
#define CC1200_RXMIX                   __CC1200_REGDEF(0x2F31)
#define CC1200_XOSC5                   __CC1200_REGDEF(0x2F32)
#define CC1200_XOSC4                   __CC1200_REGDEF(0x2F33)
#define CC1200_XOSC3                   __CC1200_REGDEF(0x2F34)
#define CC1200_XOSC2                   __CC1200_REGDEF(0x2F35)
#define CC1200_XOSC1                   __CC1200_REGDEF(0x2F36)
#define CC1200_XOSC0                   __CC1200_REGDEF(0x2F37)
#define CC1200_ANALOG_SPARE            __CC1200_REGDEF(0x2F38)
#define CC1200_PA_CFG3                 __CC1200_REGDEF(0x2F39)
#define CC1200_IRQ0M                   __CC1200_REGDEF(0x2F3F)
#define CC1200_IRQ0F                   __CC1200_REGDEF(0x2F40)

#define CC1200_WOR_TIME1               __CC1200_REGDEF(0x2F64)
#define CC1200_WOR_TIME0               __CC1200_REGDEF(0x2F65)
#define CC1200_WOR_CAPTURE1            __CC1200_REGDEF(0x2F66)
#define CC1200_WOR_CAPTURE0            __CC1200_REGDEF(0x2F67)
#define CC1200_BIST                    __CC1200_REGDEF(0x2F68)
#define CC1200_DCFILTOFFSET_I1         __CC1200_REGDEF(0x2F69)
#define CC1200_DCFILTOFFSET_I0         __CC1200_REGDEF(0x2F6A)
#define CC1200_DCFILTOFFSET_Q1         __CC1200_REGDEF(0x2F6B)
#define CC1200_DCFILTOFFSET_Q0         __CC1200_REGDEF(0x2F6C)
#define CC1200_IQIE_I1                 __CC1200_REGDEF(0x2F6D)
#define CC1200_IQIE_I0                 __CC1200_REGDEF(0x2F6E)
#define CC1200_IQIE_Q1                 __CC1200_REGDEF(0x2F6F)
#define CC1200_IQIE_Q0                 __CC1200_REGDEF(0x2F70)
#define CC1200_RSSI1                   __CC1200_REGDEF(0x2F71)
#define CC1200_RSSI0                   __CC1200_REGDEF(0x2F72)
#define CC1200_MARCSTATE               __CC1200_REGDEF(0x2F73)
#define CC1200_LQI_VAL                 __CC1200_REGDEF(0x2F74)
#define CC1200_PQT_SYNC_ERR            __CC1200_REGDEF(0x2F75)
#define CC1200_DEM_STATUS              __CC1200_REGDEF(0x2F76)
#define CC1200_FREQOFF_EST1            __CC1200_REGDEF(0x2F77)
#define CC1200_FREQOFF_EST0            __CC1200_REGDEF(0x2F78)
#define CC1200_AGC_GAIN3               __CC1200_REGDEF(0x2F79)
#define CC1200_AGC_GAIN2               __CC1200_REGDEF(0x2F7A)
#define CC1200_AGC_GAIN1               __CC1200_REGDEF(0x2F7B)
#define CC1200_AGC_GAIN0               __CC1200_REGDEF(0x2F7C)
#define CC1200_CFM_RX_DATA_OUT         __CC1200_REGDEF(0x2F7D)
#define CC1200_CFM_TX_DATA_IN          __CC1200_REGDEF(0x2F7E)
#define CC1200_ASK_SOFT_RX_DATA        __CC1200_REGDEF(0x2F7F)
#define CC1200_RNDGEN                  __CC1200_REGDEF(0x2F80)
#define CC1200_MAGN2                   __CC1200_REGDEF(0x2F81)
#define CC1200_MAGN1                   __CC1200_REGDEF(0x2F82)
#define CC1200_MAGN0                   __CC1200_REGDEF(0x2F83)
#define CC1200_ANG1                    __CC1200_REGDEF(0x2F84)
#define CC1200_ANG0                    __CC1200_REGDEF(0x2F85)
#define CC1200_CHFILT_I2               __CC1200_REGDEF(0x2F86)
#define CC1200_CHFILT_I1               __CC1200_REGDEF(0x2F87)
#define CC1200_CHFILT_I0               __CC1200_REGDEF(0x2F88)
#define CC1200_CHFILT_Q2               __CC1200_REGDEF(0x2F89)
#define CC1200_CHFILT_Q1               __CC1200_REGDEF(0x2F8A)
#define CC1200_CHFILT_Q0               __CC1200_REGDEF(0x2F8B)
#define CC1200_GPIO_STATUS             __CC1200_REGDEF(0x2F8C)
#define CC1200_FSCAL_CTRL              __CC1200_REGDEF(0x2F8D)
#define CC1200_PHASE_ADJUST            __CC1200_REGDEF(0x2F8E)
#define CC1200_PARTNUMBER              __CC1200_REGDEF(0x2F8F)
#define CC1200_PARTVERSION             __CC1200_REGDEF(0x2F90)
#define CC1200_SERIAL_STATUS           __CC1200_REGDEF(0x2F91)
#define CC1200_MODEM_STATUS1           __CC1200_REGDEF(0x2F92)
#define CC1200_MODEM_STATUS0           __CC1200_REGDEF(0x2F93)
#define CC1200_MARC_STATUS1            __CC1200_REGDEF(0x2F94)
#define CC1200_MARC_STATUS0            __CC1200_REGDEF(0x2F95)
#define CC1200_PA_IFAMP_TEST           __CC1200_REGDEF(0x2F96)
#define CC1200_FSRF_TEST               __CC1200_REGDEF(0x2F97)
#define CC1200_PRE_TEST                __CC1200_REGDEF(0x2F98)
#define CC1200_PRE_OVR                 __CC1200_REGDEF(0x2F99)
#define CC1200_ADC_TEST                __CC1200_REGDEF(0x2F9A)
#define CC1200_DVC_TEST                __CC1200_REGDEF(0x2F9B)
#define CC1200_ATEST                   __CC1200_REGDEF(0x2F9C)
#define CC1200_ATEST_LVDS              __CC1200_REGDEF(0x2F9D)
#define CC1200_ATEST_MODE              __CC1200_REGDEF(0x2F9E)
#define CC1200_XOSC_TEST1              __CC1200_REGDEF(0x2F9F)
#define CC1200_XOSC_TEST0              __CC1200_REGDEF(0x2FA0)
#define CC1200_AES                     __CC1200_REGDEF(0x2FA1)
#define CC1200_MDM_TEST                __CC1200_REGDEF(0x2FA2)

#define CC1200_RXFIRST                 __CC1200_REGDEF(0x2FD2)
#define CC1200_TXFIRST                 __CC1200_REGDEF(0x2FD3)
#define CC1200_RXLAST                  __CC1200_REGDEF(0x2FD4)
#define CC1200_TXLAST                  __CC1200_REGDEF(0x2FD5)
#define CC1200_NUM_TXBYTES             __CC1200_REGDEF(0x2FD6)
#define CC1200_NUM_RXBYTES             __CC1200_REGDEF(0x2FD7)
#define CC1200_FIFO_NUM_TXBYTES        __CC1200_REGDEF(0x2FD8)
#define CC1200_FIFO_NUM_RXBYTES        __CC1200_REGDEF(0x2FD9)
#define CC1200_RXFIFO_PRE_BUF          __CC1200_REGDEF(0x2FDA)

#define CC1200_FIFO_ACCESS             __CC1200_REGDEF(0x003F)

#define CC1200_AES_KEY                 __CC1200_REGDEF(0x2FE0)
#define CC1200_AES_KEY15           __CC1200_REGDEF(0x2FE0)
#define CC1200_AES_KEY14           __CC1200_REGDEF(0x2FE1)
#define CC1200_AES_KEY13           __CC1200_REGDEF(0x2FE2)
#define CC1200_AES_KEY12           __CC1200_REGDEF(0x2FE3)
#define CC1200_AES_KEY11           __CC1200_REGDEF(0x2FE4)
#define CC1200_AES_KEY10           __CC1200_REGDEF(0x2FE5)
#define CC1200_AES_KEY9                __CC1200_REGDEF(0x2FE6)
#define CC1200_AES_KEY8                __CC1200_REGDEF(0x2FE7)
#define CC1200_AES_KEY7                __CC1200_REGDEF(0x2FE8)
#define CC1200_AES_KEY6                __CC1200_REGDEF(0x2FE9)
#define CC1200_AES_KEY5                __CC1200_REGDEF(0x2FE10)
#define CC1200_AES_KEY4                __CC1200_REGDEF(0x2FE11)
#define CC1200_AES_KEY3                __CC1200_REGDEF(0x2FE12)
#define CC1200_AES_KEY2                __CC1200_REGDEF(0x2FE13)
#define CC1200_AES_KEY1                __CC1200_REGDEF(0x2FE14)
#define CC1200_AES_KEY0                __CC1200_REGDEF(0x2FE15)

#define CC1200_AES_BUFFER              __CC1200_REGDEF(0x2FF0)
#define CC1200_AES_BUFFER15    __CC1200_REGDEF(0x2FF0)
#define CC1200_AES_BUFFER14    __CC1200_REGDEF(0x2FF1)
#define CC1200_AES_BUFFER13    __CC1200_REGDEF(0x2FF2)
#define CC1200_AES_BUFFER12    __CC1200_REGDEF(0x2FF3)
#define CC1200_AES_BUFFER11    __CC1200_REGDEF(0x2FF4)
#define CC1200_AES_BUFFER10    __CC1200_REGDEF(0x2FF5)
#define CC1200_AES_BUFFER9     __CC1200_REGDEF(0x2FF6)
#define CC1200_AES_BUFFER8     __CC1200_REGDEF(0x2FF7)
#define CC1200_AES_BUFFER7     __CC1200_REGDEF(0x2FF8)
#define CC1200_AES_BUFFER6     __CC1200_REGDEF(0x2FF9)
#define CC1200_AES_BUFFER5     __CC1200_REGDEF(0x2FF10)
#define CC1200_AES_BUFFER4     __CC1200_REGDEF(0x2FF11)
#define CC1200_AES_BUFFER3     __CC1200_REGDEF(0x2FF12)
#define CC1200_AES_BUFFER2     __CC1200_REGDEF(0x2FF13)
#define CC1200_AES_BUFFER1     __CC1200_REGDEF(0x2FF14)
#define CC1200_AES_BUFFER0     __CC1200_REGDEF(0x2FF15)

#define CC1200_LQI_CRC_OK_BM           __CC1200_FLGDEF(0x80)
#define CC1200_LQI_EST_BM              __CC1200_FLGDEF(0x7F)

#define CC1200_SRES                    __CC1200_FLGDEF(0x30      /*  SRES    - Reset chip. */)
#define CC1200_SFSTXON                 __CC1200_FLGDEF(0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */)
#define CC1200_SXOFF                   __CC1200_FLGDEF(0x32      /*  SXOFF   - Turn off crystal oscillator. */)
#define CC1200_SCAL                    __CC1200_FLGDEF(0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */)
#define CC1200_SRX                     __CC1200_FLGDEF(0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */)
#define CC1200_STX                     __CC1200_FLGDEF(0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */)
#define CC1200_SIDLE                   __CC1200_FLGDEF(0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */)
#define CC1200_SAFC                    __CC1200_FLGDEF(0x37      /*  AFC     - Automatic Frequency Correction */)
#define CC1200_SWOR                    __CC1200_FLGDEF(0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */)
#define CC1200_SPWD                    __CC1200_FLGDEF(0x39      /*  SPWD    - Enter power down mode when CSn goes high. */)
#define CC1200_SFRX                    __CC1200_FLGDEF(0x3A      /*  SFRX    - Flush the RX FIFO buffer. */)
#define CC1200_SFTX                    __CC1200_FLGDEF(0x3B      /*  SFTX    - Flush the TX FIFO buffer. */)
#define CC1200_SWORRST                 __CC1200_FLGDEF(0x3C      /*  SWORRST - Reset real time clock. */)
#define CC1200_SNOP                    __CC1200_FLGDEF(0x3D      /*  SNOP    - No operation. Returns status byte. */)

/* IOCFGx.GPIOx_CFG bits (incomplete)
 */
#define CC1200_IOCFG_GPIO_CFG_M                     __CC1200_FLGDEF(0x3F)
#define CC1200_IOCFG_GPIO_CFG_NONE                  __CC1200_FLGDEF(0x00)
#define CC1200_IOCFG_GPIO_CFG_PKT_SYNC_RXTX         __CC1200_FLGDEF(0x06 /* Sync word detect/send on RX/TX asserted until end of packet */)
#define CC1200_IOCFG_GPIO_CFG_RSSI_VALID            __CC1200_FLGDEF(0x0D /* RSSI valid */)
#define CC1200_IOCFG_GPIO_CFG_MCU_WAKEUP            __CC1200_FLGDEF(0x14 /* MCU wake up signal */)
#define CC1200_IOCFG_GPIO_CFG_RX0TX1                __CC1200_FLGDEF(0x1A /* Indicates RX or TX (0 indicates IDLE) */)
#define CC1200_IOCFG_GPIO_CFG_MARC_2PIN_STATUS1     __CC1200_FLGDEF(0x25 /* Partial MARC state status */)
#define CC1200_IOCFG_GPIO_CFG_MARC_2PIN_STATUS0     __CC1200_FLGDEF(0x26 /* Partial MARC state status */)
#define CC1200_IOCFG_GPIO_CFG_HIGHZ                 __CC1200_FLGDEF(0x30 /* High impedance (tri-state) */)
#define CC1200_IOCFG_GPIO_CFG_HW0                   __CC1200_FLGDEF(0x33 /* Hardwire to zero */)


/* CC1200_CHAN_BW bits
 */
#define CC1200_CHAN_BW_ADC_CIC_DECFACT_M            __CC1200_FLGDEF(0xC0)
#define CC1200_CHAN_BW_ADC_CIC_DECFACT_S            __CC1200_FLGDEF(6)
#define CC1200_CHAN_BW_ADC_CIC_DECFACT_MIN          __CC1200_FLGDEF(0)
#define CC1200_CHAN_BW_ADC_CIC_DECFACT_MAX          __CC1200_FLGDEF(2)
#define CC1200_CHAN_BW_ADC_CIC_DECFACT_BASE         __CC1200_FLGDEF(12)
#define CC1200_CHAN_BW_BB_CIC_DECFACT_M             __CC1200_FLGDEF(0x3F)
#define CC1200_CHAN_BW_BB_CIC_DECFACT_S             __CC1200_FLGDEF(0)
#define CC1200_CHAN_BW_BB_CIC_DECFACT_MIN           __CC1200_FLGDEF(1)
#define CC1200_CHAN_BW_BB_CIC_DECFACT_MAX           __CC1200_FLGDEF(44)

/* CC1200_SYNC_CFG1 bits
 */
#define CC1200_SYNC_CFG1_SYNC_MODE_M                __CC1200_FLGDEF(0xE0)
#define CC1200_SYNC_CFG1_SYNC_MODE_NONE             __CC1200_FLGDEF(0x00)
#define CC1200_SYNC_CFG1_SYNC_MODE_11               __CC1200_FLGDEF(0x20)
#define CC1200_SYNC_CFG1_SYNC_MODE_16               __CC1200_FLGDEF(0x40)
#define CC1200_SYNC_CFG1_SYNC_MODE_18               __CC1200_FLGDEF(0x60)
#define CC1200_SYNC_CFG1_SYNC_MODE_24               __CC1200_FLGDEF(0x80)
#define CC1200_SYNC_CFG1_SYNC_MODE_32               __CC1200_FLGDEF(0xA0)
#define CC1200_SYNC_CFG1_SYNC_MODE_16H              __CC1200_FLGDEF(0xC0)
#define CC1200_SYNC_CFG1_SYNC_MODE_16D              __CC1200_FLGDEF(0xE0)
#define CC1200_SYNC_CFG1_SYNC_THR_M                 __CC1200_FLGDEF(0x1F)

/* CC1200_MARC_STATUS1 bits (aka MARC_STATUS_OUT)
 */
#define CC1200_MARC_STATUS1_NO_FAILURE             __CC1200_FLGDEF(0x00)
#define CC1200_MARC_STATUS1_RX_TIMEOUT             __CC1200_FLGDEF(0x01)
#define CC1200_MARC_STATUS1_RX_TERMINATION         __CC1200_FLGDEF(0x02)
#define CC1200_MARC_STATUS1_EWOR_SYNC_LOST         __CC1200_FLGDEF(0x03)
#define CC1200_MARC_STATUS1_MAXIMUM_LENGTH         __CC1200_FLGDEF(0x04)
#define CC1200_MARC_STATUS1_ADDRESS                __CC1200_FLGDEF(0x05)
#define CC1200_MARC_STATUS1_CRC                    __CC1200_FLGDEF(0x06)
#define CC1200_MARC_STATUS1_TX_FIFO_OVERFLOW       __CC1200_FLGDEF(0x07)
#define CC1200_MARC_STATUS1_TX_FIFO_UNDERFLOW      __CC1200_FLGDEF(0x08)
#define CC1200_MARC_STATUS1_RX_FIFO_OVERFLOW       __CC1200_FLGDEF(0x09)
#define CC1200_MARC_STATUS1_RX_FIFO_UNDERFLOW      __CC1200_FLGDEF(0x0A)
#define CC1200_MARC_STATUS1_TX_ON_CCA_FAILED       __CC1200_FLGDEF(0x0B)
#define CC1200_MARC_STATUS1_TX_FINISHED            __CC1200_FLGDEF(0x40)
#define CC1200_MARC_STATUS1_RX_FINISHED            __CC1200_FLGDEF(0x80)


/* CC1200_MARC_STATUS0 bits
 */
#define CC1200_MARC_STATUS0_TXONCCA_FAILED  __CC1200_FLGDEF(0x04)


/* CC1200_MARCSTATE bits
 */
#define CC1200_MARC_2PIN_STATE_M             __CC1200_FLGDEF(0x60)
#define CC1200_MARC_2PIN_STATE_SETTLING      __CC1200_FLGDEF(0x00)
#define CC1200_MARC_2PIN_STATE_TX            __CC1200_FLGDEF(0x20)
#define CC1200_MARC_2PIN_STATE_IDLE          __CC1200_FLGDEF(0x40)
#define CC1200_MARC_2PIN_STATE_RX            __CC1200_FLGDEF(0x60)
#define CC1200_MARC_STATE_M                  __CC1200_FLGDEF(0x1F)
#define CC1200_MARC_STATE_SLEEP              __CC1200_FLGDEF(0x00)
#define CC1200_MARC_STATE_IDLE               __CC1200_FLGDEF(0x01)
#define CC1200_MARC_STATE_XOFF               __CC1200_FLGDEF(0x02)
#define CC1200_MARC_STATE_BIAS_SETTLE_MC     __CC1200_FLGDEF(0x03)
#define CC1200_MARC_STATE_REG_SETTLE_MC      __CC1200_FLGDEF(0x04)
#define CC1200_MARC_STATE_MANCAL             __CC1200_FLGDEF(0x05)
#define CC1200_MARC_STATE_BIAS_SETTLE        __CC1200_FLGDEF(0x06)
#define CC1200_MARC_STATE_REG_SETTLE         __CC1200_FLGDEF(0x07)
#define CC1200_MARC_STATE_STARTCAL           __CC1200_FLGDEF(0x08)
#define CC1200_MARC_STATE_BWBOOST            __CC1200_FLGDEF(0x09)
#define CC1200_MARC_STATE_FS_LOCK            __CC1200_FLGDEF(0x0A)
#define CC1200_MARC_STATE_IFADCON            __CC1200_FLGDEF(0x0B)
#define CC1200_MARC_STATE_ENDCAL             __CC1200_FLGDEF(0x0C)
#define CC1200_MARC_STATE_RX                 __CC1200_FLGDEF(0x0D)
#define CC1200_MARC_STATE_RX_END             __CC1200_FLGDEF(0x0E)
#define CC1200_MARC_STATE_RXDCM              __CC1200_FLGDEF(0x0F)
#define CC1200_MARC_STATE_TXRX_SWITCH        __CC1200_FLGDEF(0x10)
#define CC1200_MARC_STATE_RX_FIFO_ERR        __CC1200_FLGDEF(0x11)
#define CC1200_MARC_STATE_FSTXON             __CC1200_FLGDEF(0x12)
#define CC1200_MARC_STATE_TX                 __CC1200_FLGDEF(0x13)
#define CC1200_MARC_STATE_TX_END             __CC1200_FLGDEF(0x14)
#define CC1200_MARC_STATE_RXTX_SWITCH        __CC1200_FLGDEF(0x15)
#define CC1200_MARC_STATE_TX_FIFO_ERR        __CC1200_FLGDEF(0x16)
#define CC1200_MARC_STATE_IFADCON_TXRX       __CC1200_FLGDEF(0x17)

#define CC1200_MARCSTATE_IDLE                (CC1200_MARC_2PIN_STATE_IDLE|CC1200_MARC_STATE_IDLE)


/* CC1200_FIFO_CFG bits
 */
#define CC1200_FIFO_CFG_CRC_AUTOFLUSH       __CC1200_FLGDEF(0x80)
#define CC1200_FIFO_CFG_FIFO_THR_M          __CC1200_FLGDEF(0x7F)


/* CC1200_RFEND_CFG1 bits
 */
#define CC1200_RFEND_CFG1_RXOFF_MODE_M          __CC1200_FLGDEF(0x30)
#define CC1200_RFEND_CFG1_RXOFF_MODE_IDLE       __CC1200_FLGDEF(0x00)
#define CC1200_RFEND_CFG1_RXOFF_MODE_FSTXON     __CC1200_FLGDEF(0x10)
#define CC1200_RFEND_CFG1_RXOFF_MODE_TX         __CC1200_FLGDEF(0x20)
#define CC1200_RFEND_CFG1_RXOFF_MODE_RX         __CC1200_FLGDEF(0x30)
#define CC1200_RFEND_CFG1_RX_TIME_M             __CC1200_FLGDEF(0x0E)
#define CC1200_RFEND_CFG1_RX_TIME_S             __CC1200_FLGDEF(1)
#define CC1200_RFEND_CFG1_RX_TIME_MIN           __CC1200_FLGDEF(0)
#define CC1200_RFEND_CFG1_RX_TIME_MAX           __CC1200_FLGDEF(6)
#define CC1200_RFEND_CFG1_RX_TIME_FOREVER       __CC1200_FLGDEF(7)
#define CC1200_RFEND_CFG1_RX_TIME_QUAL_M        __CC1200_FLGDEF(0x01)


/* CC1200_RFEND_CFG0 bits
 */
#define CC1200_RFEND_CFG0_CAL_END_WAKE_UP_EN    __CC1200_FLGDEF(0x40)
#define CC1200_RFEND_CFG0_TXOFF_MODE_M          __CC1200_FLGDEF(0x30)
#define CC1200_RFEND_CFG0_TXOFF_MODE_IDLE       __CC1200_FLGDEF(0x00)
#define CC1200_RFEND_CFG0_TXOFF_MODE_FSTXON     __CC1200_FLGDEF(0x10)
#define CC1200_RFEND_CFG0_TXOFF_MODE_TX         __CC1200_FLGDEF(0x20)
#define CC1200_RFEND_CFG0_TXOFF_MODE_RX         __CC1200_FLGDEF(0x30)
#define CC1200_RFEND_CFG0_TERM_ON_BAD_PACKET_EN __CC1200_FLGDEF(0x08)
#define CC1200_RFEND_CFG0_ANT_DIV_RX_TERM_CFG_M __CC1200_FLGDEF(0x07)


/* CC1200_DEM_STATUS bits
 */
#define CC1200_DEM_STATUS_SYNC_LOW0_HIGH1   __CC1200_FLGDEF(0x20)


/* CC1200_PKT_CFG2 bits
 */
#define CC1200_PKT_CFG2_CCA_MODE_M                  __CC1200_FLGDEF(0x1C)
#define CC1200_PKT_CFG2_CCA_MODE_ALWAYS             __CC1200_FLGDEF(0x00)
#define CC1200_PKT_CFG2_CCA_MODE_RSSI_THR           __CC1200_FLGDEF(0x04)
#define CC1200_PKT_CFG2_CCA_MODE_NOT_RX             __CC1200_FLGDEF(0x08)
#define CC1200_PKT_CFG2_CCA_MODE_RSSI_THR_NOT_RX    __CC1200_FLGDEF(0x0C)
#define CC1200_PKT_CFG2_CCA_MODE_RSSI_THR_ETSI_LBT  __CC1200_FLGDEF(0x10)


/* CC1200_PKT_CFG1 bits
 */
#define CC1200_PKT_CFG1_FEC_EN                          __CC1200_FLGDEF(0x80)
#define CC1200_PKT_CFG1_WHITE_DATA                      __CC1200_FLGDEF(0x40)
#define CC1200_PKT_CFG1_PN9_SWAP_EN                     __CC1200_FLGDEF(0x20)
#define CC1200_PKT_CFG1_ADDR_CHECK_CFG_M                __CC1200_FLGDEF(0x18)
#define CC1200_PKT_CFG1_ADDR_CHECK_CFG_OFF              __CC1200_FLGDEF(0x00)
#define CC1200_PKT_CFG1_ADDR_CHECK_CFG_ON_NO_BCAST      __CC1200_FLGDEF(0x08)
#define CC1200_PKT_CFG1_ADDR_CHECK_CFG_ON_BCAST_00      __CC1200_FLGDEF(0x10)
#define CC1200_PKT_CFG1_ADDR_CHECK_CFG_ON_BCAST_00_FF   __CC1200_FLGDEF(0x10)
#define CC1200_PKT_CFG1_CRC_CFG_M                       __CC1200_FLGDEF(0x06)
#define CC1200_PKT_CFG1_CRC_CFG_OFF                     __CC1200_FLGDEF(0x00)
#define CC1200_PKT_CFG1_CRC_CFG_ON_INIT_FFFF            __CC1200_FLGDEF(0x02)
#define CC1200_PKT_CFG1_CRC_CFG_ON_INIT_0000            __CC1200_FLGDEF(0x04)
#define CC1200_PKT_CFG1_CRC_CFG_ON_INIT_1D0F            __CC1200_FLGDEF(0x06)
#define CC1200_PKT_CFG1_APPEND_STATUS                   __CC1200_FLGDEF(0x01)


/* CC1200_PKT_CFG0 bits
 */
#define CC1200_PKT_CFG0_LENGTH_CONFIG_M                 __CC1200_FLGDEF(0x60)
#define CC1200_PKT_CFG0_LENGTH_CONFIG_FIXED             __CC1200_FLGDEF(0x00)
#define CC1200_PKT_CFG0_LENGTH_CONFIG_VARIABLE          __CC1200_FLGDEF(0x20)
#define CC1200_PKT_CFG0_LENGTH_CONFIG_INFINITE          __CC1200_FLGDEF(0x40)
#define CC1200_PKT_CFG0_LENGTH_CONFIG_VARIABLE_5LSB     __CC1200_FLGDEF(0x60)


/* CC1200_RSSI0 bits
 */
#define CC1200_RSSI0_RSSI_VALID             __CC1200_FLGDEF(0x01)
#define CC1200_RSSI0_CARRIER_SENSE_VALID    __CC1200_FLGDEF(0x02)
#define CC1200_RSSI0_CARRIER_SENSE          __CC1200_FLGDEF(0x04)


/* CC1200_RNDGEN bits
 */
#define CC1200_RNDGEN_EN                 __CC1200_FLGDEF(0x80)
#define CC1200_RNDGEN_VALUE_M            __CC1200_FLGDEF(0x7F)

/* CC1200_FS_CFG bits
 */
#define CC1200_FS_CFG_FS_LOCK_EN                    __CC1200_FLGDEF(0x10)
#define CC1200_FS_CFG_FSD_BANDSELECT_M              __CC1200_FLGDEF(0x0F)
#define CC1200_FS_CFG_FSD_BANDSELECT_LO_04_820_960  __CC1200_FLGDEF(0x02)
#define CC1200_FS_CFG_FSD_BANDSELECT_LO_08_410_480  __CC1200_FLGDEF(0x04)
#define CC1200_FS_CFG_FSD_BANDSELECT_LO_12_273_320  __CC1200_FLGDEF(0x06)
#define CC1200_FS_CFG_FSD_BANDSELECT_LO_16_205_240  __CC1200_FLGDEF(0x08)
#define CC1200_FS_CFG_FSD_BANDSELECT_LO_20_164_192  __CC1200_FLGDEF(0x0A)
#define CC1200_FS_CFG_FSD_BANDSELECT_LO_24_136_160  __CC1200_FLGDEF(0x0B)


/* CC1200_WOR_CFG1 bits
 */
#define CC1200_WOR_CFG1_WOR_RES_M                   __CC1200_FLGDEF(0xC0)
#define CC1200_WOR_CFG1_WOR_RES_S                   __CC1200_FLGDEF(6)
#define CC1200_WOR_CFG1_WOR_RES_MIN                 __CC1200_FLGDEF(0)
#define CC1200_WOR_CFG1_WOR_RES_MAX                 __CC1200_FLGDEF(3)
#define CC1200_WOR_CFG1_WOR_MODE_M                  __CC1200_FLGDEF(0x38)
#define CC1200_WOR_CFG1_WOR_MODE_S                  __CC1200_FLGDEF(3)
#define CC1200_WOR_CFG1_EVENT1_M                    __CC1200_FLGDEF(0x07)
#define CC1200_WOR_CFG1_EVENT1_S                    __CC1200_FLGDEF(0)

/* CC1200_MODCFG_DEV_E bits
 */
#define CC1200_MODCFG_DEV_E_MODEM_MODE_M                __CC1200_FLGDEF(0xC0)
#define CC1200_MODCFG_DEV_E_MODEM_MODE_NORMAL           __CC1200_FLGDEF(0x00)
#define CC1200_MODCFG_DEV_E_MODEM_MODE_DSSS_REPEAT      __CC1200_FLGDEF(0x40)
#define CC1200_MODCFG_DEV_E_MODEM_MODE_DSSS_PN          __CC1200_FLGDEF(0x80)
#define CC1200_MODCFG_DEV_E_MODEM_MODE_CARRIER_SENSE    __CC1200_FLGDEF(0xC0)
#define CC1200_MODCFG_DEV_E_MOD_FORMAT_M                __CC1200_FLGDEF(0x38)
#define CC1200_MODCFG_DEV_E_MOD_FORMAT_2_FSK            __CC1200_FLGDEF(0x00)
#define CC1200_MODCFG_DEV_E_MOD_FORMAT_2_GFSK           __CC1200_FLGDEF(0x08)
#define CC1200_MODCFG_DEV_E_MOD_FORMAT_ASK_OOK          __CC1200_FLGDEF(0x18)
#define CC1200_MODCFG_DEV_E_MOD_FORMAT_4_FSK            __CC1200_FLGDEF(0x20)
#define CC1200_MODCFG_DEV_E_MOD_FORMAT_4_GFSK           __CC1200_FLGDEF(0x28)
#define CC1200_MODCFG_DEV_E_DEV_E_M                     __CC1200_FLGDEF(0x07)
#define CC1200_MODCFG_DEV_E_DEV_E_S                     __CC1200_FLGDEF(0x00)

/* CC1200_SYMBOL_RATE2 bits
 */
#define CC1200_SYMBOL_RATE2_SRATE_E_M   __CC1200_FLGDEF(0xF0)
#define CC1200_SYMBOL_RATE2_SRATE_E_S   __CC1200_FLGDEF(0x04)

/* CC1200_GPIO_STATUS bits
 */
#define CC1200_GPIO_STATUS_GPIO_STATE_M __CC1200_FLGDEF(0x0F)


/* CC1200_SERIAL_STATUS bits
 */
#define CC1200_SERIAL_STATUS_IOC_SYNC_PINS_EN   __CC1200_FLGDEF(0x08)


/* Status bits
 */
#define CC1200_STATUS_CHIP_RDYn          __CC1200_FLGDEF(0x80)
#define CC1200_STATUS_STATE_M            __CC1200_FLGDEF(0x70)
#define CC1200_STATUS_EXTRA_M            __CC1200_FLGDEF(0x0F)

#define CC1200_STATE_IDLE                __CC1200_FLGDEF(0x00)
#define CC1200_STATE_RX                  __CC1200_FLGDEF(0x10)
#define CC1200_STATE_TX                  __CC1200_FLGDEF(0x20)
#define CC1200_STATE_FSTXON              __CC1200_FLGDEF(0x30)
#define CC1200_STATE_CALIBRATE           __CC1200_FLGDEF(0x40)
#define CC1200_STATE_SETTLING            __CC1200_FLGDEF(0x50)
#define CC1200_STATE_RXFIFO_ERROR        __CC1200_FLGDEF(0x60)
#define CC1200_STATE_TXFIFO_ERROR        __CC1200_FLGDEF(0x70)
