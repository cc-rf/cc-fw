#pragma once

#include <cc/common.h>
#include <cc/io.h>

// TODO: implement a more robust config management system

struct cc_cfg_reg {
    u16 addr;
    u8 data;
};

static inline bool cc_cfg_regs(cc_dev_t dev, const struct cc_cfg_reg regs[], u32 len)
{
    for (u32 i = 0; i < len; i++) {
        cc_set(dev, regs[i].addr, regs[i].data);
        if (regs[i].addr == CC1200_RNDGEN) continue;
        const u8 data = cc_get(dev, regs[i].addr);
        if (data != regs[i].data) {
            cc_dbg("[%u] validation error: addr=0x%04X data=0x%02X data[read-back]=0x%02X", dev, regs[i].addr, regs[i].data, data);
            return false;
        }
    }

    return true;
}

// -- This is a good low-bandwidth configuration
// RX filter BW = 104.166667
// Carrier frequency = 920.599976
// Symbol rate = 38.4
// Manchester enable = false
// Address config = No address check
// Whitening = false
// Packet length mode = Variable
// Bit rate = 38.4
// Deviation = 19.989014
// Modulation format = 2-GFSK
// Device address = 0
// Packet length = 255
// Packet bit length = 0

static const struct cc_cfg_reg CC_CFG_DEFAULT[] = {
        {CC1200_SYNC3,             0x6F}, // Sync Word Configuration [31:24]
        {CC1200_SYNC2,             0x4E}, // Sync Word Configuration [23:16]
        {CC1200_SYNC1,             0x90}, // Sync Word Configuration [15:8]
        {CC1200_SYNC0,             0x4E}, // Sync Word Configuration [7:0]
        {CC1200_SYNC_CFG1,         0xE9}, // Sync Word Detection Configuration Reg. 1
        {CC1200_MODCFG_DEV_E,      0x0B}, // Modulation Format and Frequency Deviation Configur..
        {CC1200_PREAMBLE_CFG1,     0x29}, // Preamble Length Configuration Reg. 1
        {CC1200_PREAMBLE_CFG0,     0x8A}, // Preamble Detection Configuration Reg. 0
        {CC1200_IQIC,              0xC8}, // Digital Image Channel Compensation Configuration
        {CC1200_CHAN_BW,           0x10}, // Channel Filter Configuration
        {CC1200_MDMCFG1,           0x42}, // General Modem Parameter Configuration Reg. 1
        {CC1200_MDMCFG0,           0x05}, // General Modem Parameter Configuration Reg. 0
        {CC1200_SYMBOL_RATE2,      0x8F}, // Symbol Rate Configuration Exponent and Mantissa [1..
        {CC1200_SYMBOL_RATE1,      0x75}, // Symbol Rate Configuration Mantissa [15:8]
        {CC1200_SYMBOL_RATE0,      0x10}, // Symbol Rate Configuration Mantissa [7:0]
        {CC1200_AGC_REF,           0x21/*0x27*/}, // AGC Reference Level Configuration
        {CC1200_AGC_CS_THR,        0x01}, // Carrier Sense Threshold Configuration
        {CC1200_AGC_CFG1,          0x11}, // Automatic Gain Control Configuration Reg. 1
        {CC1200_AGC_CFG0,          0x94}, // Automatic Gain Control Configuration Reg. 0
        {CC1200_FIFO_CFG,          0x00}, // FIFO Configuration
        {CC1200_FS_CFG,            0x12}, // Frequency Synthesizer Configuration
        {CC1200_PKT_CFG2,          0x00}, // Packet Configuration Reg. 2
        {CC1200_PKT_CFG0,          0x20}, // Packet Configuration Reg. 0
        {CC1200_PKT_LEN,           0xFF}, // Packet Length Configuration
        {CC1200_IF_MIX_CFG,        0x1C}, // IF Mix Configuration
        {CC1200_TOC_CFG,           0x03}, // Timing Offset Correction Configuration
        {CC1200_MDMCFG2,           0x02}, // General Modem Parameter Configuration Reg. 2
        {CC1200_FREQ2,             0x5C}, // Frequency Configuration [23:16]
        {CC1200_FREQ1,             0x0F}, // Frequency Configuration [15:8]
        {CC1200_FREQ0,             0x5C}, // Frequency Configuration [7:0]
        {CC1200_IF_ADC1,           0xEE}, // Analog to Digital Converter Configuration Reg. 1
        {CC1200_IF_ADC0,           0x10}, // Analog to Digital Converter Configuration Reg. 0
        {CC1200_FS_DIG1,           0x04}, // Frequency Synthesizer Digital Reg. 1
        {CC1200_FS_DIG0,           0x55}, // Frequency Synthesizer Digital Reg. 0
        {CC1200_FS_CAL1,           0x40}, // Frequency Synthesizer Calibration Reg. 1
        {CC1200_FS_CAL0,           0x0E}, // Frequency Synthesizer Calibration Reg. 0
        {CC1200_FS_DIVTWO,         0x03}, // Frequency Synthesizer Divide by 2
        {CC1200_FS_DSM0,           0x33}, // FS Digital Synthesizer Module Configuration Reg. 0
        {CC1200_FS_DVC0,           0x17}, // Frequency Synthesizer Divider Chain Configuration ..
        {CC1200_FS_PFD,            0x00}, // Frequency Synthesizer Phase Frequency Detector Con..
        {CC1200_FS_PRE,            0x6E}, // Frequency Synthesizer Prescaler Configuration
        {CC1200_FS_REG_DIV_CML,    0x1C}, // Frequency Synthesizer Divider Regulator Configurat..
        {CC1200_FS_SPARE,          0xAC}, // Frequency Synthesizer Spare
        {CC1200_FS_VCO0,           0xB5}, // FS Voltage Controlled Oscillator Configuration Reg..
        {CC1200_IFAMP,             0x09}, // Intermediate Frequency Amplifier Configuration
        {CC1200_XOSC5,             0x0E}, // Crystal Oscillator Configuration Reg. 5
        {CC1200_XOSC1,             0x03}, // Crystal Oscillator Configuration Reg. 1
};

// -- This is a good low-bandwidth configuration
// CHANGES: MODULATION -> FSK (MODCFG_DEV_E)
// RX filter BW = 104.166667
// Carrier frequency = 920.599976
// Symbol rate = 38.4
// Manchester enable = false
// Address config = No address check
// Whitening = false
// Packet length mode = Variable
// Bit rate = 38.4
// Deviation = 19.989014
// Modulation format = 2-GFSK
// Device address = 0
// Packet length = 255
// Packet bit length = 0

static const struct cc_cfg_reg CC_CFG_DEFAULT_0_MOD[] = {
        {CC1200_PA_CFG1,           (1<<6) | 0x2B},

        {CC1200_SYNC3,             0x6F}, // Sync Word Configuration [31:24]
        {CC1200_SYNC2,             0x4E}, // Sync Word Configuration [23:16]
        {CC1200_SYNC1,             0x90}, // Sync Word Configuration [15:8]
        {CC1200_SYNC0,             0x4E}, // Sync Word Configuration [7:0]
        {CC1200_SYNC_CFG1,         0xE9}, // Sync Word Detection Configuration Reg. 1
        {CC1200_MODCFG_DEV_E,      0x3}, // Modulation Format and Frequency Deviation Configur..
        {CC1200_PREAMBLE_CFG1,     0x29}, // Preamble Length Configuration Reg. 1
        {CC1200_PREAMBLE_CFG0,     0x8A}, // Preamble Detection Configuration Reg. 0
        {CC1200_IQIC,              0xC8}, // Digital Image Channel Compensation Configuration
        {CC1200_CHAN_BW,           0x10}, // Channel Filter Configuration
        {CC1200_MDMCFG1,           0x42}, // General Modem Parameter Configuration Reg. 1
        {CC1200_MDMCFG0,           0x05}, // General Modem Parameter Configuration Reg. 0
        {CC1200_SYMBOL_RATE2,      0x8F}, // Symbol Rate Configuration Exponent and Mantissa [1..
        {CC1200_SYMBOL_RATE1,      0x75}, // Symbol Rate Configuration Mantissa [15:8]
        {CC1200_SYMBOL_RATE0,      0x10}, // Symbol Rate Configuration Mantissa [7:0]
        {CC1200_AGC_REF,           0x27}, // AGC Reference Level Configuration
        {CC1200_AGC_CS_THR,        0x01}, // Carrier Sense Threshold Configuration
        {CC1200_AGC_CFG1,          0x11}, // Automatic Gain Control Configuration Reg. 1
        {CC1200_AGC_CFG0,          0x94}, // Automatic Gain Control Configuration Reg. 0
        {CC1200_FIFO_CFG,          0x00}, // FIFO Configuration
        {CC1200_FS_CFG,            0x12}, // Frequency Synthesizer Configuration
        {CC1200_PKT_CFG2,          0x00}, // Packet Configuration Reg. 2
        {CC1200_PKT_CFG0,          0x20}, // Packet Configuration Reg. 0
        {CC1200_PKT_LEN,           0xFF}, // Packet Length Configuration
        {CC1200_IF_MIX_CFG,        0x1C}, // IF Mix Configuration
        {CC1200_TOC_CFG,           0x03}, // Timing Offset Correction Configuration
        {CC1200_MDMCFG2,           0x02}, // General Modem Parameter Configuration Reg. 2
        {CC1200_FREQ2,             0x5C}, // Frequency Configuration [23:16]
        {CC1200_FREQ1,             0x0F}, // Frequency Configuration [15:8]
        {CC1200_FREQ0,             0x5C}, // Frequency Configuration [7:0]
        {CC1200_IF_ADC1,           0xEE}, // Analog to Digital Converter Configuration Reg. 1
        {CC1200_IF_ADC0,           0x10}, // Analog to Digital Converter Configuration Reg. 0
        {CC1200_FS_DIG1,           0x04}, // Frequency Synthesizer Digital Reg. 1
        {CC1200_FS_DIG0,           0x55}, // Frequency Synthesizer Digital Reg. 0
        {CC1200_FS_CAL1,           0x40}, // Frequency Synthesizer Calibration Reg. 1
        {CC1200_FS_CAL0,           0x0E}, // Frequency Synthesizer Calibration Reg. 0
        {CC1200_FS_DIVTWO,         0x03}, // Frequency Synthesizer Divide by 2
        {CC1200_FS_DSM0,           0x33}, // FS Digital Synthesizer Module Configuration Reg. 0
        {CC1200_FS_DVC0,           0x17}, // Frequency Synthesizer Divider Chain Configuration ..
        {CC1200_FS_PFD,            0x00}, // Frequency Synthesizer Phase Frequency Detector Con..
        {CC1200_FS_PRE,            0x6E}, // Frequency Synthesizer Prescaler Configuration
        {CC1200_FS_REG_DIV_CML,    0x1C}, // Frequency Synthesizer Divider Regulator Configurat..
        {CC1200_FS_SPARE,          0xAC}, // Frequency Synthesizer Spare
        {CC1200_FS_VCO0,           0xB5}, // FS Voltage Controlled Oscillator Configuration Reg..
        {CC1200_IFAMP,             0x09}, // Intermediate Frequency Amplifier Configuration
        {CC1200_XOSC5,             0x0E}, // Crystal Oscillator Configuration Reg. 5
        {CC1200_XOSC1,             0x03}, // Crystal Oscillator Configuration Reg. 1
};



// -- More conservative as it seems that RPi cannot keep up with the interrupts
// Deviation = 49.896240
// Packet length = 255
// RX filter BW = 208.333333
// Packet bit length = 0
// Bit rate = 100
// Carrier frequency = 920.599976
// Manchester enable = false
// Symbol rate = 100
// Device address = 0
// Whitening = false
// Address config = No address check
// Modulation format = 2-GFSK
// Packet length mode = Variable

static const struct cc_cfg_reg CC_CFG_DEFAULT_1[] = {
        {CC1200_SYNC3,             0x6F},
        {CC1200_SYNC2,             0x4E},
        {CC1200_SYNC1,             0x90}, // Sync Word Configuration [15:8]
        {CC1200_SYNC0,             0x4E}, // Sync Word Configuration [7:0]

        {CC1200_SYNC_CFG1,         0xA8}, // Sync Word Detection Configuration Reg. 1
        {CC1200_SYNC_CFG0,         0x23}, // Sync Word Detection Configuration Reg. 0
        {CC1200_DEVIATION_M,       0x47}, // Frequency Deviation Configuration
        {CC1200_MODCFG_DEV_E,      0x0C}, // Modulation Format and Frequency Deviation Configur..
        {CC1200_DCFILT_CFG,        0x4B}, // Digital DC Removal Configuration
        {CC1200_PREAMBLE_CFG0,     0x8A}, // Preamble Detection Configuration Reg. 0
        {CC1200_IQIC,              0xD8}, // Digital Image Channel Compensation Configuration
        {CC1200_CHAN_BW,           0x08}, // Channel Filter Configuration
        {CC1200_MDMCFG1,           0x42}, // General Modem Parameter Configuration Reg. 1
        {CC1200_MDMCFG0,           0x05}, // General Modem Parameter Configuration Reg. 0
        {CC1200_SYMBOL_RATE2,      0xA4}, // Symbol Rate Configuration Exponent and Mantissa [1..
        {CC1200_SYMBOL_RATE1,      0x7A}, // Symbol Rate Configuration Mantissa [15:8]
        {CC1200_SYMBOL_RATE0,      0xE1}, // Symbol Rate Configuration Mantissa [7:0]
        {CC1200_AGC_REF,           /*0xEA*//*0xA1*//*0x31*/0x2A}, // AGC Reference Level Configuration
        {CC1200_AGC_CS_THR,        0x01/*0x11*/}, // Carrier Sense Threshold Configuration
        {CC1200_AGC_CFG1,          0x12}, // Automatic Gain Control Configuration Reg. 1
        {CC1200_AGC_CFG0,          /*0x8b*/0x80}, // Automatic Gain Control Configuration Reg. 0
        {CC1200_FIFO_CFG,          0x00}, // FIFO Configuration
        {CC1200_FS_CFG,            0x12}, // Frequency Synthesizer Configuration
        {CC1200_PKT_CFG2,          0x00}, // Packet Configuration Reg. 2
        {CC1200_PKT_CFG0,          0x20}, // Packet Configuration Reg. 0
        {CC1200_PKT_LEN,           0xFF}, // Packet Length Configuration
        {CC1200_IF_MIX_CFG,        0x1C}, // IF Mix Configuration
        {CC1200_TOC_CFG,           0x03}, // Timing Offset Correction Configuration
        {CC1200_MDMCFG2,           0x02}, // General Modem Parameter Configuration Reg. 2
        {CC1200_FREQ2,             0x5C}, // Frequency Configuration [23:16]
        {CC1200_FREQ1,             0x0F}, // Frequency Configuration [15:8]
        {CC1200_FREQ0,             0x5C}, // Frequency Configuration [7:0]
        {CC1200_IF_ADC1,           0xEE}, // Analog to Digital Converter Configuration Reg. 1
        {CC1200_IF_ADC0,           0x10}, // Analog to Digital Converter Configuration Reg. 0
        {CC1200_FS_DIG1,           0x04}, // Frequency Synthesizer Digital Reg. 1
        {CC1200_FS_DIG0,           0x55}, // Frequency Synthesizer Digital Reg. 0
        {CC1200_FS_CAL1,           0x40}, // Frequency Synthesizer Calibration Reg. 1
        {CC1200_FS_CAL0,           0x0E}, // Frequency Synthesizer Calibration Reg. 0
        {CC1200_FS_DIVTWO,         0x03}, // Frequency Synthesizer Divide by 2
        {CC1200_FS_DSM0,           0x33}, // FS Digital Synthesizer Module Configuration Reg. 0
        {CC1200_FS_DVC0,           0x17}, // Frequency Synthesizer Divider Chain Configuration ..
        {CC1200_FS_PFD,            0x00}, // Frequency Synthesizer Phase Frequency Detector Con..
        {CC1200_FS_PRE,            0x6E}, // Frequency Synthesizer Prescaler Configuration
        {CC1200_FS_REG_DIV_CML,    0x1C}, // Frequency Synthesizer Divider Regulator Configurat..
        {CC1200_FS_SPARE,          0xAC}, // Frequency Synthesizer Spare
        {CC1200_FS_VCO0,           0xB5}, // FS Voltage Controlled Oscillator Configuration Reg..
        {CC1200_IFAMP,             0x09}, // Intermediate Frequency Amplifier Configuration
        {CC1200_XOSC5,             0x0E}, // Crystal Oscillator Configuration Reg. 5
        {CC1200_XOSC1,             0x03}, // Crystal Oscillator Configuration Reg. 1
};