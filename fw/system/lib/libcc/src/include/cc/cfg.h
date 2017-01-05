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

        u8 data = cc_get(dev, regs[i].addr);

        if (regs[i].addr == CC1200_SERIAL_STATUS) {
            // Mask out clock signals etc.
            data &= regs[i].data;
        }

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
// Symbol rate = 50 (was 38.4)
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

static const struct cc_cfg_reg CC_CFG_DEFAULT_0[] = {
        {CC1200_SYNC3,             0x6F}, // Sync Word Configuration [31:24]
        {CC1200_SYNC2,             0x4E}, // Sync Word Configuration [23:16]
        {CC1200_SYNC1,             0x90}, // Sync Word Configuration [15:8]
        {CC1200_SYNC0,             0x4E}, // Sync Word Configuration [7:0]
        {CC1200_SYNC_CFG1,         0xA5/*0xE9*/}, // Sync Word Detection Configuration Reg. 1
        //{CC1200_SYNC_CFG0,         0x23},
        //{CC1200_DEVIATION_M,       0x06},
        {CC1200_MODCFG_DEV_E,      0x0B}, // Modulation Format and Frequency Deviation Configur..
        ///*NEW-PA*/{CC1200_DCFILT_CFG,        0x56}, // Digital DC Removal Configuration
        {CC1200_PREAMBLE_CFG1,     0x29}, // Preamble Length Configuration Reg. 1
        {CC1200_PREAMBLE_CFG0,     0x8A}, // Preamble Detection Configuration Reg. 0
        /*NEW-PA*/{CC1200_IQIC,              0x4B/*0xC8*/}, // Digital Image Channel Compensation Configuration
        {CC1200_CHAN_BW,           0x84/*0x10*/}, // Channel Filter Configuration
        {CC1200_MDMCFG1,           0x40/*0x42*/}, // General Modem Parameter Configuration Reg. 1
        {CC1200_MDMCFG0,           0x05}, // General Modem Parameter Configuration Reg. 0
        {CC1200_SYMBOL_RATE2,      0x94/*0x8F*/}, // Symbol Rate Configuration Exponent and Mantissa [1..
        {CC1200_SYMBOL_RATE1,      0x7A/*0x75*/}, // Symbol Rate Configuration Mantissa [15:8]
        {CC1200_SYMBOL_RATE0,      0xE1/*0x10*/}, // Symbol Rate Configuration Mantissa [7:0]
        /*NEW-PA*/{CC1200_AGC_REF,           0x4E/*0x21*//*0x27*/}, // AGC Reference Level Configuration
        /*NEW-PA*/{CC1200_AGC_CS_THR,        0xEC/*0x01*/}, // Carrier Sense Threshold Configuration
        {CC1200_AGC_CFG1,          0x11}, // Automatic Gain Control Configuration Reg. 1
        /*NEW-PA*/{CC1200_AGC_CFG0,          /*0x90*/0x94}, // Automatic Gain Control Configuration Reg. 0
        {CC1200_FIFO_CFG,          0x00}, // FIFO Configuration
        {CC1200_FS_CFG,            0x12}, // Frequency Synthesizer Configuration
        {CC1200_PKT_CFG2,          0x00}, // Packet Configuration Reg. 2
        {CC1200_PKT_CFG0,          0x20}, // Packet Configuration Reg. 0
        {CC1200_PKT_LEN,           0xFF}, // Packet Length Configuration
        /*NEW-PA*/{CC1200_IF_MIX_CFG,        /*0x18*/0x1C}, // IF Mix Configuration
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
        /*NEW-PA*/{CC1200_IFAMP,             /*0x05*/0x09}, // Intermediate Frequency Amplifier Configuration
        {CC1200_XOSC5,             0x0E}, // Crystal Oscillator Configuration Reg. 5
        {CC1200_XOSC1,             0x03}, // Crystal Oscillator Configuration Reg. 1
        ///*NEW-PA*/{CC1200_PA_CFG0,           0x53}
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


// Address Config = No address check
// Bit Rate = 200 (was 250)
// Carrier Frequency = 920.599976
// Deviation = 70 (was ~87.5 (was 62.561035))
// Device Address = 0
// Manchester Enable = false
// Modulation Format = 2-GFSK
// Packet Bit Length = 0
// Packet Length = 255
// Packet Length Mode = Variable
// RX Filter BW = 333 (was 555.555556)
// Symbol rate = 200 (was 250)
// Whitening = false

static const struct cc_cfg_reg CC_CFG_DEFAULT[] = {
        {CC1200_SYNC3,             0x6F},
        {CC1200_SYNC2,             0x4E},
        {CC1200_SYNC1,             0x90}, // Sync Word Configuration [15:8]
        {CC1200_SYNC0,             0x4E}, // Sync Word Configuration [7:0]

        {CC1200_SYNC_CFG1,         0x49/*new: forgot to change sync word length to 2 bytes. thr is 0x07.*//*0xA7*//*0xA9*//*0xA8*/},
        /*!!UNCONFIRMED CHANGE*/{CC1200_SYNC_CFG0,         0x33/*0x13*/}, // 2byte sync. bit 5: auto clear (|==0x33) (0x23: auto clear without rx bw cfg limitation)
        /*!!UNCONFIRMED CHANGE*/{CC1200_DEVIATION_M,       /*0x48*//*0xCB*/0x1F/*0x9A*/},
        /*!!UNCONFIRMED CHANGE*/{CC1200_MODCFG_DEV_E,      0x0D/*new,same but no DSSS*/ /*0x8D*//*0x8C*//*0x8D*//*0x8C*/}, // 0x80: coding gain
        {CC1200_DCFILT_CFG,        0x4B},
        /*!!UNCONFIRMED CHANGE*/{CC1200_PREAMBLE_CFG1,     0x10/*new,2byte*/ /*0x18*//*0x34*/}, // 0x34: 30-byte preamble (4 == 0x18)
        {CC1200_PREAMBLE_CFG0,     0x8A},
        {CC1200_IQIC,              0x58/*NOTE: changing back to 0x58 improved AGC/RSSI performance*//*0x80*//*0x58*/},
        /*!!UNCONFIRMED CHANGE*/{CC1200_CHAN_BW,           /*0x05*//*0x04*/0x03},
        {CC1200_MDMCFG1,           0x42 /*new: enable collision detect*/|0x08}, // |=0x80: CS gates sync search   |=((0x00-0x03)<<1) DVGA gain 0/-18/3/6 dB
        {CC1200_MDMCFG0,           0x05},
        {CC1200_SYMBOL_RATE2,      /*0xB4*/0xB9},
        {CC1200_SYMBOL_RATE1,      /*0x7A*/0x99},
        {CC1200_SYMBOL_RATE0,      /*0xE1*/0x9A},
        /*!!UNCONFIRMED CHANGE*/{CC1200_AGC_REF,           0x2E/*0x3D*//*0x37*/}, // Calibrated setting: 0x2E
        {CC1200_AGC_CS_THR,        (u8)/*-103*/-77},
        /*!!UNCONFIRMED CHANGE*/{CC1200_AGC_CFG2,          /*0x80*/0x40/*new: put back to 0x40 -- does this depend on the PA state?*/}, //  Calibrated setting: 0x40. Default: 0x20 (Normal mode, max gain 39dB). 0x4:Performance/max=27dB
        {CC1200_AGC_CFG1,          0x16},
        {CC1200_AGC_CFG0,          0x84},
        {CC1200_FIFO_CFG,          0x00},
        {CC1200_FS_CFG,            0x12},
        {CC1200_PKT_CFG2,          0x00},
        {CC1200_PKT_CFG0,          0x20},
        {CC1200_PA_CFG1,           0x77/*0x63*//*0x55*/}, // w/pa: 0x55 == 17dBm 0x77 == 26+dBm other: 0x63 == 0dBm 0x43 == min
        {CC1200_PA_CFG0,           0x51},
        {CC1200_PKT_LEN,           0xFF},
        /*!!UNCONFIRMED CHANGE*/{CC1200_IF_MIX_CFG,        0x1C/*0x1C*/}, // IF: highest freq
        ///*!!UNCONFIRMED CHANGE*/{CC1200_FREQOFF_CFG,       0x3D}, // FOC with low gain and in fast loop with high gain after sync detect
        {CC1200_TOC_CFG,           0x03},
        {CC1200_MDMCFG2,           0x02},
        {CC1200_FREQ2,             0x5C},
        {CC1200_FREQ1,             0x0F},
        {CC1200_FREQ0,             0x5C},
        {CC1200_IF_ADC1,           0xEE},
        {CC1200_IF_ADC0,           0x10},
        {CC1200_FS_DIG1,           0x04},
        {CC1200_FS_DIG0,           0x55},
        /*!!UNCONFIRMED CHANGE*/{CC1200_FS_CAL3,           0x40}, // More accurate calibration
        {CC1200_FS_CAL1,           0x40},
        {CC1200_FS_CAL0,           0x0E},
        {CC1200_FS_DIVTWO,         0x03},
        {CC1200_FS_DSM0,           0x33},
        {CC1200_FS_DVC0,           0x17},
        {CC1200_FS_PFD,            0x00},
        {CC1200_FS_PRE,            0x6E},
        {CC1200_FS_REG_DIV_CML,    0x1C},
        {CC1200_FS_SPARE,          0xAC},
        {CC1200_FS_VCO0,           0xB5},
        /*!!UNCONFIRMED CHANGE*/{CC1200_IFAMP,             0x09/*0x01*//*0x09*/}, // IFAMP (SS) BW 1000kHz -> 300kHz [NOTE: Seems to affect RSSI level, unsure of calibration after this. Keeping because it raises LQI. Also technically breaks the rule outlined in the user guide.]
        {CC1200_XOSC5,             0x0E},
        {CC1200_XOSC1,             0x03},
};

// Address Config = No address check
// Bit Rate = 30 (was 38.4)
// Carrier Frequency = 920.599976
// Deviation = 15 (was 7.5 (was 19.989014))
// Device Address = 0
// Manchester Enable = false
// Modulation Format = 2-GFSK
// Packet Bit Length = 0
// Packet Length = 255
// Packet Length Mode = Variable
// RX Filter BW = 92.592593  (was 79.365079)
// Symbol rate = 30
// Whitening = true

static const struct cc_cfg_reg CC_CFG_DEFAULT_NB[] = {
        {CC1200_SYNC_CFG1,         0xA9},
        {CC1200_DEVIATION_M,       0x89/*DEFAULT*/},
        {CC1200_MODCFG_DEV_E,      0x0A/*0x09*//*0x0B*/},
        {CC1200_PREAMBLE_CFG0,     0x8A},
        {CC1200_IQIC,              0xC8},
        {CC1200_CHAN_BW,           0x12/*0x0E*//*0x15*/},
        {CC1200_MDMCFG1,           0x42},
        {CC1200_MDMCFG0,           0x05},
        {CC1200_SYMBOL_RATE2,      0x88/*0x8F*/},
        {CC1200_SYMBOL_RATE1,      0x93/*0x75*/},
        {CC1200_SYMBOL_RATE0,      0x75/*0x10*/},
        {CC1200_AGC_REF,           0x2B},
        {CC1200_AGC_CS_THR,        (u8)-99},
        /*!!UNCONFIRMED CHANGE*/{CC1200_AGC_CFG2,          0x80}, // Added, unconfirmed
        {CC1200_AGC_CFG1,          0x11},
        {CC1200_AGC_CFG0,          0x94},
        {CC1200_FIFO_CFG,          0x00},
        {CC1200_FS_CFG,            0x12},
        {CC1200_PKT_CFG2,          0x00},
        {CC1200_PKT_CFG1,          0x43},
        {CC1200_PKT_CFG0,          0x20},
        {CC1200_PA_CFG1,           0x55},
        {CC1200_PKT_LEN,           0xFF},
        {CC1200_IF_MIX_CFG,        0x1C},
        //{CC1200_FREQOFF_CFG,       0x3C},
        {CC1200_TOC_CFG,           0x03},
        {CC1200_MDMCFG2,           0x02},
        {CC1200_FREQ2,             0x5C},
        {CC1200_FREQ1,             0x0F},
        {CC1200_FREQ0,             0x5C},
        {CC1200_IF_ADC1,           0xEE},
        {CC1200_IF_ADC0,           0x10},
        {CC1200_FS_DIG1,           0x04},
        {CC1200_FS_DIG0,           0x55},
        {CC1200_FS_CAL1,           0x40},
        {CC1200_FS_CAL0,           0x0E},
        {CC1200_FS_DIVTWO,         0x03},
        {CC1200_FS_DSM0,           0x33},
        {CC1200_FS_DVC0,           0x17},
        {CC1200_FS_PFD,            0x00},
        {CC1200_FS_PRE,            0x6E},
        {CC1200_FS_REG_DIV_CML,    0x1C},
        {CC1200_FS_SPARE,          0xAC},
        {CC1200_FS_VCO0,           0xB5},
        {CC1200_IFAMP,             0x09},
        {CC1200_XOSC5,             0x0E},
        {CC1200_XOSC1,             0x03},
};
