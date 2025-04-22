// RH_RF24.h
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2014 Mike McCauley
// $Id: RH_RF24.h,v 1.18 2017/07/25 05:26:50 mikem Exp $
//
// Supports RF24/RF26 and RFM24/RFM26 modules in FIFO mode
// also Si4464/63/62/61/60-A1
// Si4063 is the same but Tx only
//
// Per http://www.hoperf.cn/upload/rf/RFM24.pdf
// and http://www.hoperf.cn/upload/rf/RFM26.pdf
// Sigh: the HopeRF documentation is utter rubbish: full of errors and incomplete. The Si446x docs are better:
// http://www.silabs.com/Support%20Documents/TechnicalDocs/Si4464-63-61-60.pdf
// http://www.silabs.com/Support%20Documents/TechnicalDocs/AN626.pdf
// http://www.silabs.com/Support%20Documents/TechnicalDocs/AN627.pdf
// http://www.silabs.com/Support%20Documents/TechnicalDocs/AN647.pdf
// http://www.silabs.com/Support%20Documents/TechnicalDocs/AN633.pdf
// http://www.silabs.com/Support%20Documents/TechnicalDocs/AN736.pdf
// http://nicerf.com/manage/upfile/indexbanner/635231050196868750.pdf    (API description)
// http://www.silabs.com/Support%20Documents/Software/Si446x%20RX_HOP%20PLL%20Calculator.xlsx
#ifndef RH_RF24_h
#define RH_RF24_h

#include <RHGenericSPI.h>
#include <RHSPIDriver.h>

// This is the maximum number of interrupts the driver can support
// Most Arduinos can handle 2, Megas can handle more
#define RH_RF24_NUM_INTERRUPTS 3

// Maximum payload length the RF24 can support, limited by our 1 octet message length
#define RH_RF24_MAX_PAYLOAD_LEN 255

// The length of the headers we add.
// The headers are inside the RF24's payload
#define RH_RF24_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for message length 4 bytes of address and header and payload to be included in payload size limit.
#ifndef RH_RF24_MAX_MESSAGE_LEN
#define RH_RF24_MAX_MESSAGE_LEN (RH_RF24_MAX_PAYLOAD_LEN - RH_RF24_HEADER_LEN - 1)
#endif

// Max number of times we will try to read CTS from the radio
#define RH_RF24_CTS_RETRIES 2500

// RF24/RF26 API commands from table 10
// also Si446X API DESCRIPTIONS table 1
#define RH_RF24_CMD_NOP                        0x00
#define RH_RF24_CMD_PART_INFO                  0x01
#define RH_RF24_CMD_POWER_UP                   0x02
#define RH_RF24_CMD_PATCH_IMAGE                0x04
#define RH_RF24_CMD_FUNC_INFO                  0x10
#define RH_RF24_CMD_SET_PROPERTY               0x11
#define RH_RF24_CMD_GET_PROPERTY               0x12
#define RH_RF24_CMD_GPIO_PIN_CFG               0x13
#define RH_RF24_CMD_GET_ADC_READING            0x14
#define RH_RF24_CMD_FIFO_INFO                  0x15
#define RH_RF24_CMD_PACKET_INFO                0x16
#define RH_RF24_CMD_IRCAL                      0x17
#define RH_RF24_CMD_PROTOCOL_CFG               0x18
#define RH_RF24_CMD_GET_INT_STATUS             0x20
#define RH_RF24_CMD_GET_PH_STATUS              0x21
#define RH_RF24_CMD_GET_MODEM_STATUS           0x22
#define RH_RF24_CMD_GET_CHIP_STATUS            0x23
#define RH_RF24_CMD_START_TX                   0x31
#define RH_RF24_CMD_START_RX                   0x32
#define RH_RF24_CMD_REQUEST_DEVICE_STATE       0x33
#define RH_RF24_CMD_CHANGE_STATE               0x34
#define RH_RF24_CMD_RX_HOP                     0x36
#define RH_RF24_CMD_READ_BUF                   0x44
#define RH_RF24_CMD_FAST_RESPONSE_A            0x50
#define RH_RF24_CMD_FAST_RESPONSE_B            0x51
#define RH_RF24_CMD_FAST_RESPONSE_C            0x53
#define RH_RF24_CMD_FAST_RESPONSE_D            0x57
#define RH_RF24_CMD_TX_FIFO_WRITE              0x66
#define RH_RF24_CMD_RX_FIFO_READ               0x77

// The Clear To Send signal from the radio
#define RH_RF24_REPLY_CTS                      0xff

//#define RH_RF24_CMD_START_TX                   0x31
#define RH_RF24_CONDITION_TX_COMPLETE_STATE      0xf0
#define RH_RF24_CONDITION_RETRANSMIT_NO          0x00
#define RH_RF24_CONDITION_RETRANSMIT_YES         0x04
#define RH_RF24_CONDITION_START_IMMEDIATE        0x00
#define RH_RF24_CONDITION_START_AFTER_WUT        0x01

//#define RH_RF24_CMD_START_RX                   0x32
#define RH_RF24_CONDITION_RX_START_IMMEDIATE     0x00

//#define RH_RF24_CMD_REQUEST_DEVICE_STATE       0x33
#define RH_RF24_DEVICE_STATE_NO_CHANGE           0x00
#define RH_RF24_DEVICE_STATE_SLEEP               0x01
#define RH_RF24_DEVICE_STATE_SPI_ACTIVE          0x02
#define RH_RF24_DEVICE_STATE_READY               0x03
#define RH_RF24_DEVICE_STATE_ALSO_READY          0x04
#define RH_RF24_DEVICE_STATE_TUNE_TX             0x05
#define RH_RF24_DEVICE_STATE_TUNE_RX             0x06
#define RH_RF24_DEVICE_STATE_TX                  0x07
#define RH_RF24_DEVICE_STATE_RX                  0x08

// Properties for API Description AN625 Section 2.2
#define RH_RF24_PROPERTY_GLOBAL_XO_TUNE                   0x0000
#define RH_RF24_PROPERTY_GLOBAL_CLK_CFG                   0x0001
#define RH_RF24_PROPERTY_GLOBAL_LOW_BATT_THRESH           0x0002
#define RH_RF24_PROPERTY_GLOBAL_CONFIG                    0x0003
#define RH_RF24_PROPERTY_GLOBAL_WUT_CONFIG                0x0004
#define RH_RF24_PROPERTY_GLOBAL_WUT_M_15_8                0x0005
#define RH_RF24_PROPERTY_GLOBAL_WUT_M_7_0                 0x0006
#define RH_RF24_PROPERTY_GLOBAL_WUT_R                     0x0007
#define RH_RF24_PROPERTY_GLOBAL_WUT_LDC                   0x0008
#define RH_RF24_PROPERTY_INT_CTL_ENABLE                   0x0100
#define RH_RF24_PROPERTY_INT_CTL_PH_ENABLE                0x0101
#define RH_RF24_PROPERTY_INT_CTL_MODEM_ENABLE             0x0102
#define RH_RF24_PROPERTY_INT_CTL_CHIP_ENABLE              0x0103
#define RH_RF24_PROPERTY_FRR_CTL_A_MODE                   0x0200
#define RH_RF24_PROPERTY_FRR_CTL_B_MODE                   0x0201
#define RH_RF24_PROPERTY_FRR_CTL_C_MODE                   0x0202
#define RH_RF24_PROPERTY_FRR_CTL_D_MODE                   0x0203
#define RH_RF24_PROPERTY_PREAMBLE_TX_LENGTH               0x1000
#define RH_RF24_PROPERTY_PREAMBLE_CONFIG_STD_1            0x1001
#define RH_RF24_PROPERTY_PREAMBLE_CONFIG_NSTD             0x1002
#define RH_RF24_PROPERTY_PREAMBLE_CONFIG_STD_2            0x1003
#define RH_RF24_PROPERTY_PREAMBLE_CONFIG                  0x1004
#define RH_RF24_PROPERTY_PREAMBLE_PATTERN_31_24           0x1005
#define RH_RF24_PROPERTY_PREAMBLE_PATTERN_23_16           0x1006
#define RH_RF24_PROPERTY_PREAMBLE_PATTERN_15_8            0x1007
#define RH_RF24_PROPERTY_PREAMBLE_PATTERN_7_0             0x1008
#define RH_RF24_PROPERTY_SYNC_CONFIG                      0x1100
#define RH_RF24_PROPERTY_SYNC_BITS_31_24                  0x1101
#define RH_RF24_PROPERTY_SYNC_BITS_23_16                  0x1102
#define RH_RF24_PROPERTY_SYNC_BITS_15_8                   0x1103
#define RH_RF24_PROPERTY_SYNC_BITS_7_0                    0x1104
#define RH_RF24_PROPERTY_PKT_CRC_CONFIG                   0x1200
#define RH_RF24_PROPERTY_PKT_CONFIG1                      0x1206
#define RH_RF24_PROPERTY_PKT_LEN                          0x1208
#define RH_RF24_PROPERTY_PKT_LEN_FIELD_SOURCE             0x1209
#define RH_RF24_PROPERTY_PKT_LEN_ADJUST                   0x120a
#define RH_RF24_PROPERTY_PKT_TX_THRESHOLD                 0x120b
#define RH_RF24_PROPERTY_PKT_RX_THRESHOLD                 0x120c
#define RH_RF24_PROPERTY_PKT_FIELD_1_LENGTH_12_8          0x120d
#define RH_RF24_PROPERTY_PKT_FIELD_1_LENGTH_7_0           0x120e
#define RH_RF24_PROPERTY_PKT_FIELD_1_CONFIG               0x120f
#define RH_RF24_PROPERTY_PKT_FIELD_1_CRC_CONFIG           0x1210
#define RH_RF24_PROPERTY_PKT_FIELD_2_LENGTH_12_8          0x1211
#define RH_RF24_PROPERTY_PKT_FIELD_2_LENGTH_7_0           0x1212
#define RH_RF24_PROPERTY_PKT_FIELD_2_CONFIG               0x1213
#define RH_RF24_PROPERTY_PKT_FIELD_2_CRC_CONFIG           0x1214
#define RH_RF24_PROPERTY_PKT_FIELD_3_LENGTH_12_8          0x1215
#define RH_RF24_PROPERTY_PKT_FIELD_3_LENGTH_7_0           0x1216
#define RH_RF24_PROPERTY_PKT_FIELD_3_CONFIG               0x1217
#define RH_RF24_PROPERTY_PKT_FIELD_3_CRC_CONFIG           0x1218
#define RH_RF24_PROPERTY_PKT_FIELD_4_LENGTH_12_8          0x1219
#define RH_RF24_PROPERTY_PKT_FIELD_4_LENGTH_7_0           0x121a
#define RH_RF24_PROPERTY_PKT_FIELD_4_CONFIG               0x121b
#define RH_RF24_PROPERTY_PKT_FIELD_4_CRC_CONFIG           0x121c
#define RH_RF24_PROPERTY_PKT_FIELD_5_LENGTH_12_8          0x121d
#define RH_RF24_PROPERTY_PKT_FIELD_5_LENGTH_7_0           0x121e
#define RH_RF24_PROPERTY_PKT_FIELD_5_CONFIG               0x121f
#define RH_RF24_PROPERTY_PKT_FIELD_5_CRC_CONFIG           0x1220
#define RH_RF24_PROPERTY_PKT_RX_FIELD_1_LENGTH_12_8       0x1221
#define RH_RF24_PROPERTY_PKT_RX_FIELD_1_LENGTH_7_0        0x1222
#define RH_RF24_PROPERTY_PKT_RX_FIELD_1_CONFIG            0x1223
#define RH_RF24_PROPERTY_PKT_RX_FIELD_1_CRC_CONFIG        0x1224
#define RH_RF24_PROPERTY_PKT_RX_FIELD_2_LENGTH_12_8       0x1225
#define RH_RF24_PROPERTY_PKT_RX_FIELD_2_LENGTH_7_0        0x1226
#define RH_RF24_PROPERTY_PKT_RX_FIELD_2_CONFIG            0x1227
#define RH_RF24_PROPERTY_PKT_RX_FIELD_2_CRC_CONFIG        0x1228
#define RH_RF24_PROPERTY_PKT_RX_FIELD_3_LENGTH_12_8       0x1229
#define RH_RF24_PROPERTY_PKT_RX_FIELD_3_LENGTH_7_0        0x122a
#define RH_RF24_PROPERTY_PKT_RX_FIELD_3_CONFIG            0x122b
#define RH_RF24_PROPERTY_PKT_RX_FIELD_3_CRC_CONFIG        0x122c
#define RH_RF24_PROPERTY_PKT_RX_FIELD_4_LENGTH_12_8       0x122d
#define RH_RF24_PROPERTY_PKT_RX_FIELD_4_LENGTH_7_0        0x122e
#define RH_RF24_PROPERTY_PKT_RX_FIELD_4_CONFIG            0x122f
#define RH_RF24_PROPERTY_PKT_RX_FIELD_4_CRC_CONFIG        0x1230
#define RH_RF24_PROPERTY_PKT_RX_FIELD_5_LENGTH_12_8       0x1231
#define RH_RF24_PROPERTY_PKT_RX_FIELD_5_LENGTH_7_0        0x1232
#define RH_RF24_PROPERTY_PKT_RX_FIELD_5_CONFIG            0x1233
#define RH_RF24_PROPERTY_PKT_RX_FIELD_5_CRC_CONFIG        0x1234
#define RH_RF24_PROPERTY_MODEM_MOD_TYPE                   0x2000
#define RH_RF24_PROPERTY_MODEM_MAP_CONTROL                0x2001
#define RH_RF24_PROPERTY_MODEM_DSM_CTRL                   0x2002
#define RH_RF24_PROPERTY_MODEM_DATA_RATE_2                0x2003
#define RH_RF24_PROPERTY_MODEM_DATA_RATE_1                0x2004
#define RH_RF24_PROPERTY_MODEM_DATA_RATE_0                0x2005
#define RH_RF24_PROPERTY_MODEM_TX_NCO_MODE_3              0x2006
#define RH_RF24_PROPERTY_MODEM_TX_NCO_MODE_2              0x2007
#define RH_RF24_PROPERTY_MODEM_TX_NCO_MODE_1              0x2008
#define RH_RF24_PROPERTY_MODEM_TX_NCO_MODE_0              0x2009
#define RH_RF24_PROPERTY_MODEM_FREQ_DEV_2                 0x200a
#define RH_RF24_PROPERTY_MODEM_FREQ_DEV_1                 0x200b
#define RH_RF24_PROPERTY_MODEM_FREQ_DEV_0                 0x200c
#define RH_RF24_PROPERTY_MODEM_TX_RAMP_DELAY              0x2018
#define RH_RF24_PROPERTY_MODEM_MDM_CTRL                   0x2019
#define RH_RF24_PROPERTY_MODEM_IF_CONTROL                 0x201a
#define RH_RF24_PROPERTY_MODEM_IF_FREQ_2                  0x201b
#define RH_RF24_PROPERTY_MODEM_IF_FREQ_1                  0x201c
#define RH_RF24_PROPERTY_MODEM_IF_FREQ_0                  0x201d
#define RH_RF24_PROPERTY_MODEM_DECIMATION_CFG1            0x201e
#define RH_RF24_PROPERTY_MODEM_DECIMATION_CFG0            0x201f
#define RH_RF24_PROPERTY_MODEM_BCR_OSR_1                  0x2022
#define RH_RF24_PROPERTY_MODEM_BCR_OSR_0                  0x2023
#define RH_RF24_PROPERTY_MODEM_BCR_NCO_OFFSET_2           0x2024
#define RH_RF24_PROPERTY_MODEM_BCR_NCO_OFFSET_1           0x2025
#define RH_RF24_PROPERTY_MODEM_BCR_NCO_OFFSET_0           0x2026
#define RH_RF24_PROPERTY_MODEM_BCR_GAIN_1                 0x2027
#define RH_RF24_PROPERTY_MODEM_BCR_GAIN_0                 0x2028
#define RH_RF24_PROPERTY_MODEM_BCR_GEAR                   0x2029
#define RH_RF24_PROPERTY_MODEM_BCR_MISC1                  0x202a
#define RH_RF24_PROPERTY_MODEM_AFC_GEAR                   0x202c
#define RH_RF24_PROPERTY_MODEM_AFC_WAIT                   0x202d
#define RH_RF24_PROPERTY_MODEM_AFC_GAIN_1                 0x202e
#define RH_RF24_PROPERTY_MODEM_AFC_GAIN_0                 0x202f
#define RH_RF24_PROPERTY_MODEM_AFC_LIMITER_1              0x2030
#define RH_RF24_PROPERTY_MODEM_AFC_LIMITER_0              0x2031
#define RH_RF24_PROPERTY_MODEM_AFC_MISC                   0x2032
#define RH_RF24_PROPERTY_MODEM_AGC_CONTROL                0x2035
#define RH_RF24_PROPERTY_MODEM_AGC_WINDOW_SIZE            0x2038
#define RH_RF24_PROPERTY_MODEM_AGC_RFPD_DECAY             0x2039
#define RH_RF24_PROPERTY_MODEM_AGC_IFPD_DECAY             0x203a
#define RH_RF24_PROPERTY_MODEM_FSK4_GAIN1                 0x203b
#define RH_RF24_PROPERTY_MODEM_FSK4_GAIN0                 0x203c
#define RH_RF24_PROPERTY_MODEM_FSK4_TH1                   0x203d
#define RH_RF24_PROPERTY_MODEM_FSK4_TH0                   0x203e
#define RH_RF24_PROPERTY_MODEM_FSK4_MAP                   0x203f
#define RH_RF24_PROPERTY_MODEM_OOK_PDTC                   0x2040
#define RH_RF24_PROPERTY_MODEM_OOK_CNT1                   0x2042
#define RH_RF24_PROPERTY_MODEM_OOK_MISC                   0x2043
#define RH_RF24_PROPERTY_MODEM_RAW_SEARCH                 0x2044
#define RH_RF24_PROPERTY_MODEM_RAW_CONTROL                0x2045
#define RH_RF24_PROPERTY_MODEM_RAW_EYE_1                  0x2046
#define RH_RF24_PROPERTY_MODEM_RAW_EYE_0                  0x2047
#define RH_RF24_PROPERTY_MODEM_ANT_DIV_MODE               0x2048
#define RH_RF24_PROPERTY_MODEM_ANT_DIV_CONTROL            0x2049
#define RH_RF24_PROPERTY_MODEM_RSSI_THRESH                0x204a
#define RH_RF24_PROPERTY_MODEM_RSSI_JUMP_THRESH           0x204b
#define RH_RF24_PROPERTY_MODEM_RSSI_CONTROL               0x204c
#define RH_RF24_PROPERTY_MODEM_RSSI_CONTROL2              0x204d
#define RH_RF24_PROPERTY_MODEM_RSSI_COMP                  0x204e
#define RH_RF24_PROPERTY_MODEM_ANT_DIV_CONT               0x2049
#define RH_RF24_PROPERTY_MODEM_CLKGEN_BAND                0x2051
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE13_7_0  0x2100
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE12_7_0  0x2101
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE11_7_0  0x2102
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE10_7_0  0x2103
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE9_7_0   0x2104
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE8_7_0   0x2105
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE7_7_0   0x2106
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE6_7_0   0x2107
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE5_7_0   0x2108
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE4_7_0   0x2109
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE3_7_0   0x210a
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE2_7_0   0x210b
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE1_7_0   0x210c
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE0_7_0   0x210d
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COEM0      0x210e
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COEM1      0x210f
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COEM2      0x2110
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COEM3      0x2111
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE13_7_0  0x2112
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE12_7_0  0x2113
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE11_7_0  0x2114
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE10_7_0  0x2115
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE9_7_0   0x2116
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE8_7_0   0x2117
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE7_7_0   0x2118
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE6_7_0   0x2119
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE5_7_0   0x211a
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE4_7_0   0x211b
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE3_7_0   0x211c
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE2_7_0   0x211d
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE1_7_0   0x211e
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE0_7_0   0x211f
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COEM0      0x2120
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COEM1      0x2121
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COEM2      0x2122
#define RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COEM3      0x2123
#define RH_RF24_PROPERTY_PA_MODE                          0x2200
#define RH_RF24_PROPERTY_PA_PWR_LVL                       0x2201
#define RH_RF24_PROPERTY_PA_BIAS_CLKDUTY                  0x2202
#define RH_RF24_PROPERTY_PA_TC                            0x2203
#define RH_RF24_PROPERTY_SYNTH_PFDCP_CPFF                 0x2300
#define RH_RF24_PROPERTY_SYNTH_PFDCP_CPINT                0x2301
#define RH_RF24_PROPERTY_SYNTH_VCO_KV                     0x2302
#define RH_RF24_PROPERTY_SYNTH_LPFILT3                    0x2303
#define RH_RF24_PROPERTY_SYNTH_LPFILT2                    0x2304
#define RH_RF24_PROPERTY_SYNTH_LPFILT1                    0x2305
#define RH_RF24_PROPERTY_SYNTH_LPFILT0                    0x2306
#define RH_RF24_PROPERTY_MATCH_VALUE_1                    0x3000
#define RH_RF24_PROPERTY_MATCH_MASK_1                     0x3001
#define RH_RF24_PROPERTY_MATCH_CTRL_1                     0x3002
#define RH_RF24_PROPERTY_MATCH_VALUE_2                    0x3003
#define RH_RF24_PROPERTY_MATCH_MASK_2                     0x3004
#define RH_RF24_PROPERTY_MATCH_CTRL_2                     0x3005
#define RH_RF24_PROPERTY_MATCH_VALUE_3                    0x3006
#define RH_RF24_PROPERTY_MATCH_MASK_3                     0x3007
#define RH_RF24_PROPERTY_MATCH_CTRL_3                     0x3008
#define RH_RF24_PROPERTY_MATCH_VALUE_4                    0x3009
#define RH_RF24_PROPERTY_MATCH_MASK_4                     0x300a
#define RH_RF24_PROPERTY_MATCH_CTRL_4                     0x300b
#define RH_RF24_PROPERTY_FREQ_CONTROL_INTE                0x4000
#define RH_RF24_PROPERTY_FREQ_CONTROL_FRAC_2              0x4001
#define RH_RF24_PROPERTY_FREQ_CONTROL_FRAC_1              0x4002
#define RH_RF24_PROPERTY_FREQ_CONTROL_FRAC_0              0x4003
#define RH_RF24_PROPERTY_FREQ_CONTROL_CHANNEL_STEP_SIZE_1 0x4004
#define RH_RF24_PROPERTY_FREQ_CONTROL_CHANNEL_STEP_SIZE_0 0x4005
#define RH_RF24_PROPERTY_FREQ_CONTROL_VCOCNT_RX_ADJ       0x4007
#define RH_RF24_PROPERTY_RX_HOP_CONTROL                   0x5000
#define RH_RF24_PROPERTY_RX_HOP_TABLE_SIZE                0x5001
#define RH_RF24_PROPERTY_RX_HOP_TABLE_ENTRY_0             0x5002

//#define RH_RF24_CMD_GPIO_PIN_CFG               0x13
#define RH_RF24_GPIO_NO_CHANGE                   0
#define RH_RF24_GPIO_DISABLED                    1
#define RH_RF24_GPIO_LOW                         2
#define RH_RF24_GPIO_HIGH                        3
#define RH_RF24_GPIO_INPUT                       4
#define RH_RF24_GPIO_32_KHZ_CLOCK                5
#define RH_RF24_GPIO_BOOT_CLOCK                  6
#define RH_RF24_GPIO_DIVIDED_MCU_CLOCK           7
#define RH_RF24_GPIO_CTS                         8
#define RH_RF24_GPIO_INV_CTS                     9
#define RH_RF24_GPIO_HIGH_ON_CMD_OVERLAP         10
#define RH_RF24_GPIO_SPI_DATA_OUT                11
#define RH_RF24_GPIO_HIGH_AFTER_RESET            12
#define RH_RF24_GPIO_HIGH_AFTER_CALIBRATION      13
#define RH_RF24_GPIO_HIGH_AFTER_WUT              14
#define RH_RF24_GPIO_UNUSED_0                    15
#define RH_RF24_GPIO_TX_DATA_CLOCK               16
#define RH_RF24_GPIO_RX_DATA_CLOCK               17
#define RH_RF24_GPIO_UNUSED_1                    18
#define RH_RF24_GPIO_TX_DATA                     19
#define RH_RF24_GPIO_RX_DATA                     20
#define RH_RF24_GPIO_RX_RAW_DATA                 21
#define RH_RF24_GPIO_ANTENNA_1_SWITCH            22
#define RH_RF24_GPIO_ANTENNA_2_SWITCH            23
#define RH_RF24_GPIO_VALID_PREAMBLE              24
#define RH_RF24_GPIO_INVALID_PREAMBLE            25
#define RH_RF24_GPIO_SYNC_DETECTED               26
#define RH_RF24_GPIO_RSSI_ABOVE_CAT              27
#define RH_RF24_GPIO_TX_STATE                    32
#define RH_RF24_GPIO_RX_STATE                    33
#define RH_RF24_GPIO_RX_FIFO_ALMOST_FULL         34
#define RH_RF24_GPIO_TX_FIFO_ALMOST_EMPTY        35
#define RH_RF24_GPIO_BATT_LOW                    36
#define RH_RF24_GPIO_RSSI_ABOVE_CAT_LOW          37
#define RH_RF24_GPIO_HOP                         38
#define RH_RF24_GPIO_HOP_TABLE_WRAPPED           39

// #define RH_RF24_CMD_GET_INT_STATUS             0x20
#define RH_RF24_INT_STATUS_CHIP_INT_STATUS                0x04
#define RH_RF24_INT_STATUS_MODEM_INT_STATUS               0x02
#define RH_RF24_INT_STATUS_PH_INT_STATUS                  0x01
#define RH_RF24_INT_STATUS_FILTER_MATCH                   0x80
#define RH_RF24_INT_STATUS_FILTER_MISS                    0x40
#define RH_RF24_INT_STATUS_PACKET_SENT                    0x20
#define RH_RF24_INT_STATUS_PACKET_RX                      0x10
#define RH_RF24_INT_STATUS_CRC_ERROR                      0x08
#define RH_RF24_INT_STATUS_TX_FIFO_ALMOST_EMPTY           0x02
#define RH_RF24_INT_STATUS_RX_FIFO_ALMOST_FULL            0x01
#define RH_RF24_INT_STATUS_INVALID_SYNC                   0x20
#define RH_RF24_INT_STATUS_RSSI_JUMP                      0x10
#define RH_RF24_INT_STATUS_RSSI                           0x08
#define RH_RF24_INT_STATUS_INVALID_PREAMBLE               0x04
#define RH_RF24_INT_STATUS_PREAMBLE_DETECT                0x02
#define RH_RF24_INT_STATUS_SYNC_DETECT                    0x01
#define RH_RF24_INT_STATUS_CAL                            0x40
#define RH_RF24_INT_STATUS_FIFO_UNDERFLOW_OVERFLOW_ERROR  0x20
#define RH_RF24_INT_STATUS_STATE_CHANGE                   0x10
#define RH_RF24_INT_STATUS_CMD_ERROR                      0x08
#define RH_RF24_INT_STATUS_CHIP_READY                     0x04
#define RH_RF24_INT_STATUS_LOW_BATT                       0x02
#define RH_RF24_INT_STATUS_WUT                            0x01

//#define RH_RF24_PROPERTY_GLOBAL_CLK_CFG                   0x0001
#define RH_RF24_CLK_CFG_DIVIDED_CLK_EN                    0x40
#define RH_RF24_CLK_CFG_DIVIDED_CLK_SEL_30                0x30
#define RH_RF24_CLK_CFG_DIVIDED_CLK_SEL_15                0x28
#define RH_RF24_CLK_CFG_DIVIDED_CLK_SEL_10                0x20
#define RH_RF24_CLK_CFG_DIVIDED_CLK_SEL_7_5               0x18
#define RH_RF24_CLK_CFG_DIVIDED_CLK_SEL_3                 0x10
#define RH_RF24_CLK_CFG_DIVIDED_CLK_SEL_2                 0x08
#define RH_RF24_CLK_CFG_DIVIDED_CLK_SEL_1                 0x00
#define RH_RF24_CLK_CFG_CLK_32K_SEL_EXTERNAL              0x02
#define RH_RF24_CLK_CFG_CLK_32K_SEL_RC                    0x01
#define RH_RF24_CLK_CFG_CLK_32K_SEL_DISABLED              0x00

//#define RH_RF24_PROPERTY_FRR_CTL_A_MODE                   0x0200
//#define RH_RF24_PROPERTY_FRR_CTL_B_MODE                   0x0201
//#define RH_RF24_PROPERTY_FRR_CTL_C_MODE                   0x0202
//#define RH_RF24_PROPERTY_FRR_CTL_D_MODE                   0x0203
#define RH_RF24_FRR_MODE_DISABLED                         0
#define RH_RF24_FRR_MODE_GLOBAL_STATUS                    1
#define RH_RF24_FRR_MODE_GLOBAL_INTERRUPT_PENDING         2
#define RH_RF24_FRR_MODE_PACKET_HANDLER_STATUS            3
#define RH_RF24_FRR_MODE_PACKET_HANDLER_INTERRUPT_PENDING 4
#define RH_RF24_FRR_MODE_MODEM_STATUS                     5
#define RH_RF24_FRR_MODE_MODEM_INTERRUPT_PENDING          6
#define RH_RF24_FRR_MODE_CHIP_STATUS                      7
#define RH_RF24_FRR_MODE_CHIP_INTERRUPT_PENDING           8
#define RH_RF24_FRR_MODE_CURRENT_STATE                    9
#define RH_RF24_FRR_MODE_LATCHED_RSSI                     10

//#define RH_RF24_PROPERTY_INT_CTL_ENABLE                   0x0100
#define RH_RF24_CHIP_INT_STATUS_EN                        0x04
#define RH_RF24_MODEM_INT_STATUS_EN                       0x02
#define RH_RF24_PH_INT_STATUS_EN                          0x01

//#define RH_RF24_PROPERTY_PREAMBLE_CONFIG                  0x1004
#define RH_RF24_PREAMBLE_FIRST_1                          0x20
#define RH_RF24_PREAMBLE_FIRST_0                          0x00
#define RH_RF24_PREAMBLE_LENGTH_NIBBLES                   0x00
#define RH_RF24_PREAMBLE_LENGTH_BYTES                     0x10
#define RH_RF24_PREAMBLE_MAN_CONST                        0x08
#define RH_RF24_PREAMBLE_MAN_ENABLE                       0x02
#define RH_RF24_PREAMBLE_NON_STANDARD                     0x00
#define RH_RF24_PREAMBLE_STANDARD_1010                    0x01
#define RH_RF24_PREAMBLE_STANDARD_0101                    0x02

//#define RH_RF24_PROPERTY_SYNC_CONFIG                      0x1100
#define RH_RF24_SYNC_CONFIG_SKIP_TX                       0x80
#define RH_RF24_SYNC_CONFIG_RX_ERRORS_MASK                0x70
#define RH_RF24_SYNC_CONFIG_4FSK                          0x08
#define RH_RF24_SYNC_CONFIG_MANCH                         0x04
#define RH_RF24_SYNC_CONFIG_LENGTH_MASK                   0x03

//#define RH_RF24_PROPERTY_PKT_CRC_CONFIG                   0x1200
#define RH_RF24_CRC_SEED_ALL_0S                           0x00
#define RH_RF24_CRC_SEED_ALL_1S                           0x80
#define RH_RF24_CRC_MASK                                  0x0f
#define RH_RF24_CRC_NONE                                  0x00
#define RH_RF24_CRC_ITU_T                                 0x01
#define RH_RF24_CRC_IEC_16                                0x02
#define RH_RF24_CRC_BIACHEVA                              0x03
#define RH_RF24_CRC_16_IBM                                0x04
#define RH_RF24_CRC_CCITT                                 0x05
#define RH_RF24_CRC_KOOPMAN                               0x06
#define RH_RF24_CRC_IEEE_802_3                            0x07
#define RH_RF24_CRC_CASTAGNOLI                            0x08

//#define RH_RF24_PROPERTY_PKT_CONFIG1                      0x1206
#define RH_RF24_PH_FIELD_SPLIT                            0x80
#define RH_RF24_PH_RX_DISABLE                             0x40
#define RH_RF24_4FSK_EN                                   0x20
#define RH_RF24_RX_MULTI_PKT                              0x10
#define RH_RF24_MANCH_POL                                 0x08
#define RH_RF24_CRC_INVERT                                0x04
#define RH_RF24_CRC_ENDIAN                                0x02
#define RH_RF24_BIT_ORDER                                 0x01

//#define RH_RF24_PROPERTY_PKT_FIELD_1_CONFIG               0x120f
//#define RH_RF24_PROPERTY_PKT_FIELD_2_CONFIG               0x1213
//#define RH_RF24_PROPERTY_PKT_FIELD_3_CONFIG               0x1217
//#define RH_RF24_PROPERTY_PKT_FIELD_4_CONFIG               0x121b
//#define RH_RF24_PROPERTY_PKT_FIELD_5_CONFIG               0x121f
#define RH_RF24_FIELD_CONFIG_4FSK                         0x10
#define RH_RF24_FIELD_CONFIG_WHITEN                       0x02
#define RH_RF24_FIELD_CONFIG_MANCH                        0x01

//#define RH_RF24_PROPERTY_PKT_RX_FIELD_1_CRC_CONFIG        0x1224
//#define RH_RF24_PROPERTY_PKT_RX_FIELD_2_CRC_CONFIG        0x1228
//#define RH_RF24_PROPERTY_PKT_RX_FIELD_3_CRC_CONFIG        0x122c
//#define RH_RF24_PROPERTY_PKT_RX_FIELD_4_CRC_CONFIG        0x1230
//#define RH_RF24_PROPERTY_PKT_RX_FIELD_5_CRC_CONFIG        0x1234
#define RH_RF24_FIELD_CONFIG_CRC_START                     0x80
#define RH_RF24_FIELD_CONFIG_SEND_CRC                      0x20
#define RH_RF24_FIELD_CONFIG_CHECK_CRC                     0x08
#define RH_RF24_FIELD_CONFIG_CRC_ENABLE                    0x02




//#define RH_RF24_PROPERTY_MODEM_MOD_TYPE                   0x2000
#define RH_RF24_TX_DIRECT_MODE_TYPE_SYNCHRONOUS           0x00
#define RH_RF24_TX_DIRECT_MODE_TYPE_ASYNCHRONOUS          0x80
#define RH_RF24_TX_DIRECT_MODE_GPIO0                      0x00
#define RH_RF24_TX_DIRECT_MODE_GPIO1                      0x20
#define RH_RF24_TX_DIRECT_MODE_GPIO2                      0x40
#define RH_RF24_TX_DIRECT_MODE_GPIO3                      0x60
#define RH_RF24_MOD_SOURCE_PACKET_HANDLER                 0x00
#define RH_RF24_MOD_SOURCE_DIRECT_MODE                    0x08
#define RH_RF24_MOD_SOURCE_RANDOM_GENERATOR               0x10
#define RH_RF24_MOD_TYPE_CW                               0x00
#define RH_RF24_MOD_TYPE_OOK                              0x01
#define RH_RF24_MOD_TYPE_2FSK                             0x02
#define RH_RF24_MOD_TYPE_2GFSK                            0x03
#define RH_RF24_MOD_TYPE_4FSK                             0x04
#define RH_RF24_MOD_TYPE_4GFSK                            0x05

//    RH_RF24_PROPERTY_PA_MODE                          0x2200
#define RH_RF24_PA_MODE_1_GROUP                           0x04
#define RH_RF24_PA_MODE_2_GROUPS                          0x08
#define RH_RF24_PA_MODE_CLASS_E                           0x00
#define RH_RF24_PA_MODE_SWITCH_CURRENT                    0x01


/////////////////////////////////////////////////////////////////////
/// \class RH_RF24 RH_RF24.h <RH_RF24.h>
/// \brief Driver to send and receive unaddressed, unreliable datagrams via an RF24 and compatible radio transceiver.
///
/// Works with 
/// - Silicon Labs Si4460/1/2/3/4 transceiver chips
/// - The equivalent HopeRF RF24/25/26/27 transceiver chips
/// - HopeRF Complete modules: RFM24W/26W/27W
///
/// \par Overview
///
/// This class provides basic functions for sending and receiving unaddressed, 
/// unreliable datagrams of arbitrary length to 250 octets per packet.
///
/// Manager classes may use this class to implement reliable, addressed datagrams and streams, 
/// mesh routers, repeaters, translators etc.
///
/// Naturally, for any 2 radios to communicate that must be configured to use the same frequency and 
/// modulation scheme.
///
/// This Driver provides an object-oriented interface for sending and receiving data messages with Hope-RF
/// RF24 and compatible radio modules, such as the RFM24W module.
///
/// The Hope-RF (http://www.hoperf.com) RF24 family is a low-cost ISM transceiver
/// chip. It supports FSK, GFSK, OOK over a wide range of frequencies and
/// programmable data rates. HopeRF also sell these chips on modules which includes
/// a crystal and antenna coupling circuits: RFM24W, RFM26W and RFM27W
///
/// This Driver provides functions for sending and receiving messages of up
/// to 250 octets on any frequency supported by the RF24, in a range of
/// predefined data rates and frequency deviations. Frequency can be set
/// to any frequency from 142.0MHz to 1050.0MHz. Caution: most modules only support a more limited
/// range of frequencies due to antenna tuning.
///
/// Up to 2 RFM24 modules can be connected to an Arduino (3 on a Mega),
/// permitting the construction of translators and frequency changers, etc.
///
/// The following modulation types are suppported with a range of modem configurations for 
/// common data rates and frequency deviations:
/// - OOK On-Off Keying
/// - GFSK Gaussian Frequency Shift Keying
/// - FSK Frequency Shift Keying
///
/// Support for other RF24 features such as on-chip temperature measurement, 
/// transmitter power control etc is also provided.
///
/// RH_RF24 uses interrupts to detect and handle events in the radio chip. The RF24 family has
/// TX and RX FIFOs of 64 bytes, but through the use of interrupt, the RH_RF24 driver can send longer 
/// messages by filling or emptying the FIFOs on-the-fly.
///
/// Tested on Anarduino Mini http://www.anarduino.com/mini/ with arduino-1.0.5
/// on OpenSuSE 13.1. Also on Anarduino Mini with arduino-1.8.1 on Kubuntu 16.04
///
/// \par Packet Format
///
/// All messages sent and received by this RH_RF24 Driver conform to this packet format:
///
/// - 4 octets PREAMBLE (configurable)
/// - 2 octets SYNC 0x2d, 0xd4 (configurable, so you can use this as a network filter)
/// - Field containing 1 octet of message length and 2 octet CRC protecting this field
/// - Field 2 containing at least 4 octets, and 2 octet CRC protecting this field:
///  + 4 octets HEADER: (TO, FROM, ID, FLAGS)
///  + 0 to 250 octets DATA 
///  + 2 octets CRC, computed on HEADER and DATA
///
/// \par Connecting RFM-24 to Arduino
///
/// For RFM24/RFM26 and Teensy 3.1 or Anarduino Mini
/// \code
///                 Teensy      RFM-24/RFM26
///                 GND----------GND (ground in)
///                 3V3----------VCC   (3.3V in)
/// interrupt 2 pin D2-----------NIRQ  (interrupt request out)
///          SS pin D10----------NSEL  (chip select in)
///         SCK pin D13----------SCK   (SPI clock in)
///        MOSI pin D11----------SDI   (SPI Data in)
///        MISO pin D12----------SDO   (SPI data out)
///                 D9-----------SDN   (shutdown in)
///                           /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
///                           \--TX_ANT (TX antenna control in) RFM22B only
///                           /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
///                           \--RX_ANT (RX antenna control in) RFM22B only
/// \endcode
/// Caution: tying the radio SDN pin to ground (though it might appear from the data sheets to make sense) 
/// does not always produce a reliable radio startup. So this driver controls the SDN pin directly.
/// Note: the GPIO0-TX_ANT and GPIO1-RX_ANT connections are not required for the 11dBm RFM24W, 
/// which has no antenna switch.
///
/// If you have an Arduino Zero, you should note that you cannot use Pin 2 for the interrupt line 
/// (Pin 2 is for the NMI only), instead you can use any other pin (we use Pin 3) and initialise RH_RF69 like this:
/// \code
/// // Slave Select is pin 10, interrupt is Pin 3
/// RH_RF24 driver(10, 3);
/// \endcode
///
/// \par Customising and configuring
///
/// The RH_RF24 module uses a radio configuration header file to configure the basic radio operation
/// frequency and modulation scheme. The radio configuration header file must be generated with the
/// Silicon Labs Wireless Development Suite (WDS) program and \#included by RH_RF24.cpp
/// 
/// The library will work out of the box and without further configuring with these parameters:
/// - Si4464 or equvalent
/// - 30MHz Crytstal
/// - 434MHz base frequncy band
/// - 2GFSK modulation
/// - 5kbps data rate
/// - 10kHz deviation
/// using the radio configuration header file
/// RF24configs/radio_config_Si4464_30_434_2GFSK_5_10.h
///      which is included in RadioHead.
///      
/// In order to use different frequency bands or modulation schemes, you must generate a new
/// radio configuration header file
/// with WDS, or select one of a small set of prebuilt headers from the RF24configs folder (see README in that
/// folder for details of the filename format).
///
/// To generate a new header file:
///
/// - Install Silicon Labs Wireless Development Suite (WDS) 3.2.11.0 or later 
///   (Windows only, we were not able to get it to run under Wine on Linux)
/// - Run WDS
/// - Menu->Start Simulation
/// - Select radio chip type Si4464, press Select Radio
/// - Select Radio Configuration Application, press Select Application
/// - Click on Standard Packet Tx
/// - On the Frequency and Power tab, Select the Frequency, crystal frequency etc. The PA power level is irrelevant, 
///   since power is set programatically 
/// - On the RF parameters tab, select the modulation type and rates
/// - Press Generate Source, Save custom radio configuration header file
/// - Enter a new unique file name in the RF24configs folder in RadioHead
/// - Edit RH_RF24.cpp to use this new header file
/// - Recompile RH_RF24
/// 
/// \par RSSI
///
/// The RSSI (Received Signal Strength Indicator) is measured and latched after the message sync bytes are received.
/// The latched RSSI is available from the lastRssi() member functionafter the complete message is received. 
/// Although lastRssi() 
/// supposedly returns a signed integer, in the case of this radio it actually returns an unsigned 8 bit integer (uint8_t)
/// and you will have to cast the return value to use it:
/// \code
/// uint8_t lastRssi = (uint8_t)rf24.lastRssi();
/// \endcode
/// The units of RSSI are arbitrary and relative, with larger unsigned numbers indicating a stronger signal. Values up to 255
/// are seen with radios in close proximity to each other. Lower limit of receivable strength is about 70.
///
/// \par Transmitter Power
///
/// You can control the transmitter power on the RF24/25/26/27 transceiver
/// with the RH_RF24::setTxPower() function. The argument can be any of
/// 0x00 to 0x4f (for RFM24/Si4460) or
/// 0x00 to 0x7f (for others)
/// 0x00 will yield no measurable power. For other settings there is a non-linear correlation with actual
/// RF power output (see below)
/// The default is 0x10. Eg:
/// \code
/// driver.setTxPower(0x10);
/// \endcode
///
/// We have made some actual power measurements against
/// programmed power
/// - Anarduino Mini with RFM24-433 and RFM26-433 at Vcc = 3.3V, in CW mode, 434MHz
/// - 10cm RG58C/U soldered direct to RFM69 module ANT and GND
/// - bnc connecteor
/// - 12dB attenuator
/// - BNC-SMA adapter
/// - MiniKits AD8307 HF/VHF Power Head (calibrated against Rohde&Schwartz 806.2020 test set)
/// - Digitech QM-1460 digital multimeter
/// \code
/// Program power           Measured Power dBm
///    HEX                  RFM24                RFM26
///    0x00                 not measurable       not measurable
///    0x01                 -20.4                -20.6
///    0x0f                 2.4                  4.8
///    0x1f                 9.4                  11.0
///    0x2f                 11.2                 14.2
///    0x3f                 11.6                 16.4
///    0x4f                 11.6                 18.0
///    0x5f                                      18.6
///    0x6f                                      19.0
///    0x7f                                      19.2
/// \endcode
/// Caution: the actual radiated power output will depend heavily on the power supply voltage and the antenna.

class RH_RF24 : public RHSPIDriver
{
public:
    /// \brief Defines property values for a set of modem configuration registers
    ///
    /// Defines property values for a set of modem configuration registers
    /// that can be passed to setModemRegisters() if none of the choices in
    /// ModemConfigChoice suit your need setModemRegisters() writes the
    /// property values from this structure to the appropriate RF24 properties
    /// to set the desired modulation type, data rate and deviation/bandwidth.
    /// OBSOLETE: no need ever to use this now
    typedef struct
    {
	uint8_t   prop_2000;   ///< Value for property RH_RF24_PROPERTY_MODEM_MOD_TYPE
	uint8_t   prop_2003;   ///< Value for property RH_RF24_PROPERTY_MODEM_DATA_RATE_2
	uint8_t   prop_2004;   ///< Value for property RH_RF24_PROPERTY_MODEM_DATA_RATE_1
	uint8_t   prop_2005;   ///< Value for property RH_RF24_PROPERTY_MODEM_DATA_RATE_0
	uint8_t   prop_2006;   ///< Value for property RH_RF24_PROPERTY_MODEM_TX_NCO_MODE_3
	uint8_t   prop_2007;   ///< Value for property RH_RF24_PROPERTY_MODEM_TX_NCO_MODE_2
	uint8_t   prop_2008;   ///< Value for property RH_RF24_PROPERTY_MODEM_TX_NCO_MODE_1
	uint8_t   prop_2009;   ///< Value for property RH_RF24_PROPERTY_MODEM_TX_NCO_MODE_0
	uint8_t   prop_200a;   ///< Value for property RH_RF24_PROPERTY_MODEM_FREQ_DEV_2
	uint8_t   prop_200b;   ///< Value for property RH_RF24_PROPERTY_MODEM_FREQ_DEV_1
	uint8_t   prop_200c;   ///< Value for property RH_RF24_PROPERTY_MODEM_FREQ_DEV_0
	uint8_t   prop_2018;   ///< Value for property RH_RF24_PROPERTY_MODEM_TX_RAMP_DELAY
	uint8_t   prop_201e;   ///< Value for property RH_RF24_PROPERTY_MODEM_DECIMATION_CFG1
	uint8_t   prop_201f;   ///< Value for property RH_RF24_PROPERTY_MODEM_DECIMATION_CFG0
	uint8_t   prop_2022;   ///< Value for property RH_RF24_PROPERTY_MODEM_BCR_OSR_1
	uint8_t   prop_2023;   ///< Value for property RH_RF24_PROPERTY_MODEM_BCR_OSR_0
	uint8_t   prop_2024;   ///< Value for property RH_RF24_PROPERTY_MODEM_BCR_NCO_OFFSET_2
	uint8_t   prop_2025;   ///< Value for property RH_RF24_PROPERTY_MODEM_BCR_NCO_OFFSET_1
	uint8_t   prop_2026;   ///< Value for property RH_RF24_PROPERTY_MODEM_BCR_NCO_OFFSET_0
	uint8_t   prop_2027;   ///< Value for property RH_RF24_PROPERTY_MODEM_BCR_GAIN_1
	uint8_t   prop_2028;   ///< Value for property RH_RF24_PROPERTY_MODEM_BCR_GAIN_0
	uint8_t   prop_2029;   ///< Value for property RH_RF24_PROPERTY_MODEM_BCR_GEAR
	uint8_t   prop_202d;   ///< Value for property RH_RF24_PROPERTY_MODEM_AFC_WAIT
	uint8_t   prop_202e;   ///< Value for property RH_RF24_PROPERTY_MODEM_AFC_GAIN_1
	uint8_t   prop_202f;   ///< Value for property RH_RF24_PROPERTY_MODEM_AFC_GAIN_0
	uint8_t   prop_2030;   ///< Value for property RH_RF24_PROPERTY_MODEM_AFC_LIMITER_1
	uint8_t   prop_2031;   ///< Value for property RH_RF24_PROPERTY_MODEM_AFC_LIMITER_0
	uint8_t   prop_2035;   ///< Value for property RH_RF24_PROPERTY_MODEM_AGC_CONTROL
	uint8_t   prop_2038;   ///< Value for property RH_RF24_PROPERTY_MODEM_AGC_WINDOW_SIZE
	uint8_t   prop_2039;   ///< Value for property RH_RF24_PROPERTY_MODEM_AGC_RFPD_DECAY
	uint8_t   prop_203a;   ///< Value for property RH_RF24_PROPERTY_MODEM_AGC_IFPD_DECAY
	uint8_t   prop_203b;   ///< Value for property RH_RF24_PROPERTY_MODEM_FSK4_GAIN1
	uint8_t   prop_203c;   ///< Value for property RH_RF24_PROPERTY_MODEM_FSK4_GAIN0
	uint8_t   prop_203d;   ///< Value for property RH_RF24_PROPERTY_MODEM_FSK4_TH1
	uint8_t   prop_203e;   ///< Value for property RH_RF24_PROPERTY_MODEM_FSK4_TH0
	uint8_t   prop_203f;   ///< Value for property RH_RF24_PROPERTY_MODEM_FSK4_MAP
	uint8_t   prop_2040;   ///< Value for property RH_RF24_PROPERTY_MODEM_OOK_PDTC
	uint8_t   prop_2043;   ///< Value for property RH_RF24_PROPERTY_MODEM_OOK_MISC
	uint8_t   prop_2045;   ///< Value for property RH_RF24_PROPERTY_MODEM_RAW_CONTROL
	uint8_t   prop_2046;   ///< Value for property RH_RF24_PROPERTY_MODEM_RAW_EYE_1
	uint8_t   prop_2047;   ///< Value for property RH_RF24_PROPERTY_MODEM_RAW_EYE_0
	uint8_t   prop_204e;   ///< Value for property RH_RF24_PROPERTY_MODEM_RSSI_COMP
	uint8_t   prop_2100;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE13_7_0
	uint8_t   prop_2101;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE12_7_0
	uint8_t   prop_2102;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE11_7_0
	uint8_t   prop_2103;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE10_7_0
	uint8_t   prop_2104;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE9_7_0
	uint8_t   prop_2105;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE8_7_0
	uint8_t   prop_2106;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE7_7_0
	uint8_t   prop_2107;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE6_7_0
	uint8_t   prop_2108;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE5_7_0
	uint8_t   prop_2109;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE4_7_0
	uint8_t   prop_210a;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE3_7_0
	uint8_t   prop_210b;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE2_7_0
	uint8_t   prop_210c;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE1_7_0
	uint8_t   prop_210d;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COE0_7_0
	uint8_t   prop_210e;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COEM0
	uint8_t   prop_210f;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COEM1
	uint8_t   prop_2110;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COEM2
	uint8_t   prop_2111;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX1_CHFLT_COEM3
	uint8_t   prop_2112;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE13_7_0
	uint8_t   prop_2113;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE12_7_0
	uint8_t   prop_2114;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE11_7_0
	uint8_t   prop_2115;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE10_7_0
	uint8_t   prop_2116;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE9_7_0
	uint8_t   prop_2117;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE8_7_0
	uint8_t   prop_2118;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE7_7_0
	uint8_t   prop_2119;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE6_7_0
	uint8_t   prop_211a;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE5_7_0
	uint8_t   prop_211b;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE4_7_0
	uint8_t   prop_211c;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE3_7_0
	uint8_t   prop_211d;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE2_7_0
	uint8_t   prop_211e;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE1_7_0
	uint8_t   prop_211f;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COE0_7_0
	uint8_t   prop_2120;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COEM0
	uint8_t   prop_2121;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COEM1
	uint8_t   prop_2122;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COEM2
	uint8_t   prop_2123;   ///< Value for property RH_RF24_PROPERTY_MODEM_CHFLT_RX2_CHFLT_COEM3
	uint8_t   prop_2203;   ///< Value for property RH_RF24_PROPERTY_PA_TC
	uint8_t   prop_2300;   ///< Value for property RH_RF24_PROPERTY_SYNTH_PFDCP_CPFF
	uint8_t   prop_2301;   ///< Value for property RH_RF24_PROPERTY_SYNTH_PFDCP_CPINT
	uint8_t   prop_2303;   ///< Value for property RH_RF24_PROPERTY_SYNTH_LPFILT3
	uint8_t   prop_2304;   ///< Value for property RH_RF24_PROPERTY_SYNTH_LPFILT2
	uint8_t   prop_2305;   ///< Value for property RH_RF24_PROPERTY_SYNTH_LPFILT1
    } ModemConfig;
  
    /// Choices for setModemConfig() for a selected subset of common
    /// modulation types, and data rates. If you need another configuration,
    /// use the register calculator.  and call setModemRegisters() with your
    /// desired settings.  
    /// These are indexes into MODEM_CONFIG_TABLE. We strongly recommend you use these symbolic
    /// definitions and not their integer equivalents: its possible that values will be
    /// changed in later versions (though we will try to avoid it).
    /// Contributions of new complete and tested ModemConfigs ready to add to this list will be readily accepted.
    /// OBSOLETE: no need ever to use this now
    typedef enum
    {
	FSK_Rb0_5Fd1 = 0,         ///< FSK  Rb = 0.5kbs, Fd = 1kHz
	FSK_Rb5Fd10,              ///< FSK  Rb = 5kbs,   Fd = 10kHz
	FSK_Rb50Fd100,            ///< FSK  Rb = 50kbs,  Fd = 100kHz
	FSK_Rb150Fd300,           ///< FSK  Rb = 50kbs,  Fd = 100kHz

	GFSK_Rb0_5Fd1,            ///< GFSK Rb = 0.5kbs, Fd = 1kHz
	GFSK_Rb5Fd10,             ///< GFSK Rb = 5kbs,   Fd = 10kHz
	GFSK_Rb50Fd100,           ///< GFSK Rb = 50kbs,  Fd = 100kHz
	GFSK_Rb150Fd300,          ///< GFSK Rb = 150kbs, Fd = 300kHz
	
	// We were unable to get any other OOKs to work
	OOK_Rb5Bw30,              ///< OOK  Rb = 5kbs,   Bw = 30kHz
	OOK_Rb10Bw40,             ///< OOK  Rb = 10kbs,  Bw = 40kHz

	// We were unable to get any 4FSK or 4GFSK schemes to work

    } ModemConfigChoice;

    /// \brief Defines the available choices for CRC
    /// Types of permitted CRC polynomials, to be passed to setCRCPolynomial()
    /// They deliberately have the same numeric values as the CRC_POLYNOMIAL field of PKT_CRC_CONFIG
    typedef enum
    {
	CRC_NONE = 0,
	CRC_ITU_T,
	CRC_IEC_16,
	CRC_Biacheva,
	CRC_16_IBM,
	CRC_CCITT,
	CRC_Koopman,
	CRC_IEEE_802_3,
	CRC_Castagnoli,
    } CRCPolynomial;

    /// \brief Defines the commands we can interrogate in printRegisters
    typedef struct
    {
	uint8_t      cmd;       ///< The command number
	uint8_t      replyLen;  ///< Number of bytes in the reply stream (after the CTS)
    }   CommandInfo;

    /// Constructor. You can have multiple instances, but each instance must have its own
    /// interrupt and slave select pin. After constructing, you must call init() to initialise the interface
    /// and the radio module. A maximum of 3 instances can co-exist on one processor, provided there are sufficient
    /// distinct interrupt lines, one for each instance.
    /// \param[in] slaveSelectPin the Arduino pin number of the output to use to select the RF24 before
    /// accessing it. Defaults to the normal SS pin for your Arduino (D10 for Diecimila, Uno etc, D53 for Mega, D10 for Maple)
    /// \param[in] interruptPin The interrupt Pin number that is connected to the RF24 DIO0 interrupt line. 
    /// Defaults to pin 2.
    /// Caution: You must specify an interrupt capable pin.
    /// On many Arduino boards, there are limitations as to which pins may be used as interrupts.
    /// On Leonardo pins 0, 1, 2 or 3. On Mega2560 pins 2, 3, 18, 19, 20, 21. On Due and Teensy, any digital pin.
    /// On other Arduinos pins 2 or 3. 
    /// See http://arduino.cc/en/Reference/attachInterrupt for more details.
    /// On Chipkit Uno32, pins 38, 2, 7, 8, 35.
    /// On other boards, any digital pin may be used.
    /// \param [in] sdnPin The pin number connected to SDN on the radio. Defaults to pin 9. 
    ///                     Connecting SDN directly to ground does not aloways provide reliable radio startup.
    /// \param[in] spi Pointer to the SPI interface object to use. 
    ///                Defaults to the standard Arduino hardware SPI interface
    RH_RF24(uint8_t slaveSelectPin = SS, uint8_t interruptPin = 2, uint8_t sdnPin = 9, RHGenericSPI& spi = hardware_spi);
  
    /// Initialises this instance and the radio module connected to it.
    /// The following steps are taken:
    /// - Initialise the slave select and shutdown pins and the SPI interface library
    /// - Checks the connected RF24 module can be communicated
    /// - Attaches an interrupt handler
    /// - Configures the RF24 module
    /// - Sets the frequency to 434.0 MHz
    /// - Sets the modem data rate to GFSK_Rb5Fd10
    /// - Sets the tranmitter power level to 16 (about 2.4dBm on RFM4)
    /// \return  true if everything was successful
    bool        init();
    
    /// Sets the chip mode that will be used when the RH_RF24 driver is idle (ie not transmitting or receiving)
    /// You can use this to control the power level consumed while idle, at the cost of slower
    /// transition to tranmit or receive states
    /// \param[in] idleMode The chip state to use when idle. Sensible choices might be RH_RF24_DEVICE_STATE_SLEEP or RH_RF24_DEVICE_STATE_READY
    void        setIdleMode(uint8_t idleMode);

    /// Sets the transmitter and receiver 
    /// centre frequency.
    /// Valid frequency ranges for RFM24/Si4460, Si4461, RFM25/Si4463 are:
    /// 142MHz to 175Mhz, 284MHz to 350MHz, 425MHz to 525MHz, 850MHz to 1050MHz.
    /// Valid frequency ranges for RFM26/Si4464 are:
    /// 119MHz to 960MHz.
    /// Caution: RFM modules are designed with antenna coupling components to suit a limited band
    /// of frequencies (marked underneath the module). It is possible to set frequencies in other bands,
    /// but you may only get little or no power radiated.
    /// Caution, you can only use this function to change frequency within the frequency band configured by
    /// the radio configuration header file. To use a frequency in a different band, you must recompile with
    /// the appropriate radio configuration header file. Setting a frequency in anotehr band will
    /// have unpredicatble results.
    /// \param[in] centre Frequency in MHz. 
    /// \param[in] afcPullInRange Not used
    /// \return true if the selected frequency is within a valid range for the connected radio and if
    ///         setting the new frequency succeeded.
    bool        setFrequency(float centre, float afcPullInRange = 0.05);

    /// OBSOLETE, do not use.
    /// To get different modulation schemes, you must generate a new radio config file
    /// as described in this documentation.
    /// Sets all the properties required to configure the data modem in the RF24, including the data rate, 
    /// bandwidths etc. You can use this to configure the modem with custom configurations if none of the 
    /// canned configurations in ModemConfigChoice suit you.
    /// \param[in] config A ModemConfig structure containing values for the modem configuration registers.
    void           setModemRegisters(const ModemConfig* config);

    /// OBSOLETE, do not use. 
    /// To get different modulation schemes, you must generate a new radio config file
    /// as described in this documentation.
    /// Select one of the predefined modem configurations. If you need a modem configuration not provided 
    /// here, use setModemRegisters() with your own ModemConfig. The default after init() is RH_RF24::GFSK_Rb5Fd10.
    /// \param[in] index The configuration choice.
    /// \return true if index is a valid choice.
    bool        setModemConfig(ModemConfigChoice index);

    /// Starts the receiver and checks whether a received message is available.
    /// This can be called multiple times in a timeout loop
    /// \return true if a complete, valid message has been received and is able to be retrieved by
    /// recv()
    bool        available();

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    bool        recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is NOT permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send (> 0)
    /// \return true if the message length was valid and it was correctly queued for transmit
    bool        send(const uint8_t* data, uint8_t len);

    /// The maximum message length supported by this driver
    /// \return The maximum message length supported by this driver
    uint8_t maxMessageLength();

    /// Sets the length of the preamble
    /// in bytes. 
    /// Caution: this should be set to the same 
    /// value on all nodes in your network. Default is 4.
    /// \param[in] bytes Preamble length in bytes.  
    void           setPreambleLength(uint16_t bytes);

    /// Sets the sync words for transmit and receive 
    /// Caution: SyncWords should be set to the same 
    /// value on all nodes in your network. Nodes with different SyncWords set will never receive
    /// each others messages, so different SyncWords can be used to isolate different
    /// networks from each other. Default is { 0x2d, 0xd4 }.
    /// \param[in] syncWords Array of sync words, 1 to 4 octets long. NULL if no sync words to be used.
    /// \param[in] len Number of sync words to set, 1 to 4. 0 if no sync words to be used.
    void           setSyncWords(const uint8_t* syncWords = NULL, uint8_t len = 0);

    /// Sets the CRC polynomial to be used to generate the CRC for both receive and transmit
    /// otherwise the default of CRC_16_IBM will be used.
    /// \param[in] polynomial One of RH_RF24::CRCPolynomial choices CRC_*
    /// \return true if polynomial is a valid option for this radio.
    bool setCRCPolynomial(CRCPolynomial polynomial);

    /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
    void           setModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the RF24.
    void           setModeRx();

    /// If current mode is Rx or Idle, changes it to Rx. F
    /// Starts the transmitter in the RF24.
    void           setModeTx();

    /// Sets the transmitter power output level register PA_PWR_LVL
    /// The power argument to this function has a non-linear correlation with the actual RF power output.
    /// See the transmitter power table above for some examples.
    /// Also the Si446x Data Sheet section 5.4.2 may be helpful.
    /// Be a good neighbour and set the lowest power level you need.
    /// Caution: legal power limits may apply in certain countries.
    /// After init(), the power will be set to 0x10.
    /// \param[in] power Transmitter power level. For RFM24/Si4460, valid values are 0x00 to 0x4f. For others, 0x00 to 0x7f
    void           setTxPower(uint8_t power);

    /// Dump the values of available command replies and properties
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// Not all commands have valid replies, therefore they are not all printed.
    /// Caution: the list is very long
    bool           printRegisters();

    /// Send a string of command bytes to the chip and get a string of reply bytes
    /// Different RFM24 commands take different numbers of command bytes and send back different numbers
    /// of reply bytes. See the Si446x documentaiton for more details.
    /// Both command bytes and reply bytes are optional
    /// \param[in] cmd The command number. One of RH_RF24_CMD_*
    /// \param[in] write_buf Pointer to write_len bytes of command input bytes to send. If there are none, set to NULL.
    /// \param[in] write_len The number of bytes to send from write_buf. If there are none, set to 0
    /// \param[out] read_buf Pointer to read_len bytes of storage where the reply stream from the comand will be written.
    ///            If none are required, set to NULL
    /// \param[in] read_len The number of bytes to read from the reply stream. If none required, set to 0.
    /// \return true if the command succeeeded.
    bool           command(uint8_t cmd, const uint8_t* write_buf = 0, uint8_t write_len = 0, uint8_t* read_buf = 0, uint8_t read_len = 0);

    /// Set one or more chip properties using the RH_RF24_CMD_SET_PROPERTY
    /// command. See the Si446x API Description AN625 for details on what properties are available.
    /// param[in] firstProperty The property number of the first property to set. The first value in the values array
    ///           will be used to set this property, and any subsequent values will be used to set the following properties.
    ///           One of RH_RF24_PROPERTY_*
    /// param[in] values Array of 0 or more values to write the firstProperty and subsequent proerties
    /// param[in] count The number of values in the values array
    /// \return true if the command succeeeded.
    bool           set_properties(uint16_t firstProperty, const uint8_t* values, uint8_t count);

    /// Get one or more chip properties using the RH_RF24_CMD_GET_PROPERTY
    /// command. See the Si446x API Description AN625 for details on what properties are available.
    /// param[in] firstProperty The property number of the first property to get. The first value in the values array
    ///           will be set with this property, and any subsequent values will be set from the following properties.
    ///           One of RH_RF24_PROPERTY_*
    /// param[out] values Array of 0 or more values to receive the firstProperty and subsequent proerties
    /// param[in] count The number of values in the values array
    /// \return true if the command succeeeded.
    bool           get_properties(uint16_t firstProperty, uint8_t* values, uint8_t count);

    /// Measures and returns the current
    /// Chip temperature.
    /// \return The current chip temperature in degrees Centigrade
    float          get_temperature();

    /// Measures and returns the current
    /// Chip Vcc supply voltage.
    /// \return The current chip Vcc supply voltage in Volts.
    float          get_battery_voltage();

    /// Measures and returns the current
    /// voltage applied to a GPIO pin (which has previously been configured as a voltage input)
    /// \param[in] gpio The GPIO pin to read. 0 to 3.
    /// \return The current pin voltage in Volts.
    float          get_gpio_voltage(uint8_t gpio);

    /// Read one of the Fast Read Response registers.
    /// The Fast Read Response register must be previously configured with the matching
    /// RH_RF24_PROPERTY_FRR_CTL_?_MODE property to select what chip property will be available in that register.
    /// \param[in] reg The index of the FRR register to read. 0 means FRR A, 1 means B etc.
    /// \return the value read from the specified Fast Read Response register.
    uint8_t        frr_read(uint8_t reg);

    /// Sets the radio into low-power sleep mode.
    /// If successful, the transport will stay in sleep mode until woken by 
    /// changing mode it idle, transmit or receive (eg by calling send(), recv(), available() etc)
    /// Caution: there is a time penalty as the radio takes a finte time to wake from sleep mode.
    /// \return true if sleep mode was successfully entered.
    virtual bool    sleep();

    /// Return the integer value of the device type
    /// as read from the device in from RH_RF24_CMD_PART_INFO.
    /// One of 0x4460, 0x4461, 0x4462 or 0x4463, depending on the type of device actually
    /// connected.
    /// \return The integer device type
    uint16_t deviceType() {return _deviceType;};

protected:
    /// This is a low level function to handle the interrupts for one instance of RF24.
    /// Called automatically by isr*()
    /// Should not need to be called by user code.
    void           handleInterrupt();

    /// Clears the chips RX FIFO
    /// \return true if successful
    bool           clearRxFifo();

    /// Clears RH_RF24's internal TX and RX buffers and counters
    void           clearBuffer();

    /// Loads the next part of the currently transmitting message 
    /// into the chips TX buffer
    void           sendNextFragment();

    /// Copies the next part of the currenrtly received message from the chips RX FIFO to the 
    /// receive buffer
    void           readNextFragment();

    /// Loads data into the chips TX FIFO
    /// \param[in] data Array of data bytes to be loaded
    /// \param[in] len Number of bytes in data to be loaded
    /// \return true if successful
    bool           writeTxFifo(uint8_t *data, uint8_t len);

    /// Checks the contents of the RX buffer.
    /// If it contans a valid message adressed to this node
    /// sets _rxBufValid.
    void           validateRxBuf();

    /// Cycles the Shutdown pin to force the cradio chip to reset
    void           power_on_reset();

    /// Sets registers, commands and properties
    /// in the ratio according to the data in the commands array
    /// \param[in] commands Array of data containing radio commands in the format provided by radio_config_Si4460.h
    /// \return true if successful
    bool           configure(const uint8_t* commands);

    /// Clears all pending interrutps in the radio chip.
    bool           cmd_clear_all_interrupts();

private:

    /// Low level interrupt service routine for RF24 connected to interrupt 0
    static void         isr0();

    /// Low level interrupt service routine for RF24 connected to interrupt 1
    static void         isr1();

    /// Low level interrupt service routine for RF24 connected to interrupt 1
    static void         isr2();

    /// Array of instances connected to interrupts 0 and 1
    static RH_RF24*     _deviceForInterrupt[];

    /// Index of next interrupt number to use in _deviceForInterrupt
    static uint8_t      _interruptCount;

    /// The configured interrupt pin connected to this instance
    uint8_t             _interruptPin;

    /// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
    /// else 0xff
    uint8_t             _myInterruptIndex;

    /// The configured pin connected to the SDN pin of the radio
    uint8_t             _sdnPin;

    /// The radio OP mode to use when mode is RHModeIdle
    uint8_t             _idleMode; 

    /// The reported PART device type
    uint16_t             _deviceType;

    /// The selected output power in dBm
    int8_t              _power;

    /// The message length in _buf
    volatile uint8_t    _bufLen;

    /// Array of octets of the last received message or the next to transmit message
    uint8_t             _buf[RH_RF24_MAX_PAYLOAD_LEN];

    /// True when there is a valid message in the Rx buffer
    volatile bool       _rxBufValid;

    /// Index into TX buffer of the next to send chunk
    volatile uint8_t    _txBufSentIndex;
  
    /// Time in millis since the last preamble was received (and the last time the RSSI was measured)
    uint32_t            _lastPreambleTime;

};

/// @example rf24_client.ino
/// @example rf24_server.ino
/// @example rf24_lowpower_client.ino
/// @example rf24_reliable_datagram_client.ino
/// @example rf24_reliable_datagram_server.ino

#endif
