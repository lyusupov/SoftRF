// SX126X.h
//
// Definitions for the Semtech SX126X series of LoRa capable radios
// https://wiki.seeedstudio.com/LoRa_E5_mini/
// https://www.rfsolutions.co.uk/downloads/1537522406DS_SX1261-2_V1.1_SEMTECH.pdf
// https://cdn.sparkfun.com/assets/6/b/5/1/4/SX1262_datasheet.pdf
// https://files.seeedstudio.com/products/317990687/res/LoRa-E5+module+datasheet_V1.0.pdf
// https://forum.seeedstudio.com/t/lora-e5-register-settings-for-oscillators/262635
// file:///home/mikem/Downloads/es0506-stm32wle5xx-stm32wle4xx-device-errata-stmicroelectronics.pdf
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2023 Mike McCauley
//

#ifndef RH_SX126x_h
#define RH_SX126x_h

#include <RHSPIDriver.h>

// This is the maximum number of interrupts the driver can support
// Most Arduinos can handle 2, Megas can handle more
#define RH_SX126x_NUM_INTERRUPTS 3

// Max number of octets the LORA Rx/Tx FIFO can hold
#define RH_SX126x_FIFO_SIZE 255

// This is the maximum number of bytes that can be carried by the LORA.
// We use some for headers, keeping fewer for RadioHead messages
#define RH_SX126x_MAX_PAYLOAD_LEN RH_SX126x_FIFO_SIZE

// The length of the headers we add.
// The headers are inside the LORA's payload
#define RH_SX126x_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
#ifndef RH_SX126x_MAX_MESSAGE_LEN
 #define RH_SX126x_MAX_MESSAGE_LEN (RH_SX126x_MAX_PAYLOAD_LEN - RH_SX126x_HEADER_LEN)
#endif

// Radio chip internal crystal frequency
#define RH_SX126x_XTAL_FREQ  32000000.0

// The Frequency Synthesizer step = RH_SX126x_XTAL_FREQ / 2^^25
#define RH_SX126x_FSTEP  (RH_SX126x_XTAL_FREQ / 33554432)

// Operational Modes Functions
#define RH_SX126x_CMD_NOP                         0x00
#define RH_SX126x_CMD_SET_SLEEP                   0x84
#define RH_SX126x_CMD_SET_STANDBY                 0x80
#define RH_SX126x_CMD_SET_FS                      0xC1
#define RH_SX126x_CMD_SET_TX                      0x83
#define RH_SX126x_CMD_SET_RX                      0x82
#define RH_SX126x_CMD_SET_STOP_TIMER_ON_PREAMBLE  0x9F
#define RH_SX126x_CMD_SET_RX_DUTY_CYCLE           0x94
#define RH_SX126x_CMD_SET_CAD                     0xC5
#define RH_SX126x_CMD_SET_TX_CONTINUOUS_WAVE      0xD1
#define RH_SX126x_CMD_SET_TX_INFINITE_PREAMBLE    0xD2
#define RH_SX126x_CMD_SET_REGULATOR_MODE          0x96
#define RH_SX126x_CMD_CALIBRATE                   0x89
#define RH_SX126x_CMD_CALIBRATE_IMAGE             0x98
#define RH_SX126x_CMD_SET_PA_CFG                  0x95
#define RH_SX126x_CMD_SET_RX_TX_FALLBACK_MODE     0x93

// Registers and buffer Access
#define RH_SX126x_CMD_WRITE_REGISTER              0x0D
#define RH_SX126x_CMD_READ_REGISTER               0x1D
#define RH_SX126x_CMD_WRITE_BUFFER                0x0E
#define RH_SX126x_CMD_READ_BUFFER                 0x1E

// DIO and IRQ Control Functions
#define RH_SX126x_CMD_SET_DIO_IRQ_PARAMS          0x08
#define RH_SX126x_CMD_GET_IRQ_STATUS              0x12
#define RH_SX126x_CMD_CLR_IRQ_STATUS              0x02
#define RH_SX126x_CMD_SET_DIO2_AS_RF_SWITCH_CTRL  0x9D
#define RH_SX126x_CMD_SET_DIO3_AS_TCXO_CTRL       0x97

// RF Modulation and Packet-Related Functions
#define RH_SX126x_CMD_SET_RF_FREQUENCY           0x86
#define RH_SX126x_CMD_SET_PKT_TYPE               0x8A
#define RH_SX126x_CMD_GET_PKT_TYPE               0x11
#define RH_SX126x_CMD_SET_TX_PARAMS              0x8E
#define RH_SX126x_CMD_SET_MODULATION_PARAMS      0x8B
#define RH_SX126x_CMD_SET_PKT_PARAMS             0x8C
#define RH_SX126x_CMD_SET_CAD_PARAMS             0x88
#define RH_SX126x_CMD_SET_BUFFER_BASE_ADDRESS    0x8F
#define RH_SX126x_CMD_SET_LORA_SYMB_NUM_TIMEOUT  0xA0

// Communication Status Information
#define RH_SX126x_CMD_GET_STATUS                 0xC0
#define RH_SX126x_CMD_GET_RX_BUFFER_STATUS       0x13
#define RH_SX126x_CMD_GET_PKT_STATUS             0x14
#define RH_SX126x_CMD_GET_RSSI_INST              0x15
#define RH_SX126x_CMD_GET_STATS                  0x10
#define RH_SX126x_CMD_RESET_STATS                0x00

// Miscellaneous
#define RH_SX126x_CMD_GET_DEVICE_ERRORS          0x17
#define RH_SX126x_CMD_CLR_DEVICE_ERRORS          0x07

// Registers

// Base address of the register retention list, 3 bytes
#define RH_SX126x_REG_RETENTION_LIST_BASE_ADDRESS 0x029F

#define RH_SX126x_REG_VERSION_STRING                      0x0320
#define RH_SX126x_REG_HOPPING_ENABLE                      0x0385
#define RH_SX126x_REG_LR_FHSS_PACKET_LENGTH               0x0386
#define RH_SX126x_REG_LR_FHSS_NUM_HOPPING_BLOCKS          0x0387
#define RH_SX126x_REG_LR_FHSS_NUM_SYMBOLS_FREQX_MSB(X)    (0x0388 + (X)*6)
#define RH_SX126x_REG_LR_FHSS_NUM_SYMBOLS_FREQX_LSB(X)    (0x0389 + (X)*6)
#define RH_SX126x_REG_LR_FHSS_FREQX_0(X)                  (0x038A + (X)*6)
#define RH_SX126x_REG_LR_FHSS_FREQX_1(X)                  (0x038B + (X)*6)
#define RH_SX126x_REG_LR_FHSS_FREQX_2(X)                  (0x038C + (X)*6)
#define RH_SX126x_REG_LR_FHSS_FREQX_3(X)                  (0x038D + (X)*6)
#define RH_SX126x_REG_SPECTRAL_SCAN_RESULT                0x0401

// Output disable
#define RH_SX126x_REG_OUT_DIS_REG                0x0580
#define RH_SX126x_REG_OUT_DIS_REG_DIO3_POS       ( 3U )
#define RH_SX126x_REG_OUT_DIS_REG_DIO3_MASK      ( 0x01UL << SX126X_REG_OUT_DIS_REG_DIO3_POS )

#define RH_SX126x_REG_DIOX_DRIVE_STRENGTH                 0x0582

// Input enable
#define RH_SX126x_REG_IN_EN_REG                  0x0583
#define RH_SX126x_REG_IN_EN_REG_DIO3_POS         ( 3U )
#define RH_SX126x_REG_IN_EN_REG_DIO3_MASK        ( 0x01UL << SX126X_REG_IN_EN_REG_DIO3_POS )

#define RH_SX126x_REG_DIOX_PULLUP                0x0584
#define RH_SX126x_REG_DIOX_PULLDOWN              0x0585

// TX bitbang B
#define RH_SX126x_REG_BITBANG_B_REG              0x0587
#define RH_SX126x_REG_BITBANG_B_REG_ENABLE_POS   ( 0U )
#define RH_SX126x_REG_BITBANG_B_REG_ENABLE_MASK  ( 0x0FUL << SX126X_REG_BITBANG_B_REG_ENABLE_POS )
#define RH_SX126x_REG_BITBANG_B_REG_ENABLE_VAL   ( 0x0CUL << SX126X_REG_BITBANG_B_REG_ENABLE_POS )

#define RH_SX126x_REG_PATCH_UPDATE_ENABLE                 0x0610

// TX bitbang A
#define RH_SX126x_REG_BITBANG_A_REG              0x0680
#define RH_SX126x_REG_BITBANG_A_REG_ENABLE_POS   ( 4U )
#define RH_SX126x_REG_BITBANG_A_REG_ENABLE_MASK  ( 0x07UL << SX126X_REG_BITBANG_A_REG_ENABLE_POS )
#define RH_SX126x_REG_BITBANG_A_REG_ENABLE_VAL   ( 0x01UL << SX126X_REG_BITBANG_A_REG_ENABLE_POS )

// The address of the register holding the first byte defining the whitening seed. 2 bytes
#define RH_SX126x_REG_WHITSEEDBASEADDRESS        0x06B8

// RX/TX payload length
#define RH_SX126x_REG_RXTX_PAYLOAD_LEN           0x06BB

// The address of the register holding the first byte defining the CRC seed. 2 bytes
#define RH_SX126x_REG_CRCSEEDBASEADDRESS         0x06BC

// The address of the register holding the first byte defining the CRC polynomial. 2 bytes
#define RH_SX126x_REG_CRCPOLYBASEADDRESS         0x06BE

// The addresses of the registers holding SyncWords values, 8 bytes
#define RH_SX126x_REG_SYNCWORDBASEADDRESS        0x06C0

// GFSK node address
// Reset value is 0x00
#define RH_SX126x_REG_GFSK_NODE_ADDRESS          0x06CD

// GFSK broadcast address
// Reset value is 0x00
#define RH_SX126x_REG_GFSK_BROADCAST_ADDRESS     0x06CE

#define RH_SX126x_REG_PAYLOAD_LENGTH                      0x0702
#define RH_SX126x_REG_PACKET_PARAMS                       0x0704

// Number of symbols given as SX126X_REG_LR_SYNCH_TIMEOUT[7:3] * 2 ^ (2*SX126X_REG_LR_SYNCH_TIMEOUT[2:0] + 1)
#define RH_SX126x_REG_LR_SYNCH_TIMEOUT           0x0706

// WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
#define RH_SX126x_REG_IQ_POLARITY                0x0736

// The addresses of the register holding LoRa Modem SyncWord value, 2 bytes
// 0x1424: LoRaWAN private network,
// 0x3444: LoRaWAN public network
#define RH_SX126x_REG_LR_SYNCWORD                0x0740

// The address of the register holding the coding rate configuration extracted from a received LoRa header
#define RH_SX126x_REG_LR_HEADER_CR               0x0749
#define RH_SX126x_REG_LR_HEADER_CR_POS           ( 4U )
#define RH_SX126x_REG_LR_HEADER_CR_MASK          ( 0x07UL << SX126X_REG_LR_HEADER_CR_POS )

// The address of the register holding the CRC configuration extracted from a received LoRa header
#define RH_SX126x_REG_FREQ_ERROR                 0x076B
#define RH_SX126x_REG_LR_HEADER_CRC              0x076B
#define RH_SX126x_REG_LR_HEADER_CRC_POS          ( 4U )
#define RH_SX126x_REG_LR_HEADER_CRC_MASK         ( 0x01UL << SX126X_REG_LR_HEADER_CRC_POS )

#define RH_SX126x_REG_SPECTRAL_SCAN_STATUS                0x07CD

// RX address pointer
#define RH_SX126x_REG_RX_ADDRESS_POINTER         0x0803

// The address of the register giving a 32-bit random number, 4 bytes
#define RH_SX126x_REG_RNGBASEADDRESS             0x0819

// WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
#define RH_SX126x_REG_TX_MODULATION              0x0889

#define RH_SX126x_REG_RF_FREQUENCY_0                      0x088B
#define RH_SX126x_REG_RF_FREQUENCY_1                      0x088C
#define RH_SX126x_REG_RF_FREQUENCY_2                      0x088D
#define RH_SX126x_REG_RF_FREQUENCY_3                      0x088E

#define RH_SX126x_REG_RSSI_AVG_WINDOW                     0x089B

// The address of the register holding RX Gain value
//     0x94: power saving,
//     0x96: rx boosted
#define RH_SX126x_REG_RXGAIN                     0x08AC

// WORKAROUND - Better resistance to antenna mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
#define RH_SX126x_REG_TX_CLAMP_CFG               0x08D8
#define RH_SX126x_REG_TX_CLAMP_CFG_POS           ( 1U )
#define RH_SX126x_REG_TX_CLAMP_CFG_MASK          ( 0x0FUL << SX126X_REG_TX_CLAMP_CFG_POS )

// The address of the register used to disable the LNA
#define RH_SX126x_REG_ANA_LNA                    0x08E2

#define RH_SX126x_REG_LNA_CAP_TUNE_N                      0x08E3
#define RH_SX126x_REG_LNA_CAP_TUNE_P                      0x08E4

// The address of the register used to disable the mixer
#define RH_SX126x_REG_ANA_MIXER                  0x08E5

// Set the current max value in the over current protection
#define RH_SX126x_REG_OCP                        0x08E7

// RTC control
#define RH_SX126x_REG_RTC_CTRL                   0x0902

// Change the value on the device internal trimming capacitor, 2 bytes
#define RH_SX126x_REG_XTATRIM                    0x0911

// Value of the trimming cap on XTB pin This register should only be
// changed while the radio is in STDBY_XOSC mode
#define RH_SX126x_REG_DIO3_OUTPUT_VOLTAGE        0x0920

// Event clear
#define RH_SX126x_REG_EVT_CLR                    0x0944
#define RH_SX126x_REG_EVT_CLR_TIMEOUT_POS        ( 1U )
#define RH_SX126x_REG_EVT_CLR_TIMEOUT_MASK       ( 0x01UL << SX126X_REG_EVT_CLR_TIMEOUT_POS )

#define RH_SX126x_REG_PATCH_MEMORY_BASE                   0x8000

// Values used in commands and registers
// RH_SX126x_CMD_SET_SLEEP
#define RH_SX126x_SLEEP_START_COLD                        0b00000000  //  sleep mode: cold start, configuration is lost (default)
#define RH_SX126x_SLEEP_START_WARM                        0b00000100  //               warm start, configuration is retained
#define RH_SX126x_SLEEP_RTC_OFF                           0b00000000  //   wake on RTC timeout: disabled
#define RH_SX126x_SLEEP_RTC_ON                            0b00000001  //                         enabled

// RH_SX126x_CMD_SET_STANDBY
#define RH_SX126x_STANDBY_RC                              0x00        //  standby mode: 13 MHz RC oscillator
#define RH_SX126x_STANDBY_XOSC                            0x01        //                32 MHz crystal oscillator

// RH_SX126x_CMD_SET_RX
#define RH_SX126x_RX_TIMEOUT_NONE                         0x000000    //  Rx timeout duration: no timeout (Rx single mode)
#define RH_SX126x_RX_TIMEOUT_INF                          0xFFFFFF    //                       infinite (Rx continuous mode)

// RH_SX126x_CMD_SET_TX
#define RH_SX126x_TX_TIMEOUT_NONE                         0x000000    //  Tx timeout duration: no timeout (Tx single mode)

// RH_SX126x_CMD_STOP_TIMER_ON_PREAMBLE
#define RH_SX126x_STOP_ON_PREAMBLE_OFF                    0x00        //  stop timer on: sync word or header (default)
#define RH_SX126x_STOP_ON_PREAMBLE_ON                     0x01        //                  preamble detection

// RH_SX126x_CMD_SET_REGULATOR_MODE
#define RH_SX126x_REGULATOR_LDO                           0x00        //  set regulator mode: LDO (default)
#define RH_SX126x_REGULATOR_DC_DC                         0x01        //                      DC-DC

// RH_SX126x_CMD_CALIBRATE
#define RH_SX126x_CALIBRATE_IMAGE_OFF                     0b00000000  //  image calibration: disabled
#define RH_SX126x_CALIBRATE_IMAGE_ON                      0b01000000  //                     enabled
#define RH_SX126x_CALIBRATE_ADC_BULK_P_OFF                0b00000000  //  ADC bulk P calibration: disabled
#define RH_SX126x_CALIBRATE_ADC_BULK_P_ON                 0b00100000  //                          enabled
#define RH_SX126x_CALIBRATE_ADC_BULK_N_OFF                0b00000000  //  ADC bulk N calibration: disabled
#define RH_SX126x_CALIBRATE_ADC_BULK_N_ON                 0b00010000  //                          enabled
#define RH_SX126x_CALIBRATE_ADC_PULSE_OFF                 0b00000000  //  ADC pulse calibration: disabled
#define RH_SX126x_CALIBRATE_ADC_PULSE_ON                  0b00001000  //                         enabled
#define RH_SX126x_CALIBRATE_PLL_OFF                       0b00000000  //  PLL calibration: disabled
#define RH_SX126x_CALIBRATE_PLL_ON                        0b00000100  //                   enabled
#define RH_SX126x_CALIBRATE_RC13M_OFF                     0b00000000  //  13 MHz RC osc. calibration: disabled
#define RH_SX126x_CALIBRATE_RC13M_ON                      0b00000010  //                              enabled
#define RH_SX126x_CALIBRATE_RC64K_OFF                     0b00000000  //  64 kHz RC osc. calibration: disabled
#define RH_SX126x_CALIBRATE_RC64K_ON                      0b00000001  //                              enabled
#define RH_SX126x_CALIBRATE_ALL                           0b01111111  //  calibrate all blocks

// RH_SX126x_CMD_CALIBRATE_IMAGE
#define RH_SX126x_CAL_IMG_430_MHZ_1                       0x6B
#define RH_SX126x_CAL_IMG_430_MHZ_2                       0x6F
#define RH_SX126x_CAL_IMG_470_MHZ_1                       0x75
#define RH_SX126x_CAL_IMG_470_MHZ_2                       0x81
#define RH_SX126x_CAL_IMG_779_MHZ_1                       0xC1
#define RH_SX126x_CAL_IMG_779_MHZ_2                       0xC5
#define RH_SX126x_CAL_IMG_863_MHZ_1                       0xD7
#define RH_SX126x_CAL_IMG_863_MHZ_2                       0xDB
#define RH_SX126x_CAL_IMG_902_MHZ_1                       0xE1
#define RH_SX126x_CAL_IMG_902_MHZ_2                       0xE9

// RH_SX126x_CMD_SET_PA_CONFIG
#define RH_SX126x_PA_CONFIG_HP_MAX                        0x07

#define RH_SX126x_PA_CONFIG_DEVICE_SEL_SX1261             0x01
#define RH_SX126x_PA_CONFIG_DEVICE_SEL_SX1262             0x00

#define RH_SX126x_PA_CONFIG_PA_LUT                        0x01
#define RH_SX126x_PA_CONFIG_SX1262_8                      0x00

// RH_SX126x_CMD_SET_RX_TX_FALLBACK_MODE
#define RH_SX126x_RX_TX_FALLBACK_MODE_FS                  0x40        //  after Rx/Tx go to: FS mode
#define RH_SX126x_RX_TX_FALLBACK_MODE_STDBY_XOSC          0x30        //                     standby with crystal oscillator
#define RH_SX126x_RX_TX_FALLBACK_MODE_STDBY_RC            0x20        //                     standby with RC oscillator (default)

// RH_SX126x_CMD_SET_DIO_IRQ_PARAMS
#define RH_SX126x_IRQ_LR_FHSS_HOP                         0b0100000000000000  //  PA ramped up during LR-FHSS hop
#define RH_SX126x_IRQ_TIMEOUT                             0b0000001000000000  //  Rx or Tx timeout
#define RH_SX126x_IRQ_CAD_DETECTED                        0b0000000100000000  //  channel activity detected
#define RH_SX126x_IRQ_CAD_DONE                            0b0000000010000000  //  channel activity detection finished
#define RH_SX126x_IRQ_CRC_ERR                             0b0000000001000000  //  wrong CRC received
#define RH_SX126x_IRQ_HEADER_ERR                          0b0000000000100000  //  LoRa header CRC error
#define RH_SX126x_IRQ_HEADER_VALID                        0b0000000000010000  //  valid LoRa header received
#define RH_SX126x_IRQ_SYNC_WORD_VALID                     0b0000000000001000  //  valid sync word detected
#define RH_SX126x_IRQ_PREAMBLE_DETECTED                   0b0000000000000100  //  preamble detected
#define RH_SX126x_IRQ_RX_DONE                             0b0000000000000010  //  packet received
#define RH_SX126x_IRQ_TX_DONE                             0b0000000000000001  //  packet transmission completed
#define RH_SX126x_IRQ_RX_DEFAULT                          0b0000001001100010  //  default for Rx (RX_DONE, TIMEOUT, CRC_ERR and HEADER_ERR)
#define RH_SX126x_IRQ_ALL                                 0b0100001111111111  //  all interrupts
#define RH_SX126x_IRQ_NONE                                0b0000000000000000  //  no interrupts

// RH_SX126x_CMD_SET_DIO2_AS_RF_SWITCH_CTRL
#define RH_SX126x_DIO2_AS_IRQ                             0x00        //  DIO2 configuration: IRQ
#define RH_SX126x_DIO2_AS_RF_SWITCH                       0x01        //                      RF switch control

// RH_SX126x_CMD_SET_DIO3_AS_TCXO_CTRL
#define RH_SX126x_DIO3_OUTPUT_1_6                         0x00        //  DIO3 voltage output for TCXO: 1.6 V
#define RH_SX126x_DIO3_OUTPUT_1_7                         0x01        //                                1.7 V
#define RH_SX126x_DIO3_OUTPUT_1_8                         0x02        //                                1.8 V
#define RH_SX126x_DIO3_OUTPUT_2_2                         0x03        //                                2.2 V
#define RH_SX126x_DIO3_OUTPUT_2_4                         0x04        //                                2.4 V
#define RH_SX126x_DIO3_OUTPUT_2_7                         0x05        //                                2.7 V
#define RH_SX126x_DIO3_OUTPUT_3_0                         0x06        //                                3.0 V
#define RH_SX126x_DIO3_OUTPUT_3_3                         0x07        //                                3.3 V

// RH_SX126x_CMD_SET_PACKET_TYPE
#define RH_SX126x_PACKET_TYPE_GFSK                        0x00        //  packet type: GFSK
#define RH_SX126x_PACKET_TYPE_LORA                        0x01        //               LoRa
#define RH_SX126x_PACKET_TYPE_LR_FHSS                     0x03        //               LR-FHSS

// RH_SX126x_CMD_SET_TX_PARAMS
#define RH_SX126x_PA_RAMP_10U                             0x00        //  ramp time: 10 us
#define RH_SX126x_PA_RAMP_20U                             0x01        //             20 us
#define RH_SX126x_PA_RAMP_40U                             0x02        //             40 us
#define RH_SX126x_PA_RAMP_80U                             0x03        //             80 us
#define RH_SX126x_PA_RAMP_200U                            0x04        //             200 us
#define RH_SX126x_PA_RAMP_800U                            0x05        //             800 us
#define RH_SX126x_PA_RAMP_1700U                           0x06        //             1700 us
#define RH_SX126x_PA_RAMP_3400U                           0x07        //             3400 us

// RH_SX126x_CMD_SET_MODULATION_PARAMS
// GFSK bandwidths
#define RH_SX126x_GFSK_RX_BW_4_8                          0x1F        //  4.8 kHz 
#define RH_SX126x_GFSK_RX_BW_5_8                          0x17        //  5.8 kHz
#define RH_SX126x_GFSK_RX_BW_7_3                          0x0F        //  7.3 kHz
#define RH_SX126x_GFSK_RX_BW_9_7                          0x1E        //  9.7 kHz
#define RH_SX126x_GFSK_RX_BW_11_7                         0x16        //  11.7 kHz
#define RH_SX126x_GFSK_RX_BW_14_6                         0x0E        //  14.6 kHz
#define RH_SX126x_GFSK_RX_BW_19_5                         0x1D        //  19.5 kHz
#define RH_SX126x_GFSK_RX_BW_23_4                         0x15        //  23.4 kHz
#define RH_SX126x_GFSK_RX_BW_29_3                         0x0D        //  29.3 kHz
#define RH_SX126x_GFSK_RX_BW_39_0                         0x1C        //  39.0 kHz
#define RH_SX126x_GFSK_RX_BW_46_9                         0x14        //  46.9 kHz
#define RH_SX126x_GFSK_RX_BW_58_6                         0x0C        //  58.6 kHz
#define RH_SX126x_GFSK_RX_BW_78_2                         0x1B        //  78.2 kHz
#define RH_SX126x_GFSK_RX_BW_93_8                         0x13        //  93.8 kHz
#define RH_SX126x_GFSK_RX_BW_117_3                        0x0B        //  117.3 kHz
#define RH_SX126x_GFSK_RX_BW_156_2                        0x1A        //  156.2 kHz
#define RH_SX126x_GFSK_RX_BW_187_2                        0x12        //  187.2 kHz
#define RH_SX126x_GFSK_RX_BW_234_3                        0x0A        //  234.3 kHz
#define RH_SX126x_GFSK_RX_BW_312_0                        0x19        //  312.0 kHz
#define RH_SX126x_GFSK_RX_BW_373_6                        0x11        //  373.6 kHz
#define RH_SX126x_GFSK_RX_BW_467_0                        0x09        //  467.0 kHz
// LORA bandwidths
#define RH_SX126x_LORA_BW_7_8                             0x00        //  7.8 kHz
#define RH_SX126x_LORA_BW_10_4                            0x08        //  10.4 kHz
#define RH_SX126x_LORA_BW_15_6                            0x01        //  15.6 kHz
#define RH_SX126x_LORA_BW_20_8                            0x09        //  20.8 kHz
#define RH_SX126x_LORA_BW_31_25                           0x02        //  31.25 kHz
#define RH_SX126x_LORA_BW_41_7                            0x0A        //  41.7 kHz
#define RH_SX126x_LORA_BW_62_5                            0x03        //  62.5 kHz
#define RH_SX126x_LORA_BW_125_0                           0x04        //  125.0 kHz
#define RH_SX126x_LORA_BW_250_0                           0x05        //  250.0 kHz
#define RH_SX126x_LORA_BW_500_0                           0x06        //  500.0 kHz
// LORA Coding rates
#define RH_SX126x_LORA_CR_4_5                             0x01        //  4/5
#define RH_SX126x_LORA_CR_4_6                             0x02        //  4/6
#define RH_SX126x_LORA_CR_4_7                             0x03        //  4/7
#define RH_SX126x_LORA_CR_4_8                             0x04        //  4/8
// LORA Spreading Factors, actually powers of 2
#define RH_SX126x_LORA_SF_32                              5           // SF5
#define RH_SX126x_LORA_SF_64                              6           // SF6
#define RH_SX126x_LORA_SF_128                             7           // SF7
#define RH_SX126x_LORA_SF_256                             8           // SF8
#define RH_SX126x_LORA_SF_512                             9           // SF9
#define RH_SX126x_LORA_SF_1024                            10          // SF10
#define RH_SX126x_LORA_SF_2048                            11          // SF11
#define RH_SX126x_LORA_SF_4096                            12          // SF12

#define RH_SX126x_LORA_LOW_DATA_RATE_OPTIMIZE_OFF         0x00        //  LoRa low data rate optimization: disabled
#define RH_SX126x_LORA_LOW_DATA_RATE_OPTIMIZE_ON          0x01        //                                   enabled

// RH_SX126x_CMD_SET_PACKET_PARAMS
#define RH_SX126x_GFSK_PREAMBLE_DETECT_OFF                0x00        //  GFSK minimum preamble length before reception starts: detector disabled
#define RH_SX126x_GFSK_PREAMBLE_DETECT_8                  0x04        //                                                        8 bits
#define RH_SX126x_GFSK_PREAMBLE_DETECT_16                 0x05        //                                                        16 bits
#define RH_SX126x_GFSK_PREAMBLE_DETECT_24                 0x06        //                                                        24 bits
#define RH_SX126x_GFSK_PREAMBLE_DETECT_32                 0x07        //                                                        32 bits
#define RH_SX126x_GFSK_ADDRESS_FILT_OFF                   0x00        //  GFSK address filtering: disabled
#define RH_SX126x_GFSK_ADDRESS_FILT_NODE                  0x01        //                          node only
#define RH_SX126x_GFSK_ADDRESS_FILT_NODE_BROADCAST        0x02        //                          node and broadcast
#define RH_SX126x_GFSK_PACKET_FIXED                       0x00        //  GFSK packet type: fixed (payload length known in advance to both sides)
#define RH_SX126x_GFSK_PACKET_VARIABLE                    0x01        //                    variable (payload length added to packet)
#define RH_SX126x_GFSK_CRC_OFF                            0x01        //  GFSK packet CRC: disabled
#define RH_SX126x_GFSK_CRC_1_BYTE                         0x00        //                   1 byte
#define RH_SX126x_GFSK_CRC_2_BYTE                         0x02        //                   2 byte
#define RH_SX126x_GFSK_CRC_1_BYTE_INV                     0x04        //                   1 byte, inverted
#define RH_SX126x_GFSK_CRC_2_BYTE_INV                     0x06        //                   2 byte, inverted
#define RH_SX126x_GFSK_WHITENING_OFF                      0x00        //  GFSK data whitening: disabled
#define RH_SX126x_GFSK_WHITENING_ON                       0x01        //                       enabled
#define RH_SX126x_LORA_PACKET_VARIABLE                    0x00
#define RH_SX126x_LORA_PACKET_FIXED                       0x01
#define RH_SX126x_LORA_HEADER_EXPLICIT                    0x00        //  LoRa header mode: explicit
#define RH_SX126x_LORA_HEADER_IMPLICIT                    0x01        //                    implicit
#define RH_SX126x_LORA_CRC_OFF                            0x00        //  LoRa CRC mode: disabled
#define RH_SX126x_LORA_CRC_ON                             0x01        //                 enabled
#define RH_SX126x_LORA_IQ_STANDARD                        0x00        //  LoRa IQ setup: standard
#define RH_SX126x_LORA_IQ_INVERTED                        0x01        //                 inverted

// RH_SX126x_CMD_SET_CAD_PARAMS
#define RH_SX126x_CAD_ON_1_SYMB                           0x00        //  number of symbols used for CAD: 1
#define RH_SX126x_CAD_ON_2_SYMB                           0x01        //                                  2
#define RH_SX126x_CAD_ON_4_SYMB                           0x02        //                                  4
#define RH_SX126x_CAD_ON_8_SYMB                           0x03        //                                  8
#define RH_SX126x_CAD_ON_16_SYMB                          0x04        //                                  16
#define RH_SX126x_CAD_GOTO_STDBY                          0x00        //  after CAD is done, always go to STDBY_RC mode
#define RH_SX126x_CAD_GOTO_RX                             0x01        //  after CAD is done, go to Rx mode if activity is detected
#define RH_SX126x_CAD_PARAM_DEFAULT                       0xFF        //  used by the CAD methods to specify default parameter value
#define RH_SX126x_CAD_PARAM_DET_MIN                       10          //  default detMin CAD parameter

// RH_SX126x_CMD_GET_STATUS
#define RH_SX126x_STATUS_MODE_STDBY_RC                    0b00100000  //  current chip mode: STDBY_RC
#define RH_SX126x_STATUS_MODE_STDBY_XOSC                  0b00110000  //                     STDBY_XOSC
#define RH_SX126x_STATUS_MODE_FS                          0b01000000  //                     FS
#define RH_SX126x_STATUS_MODE_RX                          0b01010000  //                     RX
#define RH_SX126x_STATUS_MODE_TX                          0b01100000  //                     TX
#define RH_SX126x_STATUS_DATA_AVAILABLE                   0b00000100  //  command status: packet received and data can be retrieved
#define RH_SX126x_STATUS_CMD_TIMEOUT                      0b00000110  //                  SPI command timed out
#define RH_SX126x_STATUS_CMD_INVALID                      0b00001000  //                  invalid SPI command
#define RH_SX126x_STATUS_CMD_FAILED                       0b00001010  //                  SPI command failed to execute
#define RH_SX126x_STATUS_TX_DONE                          0b00001100  //                  packet transmission done
#define RH_SX126x_STATUS_SPI_FAILED                       0b11111111  //  SPI transaction failed

// RH_SX126x_CMD_GET_PACKET_STATUS
#define RH_SX126x_GFSK_RX_STATUS_PREAMBLE_ERR             0b10000000  //  GFSK Rx status: preamble error
#define RH_SX126x_GFSK_RX_STATUS_SYNC_ERR                 0b01000000  //                  sync word error
#define RH_SX126x_GFSK_RX_STATUS_ADRS_ERR                 0b00100000  //                  address error
#define RH_SX126x_GFSK_RX_STATUS_CRC_ERR                  0b00010000  //                  CRC error
#define RH_SX126x_GFSK_RX_STATUS_LENGTH_ERR               0b00001000  //                  length error
#define RH_SX126x_GFSK_RX_STATUS_ABORT_ERR                0b00000100  //                  abort error
#define RH_SX126x_GFSK_RX_STATUS_PACKET_RECEIVED          0b00000010  //                  packet received
#define RH_SX126x_GFSK_RX_STATUS_PACKET_SENT              0b00000001  //                  packet sent

// RH_SX126x_CMD_GET_DEVICE_ERRORS
#define RH_SX126x_PA_RAMP_ERR                             0b100000000 //  device errors: PA ramping failed
#define RH_SX126x_PLL_LOCK_ERR                            0b001000000 //                 PLL failed to lock
#define RH_SX126x_XOSC_START_ERR                          0b000100000 //                 crystal oscillator failed to start
#define RH_SX126x_IMG_CALIB_ERR                           0b000010000 //                 image calibration failed
#define RH_SX126x_ADC_CALIB_ERR                           0b000001000 //                 ADC calibration failed
#define RH_SX126x_PLL_CALIB_ERR                           0b000000100 //                 PLL calibration failed
#define RH_SX126x_RC13M_CALIB_ERR                         0b000000010 //                 RC13M calibration failed
#define RH_SX126x_RC64K_CALIB_ERR                         0b000000001 //                 RC64K calibration failed

// RH_SX126x_CMD_SET_LBT_SCAN_PARAMS + RH_SX126x_CMD_SET_SPECTR_SCAN_PARAMS
#define RH_SX126x_SCAN_INTERVAL_7_68_US                   10          //  RSSI reading interval: 7.68 us
#define RH_SX126x_SCAN_INTERVAL_8_20_US                   11          //                         8.20 us
#define RH_SX126x_SCAN_INTERVAL_8_68_US                   12          //                         8.68 us

// SX126X SPI register variables
// RH_SX126x_REG_HOPPING_ENABLE
#define RH_SX126x_HOPPING_ENABLED                         0b00000001  //  intra-packet hopping for LR-FHSS: enabled
#define RH_SX126x_HOPPING_DISABLED                        0b00000000  //                                    (disabled)

// RH_SX126x_REG_LORA_SYNC_WORD_MSB + LSB
#define RH_SX126x_SYNC_WORD_PUBLIC                        0x34        // actually 0x3444  NOTE: The low nibbles in each byte (0x_4_4) are masked out since apparently, they're reserved.
#define RH_SX126x_SYNC_WORD_PRIVATE                       0x12        // actually 0x1424        You couldn't make this up if you tried.

// RH_SX126x_REG_TX_BITBANG_ENABLE_1
#define RH_SX126x_TX_BITBANG_1_DISABLED                   0b00000000  //  Tx bitbang: disabled (default)
#define RH_SX126x_TX_BITBANG_1_ENABLED                    0b00010000  //              enabled

// RH_SX126x_REG_TX_BITBANG_ENABLE_0
#define RH_SX126x_TX_BITBANG_0_DISABLED                   0b00000000  //  Tx bitbang: disabled (default)
#define RH_SX126x_TX_BITBANG_0_ENABLED                    0b00001100  //              enabled

// RH_SX126x_REG_DIOX_OUT_ENABLE
#define RH_SX126x_DIO1_OUT_DISABLED                       0b00000010  //  DIO1 output: disabled
#define RH_SX126x_DIO1_OUT_ENABLED                        0b00000000  //               enabled
#define RH_SX126x_DIO2_OUT_DISABLED                       0b00000100  //  DIO2 output: disabled
#define RH_SX126x_DIO2_OUT_ENABLED                        0b00000000  //               enabled
#define RH_SX126x_DIO3_OUT_DISABLED                       0b00001000  //  DIO3 output: disabled
#define RH_SX126x_DIO3_OUT_ENABLED                        0b00000000  //               enabled

// RH_SX126x_REG_DIOX_IN_ENABLE
#define RH_SX126x_DIO1_IN_DISABLED                        0b00000000  //  DIO1 input: disabled
#define RH_SX126x_DIO1_IN_ENABLED                         0b00000010  //              enabled
#define RH_SX126x_DIO2_IN_DISABLED                        0b00000000  //  DIO2 input: disabled
#define RH_SX126x_DIO2_IN_ENABLED                         0b00000100  //              enabled
#define RH_SX126x_DIO3_IN_DISABLED                        0b00000000  //  DIO3 input: disabled
#define RH_SX126x_DIO3_IN_ENABLED                         0b00001000  //              enabled

// RH_SX126x_REG_RX_GAIN
#define RH_SX126x_RX_GAIN_BOOSTED                         0x96        //  Rx gain: boosted
#define RH_SX126x_RX_GAIN_POWER_SAVING                    0x94        //           power saving
#define RH_SX126x_RX_GAIN_SPECTRAL_SCAN                   0xCB        //           spectral scan

// RH_SX126x_REG_PATCH_UPDATE_ENABLE
#define RH_SX126x_PATCH_UPDATE_DISABLED                   0b00000000  //  patch update: disabled
#define RH_SX126x_PATCH_UPDATE_ENABLED                    0b00010000  //                enabled

// RH_SX126x_REG_SPECTRAL_SCAN_STATUS
#define RH_SX126x_SPECTRAL_SCAN_NONE                      0x00        //  spectral scan status: none
#define RH_SX126x_SPECTRAL_SCAN_ONGOING                   0x0F        //                        ongoing
#define RH_SX126x_SPECTRAL_SCAN_ABORTED                   0xF0        //                        aborted
#define RH_SX126x_SPECTRAL_SCAN_COMPLETED                 0xFF        //                        completed

// RH_SX126x_REG_RSSI_AVG_WINDOW
#define RH_SX126x_SPECTRAL_SCAN_WINDOW_DEFAULT            (0x05 << 2) //  default RSSI average window

// RH_SX126x_REG_ANA_LNA
#define RH_SX126x_LNA_RNG_DISABLED                        0b00000001  //  random number: disabled
#define RH_SX126x_LNA_RNG_ENABLED                         0b00000000  //                 enabled

// RH_SX126x_REG_ANA_MIXER
#define RH_SX126x_MIXER_RNG_DISABLED                      0b00000001  //  random number: disabled
#define RH_SX126x_MIXER_RNG_ENABLED                       0b00000000  //                 enabled

// size of the spectral scan result
#define RH_SX126x_SPECTRAL_SCAN_RES_SIZE                  (33)





/////////////////////////////////////////////////////////////////////
/*! \class RH_SX126x RH_SX126x.h <RH_SX126x.h>
\brief Driver to send and receive unaddressed, unreliable datagrams via a
Semtech SX126X family LoRa capable radio transceiver.

Works with NiceRF LoRa1262-915 and Teensy 3.1. Will probably work with any other SX1262 module.

\par Overview

This class provides basic functions for sending and receiving unaddressed, 
unreliable datagrams of arbitrary length to 251 octets per packet.

Manager classes may use this class to implement reliable, addressed datagrams and streams, 
mesh routers, repeaters, translators etc.

Naturally, for any 2 radios to communicate that must be configured to use the same frequency and 
modulation scheme.

Predefined modulation schemes are available for various LoRa
modulation speeds and bandwidths. GFSK modulation is also supported.

The SX126x family of radio chips are availabel as discrete comonents with an SPI interface.
In some hardware (eg the STM32WLE5xx STM32WLE4xx processors) the radio
is built into a microprocessor.

\par Packet Format

All messages sent and received by this RH_SX126x Driver conform to this packet format, which is compatible with RH_RF95:

- LoRa mode:
- 8 symbol PREAMBLE
- Explicit header with header CRC (default CCITT, handled internally by the radio)
- 4 octets HEADER: (TO, FROM, ID, FLAGS)
- 0 to 251 octets DATA 
- CRC (default CCITT, handled internally by the radio)

\par Interrupts

The RH_SX126x driver uses interrupts to react to events in the radio,
such as the reception of a new packet, or the completion of
transmission of a packet. The driver configures the radio so the
required interrupt is generated by the radio's DIO1 pin.  The
RH_SX126x driver interrupt service routine reads status from and
writes data to the the radio module via an SPI interface. It is very
important therefore, that if you are using the RH_SX126x driver with
another SPI based deviced, that you disable interrupts while you
transfer data to and from that other device.  Use cli() to disable
interrupts and sei() to reenable them. (however note that the
RH_STM32WLx subclass uses the dedicated internal SPI interface that is
connected only to the radio).

\par Memory

The RH_SX126x driver requires non-trivial amounts of memory. The sample
programs all compile to about 35 kbytes each, which will fit in the
flash proram memory of most Arduinos. However, the RAM requirements are
more critical. Therefore, you should be vary sparing with RAM use in
programs that use the RH_SX126x driver.

\par Compatibility with RH_RF95

The predefined modulation schemes have been show to interoperate with
the RH_RF95 driver with similarly named modulation schemes.

For example the (default) RH_SX126x::LoRa_Bw125Cr45Sf128 is compatible
with the (default) RH_RF95::Bw125Cr45Sf128.

The RH_SX126x driver sets the LoRa Sync word to 0x1424, which is compatible with single byte 0x12 default for RH_RF95.
// https://forum.lora-developers.semtech.com/t/sx1272-and-sx1262-lora-sync-word-compatibility/988/13

\par Transmitter Power

We measured the RF power output from a Wio-E5 mini at 868.0 MHz, with
the radio set to continuous CW transmission using
setTxContinuous(). On this chip that implies the high power amplifier.
We set various power outputs with setTxPower() from -9 to 22 and
measured the RF output power with a HP 5342A Microwave Frequency
Counter. Note that the drivers setTxPower() sets the optimum
transmitter control registers per section 13.1.14.1 of the datasheet
SX1262_datasheet.pdf

\code
Program power              Measured power
     dBm                      dBm
    -9                       -5.0
     0                       -2.9
     5                        6.9
    10                        8.9
    15                       13.3
    16                       14.1
    17                       14.6
    18                       16.7
    19                       17.3
    20                       17.8
    21                       18.7
    22                       19.4
\endcode

With the transmitter frequency set to 868.0 MHz, the actual centre
frequency measured with the HP 5342A Microwave Frequency Counter on 2
instances of Wio-E5 mini were 867.999826 and 867.999652 MHz.

\par Differences between models

SX126x compatible chips are available in at least 4 types:

-SX1261 Has only one (low power) PA, -17 to +15 dBm

-SX1262 Has only one (high power) PA, -9 to +22 dBm

-SX1268 Has 2 PAs, low power (-17 to +15 dBm) and high power (-9 to +22 dBm)

-STM32WLE5JC has 2 PAs, low power (-17 to +15 dBm) and high power (-9 to +22 dBm).

Even if the radio has 2 PAs, depending on your radio module, maybe
only one is connected. It also includes a dedicated SPI interface for
the radio plus some internllay connected reset and interrupt pins

Some radio modules might also include an antenna switch, and the
driver MUST be configured so that it knows how to turn any
control pins on and off for receiving and transmitting. See setRadioPinConfig().

\par Configuring the driver for your particular type of radio

You will almost certainly have to configure this driver to suit the
particular radio hardware in your system.  Its boring but you MUST pay
attention to this otherwise you may not be able to transmit or receive
successfully.

This issues you will have to consider are:

- What model radio do you have?

- What SPI bus is the radio connected to?

- Is there a TCXO to configure?

- What pin is used for the SPI slave select for the radio chip?

- Is the radio reset pin connected to the CPU?

- Are there any pins required to be set to control the external radio
  interface, such as RF switches, external PAs etc,

If you are using a ST Microelectronics STM32WLE5xx or STM32WLE4xx
processors and its built in radio, you can use the RH_STM32WLx and
ignore most or all of these issues.

If you are using a Heltec CubeCell, such as HTCC-AB01, initialise the driver with:
\code
RH_SX126x driver(RADIO_NSS, RADIO_DIO_1, RADIO_BUSY, RADIO_RESET);
\endcode

\par Range

No range tests have yet been conducted.

\par Connecting SX126x modules to Arduino

Note, if you are using a STM32WLE5JC, see the intructions for that in RH_STM32WLx.h

Connecting a NiceRF LoRa1262-915 to a Teensy 3.1:

https://www.nicerf.com/lora-module/915mhz-lora-module-lora1262.html

We got one on a breakout board
which already has a small helical antenna connected. The module appears to contain a 3.3V TCXO, nd an antenna switch connected to DIO2
You should be able to use a
similar pinout for any 3.3V Arduino compatible board.


\code
                Teensy 3.1    G-Nice RF LoRa1262-915
                GND----------GND   (Ground)
                3V3----------VCC   (3.3V in)
            pin D7-----------DIO1  (radio interrupt request out, active high)
	    pin D8-----------BUSY  (radio busy output, active high)
            pin D9-----------NRESET (radio reset in: pulled low for 2ms at startup)
         SS pin D10----------NSS   (chip select in)
        SCK pin D13----------SCK   (SPI clock in)
       MOSI pin D11----------MOSI  (SPI Data in)
       MISO pin D12----------MISO  (SPI Data out)
                
With these connections you can then use the constructor:

RH_SX126x driver(SS, 7, 8, 9);
\endcode

RAKwireless RAK4360/RAK4361

RHHardwareSPI uses the default LoRa radio SPI pins as defined by the platform. You can use the contructor:

\code
RH_SX126x driver(42, 47, 46, 38); // NSS, DIO1, BUSY, NRESET
\endcode





*/

class RH_SX126x : public RHSPIDriver
{
public:
    /// Packet types the modem can be configured for
    typedef enum
    {
	PacketTypeLoRa = 0,  ///< Use LoRA packets
	PacketTypeGFSK,      ///< Use GFSK packets
    } PacketType;

    /// \brief Defines register values for a set of modem configuration registers
    ///
    /// Defines register values for a set of modem configuration registers
    /// that can be passed to setModulationParameters() if none of the choices in
    /// ModemConfigChoice suit your need setModemRegisters() writes the
    /// register values from this structure to the appropriate registers
    /// to set the desired spreading factor, coding rate and bandwidth
    typedef struct
    {
	PacketType packetType;
	uint8_t    p1;   ///< Value for setModulationParameters parameter 1
	uint8_t    p2;   ///< Value for setModulationParameters parameter 2
	uint8_t    p3;   ///< Value for setModulationParameters parameter 3
	uint8_t    p4;   ///< Value for setModulationParameters parameter 4
	uint8_t    p5;   ///< Value for setModulationParameters parameter 5
	uint8_t    p6;   ///< Value for setModulationParameters parameter 6
	uint8_t    p7;   ///< Value for setModulationParameters parameter 7
	uint8_t    p8;   ///< Value for setModulationParameters parameter 8
    } ModemConfig;
  
    /// Choices for setModemConfig() for a selected subset of common
    /// data rates. If you need another configuration,
    /// determine the necessary settings and call setModemRegisters() with your
    /// desired settings. It might be helpful to use the LoRa calculator mentioned in 
    /// http://www.semtech.com/images/datasheet/LoraDesignGuide_STD.pdf
    /// These are indexes into MODEM_CONFIG_TABLE. We strongly recommend you use these symbolic
    /// definitions and not their integer equivalents: its possible that new values will be
    /// introduced in later versions (though we will try to avoid it).
    /// Caution: if you are using slow packet rates and long packets with RHReliableDatagram or subclasses
    /// you may need to change the RHReliableDatagram timeout for reliable operations.
    /// Caution: for some slow rates nad with ReliableDatagrams you may need to increase the reply timeout 
    /// with manager.setTimeout() to
    /// deal with the long transmission times.
    /// Caution: SX1276 family errata suggests alternate settings for some LoRa registers when 500kHz bandwidth
    /// is in use. See the Semtech SX1276/77/78 Errata Note. These are not implemented by RH_SX126x.
    /// In general, the LoRa_* configurations are compatible with the similarly named RH_RF95 configurations
    typedef enum
    {
	LoRa_Bw125Cr45Sf128 = 0,   ///< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
	LoRa_Bw500Cr45Sf128,	   ///< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
	LoRa_Bw31_25Cr48Sf512,	   ///< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
	LoRa_Bw125Cr48Sf4096,      ///< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, low data rate, CRC on. Slow+long range
	LoRa_Bw125Cr45Sf2048,      ///< Bw = 125 kHz, Cr = 4/5, Sf = 2048chips/symbol, CRC on. Slow+long range
    } ModemConfigChoice;

    /// Structures and enums for Tx/Rx pin configuration
    /// These structures allow you to specify what pins are to be automaticall set or cleared to control radio power amp and receivers
    /// and how there are to be set for each radio mode, IDLE, TX or RX.
    /// They can be used to automatically configure any RF switch or external power amp etc.
    /// You will probably need these to configure the driver for your specific hardware
    typedef enum
    {
	RadioPinConfigMode_EOT = 0,         // End of table. Must be the last item in the pin configuration table
	RadioPinConfigMode_IDLE,            // This config is for the radio idle
	RadioPinConfigMode_RX,              // This config is for receiving
	RadioPinConfigMode_TX_LOW_POWER,    // This config is for transmitting with low power PA
	RadioPinConfigMode_TX_HIGH_POWER,   // This config is for transmitting with high power PA
    } RadioPinConfigMode;
    
    // Maximum bumber of entries permitted in a RadioPinConfigTable
    #define RH_SX126x_MAX_RADIO_PIN_CONFIG_MODES (RadioPinConfigMode_TX_HIGH_POWER + 1)

    // The number of pins that might need to be controlled
    #define RH_SX126x_MAX_RADIO_CONTROL_PINS     (3)
    
    // Tells how to set the pins in PinConfig for each a particular transmit or receive condition
    typedef struct
    {
	/// The type of radio condition for these pin settings. PinConfigEntry_EOT for last item in table
	RadioPinConfigMode mode = RadioPinConfigMode_EOT;
	/// The state (HIGH or LOW) to set each of the radio control pins to when this state is reached
	bool               pinState[RH_SX126x_MAX_RADIO_CONTROL_PINS];
    } RadioPinConfigEntry;

    /// Pointer to structure can be passed to the contructor or setRadioPinConfig() to configure the
    /// how various pins are to be set to configure your radio hardware (RF swithes etc) for various radio modes.
    typedef struct
    {
	/// Pin number of each pin to be automcatically controlled
	uint8_t            pinNumber[RH_SX126x_MAX_RADIO_CONTROL_PINS]; // Pin number or RH_INVALID_PIN
	/// One entry for each radio state supported by your hardware
	// The last entry must have mode = RadioPinConfigMode_EOT
	RadioPinConfigEntry configState[RH_SX126x_MAX_RADIO_PIN_CONFIG_MODES];
    } RadioPinConfig;
    
    /// Constructor. You can have multiple instances, but each instance must have its own
    /// interrupt and slave select pin. After constructing, you must call init() to initialise the interface
    /// and the radio module. A maximum of 3 instances can co-exist on one processor, provided there are sufficient
    /// distinct interrupt lines, one for each instance.
    /// \param[in] slaveSelectPin the Arduino pin number of the output to use to select the RH_RF22 before
    /// accessing it. Defaults to the normal SS pin for your Arduino (D10 for Diecimila, Uno etc, D53 for Mega, D10 for Maple)
    /// \param[in] interruptPin The interrupt Pin number that is connected to the RFM DIO0 interrupt line. 
    /// Defaults to pin 2, as required by Anarduino MinWirelessLoRa module.
    /// Caution: You must specify an interrupt capable pin.
    /// On many Arduino boards, there are limitations as to which pins may be used as interrupts.
    /// On Leonardo pins 0, 1, 2 or 3. On Mega2560 pins 2, 3, 18, 19, 20, 21. On Due and Teensy, any digital pin.
    /// On Arduino Zero from arduino.cc, any digital pin other than 4.
    /// On Arduino M0 Pro from arduino.org, any digital pin other than 2.
    /// On other Arduinos pins 2 or 3. 
    /// See http://arduino.cc/en/Reference/attachInterrupt for more details.
    /// On Chipkit Uno32, pins 38, 2, 7, 8, 35.
    /// On other boards, any digital pin may be used.
    /// \param[in] busyPin Pin number of pin connected to the radio's busy pin. The radio sets the busy pin high while it is busy
    /// If this is not set to RH_INVALID_PIN (the default) then this module will wait for the busy pin to go low before
    /// initialting the next SPI transfer. It is strongly recommended that you use this.
    /// \param[in] resetPin Pin number of the pin connected to the radio's reset pin. If this is not set to RH_INVALID_PIN (the default) then this module will
    /// assert the reset pin low for 2 ms during init() in order to reset the radio. It is strongly recommended that you use this
    /// \param[in] spi Pointer to the SPI interface object to use.
    /// \param[in] radioPinConfig pinter to a strucure that describes what pins are to be automatically set when changing
    /// the radio mode. This can be used to configure any external RF switches, RF amplifiers etc.
    ///                Defaults to the standard Arduino hardware SPI interface
    RH_SX126x(uint8_t slaveSelectPin = SS, uint8_t interruptPin = 2, uint8_t busyPin = RH_INVALID_PIN, uint8_t resetPin = RH_INVALID_PIN,
	      RHGenericSPI& spi = hardware_spi, RadioPinConfig* radioPinConfig = NULL);
    
    /// Initialise the Driver transport hardware and software.
    /// Leaves the radio in idle mode,
    /// with default configuration of: 915.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
    /// \return true if initialisation succeeded.
    virtual bool    init();

    /// Prints the value of selected radio chip registers
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// For debugging purposes only.
    /// \param[in] address The register adddress of the first register to print
    /// \param[in] count The number of registers to print
    /// \return true on success
    bool printRegisters(uint16_t address, uint8_t count);

    /// Sets all the registers required to configure the data modem in the radio, including the bandwidth, 
    /// spreading factor etc. You can use this to configure the modem with custom configurations if none of the 
    /// canned configurations in ModemConfigChoice suit you.
    /// \param[in] config A ModemConfig structure containing values for the modem configuration registers.
    /// \return true if modem was successfully reconfigured
    bool           setModemRegisters(const ModemConfig* config);

    /// Select one of the predefined modem configurations. If you need a modem configuration not provided 
    /// here, use setModemRegisters() with your own ModemConfig.
    /// Caution: the slowest protocols may require a radio module with TCXO temperature controlled oscillator
    /// for reliable operation.
    /// \param[in] index The configuration choice.
    /// \return true if index is a valid choice.
    bool        setModemConfig(ModemConfigChoice index);

    /// Tests whether a new message is available from the Driver. 
    /// On most drivers, this will also put the Driver into RHModeRx mode until
    /// a message is actually received by the transport, when it will be returned to RHModeIdle.
    /// This can be called multiple times in a timeout loop
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    virtual bool    available();

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    virtual bool    recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then optionally waits for Channel Activity Detection (CAD) 
    /// to show the channnel is clear (if the radio supports CAD) by calling waitCAD().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send
    /// specify the maximum time in ms to wait. If 0 (the default) do not wait for CAD before transmitting.
    /// \return true if the message length was valid and it was correctly queued for transmit. Return false
    /// if CAD was requested and the CAD timeout timed out before clear channel was detected.
    virtual bool    send(const uint8_t* data, uint8_t len);

    /// Sets the length of the preamble
    /// in bytes. 
    /// Caution: this should be set to the same 
    /// value on all nodes in your network. Default is 8.
    /// Sets the message preamble length in RH_SX126x_REG_??_PREAMBLE_?SB
    /// \param[in] bytes Preamble length in bytes.  
    void           setPreambleLength(uint16_t bytes);

    /// Returns the maximum message length 
    /// available in this Driver.
    /// \return The maximum legal message length
    virtual uint8_t maxMessageLength();

    /// Sets the transmitter and receiver 
    /// centre frequency.
    /// \param[in] centre Frequency in MHz. 137.0 to 1020.0. Caution: RFM95/96/97/98 comes in several
    /// different frequency ranges, and setting a frequency outside that range of your radio will probably not work
    /// \param[i] calibrate set true if the radio modules are to be automatically recalibrated for this frequency
    /// \return true if the selected frquency centre is within range, and the radio frequency is successfully set
    bool        setFrequency(float centre, bool calibrate = true);

    /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
    void           setModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the SX126X/96/97/98.
    void           setModeRx();

    /// If current mode is Rx or Idle, changes it to Rx. F
    /// Starts the transmitter in the SX126X/96/97/98.
    void           setModeTx();

    /// Sets the transmitter power output level
    /// Be a good neighbour and set the lowest power level you need.
    /// Caution: legal power limits may apply in certain countries.
    /// After init(), the power will be set to 13dBm.
    /// \param[in] power Transmitter power level in dBm.
    /// For SX1261, limits are -17 to +15 dBm
    /// For SX1262, limits are -9 to +22 dBm
    /// For STM32WLx with low power PA configured by radioPinConfig, same as SX1261.
    /// For STM32WLx with high power PA configured by radioPinConfig, same as SX1262.
    /// \return true if successful. Returns false if radioPinConfig has not been properly
    /// configured for the requested power setting
    virtual bool           setTxPower(int8_t power);

    /// Sets the radio into low-power sleep mode.
    /// If successful, the transport will stay in sleep mode until woken by 
    /// changing mode it idle, transmit or receive (eg by calling send(), recv(), available() etc)
    /// Caution: there is a time penalty as the radio takes a finite time to wake from sleep mode.
    /// \return true if sleep mode was successfully entered.
    virtual bool    sleep();

    /// Use the radio's Channel Activity Detect (CAD) function to detect channel activity.
    /// Sets the SX126X radio into CAD mode and waits until CAD detection is complete.
    /// To be used in a listen-before-talk mechanism (Collision Avoidance)
    /// with a reasonable time backoff algorithm.
    /// This is called automatically by waitCAD().
    /// NOT YET WORKING.
    /// \return true if channel is in use.  
    virtual bool    isChannelActive();

    /// Returns the Signal-to-noise ratio (SNR) of the last received message, as measured
    /// by the receiver.
    /// \return SNR of the last received message in dB
    int lastSNR();

    // Support for configurable RX and TX pins
    void setRadioPinConfig(RadioPinConfig* config);

    /// Set the radio into continuous transmission mode. A carrier
    /// wave will be transmitted on the configured centre frequency until available(), recv() or send() are called
    /// CAUTION: use this only for testing in controlled conditions with a dummy load. It may be illegal for you to transmit
    /// a continuous carrier wave to air.
    bool setTxContinuous();
    
    /// Read and return the radio status byte
    uint8_t getStatus();

    /// Return the last interrupt mask, for debugging
    uint16_t lastIrq() {return _lastirq;};

    /// Return true if an interrupt has occurred since the last clearIflag(). For debugging
    bool getIflag() {return _iflag;};

    /// Reset the interrupt flag. For debugging
    void clearIflag() {_iflag=false;};

    /// REsets the last interrupt mask. For debugging
    void clearLastIrq() {_lastirq=0;};

    /// Enable or disable the ability to detect CRC errors
    void enableCrcErrorIrq(bool enable);

    /// Tells the driver to enable RAW mode, which prevents the transmissions of the 4 byte address header.
    void enableRawMode(bool enable);

    /// Returns the frequency error from the last received packet
    float getFrequencyError();    

protected:

    ///////////////////////////////////////////////////////////////////
    // Follow are low level functions for communicating with the SX126x
    // Caution should be used if accessing them in subclasses

    /// Wait until the busy pin (if speecified in the contructor) is no longer low
    /// On timeout, prints an error to eSerial and returns false. Else returns true.
    virtual bool waitUntilNotBusy();
    

    /// Send a command with multi-byte data to the radio
    bool sendCommand(uint8_t command, uint8_t data[], uint8_t len);

    /// Send a command with a single data byte to the radio
    bool sendCommand(uint8_t command, uint8_t value);

    /// Send a command without any data to the radio
    bool sendCommand(uint8_t command);

    /// Send a command to the radio and get a multi-byte respose
    bool getCommand(uint8_t command, uint8_t data[], uint8_t len);

    /// Read multiple registers from the radio
    bool readRegisters(uint16_t address, uint8_t data[], uint8_t len);

    /// Read and return a single register byte from the radio
    uint8_t readRegister(uint16_t address);

    /// Write multibyte data to the given register and sunbsequent registers
    bool writeRegisters(uint16_t address, uint8_t data[], uint8_t len);

    /// Write a single byte to the given register
    bool writeRegister(uint16_t address, uint8_t data);

    /// Write multibyte data to the radio IO buffer at the current buffer address
    bool writeBuffer(uint8_t offset, const uint8_t data[], uint8_t len);

    /// Write a single byte to the radio IO bufferat the current buffer address
    bool writeBuffer(uint8_t offset, const char* text);

    /// Read multibyte data from the radio IO buffer at the current buffer address
    bool readBuffer(uint8_t offset, uint8_t data[], uint8_t len);

    /// Set the radio Power Amplifier configuration
    bool setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut);

    /// Set the radio power output. CAUTION: for internal use only. Users should use setTxPower()
    bool setTxParams(uint8_t power, uint8_t rampTime);

    /// Clear the radio error byte
    bool clearDeviceErrors();

    /// Sets whether DIO2 is to be used to automatically control an external radio RF switch.
    /// Normall you should use the pinConfig in the constructor or setRadioPinConfig() to automatically control any
    /// radio control pins
    bool setDIO2AsRfSwitchCtrl(bool value);

    /// Sets the mode that the radio will change to after a transmit or receive is complete
    bool setRxFallbackMode(uint8_t mode);

    /// Set the low-level registers for any desired modulation scheme
    bool setModulationParameters(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, uint8_t p6, uint8_t p7, uint8_t p8);

    /// Set the low-level registers for the desired LoRA modulation scheme
    bool setModulationParametersLoRa(uint8_t sf, float bw, uint8_t cr, bool ldro);

    /// Set the low-level registers for the desired GFSK modulation scheme
    bool setModulationParametersGFSK(uint32_t br, uint8_t sh, uint8_t rxBw, uint32_t freqDev);

    /// Cause the radio to calibrate all its sections at the currently selected frequency
    bool calibrate(uint8_t calib_param);

    /// Allows the user to calibrate the image rejection of the device for the device operating frequency band
    bool calibrateImage(uint8_t f1, uint8_t f2);

    /// Set the 16 bit LoRa sync word
    bool setLoRaSyncWord(uint16_t sync);

    /// Set the radio power amplifier over-current protection
    bool setOCPConfiguration(uint8_t setting);

    /// Configures the radio to use an external temperature controlled crystal oscillator (TCXO) and the oven voltage to use.
    /// For low level internal use only
    bool setDIO3AsTcxoCtrl(uint8_t voltage, uint32_t delay);

    /// Set the voltage to use for the TCXO oven
    bool setTCXO(float voltage, uint32_t delay);

    /// Low level function to set the radio packet confiuration
    bool setPacketParams(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4, uint8_t p5, uint8_t p6, uint8_t p7, uint8_t p8, uint8_t p9);

    /// Set the necessary radio packet parameters for a forthcoming transmission of payload_length bytes
    bool setPacketParametersLoRa(uint8_t payload_length);

    /// Low level function to set the address wherge the next radBuffer or writeBuffer will occur
    bool setBufferBaseAddress(uint8_t txbase, uint8_t rxbase);

    /// Set the radio to sleep mode. Automatically configures the radio control pins to the configuration RadioPinConfigMode_IDLE.
    bool setSleep(uint8_t config);

    /// Set the radio to sleep mode. Automatically configures the radio control pins to the configuration RadioPinConfigMode_IDLE.
    bool setStandby(uint8_t config);

    /// Set the radio to transmit mode. Automatically configures the radio control pins to the configuration required
    /// for the most recently requested power in setTxPower (RadioPinConfigMode_TX_HIGH_POWER or RadioPinConfigMode_TX_LOW_POWER )
    bool setTx(uint32_t timeout);

    /// Starts the radio in Clear Air Detect (CAD) mode. CAUTION: NOT YET WORKING, always retuns false.
    bool setCad();

    /// Set the radio to receive mode. Automatically configures the radio control pins to the configuration RadioPinConfigMode_RX.
    bool setRx(uint32_t timeout);

    /// Sets whether radios receiver gain boost should be enabled instead of the default power saving mode.
    bool setRxBoostMode(bool boost, bool retain);

    /// Sets the chip regulator mode to either LDO or DC-DC SMPS.
    /// Set to DC-DC SMPS by default
    bool setRegulatorMode(uint8_t mode);

    /// Configures the conditions under which the radio will enable an interrupt, and for which DIO pins
    bool setDioIrqParams(uint16_t irqmask, uint16_t dio1mask, uint16_t dio2mask, uint16_t dio3mask);

    /// Clear the radio IRQ state
    bool clearIrqStatus(uint16_t mask);

    /// Return the radio IRQ state
    uint16_t getIrqStatus();

    /// return the current packet type
    uint8_t getPacketType();

    /// From SX1262_datasheet.pdf: "When exchanging LoRa® packets with inverted IQ polarity,
    /// some packet losses may be observed for longer packet". THis function enables the workaround described in that section
    void setInvertIQ(bool invertIQ);

    /// Per SX1262_datasheet.pdf Rev 1.2 section 15.2, this fixes an error in the radio Power Amplifier clamping
    bool fixPAClamping(bool enable);
    
    /// Do whatever is necesary to establish the interrupt handler. Subclasses may have different needs 
    virtual bool setupInterruptHandler();

    /// This is a low level function to handle the interrupts for one instance of RH_SX126x.
    /// Called automatically by isr*()
    /// Should not need to be called by user code.
    void           handleInterrupt();

    /// Examine the revceive buffer to determine whether the message is for this node
    void validateRxBuf();

    /// Clear our local receive buffer
    void clearRxBuf();

    /// Called by RH_SX126x when the radio mode is about to change to a new setting.
    /// Can be used by subclasses to implement antenna switching etc.
    /// \param[in] mode RHMode the new mode about to take effect
    /// \return true if the subclasses changes successful
    virtual bool modeWillChange(RHMode) {return true;}

    /// Sets the pins configured in radioPinConfig as required for the desired mode.
    /// Called just before the radio is set to the new mode.
    /// \return true if succcessful, false is there is no radioPinConfig configuration,
    /// or no entry for the requested mode
    virtual bool setRadioPinsForMode(RadioPinConfigMode mode);

    /// Find the pin configuration entry for a desired radio mode
    virtual RadioPinConfigEntry* findRadioPinConfigEntry(RadioPinConfigMode mode);

private:
    /// Low level interrupt service routine for device connected to interrupt 0
    static void         isr0();

    /// Low level interrupt service routine for device connected to interrupt 1
    static void         isr1();

    /// Low level interrupt service routine for device connected to interrupt 1
    static void         isr2();

    /// Array of instances connected to interrupts 0 and 1
    static RH_SX126x*     _deviceForInterrupt[];

    /// The configured interrupt pin connected to this instance
    uint8_t             _interruptPin;

    /// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
    /// else 0xff
    uint8_t             _myInterruptIndex;

    /// Number of octets in the buffer
    volatile uint8_t    _bufLen;
    
    /// The receiver/transmitter buffer
    uint8_t             _buf[RH_SX126x_MAX_PAYLOAD_LEN];

    /// True when there is a valid message in the buffer
    volatile bool       _rxBufValid;

    /// True if we are using the HF port (779.0 MHz and above)
    bool                _usingHFport;

    /// Last measured SNR, dB
    int8_t              _lastSNR;

    /// If true, sends CRCs in every packet and requires a valid CRC in every received packet
    bool                _enableCRC;

    /// Sets the preamble length for LoRa packets
    uint16_t            _preambleLength = 8;

    /// Whether the modem is to be configured for INverted IQ
    bool                _invertIQ = false;

    /// The type of packet to configure the modem for
    PacketType          _packetType = PacketTypeLoRa;

    /// If the current LoRa bandwidth is 500kHz, we need to remeber this in order to implement the
    /// modulation quality workaround in setTx()
    bool                _lorabw500 = false;

    /// Support for optional configurable radio control pins for RX and TX modes
    RadioPinConfig*      _radioPinConfig = NULL;
    
    /// Remember what PA type is required, depending on device type and radio pin configurations
    RadioPinConfigMode  _requiredPAMode = RadioPinConfigMode_IDLE; // One of PinConfigMode_TX_LOW_POWER PinConfigMode_TX_HIGH_POWER

    /// Pin number of the radio BUSY pin, if available, else RH_INVALID_PIN
    uint8_t             _busyPin;

    /// Pin number of the radio NRESET pin, if available, else RH_INVALID_PIN
    uint8_t             _resetPin;

    /// Currently selected bandwidth, required for frequencey error calculations
    float               _bandwidth = 0.0;

    /// Whether we are in raw mode, bypassing address bytes prefix
    bool                _raw = false;

    /// Vale of the last interrupt flags, for debugging
    volatile uint16_t   _lastirq;

    /// Whether an interupt has occurred since the last clearIflag(). For debugging
    volatile bool       _iflag = false;
    
    // These are the interrupts we are willing to process
    uint16_t            _irqMask = RH_SX126x_IRQ_CAD_DETECTED
                                    | RH_SX126x_IRQ_CAD_DONE
                                    | RH_SX126x_IRQ_CRC_ERR
                                    | RH_SX126x_IRQ_HEADER_ERR
                                    | RH_SX126x_IRQ_RX_DONE
                                    | RH_SX126x_IRQ_TX_DONE;
};

/// @example sx1262_client.ino
/// @example sx1262_server.ino

#endif
