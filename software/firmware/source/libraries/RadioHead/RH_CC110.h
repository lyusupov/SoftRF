// RH_CC110.h
//
// Definitions for Texas Instruments CC110L transceiver.
// http://www.ti.com/lit/ds/symlink/cc110l.pdf
// As used in Anaren CC110L Air Module BoosterPack
// https://www.anaren.com/air/cc110l-air-module-boosterpack-embedded-antenna-module-anaren
// 
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2016 Mike McCauley
// $Id: RH_CC110.h,v 1.9 2020/01/05 07:02:23 mikem Exp $
// 

#ifndef RH_CC110_h
#define RH_CC110_h


#include <RHNRFSPIDriver.h>

// This is the maximum number of interrupts the driver can support
// Most Arduinos can handle 2, Megas can handle more
#define RH_CC110_NUM_INTERRUPTS 3

// Max number of octets the FIFO can hold
#define RH_CC110_FIFO_SIZE 64

// This is the maximum number of bytes that can be carried by the chip
// We use some for headers, keeping fewer for RadioHead messages
#define RH_CC110_MAX_PAYLOAD_LEN RH_CC110_FIFO_SIZE

// The length of the headers we add.
// The headers are inside the chip payload
#define RH_CC110_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data 
#ifndef RH_CC110_MAX_MESSAGE_LEN
 #define RH_CC110_MAX_MESSAGE_LEN (RH_CC110_MAX_PAYLOAD_LEN - RH_CC110_HEADER_LEN - 1)
#endif

#define RH_CC110_SPI_READ_MASK  0x80
#define RH_CC110_SPI_BURST_MASK 0x40

// Register definitions from Table 5-22
#define RH_CC110_REG_00_IOCFG2                 0x00
#define RH_CC110_REG_01_IOCFG1                 0x01
#define RH_CC110_REG_02_IOCFG0                 0x02
#define RH_CC110_REG_03_FIFOTHR                0x03
#define RH_CC110_REG_04_SYNC1                  0x04
#define RH_CC110_REG_05_SYNC0                  0x05
#define RH_CC110_REG_06_PKTLEN                 0x06
#define RH_CC110_REG_07_PKTCTRL1               0x07
#define RH_CC110_REG_08_PKTCTRL0               0x08
#define RH_CC110_REG_09_ADDR                   0x09
#define RH_CC110_REG_0A_CHANNR                 0x0a
#define RH_CC110_REG_0B_FSCTRL1                0x0b
#define RH_CC110_REG_0C_FSCTRL0                0x0c
#define RH_CC110_REG_0D_FREQ2                  0x0d
#define RH_CC110_REG_0E_FREQ1                  0x0e
#define RH_CC110_REG_0F_FREQ0                  0x0f
#define RH_CC110_REG_10_MDMCFG4                0x10
#define RH_CC110_REG_11_MDMCFG3                0x11
#define RH_CC110_REG_12_MDMCFG2                0x12
#define RH_CC110_REG_13_MDMCFG1                0x13
#define RH_CC110_REG_14_MDMCFG0                0x14
#define RH_CC110_REG_15_DEVIATN                0x15
#define RH_CC110_REG_16_MCSM2                  0x16
#define RH_CC110_REG_17_MCSM1                  0x17
#define RH_CC110_REG_18_MCSM0                  0x18
#define RH_CC110_REG_19_FOCCFG                 0x19
#define RH_CC110_REG_1A_BSCFG                  0x1a
#define RH_CC110_REG_1B_AGCCTRL2               0x1b
#define RH_CC110_REG_1C_AGCCTRL1               0x1c
#define RH_CC110_REG_1D_AGCCTRL0               0x1d
#define RH_CC110_REG_1E_WOREVT1                0x1e
#define RH_CC110_REG_1F_WOREVT0                0x1f
#define RH_CC110_REG_20_WORCTRL                0x20
#define RH_CC110_REG_21_FREND1                 0x21
#define RH_CC110_REG_22_FREND0                 0x22
#define RH_CC110_REG_23_FSCAL3                 0x23
#define RH_CC110_REG_24_FSCAL2                 0x24
#define RH_CC110_REG_25_FSCAL1                 0x25
#define RH_CC110_REG_26_FSCAL0                 0x26
#define RH_CC110_REG_27_RCCTRL1                0x28
#define RH_CC110_REG_28_RCCTRL0                0x29
#define RH_CC110_REG_29_FSTEST                 0x2a
#define RH_CC110_REG_2A_PTEST                  0x2b
#define RH_CC110_REG_2B_AGCTEST                0x2c
#define RH_CC110_REG_2C_TEST2                  0x2c
#define RH_CC110_REG_2D_TEST1                  0x2d
#define RH_CC110_REG_2E_TEST0                  0x2e

// Single byte read and write version of registers 0x30 to 0x3f. Strobes
// use spiCommand()
#define RH_CC110_STROBE_30_SRES                0x30
#define RH_CC110_STROBE_31_SFSTXON             0x31
#define RH_CC110_STROBE_32_SXOFF               0x32
#define RH_CC110_STROBE_33_SCAL                0x33
#define RH_CC110_STROBE_34_SRX                 0x34
#define RH_CC110_STROBE_35_STX                 0x35
#define RH_CC110_STROBE_36_SIDLE               0x36

#define RH_CC110_STROBE_39_SPWD                0x39
#define RH_CC110_STROBE_3A_SFRX                0x3a
#define RH_CC110_STROBE_3B_SFTX                0x3b

#define RH_CC110_STROBE_3D_SNOP                0x3d


// Burst read from these registers gives more data:
// use spiBurstReadRegister()
#define RH_CC110_REG_30_PARTNUM                0x30
#define RH_CC110_REG_31_VERSION                0x31
#define RH_CC110_REG_32_FREQEST                0x32
#define RH_CC110_REG_33_CRC_REG                0x33
#define RH_CC110_REG_34_RSSI                   0x34
#define RH_CC110_REG_35_MARCSTATE              0x35

#define RH_CC110_REG_38_PKTSTATUS              0x38

#define RH_CC110_REG_3A_TXBYTES                0x3a
#define RH_CC110_REG_3B_RXBYTES                0x3b

// PATABLE, TXFIFO, RXFIFO also support burst
#define RH_CC110_REG_3E_PATABLE                0x3e
#define RH_CC110_REG_3F_FIFO                   0x3f

// Status Byte
#define RH_CC110_STATUS_CHIP_RDY            0x80
#define RH_CC110_STATUS_STATE               0x70
#define RH_CC110_STATUS_IDLE                0x00
#define RH_CC110_STATUS_RX                  0x10
#define RH_CC110_STATUS_TX                  0x20
#define RH_CC110_STATUS_FSTXON              0x30
#define RH_CC110_STATUS_CALIBRATE           0x40
#define RH_CC110_STATUS_SETTLING            0x50
#define RH_CC110_STATUS_RXFIFO_OVERFLOW     0x60
#define RH_CC110_STATUS_TXFIFO_UNDERFLOW    0x70
#define RH_CC110_STATUS_FIFOBYTES_AVAILABLE 0x0f

// Register contents
// Chip Status Byte, read from header, data or command strobe
#define RH_CC110_CHIP_RDY                   0x80
#define RH_CC110_STATE                      0x70
#define RH_CC110_FIFO_BYTES_AVAILABLE       0x0f

// Register bit field definitions
// #define RH_CC110_REG_00_IOCFG2                 0x00
// #define RH_CC110_REG_01_IOCFG1                 0x01
// #define RH_CC110_REG_02_IOCFG0                 0x02
#define RH_CC110_GDO_CFG_RX_FIFO_THR              0x00
#define RH_CC110_GDO_CFG_RX_FIFO_FULL             0x01
#define RH_CC110_GDO_CFG_TX_FIFO_THR              0x02
#define RH_CC110_GDO_CFG_TX_FIFO_EMPTY            0x03
#define RH_CC110_GDO_CFG_RX_FIFO_OVERFLOW         0x04
#define RH_CC110_GDO_CFG_TX_FIFO_UNDEFLOOW        0x05
#define RH_CC110_GDO_CFG_SYNC                     0x06
#define RH_CC110_GDO_CFG_CRC_OK_AUTORESET         0x07
#define RH_CC110_GDO_CFG_CCA                      0x09
#define RH_CC110_GDO_CFG_LOCK_DETECT              0x0a
#define RH_CC110_GDO_CFG_SERIAL_CLOCK             0x0b
#define RH_CC110_GDO_CFG_SYNCHRONOUS_SDO          0x0c
#define RH_CC110_GDO_CFG_SDO                      0x0d
#define RH_CC110_GDO_CFG_CARRIER                  0x0e
#define RH_CC110_GDO_CFG_CRC_OK                   0x0f
#define RH_CC110_GDO_CFG_PA_PD                    0x1b
#define RH_CC110_GDO_CFG_LNA_PD                   0x1c
#define RH_CC110_GDO_CFG_CLK_32K                  0x27
#define RH_CC110_GDO_CFG_CHIP_RDYN                0x29
#define RH_CC110_GDO_CFG_XOSC_STABLE              0x2b
#define RH_CC110_GDO_CFG_HIGH_IMPEDANCE           0x2e
#define RH_CC110_GDO_CFG_0                        0x2f
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_1           0x30
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_1_5         0x31
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_2           0x32
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_3           0x33
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_4           0x34
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_6           0x35
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_8           0x36
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_12          0x37
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_16          0x38
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_24          0x39
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_32          0x3a
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_48          0x3b
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_64          0x3c
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_96          0x3d
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_128         0x3e
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_192         0x3f

// #define RH_CC110_REG_03_FIFOTHR                0x03
#define RH_CC110_ADC_RETENTION                    0x80

#define RH_CC110_CLOSE_IN_RX                      0x30
#define RH_CC110_CLOSE_IN_RX_0DB                  0x00
#define RH_CC110_CLOSE_IN_RX_6DB                  0x10
#define RH_CC110_CLOSE_IN_RX_12DB                 0x20
#define RH_CC110_CLOSE_IN_RX_18DB                 0x30

#define RH_CC110_FIFO_THR                         0x0f

// #define RH_CC110_REG_04_SYNC1                  0x04
// #define RH_CC110_REG_05_SYNC0                  0x05
// #define RH_CC110_REG_06_PKTLEN                 0x06
// #define RH_CC110_REG_07_PKTCTRL1               0x07
#define RH_CC110_CRC_AUTOFLUSH                    0x08
#define RH_CC110_APPEND_STATUS                    0x04
#define RH_CC110_ADDR_CHK                         0x03
// can or the next 2:
#define RH_CC110_ADDR_CHK_ADDRESS                 0x01
#define RH_CC110_ADDR_CHK_BROADCAST               0x02


// #define RH_CC110_REG_08_PKTCTRL0               0x08
#define RH_CC110_PKT_FORMAT                       0x30
#define RH_CC110_PKT_FORMAT_NORMAL                0x00
#define RH_CC110_PKT_FORMAT_SYNC_SERIAL           0x10
#define RH_CC110_PKT_FORMAT_RANDOM_TX             0x20
#define RH_CC110_PKT_FORMAT_ASYNC_SERIAL          0x30

#define RH_CC110_CRC_EN                           0x04

#define RH_CC110_LENGTH_CONFIG                    0x03
#define RH_CC110_LENGTH_CONFIG_FIXED              0x00
#define RH_CC110_LENGTH_CONFIG_VARIABLE           0x01
#define RH_CC110_LENGTH_CONFIG_INFINITE           0x02

// #define RH_CC110_REG_09_ADDR                   0x09
// #define RH_CC110_REG_0A_CHANNR                 0x0a
// #define RH_CC110_REG_0B_FSCTRL1                0x0b
// #define RH_CC110_REG_0C_FSCTRL0                0x0c
// #define RH_CC110_REG_0D_FREQ2                  0x0d
// #define RH_CC110_REG_0E_FREQ1                  0x0e
// #define RH_CC110_REG_0F_FREQ0                  0x0f
// #define RH_CC110_REG_10_MDMCFG4                0x10
#define RH_CC110_CHANBW_E                         0xc0
#define RH_CC110_CHANBW_M                         0x30
#define RH_CC110_DRATE_E                          0x0f

// #define RH_CC110_REG_11_MDMCFG3                0x11
// #define RH_CC110_REG_12_MDMCFG2                0x12
#define RH_CC110_DEM_DCFILT_OFF                   0x80
#define RH_CC110_MOD_FORMAT                       0x70
#define RH_CC110_MOD_FORMAT_2FSK                  0x00
#define RH_CC110_MOD_FORMAT_GFSK                  0x10
#define RH_CC110_MOD_FORMAT_OOK                   0x30
#define RH_CC110_MOD_FORMAT_4FSK                  0x40
#define RH_CC110_MANCHESTER_EN                    0x08
#define RH_CC110_SYNC_MODE                        0x07
#define RH_CC110_SYNC_MODE_NONE                   0x00
#define RH_CC110_SYNC_MODE_15_16                  0x01
#define RH_CC110_SYNC_MODE_16_16                  0x02
#define RH_CC110_SYNC_MODE_30_32                  0x03
#define RH_CC110_SYNC_MODE_NONE_CARRIER           0x04
#define RH_CC110_SYNC_MODE_15_16_CARRIER          0x05
#define RH_CC110_SYNC_MODE_16_16_CARRIER          0x06
#define RH_CC110_SYNC_MODE_30_32_CARRIER          0x07

// #define RH_CC110_REG_13_MDMCFG1                0x13
#define RH_CC110_NUM_PREAMBLE                     0x70
#define RH_CC110_NUM_PREAMBLE_2                   0x00
#define RH_CC110_NUM_PREAMBLE_3                   0x10
#define RH_CC110_NUM_PREAMBLE_4                   0x20
#define RH_CC110_NUM_PREAMBLE_6                   0x30
#define RH_CC110_NUM_PREAMBLE_8                   0x40
#define RH_CC110_NUM_PREAMBLE_12                  0x50
#define RH_CC110_NUM_PREAMBLE_16                  0x60
#define RH_CC110_NUM_PREAMBLE_24                  0x70

#define RH_CC110_CHANSPC_E                        0x03

// #define RH_CC110_REG_14_MDMCFG0                0x14
// #define RH_CC110_REG_15_DEVIATN                0x15
#define RH_CC110_DEVIATION_E                      0x70
#define RH_CC110_DEVIATION_M                      0x07

// #define RH_CC110_REG_16_MCSM2                  0x16
#define RH_CC110_RX_TIME_RSSI                     0x10

// #define RH_CC110_REG_17_MCSM1                  0x17
#define RH_CC110_CCA_MODE                         0x30
#define RH_CC110_CCA_MODE_ALWAYS                  0x00
#define RH_CC110_CCA_MODE_RSSI                    0x10
#define RH_CC110_CCA_MODE_PACKET                  0x20
#define RH_CC110_CCA_MODE_RSSI_PACKET             0x30
#define RH_CC110_RXOFF_MODE                       0x0c
#define RH_CC110_RXOFF_MODE_IDLE                  0x00
#define RH_CC110_RXOFF_MODE_FSTXON                0x04
#define RH_CC110_RXOFF_MODE_TX                    0x08
#define RH_CC110_RXOFF_MODE_RX                    0x0c
#define RH_CC110_TXOFF_MODE                       0x03
#define RH_CC110_TXOFF_MODE_IDLE                  0x00
#define RH_CC110_TXOFF_MODE_FSTXON                0x01
#define RH_CC110_TXOFF_MODE_TX                    0x02
#define RH_CC110_TXOFF_MODE_RX                    0x03

// #define RH_CC110_REG_18_MCSM0                  0x18
#define RH_CC110_FS_AUTOCAL                       0x30
#define RH_CC110_FS_AUTOCAL_NEVER                 0x00
#define RH_CC110_FS_AUTOCAL_FROM_IDLE             0x10
#define RH_CC110_FS_AUTOCAL_TO_IDLE               0x20
#define RH_CC110_FS_AUTOCAL_TO_IDLE_4             0x30
#define RH_CC110_PO_TIMEOUT                       0x0c
#define RH_CC110_PO_TIMEOUT_1                     0x00
#define RH_CC110_PO_TIMEOUT_16                    0x04
#define RH_CC110_PO_TIMEOUT_64                    0x08
#define RH_CC110_PO_TIMEOUT_256                   0x0c
#define RH_CC110_XOSC_FORCE_ON                    0x01

// #define RH_CC110_REG_19_FOCCFG                 0x19
#define RH_CC110_FOC_BS_CS_GATE                   0x20
#define RH_CC110_FOC_PRE_K                        0x18
#define RH_CC110_FOC_PRE_K_0                      0x00
#define RH_CC110_FOC_PRE_K_1                      0x08
#define RH_CC110_FOC_PRE_K_2                      0x10
#define RH_CC110_FOC_PRE_K_3                      0x18
#define RH_CC110_FOC_POST_K                       0x04
#define RH_CC110_FOC_LIMIT                        0x03
#define RH_CC110_FOC_LIMIT_0                      0x00
#define RH_CC110_FOC_LIMIT_8                      0x01
#define RH_CC110_FOC_LIMIT_4                      0x02
#define RH_CC110_FOC_LIMIT_2                      0x03

// #define RH_CC110_REG_1A_BSCFG                  0x1a
#define RH_CC110_BS_PRE_K                         0xc0
#define RH_CC110_BS_PRE_K_1                       0x00
#define RH_CC110_BS_PRE_K_2                       0x40
#define RH_CC110_BS_PRE_K_3                       0x80
#define RH_CC110_BS_PRE_K_4                       0xc0
#define RH_CC110_BS_PRE_KP                        0x30
#define RH_CC110_BS_PRE_KP_1                      0x00
#define RH_CC110_BS_PRE_KP_2                      0x10
#define RH_CC110_BS_PRE_KP_3                      0x20
#define RH_CC110_BS_PRE_KP_4                      0x30
#define RH_CC110_BS_POST_KI                       0x08
#define RH_CC110_BS_POST_KP                       0x04
#define RH_CC110_BS_LIMIT                         0x03
#define RH_CC110_BS_LIMIT_0                       0x00
#define RH_CC110_BS_LIMIT_3                       0x01
#define RH_CC110_BS_LIMIT_6                       0x02
#define RH_CC110_BS_LIMIT_12                      0x03

// #define RH_CC110_REG_1B_AGCCTRL2               0x1b
#define RH_CC110_MAX_DVA_GAIN                     0xc0
#define RH_CC110_MAX_DVA_GAIN_ALL                 0x00
#define RH_CC110_MAX_DVA_GAIN_ALL_LESS_1          0x40
#define RH_CC110_MAX_DVA_GAIN_ALL_LESS_2          0x80
#define RH_CC110_MAX_DVA_GAIN_ALL_LESS_3          0xc0
#define RH_CC110_MAX_LNA_GAIN                     0x38

#define RH_CC110_MAGN_TARGET                      0x07
#define RH_CC110_MAGN_TARGET_24DB                 0x00
#define RH_CC110_MAGN_TARGET_27DB                 0x01
#define RH_CC110_MAGN_TARGET_30DB                 0x02
#define RH_CC110_MAGN_TARGET_33DB                 0x03
#define RH_CC110_MAGN_TARGET_36DB                 0x04
#define RH_CC110_MAGN_TARGET_38DB                 0x05
#define RH_CC110_MAGN_TARGET_40DB                 0x06
#define RH_CC110_MAGN_TARGET_42DB                 0x07

// #define RH_CC110_REG_1C_AGCCTRL1               0x1c
#define RH_CC110_AGC_LNA_PRIORITY                 0x40
#define RH_CC110_CARRIER_SENSE_REL_THR            0x30
#define RH_CC110_CARRIER_SENSE_REL_THR_0DB        0x00
#define RH_CC110_CARRIER_SENSE_REL_THR_6DB        0x10
#define RH_CC110_CARRIER_SENSE_REL_THR_10DB       0x20
#define RH_CC110_CARRIER_SENSE_REL_THR_14DB       0x30
#define RH_CC110_CARRIER_SENSE_ABS_THR            0x0f

// #define RH_CC110_REG_1D_AGCCTRL0               0x1d
#define RH_CC110_HYST_LEVEL                       0xc0
#define RH_CC110_HYST_LEVEL_NONE                  0x00
#define RH_CC110_HYST_LEVEL_LOW                   0x40
#define RH_CC110_HYST_LEVEL_MEDIUM                0x80
#define RH_CC110_HYST_LEVEL_HIGH                  0xc0
#define RH_CC110_WAIT_TIME                        0x30
#define RH_CC110_WAIT_TIME_8                      0x00
#define RH_CC110_WAIT_TIME_16                     0x10
#define RH_CC110_WAIT_TIME_24                     0x20
#define RH_CC110_WAIT_TIME_32                     0x30
#define RH_CC110_AGC_FREEZE                       0x0c
#define RH_CC110_AGC_FILTER_LENGTH                0x03
#define RH_CC110_AGC_FILTER_LENGTH_8              0x00
#define RH_CC110_AGC_FILTER_LENGTH_16             0x01
#define RH_CC110_AGC_FILTER_LENGTH_32             0x02
#define RH_CC110_AGC_FILTER_LENGTH_64             0x03

// #define RH_CC110_REG_1E_WOREVT1                0x1e
// #define RH_CC110_REG_1F_WOREVT0                0x1f
// #define RH_CC110_REG_20_WORCTRL                0x20
// #define RH_CC110_REG_21_FREND1                 0x21
#define RH_CC110_LNA_CURRENT                      0xc0
#define RH_CC110_LNA2MIX_CURRENT                  0x30
#define RH_CC110_LODIV_BUF_CURRENT_RX             0x0c
#define RH_CC110_MIX_CURRENT                      0x03

// #define RH_CC110_REG_22_FREND0                 0x22
#define RH_CC110_LODIV_BUF_CURRENT_TX             0x30
#define RH_CC110_PA_POWER                         0x07

// #define RH_CC110_REG_23_FSCAL3                 0x23
#define RH_CC110_FSCAL3_7_6                       0xc0
#define RH_CC110_CHP_CURR_CAL_EN                  0x30
#define RH_CC110_FSCAL3_3_0                       0x0f

// #define RH_CC110_REG_24_FSCAL2                 0x24
#define RH_CC110_VCO_CORE_H_EN                    0x20
#define RH_CC110_FSCAL2                           0x1f

// #define RH_CC110_REG_25_FSCAL1                 0x25
#define RH_CC110_FSCAL1                           0x3f

// #define RH_CC110_REG_26_FSCAL0                 0x26
#define RH_CC110_FSCAL0                           0x7f

// #define RH_CC110_REG_27_RCCTRL1                0x28
// #define RH_CC110_REG_28_RCCTRL0                0x29
// #define RH_CC110_REG_29_FSTEST                 0x2a
// #define RH_CC110_REG_2A_PTEST                  0x2b
// #define RH_CC110_REG_2B_AGCTEST                0x2c
// #define RH_CC110_REG_2C_TEST2                  0x2c
// #define RH_CC110_REG_2D_TEST1                  0x2d
// #define RH_CC110_REG_2E_TEST0                  0x2e
#define RH_CC110_TEST0_7_2                        0xfc
#define RH_CC110_VCO_SEL_CAL_EN                   0x02
#define RH_CC110_TEST0_0                          0x01

// #define RH_CC110_REG_30_PARTNUM                0x30
// #define RH_CC110_REG_31_VERSION                0x31
// #define RH_CC110_REG_32_FREQEST                0x32
// #define RH_CC110_REG_33_CRC_REG                0x33
#define RH_CC110_CRC_REG_CRC_OK                   0x80

// #define RH_CC110_REG_34_RSSI                   0x34
// #define RH_CC110_REG_35_MARCSTATE              0x35
#define RH_CC110_MARC_STATE                       0x1f
#define RH_CC110_MARC_STATE_SLEEP                 0x00
#define RH_CC110_MARC_STATE_IDLE                  0x01
#define RH_CC110_MARC_STATE_XOFF                  0x02
#define RH_CC110_MARC_STATE_VCOON_MC              0x03
#define RH_CC110_MARC_STATE_REGON_MC              0x04
#define RH_CC110_MARC_STATE_MANCAL                0x05
#define RH_CC110_MARC_STATE_VCOON                 0x06
#define RH_CC110_MARC_STATE_REGON                 0x07
#define RH_CC110_MARC_STATE_STARTCAL              0x08
#define RH_CC110_MARC_STATE_BWBOOST               0x09
#define RH_CC110_MARC_STATE_FS_LOCK               0x0a
#define RH_CC110_MARC_STATE_IFADCON               0x0b
#define RH_CC110_MARC_STATE_ENDCAL                0x0c
#define RH_CC110_MARC_STATE_RX                    0x0d
#define RH_CC110_MARC_STATE_RX_END                0x0e
#define RH_CC110_MARC_STATE_RX_RST                0x0f
#define RH_CC110_MARC_STATE_TXRX_SWITCH           0x10
#define RH_CC110_MARC_STATE_RXFIFO_OVERFLOW       0x11
#define RH_CC110_MARC_STATE_FSTXON                0x12
#define RH_CC110_MARC_STATE_TX                    0x13
#define RH_CC110_MARC_STATE_TX_END                0x14
#define RH_CC110_MARC_STATE_RXTX_SWITCH           0x15
#define RH_CC110_MARC_STATE_TXFIFO_UNDERFLOW      0x16

// #define RH_CC110_REG_38_PKTSTATUS              0x38
#define RH_CC110_PKTSTATUS_CRC_OK                 0x80
#define RH_CC110_PKTSTATUS_CS                     0x40
#define RH_CC110_PKTSTATUS_CCA                    0x10
#define RH_CC110_PKTSTATUS_SFD                    0x08
#define RH_CC110_PKTSTATUS_GDO2                   0x04
#define RH_CC110_PKTSTATUS_GDO0                   0x01

// #define RH_CC110_REG_3A_TXBYTES                0x3a
#define RH_CC110_TXFIFO_UNDERFLOW                 0x80
#define RH_CC110_NUM_TXBYTES                      0x7f

// #define RH_CC110_REG_3B_RXBYTES                0x3b
#define RH_CC110_RXFIFO_UNDERFLOW                 0x80
#define RH_CC110_NUM_RXBYTES                      0x7f

/////////////////////////////////////////////////////////////////////
/// \class RH_CC110 RH_CC110.h <RH_CC110.h>
/// \brief Send and receive unaddressed, unreliable, datagrams by Texas Instruments CC110L and compatible transceivers and modules.
///
/// The TI CC110L is a low cost tranceiver chip capable of 300 to 928MHz and with a wide range of modulation types and speeds.
/// The chip is typically provided on a module that also includes the antenna and coupling hardware
/// and is therefore capable of a more restricted frequency range.
///
/// Supported modules include:
/// - Anaren AIR BoosterPack 430BOOST-CC110L 
///
/// This base class provides basic functions for sending and receiving unaddressed, unreliable datagrams
/// of arbitrary length to 59 octets per packet at a selected data rate and modulation type. 
/// Use one of the Manager classes to get addressing and 
/// acknowledgement reliability, routing, meshes etc.
///
/// Naturally, for any 2 radios to communicate that must be configured to use the same frequency and 
/// data rate, and with identical network addresses.
///
/// Several CC110L modules can be connected to an Arduino, permitting the construction of translators
/// and frequency changers, etc.
///
/// Several GFSK modulation schemes are provided and may be selected by calling setModemConfig(). No FSK or OOK 
/// modulation schemes are provided though the implementor may configure the mnodem characteristics directly 
/// by calling setModemRegisters().
///
/// Implementation based on:
/// http://www.ti.com/lit/ds/symlink/cc110l.pdf
/// and
/// https://www.anaren.com/air/cc110l-air-module-boosterpack-embedded-antenna-module-anaren
///
/// \par Crystal Frequency
///
/// Modules based on the CC110L may contain a crystal oscillator with one of 2 possible frequencies: 26MHz or 27MHz.
/// A number of radio configuration parameters (including carrier frequency and data rates) depend on the
/// crystal oscillator frequency. The chip has no knowledge of the frequency, so it is up to the implementer
/// to tell the driver the oscillator frequency by passing in the appropriate value of is27MHz to the constructor (default 26MHz)
/// or by calling setIs27MHz() before calling init().
/// Failure to correctly set this flag will cause incorrect frequency and modulation
/// characteristics to be used. 
///
/// Caution: it is not easy to determine what the actual crystal frequency is on some modules. For example, 
/// the documentation for the Anaren BoosterPack indictes a 26MHz crystal, but measurements on the devices delivered here
/// indicate a 27MHz crystal is actually installed. TI recommend 27MHz for 
///
/// \par Packet Format
///
/// - 2 octets sync (a configurable network address)
/// - 1 octet message length
/// - 4 to 63 octets of payload consisting of:
///   - 1 octet TO header
///   - 1 octet FROM header
///   - 1 octet ID header
///   - 1 octet FLAGS header
///   - 0 to 59 octets of user message
/// - 2 octets CRC 
///
/// \par Connecting CC110L to Arduino
/// 
/// Warning: the CC110L is a 3.3V part, and exposing it to 5V on any pin will damage it. Ensure you are using a 3.3V 
/// MCU or use level shifters. We tested with Teensy 3.1.
///
/// The electrical connection between a CC110L module and the Arduino or other processor
/// require 3.3V, the 3 x SPI pins (SCK, SDI, SDO), 
/// a Chip Select pin and an Interrupt pin.
/// Examples below assume the Anaren BoosterPack. Caution: the pin numbering on the Anaren BoosterPack
/// is a bit counter-intuitive: the direction of number on J1 is the reverse of J2. Check the pin numbers
/// stenciled on the front of the board to be sure.
///
/// \code
///                 Teensy 3.1   CC110L pin name         Anaren BoosterPack pin
///                 3.3V---------VDD   (3.3V in)              J1-1
///          SS pin D10----------CSn   (chip select in)       J2-8
///         SCK pin D13----------SCLK  (SPI clock in)         J1-7
///        MOSI pin D11----------MOSI  (SPI data in)          J2-5
///        MISO pin D12----------MISO  (SPI data out)         J2-4
///                 D2-----------GDO0  (Interrupt output)     J2-9
///                 GND----------GND   (ground in)            J2-10
/// \endcode
/// and use the default RH_CC110 constructor. You can use other pins by passing the appropriate arguments
/// to the RH_CC110 constructor, depending on what your MCU supports.
///
/// For the Particle Photon:
/// \code
///                 Photon   CC110L pin name         Anaren BoosterPack pin
///                 3.3V---------VDD   (3.3V in)              J1-1
///          SS pin A2-----------CSn   (chip select in)       J2-8
///         SCK pin A3-----------SCLK  (SPI clock in)         J1-7
///        MOSI pin A5-----------MOSI  (SPI data in)          J2-5
///        MISO pin A4-----------MISO  (SPI data out)         J2-4
///                 D2-----------GDO0  (Interrupt output)     J2-9
///                 GND----------GND   (ground in)            J2-10
/// \endcode
/// and use the default RH_CC110 constructor. You can use other pins by passing the appropriate arguments
/// to the RH_CC110 constructor, depending on what your MCU supports.
///
/// \par Example programs
///
/// Several example programs are provided.
///
/// \par Radio operating strategy and defaults
///
/// The radio is enabled at all times and switched between RX, TX and IDLE modes.
/// When RX is enabled (by calling available() or setModeRx()) the radio will stay in RX mode until a 
/// valid CRC correct message addressed to thiis node is received, when it will transition to IDLE.
/// When TX is enabled (by calling send()) it will stay in TX mode until the message has ben sent
/// and waitPacketSent() is called when it wil transition to IDLE 
///(this radio has no 'packet sent' interrupt that could be used, so polling
/// with waitPacketSent() is required
///
/// The modulation schemes supported include the GFSK schemes provided by default in the TI SmartRF Suite.
/// This software allows you to get the correct register values for diferent modulation schemes. All the modulation
/// schemes prvided in the driver are based on the recommended register values given by SmartRF.
/// Other schemes such a 2-FSK, 4-FSK and OOK are suported by the chip, but canned configurations are not provided with this driver. 
/// The implementer may choose to create their own modem configurations and pass them to setModemRegisters().
///
class RH_CC110 : public RHNRFSPIDriver
{
public:

    /// \brief Defines register configuration values for a desired modulation
    ///
    /// Defines values for various configuration fields and registers to 
    /// achieve a desired modulation speed and frequency deviation.
    typedef struct
    {
	uint8_t reg_0b;    ///< RH_CC110_REG_0B_FSCTRL1
	uint8_t reg_0c;    ///< RH_CC110_REG_0C_FSCTRL0
	uint8_t reg_10;    ///< RH_CC110_REG_10_MDMCFG4
	uint8_t reg_11;    ///< RH_CC110_REG_11_MDMCFG3
	uint8_t reg_12;    ///< RH_CC110_REG_12_MDMCFG2
	uint8_t reg_15;    ///< RH_CC110_REG_15_DEVIATN
	uint8_t reg_19;    ///< RH_CC110_REG_19_FOCCFG
	uint8_t reg_1a;    ///< RH_CC110_REG_1A_BSCFG
	uint8_t reg_1b;    ///< RH_CC110_REG_1B_AGCCTRL2
	uint8_t reg_1c;    ///< RH_CC110_REG_1C_AGCCTRL1
	uint8_t reg_1d;    ///< RH_CC110_REG_1D_AGCCTRL0
	uint8_t reg_21;    ///< RH_CC110_REG_21_FREND1
	uint8_t reg_22;    ///< RH_CC110_REG_22_FREND0
	uint8_t reg_23;    ///< RH_CC110_REG_23_FSCAL3
	uint8_t reg_24;    ///< RH_CC110_REG_24_FSCAL2
	uint8_t reg_25;    ///< RH_CC110_REG_25_FSCAL1
	uint8_t reg_26;    ///< RH_CC110_REG_26_FSCAL0
	uint8_t reg_2c;    ///< RH_CC110_REG_2C_TEST2
	uint8_t reg_2d;    ///< RH_CC110_REG_2D_TEST1
	uint8_t reg_2e;    ///< RH_CC110_REG_2E_TEST0
    } ModemConfig;


    /// Choices for setModemConfig() for a selected subset of common modulation types,
    /// and data rates. If you need another configuration, use the register calculator.
    /// and call setModemRegisters() with your desired settings.
    /// These are indexes into MODEM_CONFIG_TABLE. We strongly recommend you use these symbolic
    /// definitions and not their integer equivalents: its possible that new values will be
    /// introduced in later versions (though we will try to avoid it).
    /// All configs use SYNC_MODE = RH_CC110_SYNC_MODE_16_16 (2 byte sync)
    typedef enum
    {
	GFSK_Rb1_2Fd5_2 = 0,   ///< GFSK, Data Rate: 1.2kBaud, Dev: 5.2kHz, RX BW 58kHz, optimised for sensitivity
	GFSK_Rb2_4Fd5_2,       ///< GFSK, Data Rate: 2.4kBaud, Dev: 5.2kHz, RX BW 58kHz, optimised for sensitivity
	GFSK_Rb4_8Fd25_4,      ///< GFSK, Data Rate: 4.8kBaud, Dev: 25.4kHz, RX BW 100kHz, optimised for sensitivity
	GFSK_Rb10Fd19,         ///< GFSK, Data Rate: 10kBaud, Dev: 19kHz, RX BW 100kHz, optimised for sensitivity
	GFSK_Rb38_4Fd20,       ///< GFSK, Data Rate: 38.4kBaud, Dev: 20kHz, RX BW 100kHz, optimised for sensitivity
	GFSK_Rb76_8Fd32,       ///< GFSK, Data Rate: 76.8kBaud, Dev: 32kHz, RX BW 232kHz, optimised for sensitivity
	GFSK_Rb100Fd47,        ///< GFSK, Data Rate: 100kBaud, Dev: 47kHz, RX BW 325kHz, optimised for sensitivity
	GFSK_Rb250Fd127,       ///< GFSK, Data Rate: 250kBaud, Dev: 127kHz, RX BW 540kHz, optimised for sensitivity
    } ModemConfigChoice;

    /// These power outputs are based on the suggested optimum values for 
    /// multilayer inductors in the 915MHz frequency band. Per table 5-15.
    /// Caution: these enum values are indexes into PaPowerValues. 
    /// Do not change one without changing the other. Use the symbolic names, not the integer values
    typedef enum
    {
	TransmitPowerM30dBm = 0,      ///< -30dBm
	TransmitPowerM20dBm,          ///< -20dBm
	TransmitPowerM15dBm,          ///< -15dBm
	TransmitPowerM10dBm,          ///< -10dBm
	TransmitPower0dBm,            ///< 0dBm
	TransmitPower5dBm,            ///< 5dBm
	TransmitPower7dBm,            ///< 7dBm
	TransmitPower10dBm,           ///< 10dBm
    } TransmitPower;

    /// Constructor. You can have multiple instances, but each instance must have its own
    /// interrupt and slave select pin. After constructing, you must call init() to initialise the interface
    /// and the radio module. A maximum of 3 instances can co-exist on one processor, provided there are sufficient
    /// distinct interrupt lines, one for each instance.
    /// \param[in] slaveSelectPin the Arduino pin number of the output to use to select the CC110L before
    /// accessing it. Defaults to the normal SS pin for your Arduino (D10 for Diecimila, Uno etc, D53 for Mega, D10 for Maple)
    /// \param[in] interruptPin The interrupt Pin number that is connected to the CC110L GDO0 interrupt line. 
    /// Defaults to pin 2.
    /// Caution: You must specify an interrupt capable pin.
    /// On many Arduino boards, there are limitations as to which pins may be used as interrupts.
    /// On Leonardo pins 0, 1, 2 or 3. On Mega2560 pins 2, 3, 18, 19, 20, 21. On Due and Teensy, any digital pin.
    /// On other Arduinos pins 2 or 3. 
    /// See http://arduino.cc/en/Reference/attachInterrupt for more details.
    /// On Chipkit Uno32, pins 38, 2, 7, 8, 35.
    /// On other boards, any digital pin may be used.
    /// \param[in] is27MHz Set to true if your CC110 is equipped with a 27MHz crystal oscillator. Defaults to false.
    /// \param[in] spi Pointer to the SPI interface object to use. 
    ///                Defaults to the standard Arduino hardware SPI interface
    RH_CC110(uint8_t slaveSelectPin = SS, uint8_t interruptPin = 2, bool is27MHz = false, RHGenericSPI& spi = hardware_spi);

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// In particular, ensure you have called setIs27MHz(true) if your module has a 27MHz crystal oscillator.
    /// After init(), the following default characteristics are set:
    /// TxPower: TransmitPower5dBm
    /// Frequency: 915.0
    /// Modulation: GFSK_Rb1_2Fd5_2 (GFSK, Data Rate: 1.2kBaud, Dev: 5.2kHz, RX BW 58kHz, optimised for sensitivity)
    /// Sync Words: 0xd3, 0x91
    /// \return true if initialisation succeeded.
    virtual bool    init();

    /// Prints the value of all chip registers
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// For debugging purposes only.
    /// \return true on success
    bool printRegisters();

    /// Blocks until the current message (if any) 
    /// has been transmitted
    /// \return true on success, false if the chip is not in transmit mode or other transmit failure
    virtual bool waitPacketSent();

    /// Tests whether a new message is available
    /// from the Driver. 
    /// On most drivers, this will also put the Driver into RHModeRx mode until
    /// a message is actually received by the transport, when it will be returned to RHModeIdle
    /// and available() will return true.
    /// This can be called multiple times in a timeout loop
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    virtual bool    available();

    /// Turns the receiver on if it not already on (after wiaint gor any currenly transmitting message to complete).
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to the number of octets available in buf. The number be reset to the actual number of octets copied.
    /// \return true if a valid message was copied to buf. The message cannot be retreived again.
    virtual bool    recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send
    /// \return true if the message length was valid and it was correctly queued for transmit
    virtual bool    send(const uint8_t* data, uint8_t len);

    /// Returns the maximum message length 
    /// available in this Driver.
    /// \return The maximum legal message length
    virtual uint8_t maxMessageLength();

    /// If current mode is Sleep, Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
    void           setModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver. The radio will stay in Rx mode until a CRC correct message addressed to this node
    /// is received, or the ode is changed to Tx, Idle or Sleep.
    void           setModeRx();

    /// If current mode is Rx or Idle, changes it to Tx.
    /// Starts the transmitter sending the current message.
    void           setModeTx();

    /// Sets the radio into low-power sleep mode.
    /// If successful, the transport will stay in sleep mode until woken by 
    /// changing mode to idle, transmit or receive (eg by calling send(), recv(), available() etc)
    /// Caution: there is a time penalty as the radio takes a finite time to wake from sleep mode.
    /// Caution: waking up from sleep loses values from registers 0x29 through 0x2e
    /// \return true if sleep mode was successfully entered.
    virtual bool    sleep();

    /// Set the Power Amplifier power setting.
    /// The PaTable settings are based on are based on the suggested optimum values for 
    /// multilayer inductors in the 915MHz frequency band. Per table 5-15.
    /// If these values are not suitable, use setPaTable() directly.
    /// Caution: be a good neighbour and use the lowest power setting compatible with your application.
    /// Caution: Permissable power settings for your area may depend on frequency and modulation characteristics: 
    /// consult local authorities.
    /// param[in] power One of TransmitPower enum values 
    bool setTxPower(TransmitPower power);

    /// Indicates the presence of 27MHz crystal oscillator.
    /// You must indicate to the driver if your CC110L is equipped with a 27MHz crystal oscillator (26MHz is the default 
    /// in the constructor).
    /// This should be called before calling init() if you have a 27MHz crystal.
    /// It can be called after calling init() but you must reset the frequency (with setFrequency()) and modulation 
    /// (with setModemConfig()) afterwards.
    /// \param[in] is27MHz Pass true if the CC110L has a 27MHz crystal (default is true).
    void setIs27MHz(bool is27MHz = true);

    /// Sets the transmitter and receiver 
    /// centre frequency.
    /// Caution: permissable frequency bands will depend on you country and area: consult local authorities.
    /// \param[in] centre Frequency in MHz. 300.0 to 928.0
    /// \return true if the selected frquency centre is within range
    bool        setFrequency(float centre);

    /// Sets all the registers required to configure the data modem in the CC110, including the data rate, 
    /// bandwidths etc. You cas use this to configure the modem with custom configuraitons if none of the 
    /// canned configurations in ModemConfigChoice suit you.
    /// \param[in] config A ModemConfig structure containing values for the modem configuration registers.
    void           setModemRegisters(const ModemConfig* config);

    /// Select one of the predefined modem configurations. If you need a modem configuration not provided 
    /// here, use setModemRegisters() with your own ModemConfig.
    /// \param[in] index The configuration choice.
    /// \return true if index is a valid choice.
    bool        setModemConfig(ModemConfigChoice index);

    /// Sets the sync words for transmit and receive in registers RH_CC110_REG_04_SYNC1 and RH_CC110_REG_05_SYNC0.
    /// Caution: SyncWords should be set to the same 
    /// value on all nodes in your network. Nodes with different SyncWords set will never receive
    /// each others messages, so different SyncWords can be used to isolate different
    /// networks from each other. Default is { 0xd3, 0x91 }.
    /// \param[in] syncWords Array of sync words, 2 octets long
    /// \param[in] len Number of sync words to set. MUST be 2.
    void setSyncWords(const uint8_t* syncWords, uint8_t len);

    /// Sets the PaTable registers directly.
    /// Ensure you use suitable PATABLE values per Tbale 5-15 or 5-16
    /// You may need to do this to implement an OOK modulation scheme.
    void setPaTable(uint8_t* patable, uint8_t patablesize);

protected:
    /// This is a low level function to handle the interrupts for one instance of RH_RF95.
    /// Called automatically by isr*()
    /// Should not need to be called by user code.
    void           handleInterrupt();

    /// Reads a single register from the CC110L
    /// \param[in] reg Register number, one of RH_CC110_REG
    /// \return The value of the register
    uint8_t spiReadRegister(uint8_t reg);

    /// Reads a single register in burst mode.
    /// On the CC110L, some registers yield different data when read in burst mode
    /// as opposed to single byte mode.
    /// \param[in] reg Register number, one of RH_CC110_REG (burst mode readable)
    /// \return The value of the register after a burst read
    uint8_t spiBurstReadRegister(uint8_t reg);

    /// Writes to a single single register on the CC110L
    /// \param[in] reg Register number, one of RH_CC110L_REG_*
    /// \param[in] val The value to write
    /// \return returns the chip status byte per table 5.2
    uint8_t spiWriteRegister(uint8_t reg, uint8_t val);

    /// Write a number of bytes to a burst capable register
    /// \param[in] reg Register number of the first register, one of RH_CC110L_REG_*
    /// \param[in] src Array of new register values to write. Must be at least len bytes
    /// \param[in] len Number of bytes to write
    /// \return the chip status byte per table 5.2
    uint8_t  spiBurstWriteRegister(uint8_t reg, const uint8_t* src, uint8_t len);
    
    /// Examine the receive buffer to determine whether the message is for this node
    /// Sets _rxBufValid.
    void validateRxBuf();

    /// Clear our local receive buffer
    void clearRxBuf();

    /// Reads and returns the status byte by issuing the SNOP strobe
    /// \return The value of the status byte per Table 5-2
    uint8_t statusRead();

    /// Handle the TX or RX overflow state of the given status
    /// \param status The status byte read from the last SPI command
    /// \return void
    void handleOverFlows(uint8_t status);


private:
    /// Low level interrupt service routine for device connected to interrupt 0
    static void         isr0();

    /// Low level interrupt service routine for device connected to interrupt 1
    static void         isr1();

    /// Low level interrupt service routine for device connected to interrupt 1
    static void         isr2();

    /// Array of instances connected to interrupts 0 and 1
    static RH_CC110*     _deviceForInterrupt[];

    /// Index of next interrupt number to use in _deviceForInterrupt
    static uint8_t      _interruptCount;

    /// The configured interrupt pin connected to this instance
    uint8_t             _interruptPin;

    /// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
    /// else 0xff
    uint8_t             _myInterruptIndex;

    /// Number of octets in the buffer
    volatile uint8_t    _bufLen;
    
    /// The receiver/transmitter buffer
    /// Allow for 2 status bytes so we can read packet RSSI
    uint8_t             _buf[RH_CC110_MAX_PAYLOAD_LEN + 2];

    /// True when there is a valid message in the buffer
    volatile bool       _rxBufValid;

    /// True if crystal oscillator is 26 MHz, not 26MHz.
    bool                _is27MHz;
};

/// @example cc110_client.ino
/// @example cc110_server.ino

#endif
