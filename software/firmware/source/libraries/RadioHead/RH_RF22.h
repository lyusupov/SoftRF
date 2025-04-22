// RH_RF22.h
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2011 Mike McCauley
// $Id: RH_RF22.h,v 1.39 2020/08/04 09:02:14 mikem Exp $
//

#ifndef RH_RF22_h
#define RH_RF22_h

#include <RHGenericSPI.h>
#include <RHSPIDriver.h>

// This is the maximum number of interrupts the library can support
// Most Arduinos can handle 2, Megas can handle more
#define RH_RF22_NUM_INTERRUPTS 3

// This is the bit in the SPI address that marks it as a write
#define RH_RF22_SPI_WRITE_MASK 0x80

// This is the maximum message length that can be supported by this library. Limited by
// the single message length octet in the header. 
// Yes, 255 is correct even though the FIFO size in the RF22 is only
// 64 octets. We use interrupts to refill the Tx FIFO during transmission and to empty the
// Rx FIFO during reception
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
#ifndef RH_RF22_MAX_MESSAGE_LEN
//#define RH_RF22_MAX_MESSAGE_LEN 255
#define RH_RF22_MAX_MESSAGE_LEN 50
#endif

// Max number of octets the RF22 Rx and Tx FIFOs can hold
#define RH_RF22_FIFO_SIZE 64

// These values we set for FIFO thresholds (4, 55) are actually the same as the POR values
#define RH_RF22_TXFFAEM_THRESHOLD 4
#define RH_RF22_RXFFAFULL_THRESHOLD 55

// Number of registers to be passed to setModemConfig(). Obsolete.
#define RH_RF22_NUM_MODEM_CONFIG_REGS 18

// Register names
#define RH_RF22_REG_00_DEVICE_TYPE                         0x00
#define RH_RF22_REG_01_VERSION_CODE                        0x01
#define RH_RF22_REG_02_DEVICE_STATUS                       0x02
#define RH_RF22_REG_03_INTERRUPT_STATUS1                   0x03
#define RH_RF22_REG_04_INTERRUPT_STATUS2                   0x04
#define RH_RF22_REG_05_INTERRUPT_ENABLE1                   0x05
#define RH_RF22_REG_06_INTERRUPT_ENABLE2                   0x06
#define RH_RF22_REG_07_OPERATING_MODE1                     0x07
#define RH_RF22_REG_08_OPERATING_MODE2                     0x08
#define RH_RF22_REG_09_OSCILLATOR_LOAD_CAPACITANCE         0x09
#define RH_RF22_REG_0A_UC_OUTPUT_CLOCK                     0x0a
#define RH_RF22_REG_0B_GPIO_CONFIGURATION0                 0x0b
#define RH_RF22_REG_0C_GPIO_CONFIGURATION1                 0x0c
#define RH_RF22_REG_0D_GPIO_CONFIGURATION2                 0x0d
#define RH_RF22_REG_0E_IO_PORT_CONFIGURATION               0x0e
#define RH_RF22_REG_0F_ADC_CONFIGURATION                   0x0f
#define RH_RF22_REG_10_ADC_SENSOR_AMP_OFFSET               0x10
#define RH_RF22_REG_11_ADC_VALUE                           0x11
#define RH_RF22_REG_12_TEMPERATURE_SENSOR_CALIBRATION      0x12
#define RH_RF22_REG_13_TEMPERATURE_VALUE_OFFSET            0x13
#define RH_RF22_REG_14_WAKEUP_TIMER_PERIOD1                0x14
#define RH_RF22_REG_15_WAKEUP_TIMER_PERIOD2                0x15
#define RH_RF22_REG_16_WAKEUP_TIMER_PERIOD3                0x16
#define RH_RF22_REG_17_WAKEUP_TIMER_VALUE1                 0x17
#define RH_RF22_REG_18_WAKEUP_TIMER_VALUE2                 0x18
#define RH_RF22_REG_19_LDC_MODE_DURATION                   0x19
#define RH_RF22_REG_1A_LOW_BATTERY_DETECTOR_THRESHOLD      0x1a
#define RH_RF22_REG_1B_BATTERY_VOLTAGE_LEVEL               0x1b
#define RH_RF22_REG_1C_IF_FILTER_BANDWIDTH                 0x1c
#define RH_RF22_REG_1D_AFC_LOOP_GEARSHIFT_OVERRIDE         0x1d
#define RH_RF22_REG_1E_AFC_TIMING_CONTROL                  0x1e
#define RH_RF22_REG_1F_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE   0x1f
#define RH_RF22_REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE    0x20
#define RH_RF22_REG_21_CLOCK_RECOVERY_OFFSET2              0x21
#define RH_RF22_REG_22_CLOCK_RECOVERY_OFFSET1              0x22
#define RH_RF22_REG_23_CLOCK_RECOVERY_OFFSET0              0x23
#define RH_RF22_REG_24_CLOCK_RECOVERY_TIMING_LOOP_GAIN1    0x24
#define RH_RF22_REG_25_CLOCK_RECOVERY_TIMING_LOOP_GAIN0    0x25
#define RH_RF22_REG_26_RSSI                                0x26
#define RH_RF22_REG_27_RSSI_THRESHOLD                      0x27
#define RH_RF22_REG_28_ANTENNA_DIVERSITY1                  0x28
#define RH_RF22_REG_29_ANTENNA_DIVERSITY2                  0x29
#define RH_RF22_REG_2A_AFC_LIMITER                         0x2a
#define RH_RF22_REG_2B_AFC_CORRECTION_READ                 0x2b
#define RH_RF22_REG_2C_OOK_COUNTER_VALUE_1                 0x2c
#define RH_RF22_REG_2D_OOK_COUNTER_VALUE_2                 0x2d
#define RH_RF22_REG_2E_SLICER_PEAK_HOLD                    0x2e
#define RH_RF22_REG_30_DATA_ACCESS_CONTROL                 0x30
#define RH_RF22_REG_31_EZMAC_STATUS                        0x31
#define RH_RF22_REG_32_HEADER_CONTROL1                     0x32
#define RH_RF22_REG_33_HEADER_CONTROL2                     0x33
#define RH_RF22_REG_34_PREAMBLE_LENGTH                     0x34
#define RH_RF22_REG_35_PREAMBLE_DETECTION_CONTROL1         0x35
#define RH_RF22_REG_36_SYNC_WORD3                          0x36
#define RH_RF22_REG_37_SYNC_WORD2                          0x37
#define RH_RF22_REG_38_SYNC_WORD1                          0x38
#define RH_RF22_REG_39_SYNC_WORD0                          0x39
#define RH_RF22_REG_3A_TRANSMIT_HEADER3                    0x3a
#define RH_RF22_REG_3B_TRANSMIT_HEADER2                    0x3b
#define RH_RF22_REG_3C_TRANSMIT_HEADER1                    0x3c
#define RH_RF22_REG_3D_TRANSMIT_HEADER0                    0x3d
#define RH_RF22_REG_3E_PACKET_LENGTH                       0x3e
#define RH_RF22_REG_3F_CHECK_HEADER3                       0x3f
#define RH_RF22_REG_40_CHECK_HEADER2                       0x40
#define RH_RF22_REG_41_CHECK_HEADER1                       0x41
#define RH_RF22_REG_42_CHECK_HEADER0                       0x42
#define RH_RF22_REG_43_HEADER_ENABLE3                      0x43
#define RH_RF22_REG_44_HEADER_ENABLE2                      0x44
#define RH_RF22_REG_45_HEADER_ENABLE1                      0x45
#define RH_RF22_REG_46_HEADER_ENABLE0                      0x46
#define RH_RF22_REG_47_RECEIVED_HEADER3                    0x47
#define RH_RF22_REG_48_RECEIVED_HEADER2                    0x48
#define RH_RF22_REG_49_RECEIVED_HEADER1                    0x49
#define RH_RF22_REG_4A_RECEIVED_HEADER0                    0x4a
#define RH_RF22_REG_4B_RECEIVED_PACKET_LENGTH              0x4b
#define RH_RF22_REG_50_ANALOG_TEST_BUS_SELECT              0x50
#define RH_RF22_REG_51_DIGITAL_TEST_BUS_SELECT             0x51
#define RH_RF22_REG_52_TX_RAMP_CONTROL                     0x52
#define RH_RF22_REG_53_PLL_TUNE_TIME                       0x53
#define RH_RF22_REG_55_CALIBRATION_CONTROL                 0x55
#define RH_RF22_REG_56_MODEM_TEST                          0x56
#define RH_RF22_REG_57_CHARGE_PUMP_TEST                    0x57
#define RH_RF22_REG_58_CHARGE_PUMP_CURRENT_TRIMMING        0x58
#define RH_RF22_REG_59_DIVIDER_CURRENT_TRIMMING            0x59
#define RH_RF22_REG_5A_VCO_CURRENT_TRIMMING                0x5a
#define RH_RF22_REG_5B_VCO_CALIBRATION                     0x5b
#define RH_RF22_REG_5C_SYNTHESIZER_TEST                    0x5c
#define RH_RF22_REG_5D_BLOCK_ENABLE_OVERRIDE1              0x5d
#define RH_RF22_REG_5E_BLOCK_ENABLE_OVERRIDE2              0x5e
#define RH_RF22_REG_5F_BLOCK_ENABLE_OVERRIDE3              0x5f
#define RH_RF22_REG_60_CHANNEL_FILTER_COEFFICIENT_ADDRESS  0x60
#define RH_RF22_REG_61_CHANNEL_FILTER_COEFFICIENT_VALUE    0x61
#define RH_RF22_REG_62_CRYSTAL_OSCILLATOR_POR_CONTROL      0x62
#define RH_RF22_REG_63_RC_OSCILLATOR_COARSE_CALIBRATION    0x63
#define RH_RF22_REG_64_RC_OSCILLATOR_FINE_CALIBRATION      0x64
#define RH_RF22_REG_65_LDO_CONTROL_OVERRIDE                0x65
#define RH_RF22_REG_66_LDO_LEVEL_SETTINGS                  0x66
#define RH_RF22_REG_67_DELTA_SIGMA_ADC_TUNING1             0x67
#define RH_RF22_REG_68_DELTA_SIGMA_ADC_TUNING2             0x68
#define RH_RF22_REG_69_AGC_OVERRIDE1                       0x69
#define RH_RF22_REG_6A_AGC_OVERRIDE2                       0x6a
#define RH_RF22_REG_6B_GFSK_FIR_FILTER_COEFFICIENT_ADDRESS 0x6b
#define RH_RF22_REG_6C_GFSK_FIR_FILTER_COEFFICIENT_VALUE   0x6c
#define RH_RF22_REG_6D_TX_POWER                            0x6d
#define RH_RF22_REG_6E_TX_DATA_RATE1                       0x6e
#define RH_RF22_REG_6F_TX_DATA_RATE0                       0x6f
#define RH_RF22_REG_70_MODULATION_CONTROL1                 0x70
#define RH_RF22_REG_71_MODULATION_CONTROL2                 0x71
#define RH_RF22_REG_72_FREQUENCY_DEVIATION                 0x72
#define RH_RF22_REG_73_FREQUENCY_OFFSET1                   0x73
#define RH_RF22_REG_74_FREQUENCY_OFFSET2                   0x74
#define RH_RF22_REG_75_FREQUENCY_BAND_SELECT               0x75
#define RH_RF22_REG_76_NOMINAL_CARRIER_FREQUENCY1          0x76
#define RH_RF22_REG_77_NOMINAL_CARRIER_FREQUENCY0          0x77
#define RH_RF22_REG_79_FREQUENCY_HOPPING_CHANNEL_SELECT    0x79
#define RH_RF22_REG_7A_FREQUENCY_HOPPING_STEP_SIZE         0x7a
#define RH_RF22_REG_7C_TX_FIFO_CONTROL1                    0x7c
#define RH_RF22_REG_7D_TX_FIFO_CONTROL2                    0x7d
#define RH_RF22_REG_7E_RX_FIFO_CONTROL                     0x7e
#define RH_RF22_REG_7F_FIFO_ACCESS                         0x7f

// These register masks etc are named wherever possible
// corresponding to the bit and field names in the RF-22 Manual
// RH_RF22_REG_00_DEVICE_TYPE                      0x00
#define RH_RF22_DEVICE_TYPE_RX_TRX                 0x08
#define RH_RF22_DEVICE_TYPE_TX                     0x07

// RH_RF22_REG_02_DEVICE_STATUS                    0x02
#define RH_RF22_FFOVL                              0x80
#define RH_RF22_FFUNFL                             0x40
#define RH_RF22_RXFFEM                             0x20
#define RH_RF22_HEADERR                            0x10
#define RH_RF22_FREQERR                            0x08
#define RH_RF22_LOCKDET                            0x04
#define RH_RF22_CPS                                0x03
#define RH_RF22_CPS_IDLE                           0x00
#define RH_RF22_CPS_RX                             0x01
#define RH_RF22_CPS_TX                             0x10

// RH_RF22_REG_03_INTERRUPT_STATUS1                0x03
#define RH_RF22_IFFERROR                           0x80
#define RH_RF22_ITXFFAFULL                         0x40
#define RH_RF22_ITXFFAEM                           0x20
#define RH_RF22_IRXFFAFULL                         0x10
#define RH_RF22_IEXT                               0x08
#define RH_RF22_IPKSENT                            0x04
#define RH_RF22_IPKVALID                           0x02
#define RH_RF22_ICRCERROR                          0x01

// RH_RF22_REG_04_INTERRUPT_STATUS2                0x04
#define RH_RF22_ISWDET                             0x80
#define RH_RF22_IPREAVAL                           0x40
#define RH_RF22_IPREAINVAL                         0x20
#define RH_RF22_IRSSI                              0x10
#define RH_RF22_IWUT                               0x08
#define RH_RF22_ILBD                               0x04
#define RH_RF22_ICHIPRDY                           0x02
#define RH_RF22_IPOR                               0x01

// RH_RF22_REG_05_INTERRUPT_ENABLE1                0x05
#define RH_RF22_ENFFERR                            0x80
#define RH_RF22_ENTXFFAFULL                        0x40
#define RH_RF22_ENTXFFAEM                          0x20
#define RH_RF22_ENRXFFAFULL                        0x10
#define RH_RF22_ENEXT                              0x08
#define RH_RF22_ENPKSENT                           0x04
#define RH_RF22_ENPKVALID                          0x02
#define RH_RF22_ENCRCERROR                         0x01

// RH_RF22_REG_06_INTERRUPT_ENABLE2                0x06
#define RH_RF22_ENSWDET                            0x80
#define RH_RF22_ENPREAVAL                          0x40
#define RH_RF22_ENPREAINVAL                        0x20
#define RH_RF22_ENRSSI                             0x10
#define RH_RF22_ENWUT                              0x08
#define RH_RF22_ENLBDI                             0x04
#define RH_RF22_ENCHIPRDY                          0x02
#define RH_RF22_ENPOR                              0x01

// RH_RF22_REG_07_OPERATING_MODE                   0x07
#define RH_RF22_SWRES                              0x80
#define RH_RF22_ENLBD                              0x40
#define RH_RF22_ENWT                               0x20
#define RH_RF22_X32KSEL                            0x10
#define RH_RF22_TXON                               0x08
#define RH_RF22_RXON                               0x04
#define RH_RF22_PLLON                              0x02
#define RH_RF22_XTON                               0x01

// RH_RF22_REG_08_OPERATING_MODE2                  0x08
#define RH_RF22_ANTDIV                             0xc0
#define RH_RF22_RXMPK                              0x10
#define RH_RF22_AUTOTX                             0x08
#define RH_RF22_ENLDM                              0x04
#define RH_RF22_FFCLRRX                            0x02
#define RH_RF22_FFCLRTX                            0x01

// RH_RF22_REG_0F_ADC_CONFIGURATION                0x0f
#define RH_RF22_ADCSTART                           0x80
#define RH_RF22_ADCDONE                            0x80
#define RH_RF22_ADCSEL                             0x70
#define RH_RF22_ADCSEL_INTERNAL_TEMPERATURE_SENSOR 0x00
#define RH_RF22_ADCSEL_GPIO0_SINGLE_ENDED          0x10
#define RH_RF22_ADCSEL_GPIO1_SINGLE_ENDED          0x20
#define RH_RF22_ADCSEL_GPIO2_SINGLE_ENDED          0x30
#define RH_RF22_ADCSEL_GPIO0_GPIO1_DIFFERENTIAL    0x40
#define RH_RF22_ADCSEL_GPIO1_GPIO2_DIFFERENTIAL    0x50
#define RH_RF22_ADCSEL_GPIO0_GPIO2_DIFFERENTIAL    0x60
#define RH_RF22_ADCSEL_GND                         0x70
#define RH_RF22_ADCREF                             0x0c
#define RH_RF22_ADCREF_BANDGAP_VOLTAGE             0x00
#define RH_RF22_ADCREF_VDD_ON_3                    0x08
#define RH_RF22_ADCREF_VDD_ON_2                    0x0c
#define RH_RF22_ADCGAIN                            0x03

// RH_RF22_REG_10_ADC_SENSOR_AMP_OFFSET            0x10
#define RH_RF22_ADCOFFS                            0x0f

// RH_RF22_REG_12_TEMPERATURE_SENSOR_CALIBRATION   0x12
#define RH_RF22_TSRANGE                            0xc0
#define RH_RF22_TSRANGE_M64_64C                    0x00
#define RH_RF22_TSRANGE_M64_192C                   0x40
#define RH_RF22_TSRANGE_0_128C                     0x80
#define RH_RF22_TSRANGE_M40_216F                   0xc0
#define RH_RF22_ENTSOFFS                           0x20
#define RH_RF22_ENTSTRIM                           0x10
#define RH_RF22_TSTRIM                             0x0f

// RH_RF22_REG_14_WAKEUP_TIMER_PERIOD1             0x14
#define RH_RF22_WTR                                0x3c
#define RH_RF22_WTD                                0x03

// RH_RF22_REG_1D_AFC_LOOP_GEARSHIFT_OVERRIDE      0x1d
#define RH_RF22_AFBCD                              0x80
#define RH_RF22_ENAFC                              0x40
#define RH_RF22_AFCGEARH                           0x38
#define RH_RF22_AFCGEARL                           0x07

// RH_RF22_REG_1E_AFC_TIMING_CONTROL               0x1e
#define RH_RF22_SWAIT_TIMER                        0xc0
#define RH_RF22_SHWAIT                             0x38
#define RH_RF22_ANWAIT                             0x07

// RH_RF22_REG_30_DATA_ACCESS_CONTROL              0x30
#define RH_RF22_ENPACRX                            0x80
#define RH_RF22_MSBFRST                            0x00
#define RH_RF22_LSBFRST                            0x40
#define RH_RF22_CRCHDRS                            0x00
#define RH_RF22_CRCDONLY                           0x20
#define RH_RF22_SKIP2PH                            0x10
#define RH_RF22_ENPACTX                            0x08
#define RH_RF22_ENCRC                              0x04
#define RH_RF22_CRC                                0x03
#define RH_RF22_CRC_CCITT                          0x00
#define RH_RF22_CRC_CRC_16_IBM                     0x01
#define RH_RF22_CRC_IEC_16                         0x02
#define RH_RF22_CRC_BIACHEVA                       0x03

// RH_RF22_REG_32_HEADER_CONTROL1                  0x32
#define RH_RF22_BCEN                               0xf0
#define RH_RF22_BCEN_NONE                          0x00
#define RH_RF22_BCEN_HEADER0                       0x10
#define RH_RF22_BCEN_HEADER1                       0x20
#define RH_RF22_BCEN_HEADER2                       0x40
#define RH_RF22_BCEN_HEADER3                       0x80
#define RH_RF22_HDCH                               0x0f
#define RH_RF22_HDCH_NONE                          0x00
#define RH_RF22_HDCH_HEADER0                       0x01
#define RH_RF22_HDCH_HEADER1                       0x02
#define RH_RF22_HDCH_HEADER2                       0x04
#define RH_RF22_HDCH_HEADER3                       0x08

// RH_RF22_REG_33_HEADER_CONTROL2                  0x33
#define RH_RF22_HDLEN                              0x70
#define RH_RF22_HDLEN_0                            0x00
#define RH_RF22_HDLEN_1                            0x10
#define RH_RF22_HDLEN_2                            0x20
#define RH_RF22_HDLEN_3                            0x30
#define RH_RF22_HDLEN_4                            0x40
#define RH_RF22_VARPKLEN                           0x00
#define RH_RF22_FIXPKLEN                           0x08
#define RH_RF22_SYNCLEN                            0x06
#define RH_RF22_SYNCLEN_1                          0x00
#define RH_RF22_SYNCLEN_2                          0x02
#define RH_RF22_SYNCLEN_3                          0x04
#define RH_RF22_SYNCLEN_4                          0x06
#define RH_RF22_PREALEN8                           0x01

// RH_RF22_REG_6D_TX_POWER                         0x6d
// https://www.sparkfun.com/datasheets/Wireless/General/RFM22B.pdf
#define RH_RF22_PAPEAKVAL                          0x80
#define RH_RF22_PAPEAKEN                           0x40
#define RH_RF22_PAPEAKLVL                          0x30
#define RH_RF22_PAPEAKLVL6_5                       0x00
#define RH_RF22_PAPEAKLVL7                         0x10
#define RH_RF22_PAPEAKLVL7_5                       0x20
#define RH_RF22_PAPEAKLVL8                         0x30
#define RH_RF22_LNA_SW                             0x08
#define RH_RF22_TXPOW                              0x07
#define RH_RF22_TXPOW_4X31                         0x08 // Not used in RFM22B
// For RFM22B:
#define RH_RF22_TXPOW_1DBM                         0x00
#define RH_RF22_TXPOW_2DBM                         0x01
#define RH_RF22_TXPOW_5DBM                         0x02
#define RH_RF22_TXPOW_8DBM                         0x03
#define RH_RF22_TXPOW_11DBM                        0x04
#define RH_RF22_TXPOW_14DBM                        0x05 
#define RH_RF22_TXPOW_17DBM                        0x06 
#define RH_RF22_TXPOW_20DBM                        0x07 
// RFM23B only:
#define RH_RF22_RF23B_TXPOW_M8DBM                  0x00 // -8dBm
#define RH_RF22_RF23B_TXPOW_M5DBM                  0x01 // -5dBm
#define RH_RF22_RF23B_TXPOW_M2DBM                  0x02 // -2dBm
#define RH_RF22_RF23B_TXPOW_1DBM                   0x03 // 1dBm
#define RH_RF22_RF23B_TXPOW_4DBM                   0x04 // 4dBm
#define RH_RF22_RF23B_TXPOW_7DBM                   0x05 // 7dBm
#define RH_RF22_RF23B_TXPOW_10DBM                  0x06 // 10dBm
#define RH_RF22_RF23B_TXPOW_13DBM                  0x07 // 13dBm
// RFM23BP only:
#define RH_RF22_RF23BP_TXPOW_28DBM                 0x05 // 28dBm
#define RH_RF22_RF23BP_TXPOW_29DBM                 0x06 // 29dBm
#define RH_RF22_RF23BP_TXPOW_30DBM                 0x07 // 30dBm

// RH_RF22_REG_71_MODULATION_CONTROL2              0x71
#define RH_RF22_TRCLK                              0xc0
#define RH_RF22_TRCLK_NONE                         0x00
#define RH_RF22_TRCLK_GPIO                         0x40
#define RH_RF22_TRCLK_SDO                          0x80
#define RH_RF22_TRCLK_NIRQ                         0xc0
#define RH_RF22_DTMOD                              0x30
#define RH_RF22_DTMOD_DIRECT_GPIO                  0x00
#define RH_RF22_DTMOD_DIRECT_SDI                   0x10
#define RH_RF22_DTMOD_FIFO                         0x20
#define RH_RF22_DTMOD_PN9                          0x30
#define RH_RF22_ENINV                              0x08
#define RH_RF22_FD8                                0x04
#define RH_RF22_MODTYP                             0x30
#define RH_RF22_MODTYP_UNMODULATED                 0x00
#define RH_RF22_MODTYP_OOK                         0x01
#define RH_RF22_MODTYP_FSK                         0x02
#define RH_RF22_MODTYP_GFSK                        0x03


// RH_RF22_REG_75_FREQUENCY_BAND_SELECT            0x75
#define RH_RF22_SBSEL                              0x40
#define RH_RF22_HBSEL                              0x20
#define RH_RF22_FB                                 0x1f

// Define this to include Serial printing in diagnostic routines
#define RH_RF22_HAVE_SERIAL

/////////////////////////////////////////////////////////////////////
/// \class RH_RF22 RH_RF22.h <RH_RF22.h>
/// \brief Driver to send and receive unaddressed, unreliable datagrams via an RF22 and compatible radio transceiver.
///
/// Works with RF22, RF23 based radio modules, and compatible chips and modules, including:
/// - RF22 bare module: http://www.sparkfun.com/products/10153
///   (Caution, that is a 3.3V part, and requires a 3.3V CPU such as Teensy etc or level shifters)
/// - RF22 shield: http://www.sparkfun.com/products/11018 
/// - RF22 integrated board http://www.anarduino.com/miniwireless
/// - RFM23BP bare module: http://www.anarduino.com/details.jsp?pid=130 
/// - Silicon Labs Si4430/31/32 based modules. S4432 is equivalent to RF22. Si4431/30 is equivalent to RF23.
///
/// Data based on https://www.sparkfun.com/datasheets/Wireless/General/RFM22B.pdf
///
/// \par Overview
///
/// This base class provides basic functions for sending and receiving unaddressed, 
/// unreliable datagrams of arbitrary length to 255 octets per packet.
///
/// Manager classes may use this class to implement reliable, addressed datagrams and streams, 
/// mesh routers, repeaters, translators etc.
///
/// On transmission, the TO and FROM addresses default to 0x00, unless changed by a subclass. 
/// On reception the TO addressed is checked against the node address (defaults to 0x00) or the
/// broadcast address (which is 0xff). The ID and FLAGS are set to 0, and not checked by this class.
/// This permits use of the this base RH_RF22 class as an 
/// unaddressed, unreliable datagram service without the use of one the RadioHead Manager classes.
///
/// Naturally, for any 2 radios to communicate that must be configured to use the same frequency and 
/// modulation scheme.
///
/// \par Details
///
/// This Driver provides an object-oriented interface for sending and receiving data messages with Hope-RF
/// RF22 and RF23 based radio modules, and compatible chips and modules, 
/// including the RFM22B transceiver module such as 
/// this bare module: http://www.sparkfun.com/products/10153
/// and this shield: http://www.sparkfun.com/products/11018
/// and this module: http://www.hoperfusa.com/details.jsp?pid=131
/// and this integrated board: http://www.anarduino.com/miniwireless
/// and RF23BP modules such as this http://www.anarduino.com/details.jsp?pid=130
///
/// The Hope-RF (http://www.hoperf.com) RFM22B (http://www.hoperf.com/rf_fsk/fsk/RFM22B.htm) 
/// is a low-cost ISM transceiver module. It supports FSK, GFSK, OOK over a wide 
/// range of frequencies and programmable data rates.
/// Manual can be found at https://www.sparkfun.com/datasheets/Wireless/General/RFM22.PDF
///
/// This library provides functions for sending and receiving messages of up to 255 octets on any 
/// frequency supported by the RF22B, in a range of predefined data rates and frequency deviations. 
/// Frequency can be set with 312Hz precision to any frequency from 240.0MHz to 960.0MHz.
///
/// Up to 3 RF22B modules can be connected to an Arduino, permitting the construction of translators
/// and frequency changers, etc.
///
/// The following modulation types are suppported with a range of modem configurations for 
/// common data rates and frequency deviations:
/// - GFSK Gaussian Frequency Shift Keying
/// - FSK Frequency Shift Keying
/// - OOK On-Off Keying
///
/// Support for other RF22B features such as on-chip temperature measurement, analog-digital 
/// converter, transmitter power control etc is also provided.
///
/// Tested on Arduino Diecimila, Uno and Mega with arduino-0021, 1.0.5
/// on OpenSuSE 13.1 and avr-libc-1.6.1-1.15,
/// cross-avr-binutils-2.19-9.1, cross-avr-gcc-4.1.3_20080612-26.5.
/// With HopeRF RFM22 modules that appear to have RF22B chips on board:
///    - Device Type Code = 0x08 (RX/TRX)
///    - Version Code = 0x06
/// Works on Duo. Works with Sparkfun RFM22 Wireless shields. Works with RFM22 modules from http://www.hoperfusa.com/
/// Works with Arduino 1.0 to at least 1.0.5. Works on Maple, Flymaple, Uno32 (with ChipKIT Core with Arduino IDE).
///
/// \par Packet Format
///
/// All messages sent and received by this Driver must conform to this packet format:
///
/// - 8 nibbles (4 octets) PREAMBLE
/// - 2 octets SYNC 0x2d, 0xd4
/// - 4 octets HEADER: (TO, FROM, ID, FLAGS)
/// - 1 octet LENGTH (0 to 255), number of octets in DATA
/// - 0 to 255 octets DATA
/// - 2 octets CRC computed with CRC16(IBM), computed on HEADER, LENGTH and DATA
///
/// For technical reasons, the message format is not protocol compatible with the 
/// 'HopeRF Radio Transceiver Message Library for Arduino' http://www.airspayce.com/mikem/arduino/HopeRF from the same author. Nor is it compatible with 
/// 'Virtual Wire' http://www.airspayce.com/mikem/arduino/VirtualWire.pdf also from the same author.
///
/// \par Connecting RFM-22 to Arduino
///
/// If you have the Sparkfun RFM22 Shield (https://www.sparkfun.com/products/11018)
/// the connections described below are done for you on the shield, no changes required, 
/// just add headers and plug it in to an Arduino (but not and Arduino Mega, see below)
///
/// The physical connection between the RF22B and the Arduino requires 3.3V,
/// the 3 x SPI pins (SCK, SDI, SDO), a Slave Select pin and an interrupt pin.
///
/// Note: some devices may need a pullup resister on the SDO line.
///
/// Note also that on the RFM22B (but not the RFM23B), it is required to control the TX_ANT and
/// RX_ANT pins of the RFM22 in order to control the antenna connection properly. The RH_RF22
/// driver is configured by default so that GPIO0 and GPIO1 outputs can
/// control TX_ANT and RX_ANT input pins respectively automatically. On RFM22, 
/// you must connect GPIO0
/// to TX_ANT and GPIO1 to RX_ANT for this automatic antenna switching to
/// occur.  See setGpioReversed() for more details. These connections are not required on RFM23B.
///
/// If you are using the Sparkfun RF22 shield, it will work with any 5V arduino without modification.
/// Connect the RFM-22 module to most Arduino's like this (Caution, Arduino Mega has different pins for SPI, 
/// see below).
/// \code
///                 Arduino      RFM-22B
///                 GND----------GND-\ (ground in)
///                              SDN-/ (shutdown in)
///                 3V3----------VCC   (3.3V in)
/// interrupt 0 pin D2-----------NIRQ  (interrupt request out)
///          SS pin D10----------NSEL  (chip select in)
///         SCK pin D13----------SCK   (SPI clock in)
///        MOSI pin D11----------SDI   (SPI Data in)
///        MISO pin D12----------SDO   (SPI data out)
///                           /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
///                           \--TX_ANT (TX antenna control in) RFM22B only
///                           /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
///                           \--RX_ANT (RX antenna control in) RFM22B only
/// \endcode
/// For an Arduino Mega:
/// \code
///                 Mega         RFM-22B
///                 GND----------GND-\ (ground in)
///                              SDN-/ (shutdown in)
///                 3V3----------VCC   (3.3V in)
/// interrupt 0 pin D2-----------NIRQ  (interrupt request out)
///          SS pin D10----------NSEL  (chip select in)
///         SCK pin D52----------SCK   (SPI clock in)
///        MOSI pin D51----------SDI   (SPI Data in)
///        MISO pin D50----------SDO   (SPI data out)
///                           /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
///                           \--TX_ANT (TX antenna control in) RFM22B only
///                           /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
///                           \--RX_ANT (RX antenna control in) RFM22B only
/// \endcode
/// For Chipkit Uno32. Caution: you must also ensure jumper JP4 on the Uno32 is set to RD4
/// \code
///                 Arduino      RFM-22B
///                 GND----------GND-\ (ground in)
///                              SDN-/ (shutdown in)
///                 3V3----------VCC   (3.3V in)
/// interrupt 0 pin D38----------NIRQ  (interrupt request out)
///          SS pin D10----------NSEL  (chip select in)
///         SCK pin D13----------SCK   (SPI clock in)
///        MOSI pin D11----------SDI   (SPI Data in)
///        MISO pin D12----------SDO   (SPI data out)
///                           /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
///                           \--TX_ANT (TX antenna control in) RFM22B only
///                           /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
///                           \--RX_ANT (RX antenna control in) RFM22B only
/// \endcode
/// For Teensy 3.1
/// \code
///                 Teensy      RFM-22B
///                 GND----------GND-\ (ground in)
///                              SDN-/ (shutdown in)
///                 3V3----------VCC   (3.3V in)
/// interrupt 2 pin D2-----------NIRQ  (interrupt request out)
///          SS pin D10----------NSEL  (chip select in)
///         SCK pin D13----------SCK   (SPI clock in)
///        MOSI pin D11----------SDI   (SPI Data in)
///        MISO pin D12----------SDO   (SPI data out)
///                           /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
///                           \--TX_ANT (TX antenna control in) RFM22B only
///                           /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
///                           \--RX_ANT (RX antenna control in) RFM22B only
/// \endcode
/// For an Arduino Due (the SPI pins do not come out on the Digital pins as for normal Arduino, but only
/// appear on the SPI header)
/// \code
///                 Due      RFM-22B
///                 GND----------GND-\ (ground in)
///                              SDN-/ (shutdown in)
///                 5V-----------VCC   (5V in)
/// interrupt 0 pin D2-----------NIRQ  (interrupt request out)
///          SS pin D10----------NSEL  (chip select in)
///       SCK SPI pin 3----------SCK   (SPI clock in)
///      MOSI SPI pin 4----------SDI   (SPI Data in)
///      MISO SPI pin 1----------SDO   (SPI data out)
///                           /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
///                           \--TX_ANT (TX antenna control in) RFM22B only
///                           /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
///                           \--RX_ANT (RX antenna control in) RFM22B only
/// \endcode
/// and use the default constructor:
/// RH_RF22 driver;
/// For connecting an Arduino to an RFM23BP module. Note that the antenna control pins are reversed 
/// compared to the RF22.
/// \code
///                 Arduino      RFM-23BP
///                 GND----------GND-\ (ground in)
///                              SDN-/ (shutdown in)
///                 5V-----------VCC   (5V in)
/// interrupt 0 pin D2-----------NIRQ  (interrupt request out)
///          SS pin D10----------NSEL  (chip select in)
///         SCK pin D13----------SCK   (SPI clock in)
///        MOSI pin D11----------SDI   (SPI Data in)
///        MISO pin D12----------SDO   (SPI data out)
///                           /--GPIO0 (GPIO0 out to control receiver antenna RXON)
///                           \--RXON   (RX antenna control in)
///                           /--GPIO1 (GPIO1 out to control transmitter antenna TXON)
///                           \--TXON   (TX antenna control in)
/// \endcode
///
/// and you can then use the default constructor RH_RF22(). 
/// You can override the default settings for the SS pin and the interrupt 
/// in the RH_RF22 constructor if you wish to connect the slave select SS to other than the normal one for your 
/// Arduino (D10 for Diecimila, Uno etc and D53 for Mega)
/// or the interrupt request to other than pin D2 (Caution, different processors have different constraints as to the 
/// pins available for interrupts).
///
/// Caution: some people have had problems with some batches of
/// RFM23BP chips burning out their nIRQ outputs for unknown
/// reasons when run at 5V. Some users assert that running RFM23BP with voltage
/// dividers at 3.3V is to be preferred. We have not tested or verified
/// either the cause or the supposed cure.
//
///
/// If you have an Arduino Zero, you should note that you cannot use Pin 2 for the interrupt line 
/// (Pin 2 is for the NMI only), instead you can use any other pin (we use Pin 3) and initialise RH_RF69 like this:
/// \code
/// // Slave Select is pin 10, interrupt is Pin 3
/// RH_RF22 driver(10, 3);
/// \endcode
///
/// If you have an ESP32 (we tested with the Geekworm EASY-KIT ESP32-B1 which has a ESP-WROOM-32 chip)
/// \code
///                 ESP32      RFM-22B
///                 GND----------GND-\ (ground in)
///                              SDN-/ (shutdown in)
///                 3V3----------VCC   (3.3V in)
/// interrupt   pin GPIO15-------NIRQ  (interrupt request out)
///          SS pin GPIO13-------NSEL  (chip select in)
///         SCK pin GPIO18-------SCK   (SPI clock in)
///        MOSI pin GPIO23-------SDI   (SPI Data in)
///        MISO pin GPIO19-------SDO   (SPI data out)
///                           /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
///                           \--TX_ANT (TX antenna control in) RFM22B only
///                           /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
///                           \--RX_ANT (RX antenna control in) RFM22B only
/// \endcode
/// and initialise like this:
/// \code
/// RH_RF22 driver(13, 15);
/// \endcode
/// You can of course use other pins for NSEL and NIRQ if you prefer.
///
/// To connect an STM32 F4 Discovery board to RF22 using Arduino and Arduino_STM32
/// connect the pins like this:
/// \code
///                 STM32      RFM-22B
///                 GND----------GND-\ (ground in)
///                              SDN-/ (shutdown in)
///                 VDD----------VCC   (3.3V in)
/// interrupt   pin PB1----------NIRQ  (interrupt request out)
///          SS pin PB0----------NSEL  (chip select in)
///         SCK pin PB3----------SCK   (SPI clock in)
///        MOSI pin PB5----------SDI   (SPI Data in)
///        MISO pin PB4----------SDO   (SPI data out)
///                           /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
///                           \--TX_ANT (TX antenna control in) RFM22B only
///                           /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
///                           \--RX_ANT (RX antenna control in) RFM22B only
/// \endcode
/// and initialise like this:
/// \code
/// RH_RF22 driver(PB0, PB1);
/// \endcode
/// You can of use other pins for NSEL and NIRQ if you prefer.
///
/// To connect an ATTiny Mega x16 such as AtTiny 3216 etc
/// (running at 5V) etc RF22 using Arduino using Spencer Kondes 
/// megaTinyCore https://github.com/SpenceKonde/megaTinyCore connect the pins like this:
/// (pin numbering based on https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/extras/ATtiny_x16.md)
/// \code
///              AtTiny x16      RFM-22B
///                 GND----------GND-\ (ground in)
///                              SDN-/ (shutdown in)
///                 VDD----------VCC   (5V in)
/// interrupt   pin PA6----------NIRQ  (interrupt request out)
///          SS pin PC0----------NSEL  (chip select in)
///         SCK pin PA3----------SCK   (SPI clock in)
///        MOSI pin PA1----------SDI   (SPI Data in)
///        MISO pin PA2----------SDO   (SPI data out)
///                           /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
///                           \--TX_ANT (TX antenna control in) RFM22B only
///                           /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
///                           \--RX_ANT (RX antenna control in) RFM22B only
/// \endcode
/// and initialise like this:
/// \code
/// RH_RF22 driver(10, 2);
/// \endcode
/// You can of use other pins for NSEL and NIRQ if you prefer.
///
/// For ESP8266-based ESP-12 modules. Caution: on some breakout boards we have seen
/// labels for D4 and D5 reversed.
/// \code
///                 ESP-12      RFM-22B
///                 GND----------GND-\ (ground in)
///                              SDN-/ (shutdown in)
///                 3V3----------VCC   (3.3V in)
/// interrupt 0 pin D4-----------NIRQ  (interrupt request out)
///          SS pin D5-----------NSEL  (chip select in)
///         SCK pin D14----------SCK   (SPI clock in)
///        MOSI pin D13----------SDI   (SPI Data in)
///        MISO pin D12----------SDO   (SPI data out)
///                           /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
///                           \--TX_ANT (TX antenna control in) RFM22B only
///                           /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
///                           \--RX_ANT (RX antenna control in) RFM22B only
/// \endcode
/// and initialise like this:
/// \code
/// RH_RF22 driver(5, 4);
/// \endcode
///
/// Note: It is possible to have 2 radios connected to one Arduino, provided each radio has its own 
/// SS and interrupt line (SCK, SDI and SDO are common to both radios)
///
/// Caution: on some Arduinos such as the Mega 2560, if you set the slave select pin to be other than the usual SS 
/// pin (D53 on  Mega 2560), you may need to set the usual SS pin to be an output to force the Arduino into SPI 
/// master mode.
///
/// Caution: Power supply requirements of the RF22 module may be relevant in some circumstances: 
/// RF22 modules are capable of pulling 80mA+ at full power, where Arduino's 3.3V line can
/// give 50mA. You may need to make provision for alternate power supply for
/// the RF22, especially if you wish to use full transmit power, and/or you have
/// other shields demanding power. Inadequate power for the RF22 is reported to cause symptoms such as:
/// - reset's/bootups terminate with "init failed" messages
/// -random termination of communication after 5-30 packets sent/received
/// -"fake ok" state, where initialization passes fluently, but communication doesn't happen
/// -shields hang Arduino boards, especially during the flashing
///
/// Caution: some RF22 breakout boards (such as the HAB-RFM22B-BOA  HAB-RFM22B-BO) reportedly 
/// have the TX_ANT and RX_ANT pre-connected to GPIO0 and GPIO1 round the wrong way. You can work with this
/// if you use setGpioReversed().
///
/// Caution: If you are using a bare RF22 module without IO level shifters, you may have difficulty connecting
/// to a 5V arduino. The RF22 module is 3.3V and its IO pins are 3.3V not 5V. Some Arduinos (Diecimila and
/// Uno) seem to work OK with this, and some (Mega) do not always work reliably. Your Mileage May Vary.
/// For best result, use level shifters, or use a RF22 shield or board with level shifters built in, 
/// such as the Sparkfun RFM22 shield http://www.sparkfun.com/products/11018. 
/// You could also use a 3.3V IO Arduino such as a Pro. 
/// It is recognised that it is difficult to connect 
/// the Sparkfun RFM22 shield to a Mega, since the SPI pins on the Mega are different to other Arduinos, 
/// But it is possible, by bending the SPI pins (D10, D11, D12, D13) on the 
/// shield out of the way before plugging it in to the Mega and jumpering the shield pins to the Mega like this:
/// \code
///   RF22 Shield        Mega
///     D10              D53
///     D13              D52
///     D11              D51
///     D12              D50
/// \endcode
/// 
/// \par Interrupts
///
/// The Driver uses interrupts to react to events in the RF22 module, 
/// such as the reception of a new packet, or the completion of transmission of a packet. 
/// The RH_RF22 interrupt service routine reads status from and writes data
/// to the the RF22 module via the SPI interface. It is very important therefore,
/// that if you are using the RF22 library with another SPI based deviced, that you
/// disable interrupts while you transfer data to and from that other device.
/// Use cli() to disable interrupts and sei() to reenable them.
///
/// \par SPI Interface
///
/// The RF22 module uses the SPI bus to communicate with the Arduino. Arduino
/// IDE includes a hardware SPI class to communicate with SPI devices using
/// the SPI facilities built into the Atmel chips, over the standard designated
/// SPI pins MOSI, MISO, SCK, which are usually on Arduino pins 11, 12 and 13
/// respectively (or 51, 50, 52 on a Mega).
///
/// By default, the RH_RF22 Driver uses the Hardware SPI interface to
/// communicate with the RF22 module. However, if your RF22 SPI is connected to
/// the Arduino through non-standard pins, or the standard Hardware SPI
/// interface will not work for you, you can instead use a bit-banged Software
/// SPI class RHSoftwareSPI, which can be configured to work on any Arduino digital IO pins.
/// See the documentation of RHSoftwareSPI for details.
///
/// The advantages of the Software SPI interface are that it can be used on
/// any Arduino pins, not just the usual dedicated hardware pins. The
/// disadvantage is that it is significantly slower then hardware.
/// If you observe reliable behaviour with the default hardware SPI RHHardwareSPI, but unreliable behaviour 
/// with Software SPI RHSoftwareSPI, it may be due to slow CPU performance.
///
/// Initialisation example with hardware SPI
/// \code
/// #include <RH_RF22.h>
/// RH_RF22 driver;
/// RHReliableDatagram manager(driver, CLIENT_ADDRESS);
/// \endcode
///
/// Initialisation example with software SPI
/// \code
/// #include <RH_RF22.h>
/// #include <RHSoftwareSPI.h>
/// RHSoftwareSPI spi;
/// RH_RF22 driver(10, 2, spi);
/// RHReliableDatagram manager(driver, CLIENT_ADDRESS);
/// \endcode
///
/// \par Memory
///
/// The RH_RF22 Driver requires non-trivial amounts of memory. The sample programs all compile to 
/// about 9 to 14kbytes each on Arduino, which will fit in the flash proram memory of most Arduinos. However, 
/// the RAM requirements are more critical. Most sample programs above will run on Duemilanova, 
/// but not on Diecimila. Even on Duemilanova, the RAM requirements are very close to the 
/// available memory of 2kbytes. Therefore, you should be vary sparing with RAM use in programs that use 
/// the RH_RF22 Driver on Duemilanova.
///
/// The sample RHRouter and RHMesh programs compile to about 14kbytes, 
/// and require more RAM than the others. 
/// They will not run on Duemilanova or Diecimila, but will run on Arduino Mega.
///
/// It is often hard to accurately identify when you are hitting RAM limits on Arduino. 
/// The symptoms can include:
/// - Mysterious crashes and restarts
/// - Changes in behaviour when seemingly unrelated changes are made (such as adding print() statements)
/// - Hanging
/// - Output from Serial.print() not appearing
/// 
/// With an Arduino Mega, with 8 kbytes of SRAM, there is much more RAM headroom for 
/// your own elaborate programs. 
/// This library is reported to work with Arduino Pro Mini, but that has not been tested by me.
///
/// The RF22M modules use an inexpensive crystal to control the frequency synthesizer, and therfore you can expect 
/// the transmitter and receiver frequencies to be subject to the usual inaccuracies of such crystals. The RF22
/// contains an AFC circuit to compensate for differences in transmitter and receiver frequencies. 
/// It does this by altering the receiver frequency during reception by up to the pull-in frequency range. 
/// This RF22 library enables the AFC and by default sets the pull-in frequency range to
/// 0.05MHz, which should be sufficient to handle most situations. However, if you observe unexplained packet losses
/// or failure to operate correctly all the time it may be because your modules have a wider frequency difference, and
/// you may need to set the afcPullInRange to a different value, using setFrequency();
///
/// \par Transmitter Power
///
/// You can control the transmitter power on the RF22 and RF23 transceivers
/// with the RH_RF22::setTxPower() function. The argument can be any of the
/// RH_RF22_TXPOW_* (for RFM22) or RH_RF22_RF23B_TXPOW_* (for RFM23) values. 
/// The default is RH_RF22_TXPOW_8DBM/RH_RF22_RF23B_TXPOW_1DBM . Eg:
/// \code
/// driver.setTxPower(RH_RF22_TXPOW_2DBM);
/// \endcode
///
/// The RF23BP has higher power capability, there are
/// several power settings that are specific to the RF23BP only:
///
/// - RH_RF22_RF23BP_TXPOW_28DBM
/// - RH_RF22_RF23BP_TXPOW_29DBM
/// - RH_RF22_RF23BP_TXPOW_30DBM
///
/// CAUTION: the high power settings available on the RFM23BP require
/// significant power supply current.  For example at +30dBm, the typical chip
/// supply current is 550mA. This will overwhelm some small CPU board power
/// regulators and USB supplies. If you use this chip at high power make sure
/// you have an adequate supply current providing full 5V to the RFM23BP (and
/// the CPU if required), otherwise you can expect strange behaviour like
/// hanging, stopping, incorrect power levels, RF power amp overheating etc.
/// You must also ensure that the RFM23BP GPIO pins are connected to the
/// antenna switch control pins like so:
////
/// \code
/// GPIO0 <-> RXON
/// GPIO1 <-> TXON
/// \endcode
///
/// The RF output impedance of the RFM22BP module is 50 ohms.  In our
/// experiments we found that the most critical issue (besides a suitable
/// power supply) is to ensure that the antenna impedance is also near 50
/// ohms. Connecting a simple 1/4 wavelength (ie a 17.3cm single wire)
/// directly to the antenna output <b>will not work at full 30dBm power</b>,
/// and will result in the transmitter hanging and/or the power amp
/// overheating. Connect a proper 50 ohm impedance transmission line or
/// antenna, and prevent RF radiation into the radio and arduino modules,
/// in order to get full, reliable power. Our tests show that a 433MHz
/// RFM23BP feeding a 50 ohm transmission line with a VHF discone antenna at
/// the end results in full power output and the power amp transistor on the
/// RFM22BP module runnning slightly warm but not hot. We recommend you use
/// the services of a competent RF engineer when trying to use this high power
/// module.
///
/// Note: with RFM23BP, the reported maximum possible power when operating on 3.3V is 27dBm.
/// The BP version is an RFM23 with a PA 
/// external to the Silicon Labs radio chip.  
/// The RFM23BP only supports the top three power settings because those three 
/// output levels from the RFM23 provide enough drive to the PA to make it 
/// saturate.  Less drive and the PA will dissipate more heat.  However, those 
/// three levels don't change the output power from the PA.
///
/// We have made some actual power measurements against
/// programmed power for Sparkfun RFM22 wireless module under the following conditions:
/// - Sparkfun RFM22 wireless module, Duemilanove, USB power
/// - 10cm RG58C/U soldered direct to RFM22 module ANT and GND
/// - bnc connecteor
/// - 12dB attenuator
/// - BNC-SMA adapter
/// - MiniKits AD8307 HF/VHF Power Head (calibrated against Rohde&Schwartz 806.2020 test set)
/// - Tektronix TDS220 scope to measure the Vout from power head
/// \code
/// Program power           Measured Power
///    dBm                         dBm
///    1                           -5.6
///    2                           -3.8
///    5                           -2.2
///    8                           -0.6
///    11                           1.2
///    14                           11.6
///    17                           14.4
///    20                           18.0
/// \endcode
/// (Caution: we dont claim laboratory accuracy for these measurements)
/// You would not expect to get anywhere near these powers to air with a simple 1/4 wavelength wire antenna.
///
/// \par Performance
///
/// Some simple speed performance tests have been conducted.
/// In general packet transmission rate will be limited by the modulation scheme.
/// Also, if your code does any slow operations like Serial printing it will also limit performance. 
/// We disabled any printing in the tests below.
/// We tested with RH_RF22::GFSK_Rb125Fd125, which is probably the fastest scheme available.
/// We tested with a 13 octet message length, over a very short distance of 10cm.
///
/// Transmission (no reply) tests with modulation RH_RF22::GFSK_Rb125Fd125 and a 
/// 13 octet message show about 330 messages per second transmitted.
///
/// Transmit-and-wait-for-a-reply tests with modulation RH_RF22::GFSK_Rb125Fd125 and a 
/// 13 octet message (send and receive) show about 160 round trips per second.
///
/// \par Compatibility with RF22 library
/// The RH_RF22 driver is based on our earlier RF22 library http://www.airspayce.com/mikem/arduino/RF22
/// We have tried hard to be as compatible as possible with the earlier RF22 library, but there are some differences:
/// - Different constructor.
/// - Indexes for some modem configurations have changed (we recommend you use the symbolic names, not integer indexes).
///
/// The major difference is that under RadioHead, you are
/// required to create 2 objects (ie RH_RF22 and a manager) instead of just one object under RF22
/// (ie RHMesh, RHRouter, RHReliableDatagram or RHDatagram).
/// It may be sufficient or you to change for example:
/// \code
/// RF22ReliableDatagram rf22(CLIENT_ADDRESS);
/// \endcode
/// to:
/// \code
/// RH_RF22 driver;
/// RHReliableDatagram rf22(driver, CLIENT_ADDRESS);
/// \endcode
/// and any instance of RF22_MAX_MESSAGE_LEN to RH_RF22_MAX_MESSAGE_LEN
///
/// RadioHead version 1.6 changed the way the interrupt pin number is
/// specified on Arduino and Uno32 platforms. If your code previously
/// specifed a non-default interrupt pin number in the RH_RF22 constructor,
/// you may need to review your code to specify the correct interrrupt pin
/// (and not the interrupt number as before).
class RH_RF22 : public RHSPIDriver
{
public:
    /// \brief Defines register values for a set of modem configuration registers
    ///
    /// Defines register values for a set of modem configuration registers
    /// that can be passed to setModemConfig()
    /// if none of the choices in ModemConfigChoice suit your need
    /// setModemConfig() writes the register values to the appropriate RH_RF22 registers
    /// to set the desired modulation type, data rate and deviation/bandwidth.
    /// Suitable values for these registers can be computed using the register calculator at
    /// http://www.hoperf.com/upload/rf/RF22B%2023B%2031B%2042B%2043B%20Register%20Settings_RevB1-v5.xls
    typedef struct
    {
	uint8_t    reg_1c;   ///< Value for register RH_RF22_REG_1C_IF_FILTER_BANDWIDTH
	uint8_t    reg_1f;   ///< Value for register RH_RF22_REG_1F_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE
	uint8_t    reg_20;   ///< Value for register RH_RF22_REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE
	uint8_t    reg_21;   ///< Value for register RH_RF22_REG_21_CLOCK_RECOVERY_OFFSET2 
	uint8_t    reg_22;   ///< Value for register RH_RF22_REG_22_CLOCK_RECOVERY_OFFSET1 
	uint8_t    reg_23;   ///< Value for register RH_RF22_REG_23_CLOCK_RECOVERY_OFFSET0
	uint8_t    reg_24;   ///< Value for register RH_RF22_REG_24_CLOCK_RECOVERY_TIMING_LOOP_GAIN1
	uint8_t    reg_25;   ///< Value for register RH_RF22_REG_25_CLOCK_RECOVERY_TIMING_LOOP_GAIN0 
	uint8_t    reg_2c;   ///< Value for register RH_RF22_REG_2C_OOK_COUNTER_VALUE_1 
	uint8_t    reg_2d;   ///< Value for register RH_RF22_REG_2D_OOK_COUNTER_VALUE_2
	uint8_t    reg_2e;   ///< Value for register RH_RF22_REG_2E_SLICER_PEAK_HOLD 
	uint8_t    reg_58;   ///< Value for register RH_RF22_REG_58_CHARGE_PUMP_CURRENT_TRIMMING
	uint8_t    reg_69;   ///< Value for register RH_RF22_REG_69_AGC_OVERRIDE1 
	uint8_t    reg_6e;   ///< Value for register RH_RF22_REG_6E_TX_DATA_RATE1
	uint8_t    reg_6f;   ///< Value for register RH_RF22_REG_6F_TX_DATA_RATE0 
	uint8_t    reg_70;   ///< Value for register RH_RF22_REG_70_MODULATION_CONTROL1
	uint8_t    reg_71;   ///< Value for register RH_RF22_REG_71_MODULATION_CONTROL2
	uint8_t    reg_72;   ///< Value for register RH_RF22_REG_72_FREQUENCY_DEVIATION
    } ModemConfig;
  
    /// Choices for setModemConfig() for a selected subset of common modulation types,
    /// and data rates. If you need another configuration, use the register calculator.
    /// and call setModemRegisters() with your desired settings.
    /// These are indexes into MODEM_CONFIG_TABLE. We strongly recommend you use these symbolic
    /// definitions and not their integer equivalents: its possible that new values will be
    /// introduced in later versions (though we will try to avoid it).
    typedef enum
    {
	UnmodulatedCarrier = 0, ///< Unmodulated carrier for testing
	FSK_PN9_Rb2Fd5,      ///< FSK, No Manchester, Rb = 2kbs, Fd = 5kHz, PN9 random modulation for testing

	FSK_Rb2Fd5,	     ///< FSK, No Manchester, Rb = 2kbs,    Fd = 5kHz
	FSK_Rb2_4Fd36,       ///< FSK, No Manchester, Rb = 2.4kbs,  Fd = 36kHz
	FSK_Rb4_8Fd45,       ///< FSK, No Manchester, Rb = 4.8kbs,  Fd = 45kHz
	FSK_Rb9_6Fd45,       ///< FSK, No Manchester, Rb = 9.6kbs,  Fd = 45kHz
	FSK_Rb19_2Fd9_6,     ///< FSK, No Manchester, Rb = 19.2kbs, Fd = 9.6kHz
	FSK_Rb38_4Fd19_6,    ///< FSK, No Manchester, Rb = 38.4kbs, Fd = 19.6kHz
	FSK_Rb57_6Fd28_8,    ///< FSK, No Manchester, Rb = 57.6kbs, Fd = 28.8kHz
	FSK_Rb125Fd125,      ///< FSK, No Manchester, Rb = 125kbs,  Fd = 125kHz
	FSK_Rb_512Fd2_5,     ///< FSK, No Manchester, Rb = 512bs,  Fd = 2.5kHz, for POCSAG compatibility
	FSK_Rb_512Fd4_5,     ///< FSK, No Manchester, Rb = 512bs,  Fd = 4.5kHz, for POCSAG compatibility

	GFSK_Rb2Fd5,         ///< GFSK, No Manchester, Rb = 2kbs,    Fd = 5kHz
	GFSK_Rb2_4Fd36,      ///< GFSK, No Manchester, Rb = 2.4kbs,  Fd = 36kHz
	GFSK_Rb4_8Fd45,      ///< GFSK, No Manchester, Rb = 4.8kbs,  Fd = 45kHz
	GFSK_Rb9_6Fd45,      ///< GFSK, No Manchester, Rb = 9.6kbs,  Fd = 45kHz
	GFSK_Rb19_2Fd9_6,    ///< GFSK, No Manchester, Rb = 19.2kbs, Fd = 9.6kHz
	GFSK_Rb38_4Fd19_6,   ///< GFSK, No Manchester, Rb = 38.4kbs, Fd = 19.6kHz
	GFSK_Rb57_6Fd28_8,   ///< GFSK, No Manchester, Rb = 57.6kbs, Fd = 28.8kHz
	GFSK_Rb125Fd125,     ///< GFSK, No Manchester, Rb = 125kbs,  Fd = 125kHz

	OOK_Rb1_2Bw75,       ///< OOK, No Manchester, Rb = 1.2kbs,  Rx Bandwidth = 75kHz
	OOK_Rb2_4Bw335,      ///< OOK, No Manchester, Rb = 2.4kbs,  Rx Bandwidth = 335kHz
	OOK_Rb4_8Bw335,      ///< OOK, No Manchester, Rb = 4.8kbs,  Rx Bandwidth = 335kHz
	OOK_Rb9_6Bw335,      ///< OOK, No Manchester, Rb = 9.6kbs,  Rx Bandwidth = 335kHz
	OOK_Rb19_2Bw335,     ///< OOK, No Manchester, Rb = 19.2kbs, Rx Bandwidth = 335kHz
	OOK_Rb38_4Bw335,     ///< OOK, No Manchester, Rb = 38.4kbs, Rx Bandwidth = 335kHz
	OOK_Rb40Bw335        ///< OOK, No Manchester, Rb = 40kbs,   Rx Bandwidth = 335kHz

    } ModemConfigChoice;

    /// \brief Defines the available choices for CRC
    /// Types of permitted CRC polynomials, to be passed to setCRCPolynomial()
    /// They deliberately have the same numeric values as the crc[1:0] field of Register
    /// RH_RF22_REG_30_DATA_ACCESS_CONTROL
    typedef enum
    {
	CRC_CCITT = 0,       ///< CCITT
	CRC_16_IBM = 1,      ///< CRC-16 (IBM) The default used by RH_RF22 driver
	CRC_IEC_16 = 2,      ///< IEC-16
	CRC_Biacheva = 3     ///< Biacheva
    } CRCPolynomial;

    /// Constructor. You can have multiple instances, but each instance must have its own
    /// interrupt and slave select pin. After constructing, you must call init() to initialise the interface
    /// and the radio module. A maximum of 3 instances can co-exist on one processor, provided there are sufficient
    /// distinct interrupt lines, one for each instance.
    /// \param[in] slaveSelectPin the Arduino pin number of the output to use to select the RH_RF22 before
    /// accessing it. Defaults to the normal SS pin for your Arduino (D10 for Diecimila, Uno etc, D53 for Mega, D10 for Maple)
    /// \param[in] interruptPin The interrupt Pin number that is connected to the RF22 NIRQ interrupt line. 
    /// Defaults to pin 2, as required by sparkfun RFM22 module shields.
    /// Caution: You must specify an interrupt capable pin.
    /// On many Arduino boards, there are limitations as to which pins may be used as interrupts.
    /// On Leonardo pins 0, 1, 2 or 3. On Mega2560 pins 2, 3, 18, 19, 20, 21. On Due and Teensy, any digital pin.
    /// On other Arduinos pins 2 or 3. 
    /// See http://arduino.cc/en/Reference/attachInterrupt for more details.
    /// On Chipkit Uno32, pins 38, 2, 7, 8, 35.
    /// On other boards, any digital pin may be used.
    /// \param[in] spi Pointer to the SPI interface object to use. 
    ///                Defaults to the standard Arduino hardware SPI interface
    RH_RF22(uint8_t slaveSelectPin = SS, uint8_t interruptPin = 2, RHGenericSPI& spi = hardware_spi);
  
    /// Initialises this instance and the radio module connected to it.
    /// The following steps are taken:
    /// - Initialise the slave select pin and the SPI interface library
    /// - Software reset the RH_RF22 module
    /// - Checks the connected RH_RF22 module is either a RH_RF22_DEVICE_TYPE_RX_TRX or a RH_RF22_DEVICE_TYPE_TX
    /// - Attaches an interrupt handler
    /// - Configures the RH_RF22 module
    /// - Sets the frequency to 434.0 MHz
    /// - Sets the modem data rate to FSK_Rb2_4Fd36
    /// \return  true if everything was successful
    bool        init();

    /// Issues a software reset to the 
    /// RH_RF22 module. Blocks for 1ms to ensure the reset is complete.
    void           reset();

    /// Reads and returns the device status register RH_RF22_REG_02_DEVICE_STATUS
    /// \return The value of the device status register
    uint8_t        statusRead();
  
    /// Reads a value from the on-chip analog-digital converter
    /// \param[in] adcsel Selects the ADC input to measure. One of RH_RF22_ADCSEL_*. Defaults to the 
    /// internal temperature sensor
    /// \param[in] adcref Specifies the refernce voltage to use. One of RH_RF22_ADCREF_*. 
    /// Defaults to the internal bandgap voltage.
    /// \param[in] adcgain Amplifier gain selection. 
    /// \param[in] adcoffs Amplifier offseet (0 to 15).
    /// \return The analog value. 0 to 255.
    uint8_t        adcRead(uint8_t adcsel = RH_RF22_ADCSEL_INTERNAL_TEMPERATURE_SENSOR,
			   uint8_t adcref = RH_RF22_ADCREF_BANDGAP_VOLTAGE,
			   uint8_t adcgain = 0, 
			   uint8_t adcoffs = 0);

    /// Reads the on-chip temperature sensor
    /// \param[in] tsrange Specifies the temperature range to use. One of RH_RF22_TSRANGE_*
    /// \param[in] tvoffs Specifies the temperature value offset. This is actually signed value 
    /// added to the measured temperature value
    /// \return The measured temperature.
    uint8_t        temperatureRead(uint8_t tsrange = RH_RF22_TSRANGE_M64_64C, uint8_t tvoffs = 0);   

    /// Reads the wakeup timer value in registers RH_RF22_REG_17_WAKEUP_TIMER_VALUE1 
    /// and RH_RF22_REG_18_WAKEUP_TIMER_VALUE2
    /// \return The wakeup timer value 
    uint16_t       wutRead();

    /// Sets the wakeup timer period registers RH_RF22_REG_14_WAKEUP_TIMER_PERIOD1,
    /// RH_RF22_REG_15_WAKEUP_TIMER_PERIOD2 and RH_RF22_R<EG_16_WAKEUP_TIMER_PERIOD3
    /// \param[in] wtm Wakeup timer mantissa value
    /// \param[in] wtr Wakeup timer exponent R value
    /// \param[in] wtd Wakeup timer exponent D value
    void           setWutPeriod(uint16_t wtm, uint8_t wtr = 0, uint8_t wtd = 0);

    /// Sets the transmitter and receiver centre frequency
    /// \param[in] centre Frequency in MHz. 240.0 to 960.0. Caution, some versions of RH_RF22 and derivatives 
    /// implemented more restricted frequency ranges.
    /// \param[in] afcPullInRange Sets the AF Pull In Range in MHz. Defaults to 0.05MHz (50kHz). 
    /// Range is 0.0 to 0.159375
    /// for frequencies 240.0 to 480MHz, and 0.0 to 0.318750MHz for  frequencies 480.0 to 960MHz, 
    /// \return true if the selected frquency centre + (fhch * fhs) is within range and the afcPullInRange 
    /// is within range
    bool        setFrequency(float centre, float afcPullInRange = 0.05);

    /// Sets the frequency hopping step size.
    /// \param[in] fhs Frequency Hopping step size in 10kHz increments
    /// \return true if centre + (fhch * fhs) is within limits
    bool        setFHStepSize(uint8_t fhs);

    /// Sets the frequncy hopping channel. Adds fhch * fhs to centre frequency
    /// \param[in] fhch The channel number
    /// \return true if the selected frquency centre + (fhch * fhs) is within range
    bool        setFHChannel(uint8_t fhch);

    /// Reads and returns the current RSSI value from register RH_RF22_REG_26_RSSI. Caution: this is
    /// in internal units (see figure 31 of RFM22B/23B documentation), not in dBm. If you want to find the RSSI in dBm
    /// of the last received message, use lastRssi() instead.
    /// \return The current RSSI value 
    uint8_t        rssiRead();

    /// Reads and returns the current EZMAC value from register RH_RF22_REG_31_EZMAC_STATUS
    /// \return The current EZMAC value
    uint8_t        ezmacStatusRead();

    /// Sets the parameters for the RH_RF22 Idle mode in register RH_RF22_REG_07_OPERATING_MODE. 
    /// Idle mode is the mode the RH_RF22 will be in when not transmitting or receiving. The default idle mode 
    /// is RH_RF22_XTON ie READY mode. 
    /// \param[in] mode Mask of mode bits, using RH_RF22_SWRES, RH_RF22_ENLBD, RH_RF22_ENWT, 
    /// RH_RF22_X32KSEL, RH_RF22_PLLON, RH_RF22_XTON.
    void           setOpMode(uint8_t mode);

    /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
    void           setModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the RH_RF22.
    void           setModeRx();

    /// If current mode is Rx or Idle, changes it to Rx. 
    /// Starts the transmitter in the RH_RF22.
    void           setModeTx();

    /// Sets the transmitter power output level in register RH_RF22_REG_6D_TX_POWER.
    /// Be a good neighbour and set the lowest power level you need.
    /// After init(), the power will be set to RH_RF22::RH_RF22_TXPOW_8DBM on RF22B
    /// or RH_RF22_RF23B_TXPOW_1DBM on an RF23B.
    /// The highest power available on RF22B is RH_RF22::RH_RF22_TXPOW_20DBM (20dBm).
    /// The highest power available on RF23B is RH_RF22::RH_RF22_RF23B_TXPOW_13DBM (13dBm).
    /// Higher powers are available on RF23BP (using RH_RF22_RF23BP_TXPOW_*), 
    /// and then only with an adequate power supply. See comments above.
    /// Caution: In some countries you may only select certain higher power levels if you
    /// are also using frequency hopping. Make sure you are aware of the legal
    /// limitations and regulations in your region.
    /// \param[in] power Transmitter power level, one of RH_RF22_*TXPOW_*
    void           setTxPower(uint8_t power);

    /// Sets all the registered required to configure the data modem in the RH_RF22, including the data rate, 
    /// bandwidths etc. You cas use this to configure the modem with custom configuraitons if none of the 
    /// canned configurations in ModemConfigChoice suit you.
    /// \param[in] config A ModemConfig structure containing values for the modem configuration registers.
    void           setModemRegisters(const ModemConfig* config);

    /// Select one of the predefined modem configurations. If you need a modem configuration not provided 
    /// here, use setModemRegisters() with your own ModemConfig.
    /// \param[in] index The configuration choice.
    /// \return true if index is a valid choice.
    bool        setModemConfig(ModemConfigChoice index);

    /// Starts the receiver and checks whether a received message is available.
    /// This can be called multiple times in a timeout loop
    /// \return true if a complete, valid message has been received and is able to be retrieved by
    /// recv()
    bool        available();

#if RH_PLATFORM == RH_PLATFORM_ESP8266
    /// Reimplementation of virtual waitPacketSent method only for ESP8266. This method calls
    /// the loopIsr method to check when a new interrupt is asserted and handles the interrupt
    /// service routine.
    bool waitPacketSent();
#endif

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

    /// Sets the length of the preamble
    /// in 4-bit nibbles. 
    /// Caution: this should be set to the same 
    /// value on all nodes in your network. Default is 8.
    /// Sets the message preamble length in RH_RF22_REG_34_PREAMBLE_LENGTH
    /// \param[in] nibbles Preamble length in nibbles of 4 bits each.  
    void           setPreambleLength(uint8_t nibbles);

    /// Sets the sync words for transmit and receive in registers RH_RF22_REG_36_SYNC_WORD3 
    /// to RH_RF22_REG_39_SYNC_WORD0
    /// Caution: SyncWords should be set to the same 
    /// value on all nodes in your network. Nodes with different SyncWords set will never receive
    /// each others messages, so different SyncWords can be used to isolate different
    /// networks from each other. Default is { 0x2d, 0xd4 }.
    /// \param[in] syncWords Array of sync words, 1 to 4 octets long
    /// \param[in] len Number of sync words to set, 1 to 4.
    void           setSyncWords(const uint8_t* syncWords, uint8_t len);

    /// Tells the receiver to accept messages with any TO address, not just messages
    /// addressed to thisAddress or the broadcast address
    /// \param[in] promiscuous true if you wish to receive messages with any TO address
    virtual void   setPromiscuous(bool promiscuous);

    /// Sets the CRC polynomial to be used to generate the CRC for both receive and transmit
    /// otherwise the default of CRC_16_IBM will be used.
    /// \param[in] polynomial One of RH_RF22::CRCPolynomial choices CRC_*
    /// \return true if polynomial is a valid option for this radio.
    bool setCRCPolynomial(CRCPolynomial polynomial);

    /// Configures GPIO pins for reversed GPIO connections to the antenna switch.
    /// Normally on RF22 modules, GPIO0(out) is connected to TX_ANT(in) to enable tx antenna during transmit
    /// and GPIO1(out) is connected to RX_ANT(in) to enable rx antenna during receive. The RH_RF22 driver
    /// configures the GPIO pins during init() so the antenna switch works as expected.
    /// However, some RF22 modules, such as HAB-RFM22B-BOA HAB-RFM22B-BO, also Si4432 sold by Dorji.com via Tindie.com
    /// have these GPIO pins reversed, so that GPIO0 is connected to RX_ANT.
    /// Call this function with a true argument after init() and before transmitting
    /// in order to configure the module for reversed GPIO pins.
    /// \param[in] gpioReversed Set to true if your RF22 module has reversed GPIO antenna switch connections.
    void setGpioReversed(bool gpioReversed = false);

    /// Returns the time in millis since the last preamble was received, and when the last
    /// RSSI measurement was made.
    uint32_t getLastPreambleTime();

    /// The maximum message length supported by this driver
    /// \return The maximum message length supported by this driver
    uint8_t maxMessageLength();

    /// Sets the radio into low-power sleep mode.
    /// If successful, the transport will stay in sleep mode until woken by 
    /// changing mode it idle, transmit or receive (eg by calling send(), recv(), available() etc)
    /// Caution: there is a time penalty as the radio takes a finite time to wake from sleep mode.
    /// \return true if sleep mode was successfully entered.
    virtual bool    sleep();

protected:
    /// This is a low level function to handle the interrupts for one instance of RH_RF22.
    /// Called automatically by isr*()
    /// Should not need to be called.
    void           handleInterrupt();

    /// Clears the receiver buffer.
    /// Internal use only
    void           clearRxBuf();

    /// Clears the transmitter buffer
    /// Internal use only
    void           clearTxBuf();

    /// Fills the transmitter buffer with the data of a mesage to be sent
    /// \param[in] data Array of data bytes to be sent (1 to 255)
    /// \param[in] len Number of data bytes in data (> 0)
    /// \return true if the message length is valid
    bool           fillTxBuf(const uint8_t* data, uint8_t len);

    /// Appends the transmitter buffer with the data of a mesage to be sent
    /// \param[in] data Array of data bytes to be sent (0 to 255)
    /// \param[in] len Number of data bytes in data
    /// \return false if the resulting message would exceed RH_RF22_MAX_MESSAGE_LEN, else true
    bool           appendTxBuf(const uint8_t* data, uint8_t len);

    /// Internal function to load the next fragment of 
    /// the current message into the transmitter FIFO
    /// Internal use only
    void           sendNextFragment();

    ///  function to copy the next fragment from 
    /// the receiver FIF) into the receiver buffer
    void           readNextFragment();

    /// Clears the RF22 Rx and Tx FIFOs
    /// Internal use only
    void           resetFifos();

    /// Clears the RF22 Rx FIFO
    /// Internal use only
    void           resetRxFifo();

    /// Clears the RF22 Tx FIFO
    /// Internal use only
    void           resetTxFifo();

    /// This function will be called by handleInterrupt() if an RF22 external interrupt occurs. 
    /// This can only happen if external interrupts are enabled in the RF22 
    /// (which they are not by default). 
    /// Subclasses may override this function to get control when an RF22 external interrupt occurs. 
    virtual void   handleExternalInterrupt();

    /// This function will be called by handleInterrupt() if an RF22 wakeup timer interrupt occurs. 
    /// This can only happen if wakeup timer interrupts are enabled in theRF22 
    /// (which they are not by default). 
    /// Subclasses may override this function to get control when an RF22 wakeup timer interrupt occurs. 
    virtual void   handleWakeupTimerInterrupt();

    /// Start the transmission of the contents 
    /// of the Tx buffer
    void           startTransmit();

    /// ReStart the transmission of the contents 
    /// of the Tx buffer after a atransmission failure
    void           restartTransmit();

    void           setThisAddress(uint8_t thisAddress);

    /// Sets the radio operating mode for the case when the driver is idle (ie not
    /// transmitting or receiving), allowing you to control the idle mode power requirements
    /// at the expense of slower transitions to transmit and receive modes.
    /// By default, the idle mode is RH_RF22_XTON,
    /// but eg setIdleMode(RH_RF22_PLL) will provide a much lower
    /// idle current but slower transitions. Call this function after init().
    /// \param[in] idleMode The chip operating mode to use when the driver is idle. One of the valid definitions for RH_RF22_REG_07_OPERATING_MODE
    void setIdleMode(uint8_t idleMode);

#if RH_PLATFORM == RH_PLATFORM_ESP8266
	/// \brief Method only for ESP8266 to avoid SPI calls from the ISRs
	///
	/// This method is used only with ESP8266 platform and must be called from
	/// the main loop. It checks if the Isr flags have been asserted and calls,
	/// from the main loop, the interrupt handler methods (where SPI calls are
	/// needed to process the communication).
	void loopIsr();
#endif

protected:
    /// Low level interrupt service routine for RF22 connected to interrupt 0
    static void         isr0();

    /// Low level interrupt service routine for RF22 connected to interrupt 1
    static void         isr1();

    /// Low level interrupt service routine for RF22 connected to interrupt 1
    static void         isr2();

    /// Array of instances connected to interrupts 0 and 1
    static RH_RF22*     _deviceForInterrupt[];

    /// Index of next interrupt number to use in _deviceForInterrupt
    static uint8_t      _interruptCount;

    /// The configured interrupt pin connected to this instance
    uint8_t             _interruptPin;

    /// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
    /// else 0xff
    uint8_t             _myInterruptIndex;

    /// The radio mode to use when mode is idle
    uint8_t             _idleMode; 

    /// The device type reported by the RF22
    uint8_t             _deviceType;

    /// The selected CRC polynomial
    CRCPolynomial       _polynomial;

    // These volatile members may get changed in the interrupt service routine
    /// Number of octets in the receiver buffer
    volatile uint8_t    _bufLen;
    
    /// The receiver buffer
    uint8_t             _buf[RH_RF22_MAX_MESSAGE_LEN];

    /// True when there is a valid message in the Rx buffer
    volatile bool       _rxBufValid;

    /// Index into TX buffer of the next to send chunk
    volatile uint8_t    _txBufSentIndex;
  
    /// Time in millis since the last preamble was received (and the last time the RSSI was measured)
    uint32_t            _lastPreambleTime;
};

/// @example rf22_client.ino
/// @example rf22_server.ino
/// @example rf22_cw.ino

#endif 
