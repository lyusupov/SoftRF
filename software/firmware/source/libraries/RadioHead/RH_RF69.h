// RH_RF69.h
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2014 Mike McCauley
// $Id: RH_RF69.h,v 1.38 2020/04/09 23:40:34 mikem Exp $
//
///


#ifndef RH_RF69_h
#define RH_RF69_h

#include <RHGenericSPI.h>
#include <RHSPIDriver.h>

// The crystal oscillator frequency of the RF69 module
#define RH_RF69_FXOSC 32000000.0

// The Frequency Synthesizer step = RH_RF69_FXOSC / 2^^19
#define RH_RF69_FSTEP  (RH_RF69_FXOSC / 524288)

// This is the maximum number of interrupts the driver can support
// Most Arduinos can handle 2, Megas can handle more
#define RH_RF69_NUM_INTERRUPTS 3

// This is the bit in the SPI address that marks it as a write
#define RH_RF69_SPI_WRITE_MASK 0x80

// Max number of octets the RH_RF69 Rx and Tx FIFOs can hold
#define RH_RF69_FIFO_SIZE 66

// Maximum encryptable payload length the RF69 can support
#define RH_RF69_MAX_ENCRYPTABLE_PAYLOAD_LEN 64

// The length of the headers we add.
// The headers are inside the RF69's payload and are therefore encrypted if encryption is enabled
#define RH_RF69_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. Limited by
// the size of the FIFO, since we are unable to support on-the-fly filling and emptying 
// of the FIFO.
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 4 bytes of address and header and payload to be included in the 64 byte encryption limit.
// the one byte payload length is not encrpyted
#ifndef RH_RF69_MAX_MESSAGE_LEN
#define RH_RF69_MAX_MESSAGE_LEN (RH_RF69_MAX_ENCRYPTABLE_PAYLOAD_LEN - RH_RF69_HEADER_LEN)
#endif

// Keep track of the mode the RF69 is in
#define RH_RF69_MODE_IDLE         0
#define RH_RF69_MODE_RX           1
#define RH_RF69_MODE_TX           2

// This is the default node address,
#define RH_RF69_DEFAULT_NODE_ADDRESS 0

// You can define the following macro (either by editing here or by passing it as a compiler definition
// to change the default value of the ishighpowermodule argument to setTxPower to true
// 
// #define RFM69_HW
#ifdef RFM69_HW
#define RH_RF69_DEFAULT_HIGHPOWER true
#else
#define RH_RF69_DEFAULT_HIGHPOWER false
#endif

// Register names
#define RH_RF69_REG_00_FIFO                                 0x00
#define RH_RF69_REG_01_OPMODE                               0x01
#define RH_RF69_REG_02_DATAMODUL                            0x02
#define RH_RF69_REG_03_BITRATEMSB                           0x03
#define RH_RF69_REG_04_BITRATELSB                           0x04
#define RH_RF69_REG_05_FDEVMSB                              0x05
#define RH_RF69_REG_06_FDEVLSB                              0x06
#define RH_RF69_REG_07_FRFMSB                               0x07
#define RH_RF69_REG_08_FRFMID                               0x08
#define RH_RF69_REG_09_FRFLSB                               0x09
#define RH_RF69_REG_0A_OSC1                                 0x0a
#define RH_RF69_REG_0B_AFCCTRL                              0x0b
#define RH_RF69_REG_0C_RESERVED                             0x0c
#define RH_RF69_REG_0D_LISTEN1                              0x0d
#define RH_RF69_REG_0E_LISTEN2                              0x0e
#define RH_RF69_REG_0F_LISTEN3                              0x0f
#define RH_RF69_REG_10_VERSION                              0x10
#define RH_RF69_REG_11_PALEVEL                              0x11
#define RH_RF69_REG_12_PARAMP                               0x12
#define RH_RF69_REG_13_OCP                                  0x13
#define RH_RF69_REG_14_RESERVED                             0x14
#define RH_RF69_REG_15_RESERVED                             0x15
#define RH_RF69_REG_16_RESERVED                             0x16
#define RH_RF69_REG_17_RESERVED                             0x17
#define RH_RF69_REG_18_LNA                                  0x18
#define RH_RF69_REG_19_RXBW                                 0x19
#define RH_RF69_REG_1A_AFCBW                                0x1a
#define RH_RF69_REG_1B_OOKPEAK                              0x1b
#define RH_RF69_REG_1C_OOKAVG                               0x1c
#define RH_RF69_REG_1D_OOKFIX                               0x1d
#define RH_RF69_REG_1E_AFCFEI                               0x1e
#define RH_RF69_REG_1F_AFCMSB                               0x1f
#define RH_RF69_REG_20_AFCLSB                               0x20
#define RH_RF69_REG_21_FEIMSB                               0x21
#define RH_RF69_REG_22_FEILSB                               0x22
#define RH_RF69_REG_23_RSSICONFIG                           0x23
#define RH_RF69_REG_24_RSSIVALUE                            0x24
#define RH_RF69_REG_25_DIOMAPPING1                          0x25
#define RH_RF69_REG_26_DIOMAPPING2                          0x26
#define RH_RF69_REG_27_IRQFLAGS1                            0x27
#define RH_RF69_REG_28_IRQFLAGS2                            0x28
#define RH_RF69_REG_29_RSSITHRESH                           0x29
#define RH_RF69_REG_2A_RXTIMEOUT1                           0x2a
#define RH_RF69_REG_2B_RXTIMEOUT2                           0x2b
#define RH_RF69_REG_2C_PREAMBLEMSB                          0x2c
#define RH_RF69_REG_2D_PREAMBLELSB                          0x2d
#define RH_RF69_REG_2E_SYNCCONFIG                           0x2e
#define RH_RF69_REG_2F_SYNCVALUE1                           0x2f
// another 7 sync word bytes follow, 30 through 36 inclusive
#define RH_RF69_REG_37_PACKETCONFIG1                        0x37
#define RH_RF69_REG_38_PAYLOADLENGTH                        0x38
#define RH_RF69_REG_39_NODEADRS                             0x39
#define RH_RF69_REG_3A_BROADCASTADRS                        0x3a
#define RH_RF69_REG_3B_AUTOMODES                            0x3b
#define RH_RF69_REG_3C_FIFOTHRESH                           0x3c
#define RH_RF69_REG_3D_PACKETCONFIG2                        0x3d
#define RH_RF69_REG_3E_AESKEY1                              0x3e
// Another 15 AES key bytes follow
#define RH_RF69_REG_4E_TEMP1                                0x4e
#define RH_RF69_REG_4F_TEMP2                                0x4f
#define RH_RF69_REG_58_TESTLNA                              0x58
#define RH_RF69_REG_5A_TESTPA1                              0x5a
#define RH_RF69_REG_5C_TESTPA2                              0x5c
#define RH_RF69_REG_6F_TESTDAGC                             0x6f
#define RH_RF69_REG_71_TESTAFC                              0x71

// These register masks etc are named wherever possible
// corresponding to the bit and field names in the RFM69 Manual

// RH_RF69_REG_01_OPMODE
#define RH_RF69_OPMODE_SEQUENCEROFF                         0x80
#define RH_RF69_OPMODE_LISTENON                             0x40
#define RH_RF69_OPMODE_LISTENABORT                          0x20
#define RH_RF69_OPMODE_MODE                                 0x1c
#define RH_RF69_OPMODE_MODE_SLEEP                           0x00
#define RH_RF69_OPMODE_MODE_STDBY                           0x04
#define RH_RF69_OPMODE_MODE_FS                              0x08
#define RH_RF69_OPMODE_MODE_TX                              0x0c
#define RH_RF69_OPMODE_MODE_RX                              0x10

// RH_RF69_REG_02_DATAMODUL
#define RH_RF69_DATAMODUL_DATAMODE                          0x60
#define RH_RF69_DATAMODUL_DATAMODE_PACKET                   0x00
#define RH_RF69_DATAMODUL_DATAMODE_CONT_WITH_SYNC           0x40
#define RH_RF69_DATAMODUL_DATAMODE_CONT_WITHOUT_SYNC        0x60
#define RH_RF69_DATAMODUL_MODULATIONTYPE                    0x18
#define RH_RF69_DATAMODUL_MODULATIONTYPE_FSK                0x00
#define RH_RF69_DATAMODUL_MODULATIONTYPE_OOK                0x08
#define RH_RF69_DATAMODUL_MODULATIONSHAPING                 0x03
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_NONE        0x00
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT1_0       0x01
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT0_5       0x02
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT0_3       0x03
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_OOK_NONE        0x00
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_OOK_BR          0x01
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_OOK_2BR         0x02

// RH_RF69_REG_11_PALEVEL
#define RH_RF69_PALEVEL_PA0ON                               0x80
#define RH_RF69_PALEVEL_PA1ON                               0x40
#define RH_RF69_PALEVEL_PA2ON                               0x20
#define RH_RF69_PALEVEL_OUTPUTPOWER                         0x1f

// RH_RF69_REG_23_RSSICONFIG
#define RH_RF69_RSSICONFIG_RSSIDONE                         0x02
#define RH_RF69_RSSICONFIG_RSSISTART                        0x01

// RH_RF69_REG_25_DIOMAPPING1
#define RH_RF69_DIOMAPPING1_DIO0MAPPING                     0xc0
#define RH_RF69_DIOMAPPING1_DIO0MAPPING_00                  0x00
#define RH_RF69_DIOMAPPING1_DIO0MAPPING_01                  0x40
#define RH_RF69_DIOMAPPING1_DIO0MAPPING_10                  0x80
#define RH_RF69_DIOMAPPING1_DIO0MAPPING_11                  0xc0

#define RH_RF69_DIOMAPPING1_DIO1MAPPING                     0x30
#define RH_RF69_DIOMAPPING1_DIO1MAPPING_00                  0x00
#define RH_RF69_DIOMAPPING1_DIO1MAPPING_01                  0x10
#define RH_RF69_DIOMAPPING1_DIO1MAPPING_10                  0x20
#define RH_RF69_DIOMAPPING1_DIO1MAPPING_11                  0x30

#define RH_RF69_DIOMAPPING1_DIO2MAPPING                     0x0c
#define RH_RF69_DIOMAPPING1_DIO2MAPPING_00                  0x00
#define RH_RF69_DIOMAPPING1_DIO2MAPPING_01                  0x04
#define RH_RF69_DIOMAPPING1_DIO2MAPPING_10                  0x08
#define RH_RF69_DIOMAPPING1_DIO2MAPPING_11                  0x0c

#define RH_RF69_DIOMAPPING1_DIO3MAPPING                     0x03
#define RH_RF69_DIOMAPPING1_DIO3MAPPING_00                  0x00
#define RH_RF69_DIOMAPPING1_DIO3MAPPING_01                  0x01
#define RH_RF69_DIOMAPPING1_DIO3MAPPING_10                  0x02
#define RH_RF69_DIOMAPPING1_DIO3MAPPING_11                  0x03

// RH_RF69_REG_26_DIOMAPPING2
#define RH_RF69_DIOMAPPING2_DIO4MAPPING                     0xc0
#define RH_RF69_DIOMAPPING2_DIO4MAPPING_00                  0x00
#define RH_RF69_DIOMAPPING2_DIO4MAPPING_01                  0x40
#define RH_RF69_DIOMAPPING2_DIO4MAPPING_10                  0x80
#define RH_RF69_DIOMAPPING2_DIO4MAPPING_11                  0xc0

#define RH_RF69_DIOMAPPING2_DIO5MAPPING                     0x30
#define RH_RF69_DIOMAPPING2_DIO5MAPPING_00                  0x00
#define RH_RF69_DIOMAPPING2_DIO5MAPPING_01                  0x10
#define RH_RF69_DIOMAPPING2_DIO5MAPPING_10                  0x20
#define RH_RF69_DIOMAPPING2_DIO5MAPPING_11                  0x30

#define RH_RF69_DIOMAPPING2_CLKOUT                          0x07
#define RH_RF69_DIOMAPPING2_CLKOUT_FXOSC_                   0x00
#define RH_RF69_DIOMAPPING2_CLKOUT_FXOSC_2                  0x01
#define RH_RF69_DIOMAPPING2_CLKOUT_FXOSC_4                  0x02
#define RH_RF69_DIOMAPPING2_CLKOUT_FXOSC_8                  0x03
#define RH_RF69_DIOMAPPING2_CLKOUT_FXOSC_16                 0x04
#define RH_RF69_DIOMAPPING2_CLKOUT_FXOSC_32                 0x05
#define RH_RF69_DIOMAPPING2_CLKOUT_FXOSC_RC                 0x06
#define RH_RF69_DIOMAPPING2_CLKOUT_FXOSC_OFF                0x07

// RH_RF69_REG_27_IRQFLAGS1
#define RH_RF69_IRQFLAGS1_MODEREADY                         0x80
#define RH_RF69_IRQFLAGS1_RXREADY                           0x40
#define RH_RF69_IRQFLAGS1_TXREADY                           0x20
#define RH_RF69_IRQFLAGS1_PLLLOCK                           0x10
#define RH_RF69_IRQFLAGS1_RSSI                              0x08
#define RH_RF69_IRQFLAGS1_TIMEOUT                           0x04
#define RH_RF69_IRQFLAGS1_AUTOMODE                          0x02
#define RH_RF69_IRQFLAGS1_SYNADDRESSMATCH                   0x01

// RH_RF69_REG_28_IRQFLAGS2
#define RH_RF69_IRQFLAGS2_FIFOFULL                          0x80
#define RH_RF69_IRQFLAGS2_FIFONOTEMPTY                      0x40
#define RH_RF69_IRQFLAGS2_FIFOLEVEL                         0x20
#define RH_RF69_IRQFLAGS2_FIFOOVERRUN                       0x10
#define RH_RF69_IRQFLAGS2_PACKETSENT                        0x08
#define RH_RF69_IRQFLAGS2_PAYLOADREADY                      0x04
#define RH_RF69_IRQFLAGS2_CRCOK                             0x02

// RH_RF69_REG_2E_SYNCCONFIG
#define RH_RF69_SYNCCONFIG_SYNCON                           0x80
#define RH_RF69_SYNCCONFIG_FIFOFILLCONDITION_MANUAL         0x40
#define RH_RF69_SYNCCONFIG_SYNCSIZE                         0x38
#define RH_RF69_SYNCCONFIG_SYNCSIZE_1                       0x00
#define RH_RF69_SYNCCONFIG_SYNCSIZE_2                       0x08
#define RH_RF69_SYNCCONFIG_SYNCSIZE_3                       0x10
#define RH_RF69_SYNCCONFIG_SYNCSIZE_4                       0x18
#define RH_RF69_SYNCCONFIG_SYNCSIZE_5                       0x20
#define RH_RF69_SYNCCONFIG_SYNCSIZE_6                       0x28
#define RH_RF69_SYNCCONFIG_SYNCSIZE_7                       0x30
#define RH_RF69_SYNCCONFIG_SYNCSIZE_8                       0x38
#define RH_RF69_SYNCCONFIG_SYNCSIZE_SYNCTOL                 0x07

// RH_RF69_REG_37_PACKETCONFIG1
#define RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE         0x80
#define RH_RF69_PACKETCONFIG1_DCFREE                        0x60
#define RH_RF69_PACKETCONFIG1_DCFREE_NONE                   0x00
#define RH_RF69_PACKETCONFIG1_DCFREE_MANCHESTER             0x20
#define RH_RF69_PACKETCONFIG1_DCFREE_WHITENING              0x40
#define RH_RF69_PACKETCONFIG1_DCFREE_RESERVED               0x60
#define RH_RF69_PACKETCONFIG1_CRC_ON                        0x10
#define RH_RF69_PACKETCONFIG1_CRCAUTOCLEAROFF               0x08
#define RH_RF69_PACKETCONFIG1_ADDRESSFILTERING              0x06
#define RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE         0x00
#define RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NODE         0x02
#define RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NODE_BC      0x04
#define RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_RESERVED     0x06

// RH_RF69_REG_3B_AUTOMODES
#define RH_RF69_AUTOMODE_ENTER_COND_NONE                    0x00
#define RH_RF69_AUTOMODE_ENTER_COND_FIFO_NOT_EMPTY          0x20
#define RH_RF69_AUTOMODE_ENTER_COND_FIFO_LEVEL              0x40
#define RH_RF69_AUTOMODE_ENTER_COND_CRC_OK                  0x60
#define RH_RF69_AUTOMODE_ENTER_COND_PAYLOAD_READY           0x80
#define RH_RF69_AUTOMODE_ENTER_COND_SYNC_ADDRESS            0xa0
#define RH_RF69_AUTOMODE_ENTER_COND_PACKET_SENT             0xc0
#define RH_RF69_AUTOMODE_ENTER_COND_FIFO_EMPTY              0xe0

#define RH_RF69_AUTOMODE_EXIT_COND_NONE                     0x00
#define RH_RF69_AUTOMODE_EXIT_COND_FIFO_EMPTY               0x04
#define RH_RF69_AUTOMODE_EXIT_COND_FIFO_LEVEL               0x08
#define RH_RF69_AUTOMODE_EXIT_COND_CRC_OK                   0x0c
#define RH_RF69_AUTOMODE_EXIT_COND_PAYLOAD_READY            0x10
#define RH_RF69_AUTOMODE_EXIT_COND_SYNC_ADDRESS             0x14
#define RH_RF69_AUTOMODE_EXIT_COND_PACKET_SENT              0x18
#define RH_RF69_AUTOMODE_EXIT_COND_TIMEOUT                  0x1c

#define RH_RF69_AUTOMODE_INTERMEDIATE_MODE_SLEEP            0x00
#define RH_RF69_AUTOMODE_INTERMEDIATE_MODE_STDBY            0x01
#define RH_RF69_AUTOMODE_INTERMEDIATE_MODE_RX               0x02
#define RH_RF69_AUTOMODE_INTERMEDIATE_MODE_TX               0x03

// RH_RF69_REG_3C_FIFOTHRESH
#define RH_RF69_FIFOTHRESH_TXSTARTCONDITION_NOTEMPTY        0x80
#define RH_RF69_FIFOTHRESH_FIFOTHRESHOLD                    0x7f

// RH_RF69_REG_3D_PACKETCONFIG2
#define RH_RF69_PACKETCONFIG2_INTERPACKETRXDELAY            0xf0
#define RH_RF69_PACKETCONFIG2_RESTARTRX                     0x04
#define RH_RF69_PACKETCONFIG2_AUTORXRESTARTON               0x02
#define RH_RF69_PACKETCONFIG2_AESON                         0x01

// RH_RF69_REG_4E_TEMP1
#define RH_RF69_TEMP1_TEMPMEASSTART                         0x08
#define RH_RF69_TEMP1_TEMPMEASRUNNING                       0x04

// RH_RF69_REG_5A_TESTPA1
#define RH_RF69_TESTPA1_NORMAL                              0x55
#define RH_RF69_TESTPA1_BOOST                               0x5d

// RH_RF69_REG_5C_TESTPA2
#define RH_RF69_TESTPA2_NORMAL                              0x70
#define RH_RF69_TESTPA2_BOOST                               0x7c

// RH_RF69_REG_6F_TESTDAGC
#define RH_RF69_TESTDAGC_CONTINUOUSDAGC_NORMAL              0x00
#define RH_RF69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAON  0x20
#define RH_RF69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAOFF 0x30

// Define this to include Serial printing in diagnostic routines
#define RH_RF69_HAVE_SERIAL


/////////////////////////////////////////////////////////////////////
/// \class RH_RF69 RH_RF69.h <RH_RF69.h>
/// \brief Driver to send and receive unaddressed, unreliable datagrams via an RF69 and compatible radio transceiver.
///
/// Works with 
/// - the excellent Moteino and Moteino-USB 
/// boards from LowPowerLab http://lowpowerlab.com/moteino/
/// - compatible chips and modules such as RFM69W, RFM69HW, RFM69CW, RFM69HCW (Semtech SX1231, SX1231H),
/// - RFM69 modules from http://www.hoperfusa.com such as http://www.hoperfusa.com/details.jsp?pid=145
/// - Anarduino MiniWireless -CW and -HW boards http://www.anarduino.com/miniwireless/ including
///  the marvellous high powered MinWireless-HW (with 20dBm output for excellent range)
/// - the excellent Rocket Scream Mini Ultra Pro with the RFM69HCW 
///   http://www.rocketscream.com/blog/product/mini-ultra-pro-with-radio/
/// - The excellent Talk2 Whisper Node boards 
///   (https://talk2.wisen.com.au/ and https://bitbucket.org/talk2/whisper-node-avr), 
///   an Arduino compatible board, which include an on-board RF69 radio, external antenna, 
///   run on 2xAAA batteries and support low power operations. RF69 examples work without modification.
///   Use Arduino Board Manager to install the Talk2 code support as described in 
///   https://bitbucket.org/talk2/whisper-node-avr. Upeload the code with an FTDI adapter set to 3.3V.
/// - The excellent Adafruit Feather. These are excellent boards that are available with a variety of radios. 
///   We tested with the 
///   Feather 32u4 with RFM69HCW radio, with Arduino IDE 1.6.8 and the Adafruit AVR Boards board manager version 1.6.10.
///   https://www.adafruit.com/products/3076
///
/// \par Overview
///
/// This class provides basic functions for sending and receiving unaddressed, 
/// unreliable datagrams of arbitrary length to 64 octets per packet.
///
/// Manager classes may use this class to implement reliable, addressed datagrams and streams, 
/// mesh routers, repeaters, translators etc.
///
/// Naturally, for any 2 radios to communicate that must be configured to use the same frequency and 
/// modulation scheme.
///
/// This Driver provides an object-oriented interface for sending and receiving data messages with Hope-RF
/// RF69B and compatible radio modules, such as the RFM69 module.
///
/// The Hope-RF (http://www.hoperf.com) RF69 is a low-cost ISM transceiver
/// chip. It supports FSK, GFSK, OOK over a wide range of frequencies and
/// programmable data rates. It also suports AES encryption of up to 64 octets
/// of payload It is available prepackaged on modules such as the RFM69W. And
/// such modules can be prepacked on processor boards such as the Moteino from
/// LowPowerLabs (which is what we used to develop the RH_RF69 driver)
///
/// This Driver provides functions for sending and receiving messages of up
/// to 60 octets on any frequency supported by the RF69, in a range of
/// predefined data rates and frequency deviations.  Frequency can be set with
/// 61Hz precision to any frequency from 240.0MHz to 960.0MHz. Caution: most modules only support a more limited
/// range of frequencies due to antenna tuning.
///
/// Up to 2 RF69B modules can be connected to an Arduino (3 on a Mega),
/// permitting the construction of translators and frequency changers, etc.
///
/// The following modulation types are suppported with a range of modem configurations for 
/// common data rates and frequency deviations:
/// - GFSK Gaussian Frequency Shift Keying
/// - FSK Frequency Shift Keying
///
/// Support for other RF69 features such as on-chip temperature measurement, 
/// transmitter power control etc is also provided.
///
/// Tested on USB-Moteino with arduino-1.0.5
/// on OpenSuSE 13.1
///
/// \par Packet Format
///
/// All messages sent and received by this RH_RF69 Driver conform to this packet format:
///
/// - 4 octets PREAMBLE
/// - 2 octets SYNC 0x2d, 0xd4 (configurable, so you can use this as a network filter)
/// - 1 octet RH_RF69 payload length
/// - 4 octets HEADER: (TO, FROM, ID, FLAGS)
/// - 0 to 60 octets DATA 
/// - 2 octets CRC computed with CRC16(IBM), computed on HEADER and DATA
///
/// For technical reasons, the message format is not protocol compatible with the
/// 'HopeRF Radio Transceiver Message Library for Arduino'
/// http://www.airspayce.com/mikem/arduino/HopeRF from the same author. Nor is
/// it compatible with messages sent by 'Virtual Wire'
/// http://www.airspayce.com/mikem/arduino/VirtualWire.pdf also from the same
/// author.  Nor is it compatible with messages sent by 'RF22'
/// http://www.airspayce.com/mikem/arduino/RF22 also from the same author.
///
/// \par Connecting RFM-69 to Arduino
///
/// We tested with Moteino, which is an Arduino Uno compatible with the RFM69W
/// module on-board. Therefore it needs no connections other than the USB
/// programming connection and an antenna to make it work.
///
/// If you have a bare RFM69W that you want to connect to an Arduino, you
/// might use these connections: CAUTION: you must use a 3.3V type
/// Arduino, otherwise you will also need voltage level shifters between the
/// Arduino and the RFM69.  CAUTION, you must also ensure you connect an
/// antenna
/// 
/// \code
///                 Arduino      RFM69W
///                 GND----------GND   (ground in)
///                 3V3----------3.3V  (3.3V in)
/// interrupt 0 pin D2-----------DIO0  (interrupt request out)
///          SS pin D10----------NSS   (chip select in)
///         SCK pin D13----------SCK   (SPI clock in)
///        MOSI pin D11----------MOSI  (SPI Data in)
///        MISO pin D12----------MISO  (SPI Data out)
/// \endcode
///
/// For Arduino Due, use these connections:
/// \code
///                 Arduino      RFM69W
///                 GND----------GND   (ground in)
///                 3V3----------3.3V  (3.3V in)
/// interrupt 0 pin D2-----------DIO0  (interrupt request out)
///          SS pin D10----------NSS   (chip select in)
///       SCK SPI pin 3----------SCK   (SPI clock in)
///      MOSI SPI pin 4----------MOSI  (SPI Data in)
///      MISO SPI pin 1----------MISO  (SPI Data out)
/// \endcode
///
/// With these connections, you can then use the default constructor RH_RF69().
/// You can override the default settings for the SS pin and the interrupt in
/// the RH_RF69 constructor if you wish to connect the slave select SS to other
/// than the normal one for your Arduino (D10 for Diecimila, Uno etc and D53
/// for Mega) or the interrupt request to other than pin D2 (Caution,
/// different processors have different constraints as to the pins available
/// for interrupts).
///
/// If you have a Teensy 3.1 and a compatible RFM69 breakout board, you will need to 
/// construct the RH_RF69 instance like this:
/// \code
/// RH_RF69 driver(15, 16);
/// \endcode
///
/// If you have a MoteinoMEGA https://lowpowerlab.com/shop/moteinomega
/// with RFM69 on board, you dont need to make any wiring connections 
/// (the RFM69 module is soldered onto the MotienoMEGA), but you must initialise the RH_RF69
/// constructor like this:
/// \code
/// RH_RF69 driver(4, 2);
/// \endcode
/// Make sure you have the MoteinoMEGA core installed in your Arduino hardware folder as described in the
/// documentation for the MoteinoMEGA.
///
/// If you have an Arduino M0 Pro from arduino.org, 
/// you should note that you cannot use Pin 2 for the interrupt line 
/// (Pin 2 is for the NMI only). The same comments apply to Pin 4 on Arduino Zero from arduino.cc.
/// Instead you can use any other pin (we use Pin 3) and initialise RH_RF69 like this:
/// \code
/// // Slave Select is pin 10, interrupt is Pin 3
/// RH_RF69 driver(10, 3);
/// \endcode
///
/// If you have a Rocket Scream Mini Ultra Pro with the RFM69HCW
/// - Ensure you have Arduino SAMD board support 1.6.5 or later in Arduino IDE 1.6.8 or later.
/// - The radio SS is hardwired to pin D5 and the DIO0 interrupt to pin D2, 
/// so you need to initialise the radio like this:
/// \code
/// RH_RF69 driver(5, 2);
/// \endcode
/// - The name of the serial port on that board is 'SerialUSB', not 'Serial', so this may be helpful at the top of our
///   sample sketches:
/// \code
/// #define Serial SerialUSB
/// \endcode
/// - You also need this in setup before radio initialisation  
/// \code
/// // Ensure serial flash is not interfering with radio communication on SPI bus
///  pinMode(4, OUTPUT);
///  digitalWrite(4, HIGH);
/// \endcode
/// - and if you have a 915MHz part, you need this after driver/manager intitalisation:
/// \code
/// rf69.setFrequency(915.0);
/// rf69.setTxPower(20);
/// \endcode
/// which adds up to modifying sample sketches something like:
/// \code
/// #include <SPI.h>
/// #include <RH_RF69.h>
/// RH_RF69 rf69(5, 2); // Rocket Scream Mini Ultra Pro with the RFM69HCW
/// #define Serial SerialUSB
/// 
/// void setup() 
/// {
///   // Ensure serial flash is not interfering with radio communication on SPI bus
///   pinMode(4, OUTPUT);
///   digitalWrite(4, HIGH);
/// 
///   Serial.begin(9600);
///   while (!Serial) ; // Wait for serial port to be available
///   if (!rf69.init())
///     Serial.println("init failed");
///   rf69.setFrequency(915.0);
///   rf69.setTxPower(20);
/// }
/// ...
/// \endcode
///
/// If you have a talk2 Whisper Node board with on-board RF69 radio, 
/// the example rf69_* sketches work without modifications. Initialise the radio like
/// with the default constructor:
/// \code
///  RH_RF69 driver;
/// \endcode
///
/// If you have a Feather 32u4 with RFM69HCW you need to initialise the driver like:
/// \code
///  RH_RF69 driver(8, 7);
/// \endcode
/// and since the radio is the high power HCW model, you must set the Tx power in the
/// range 14 to 20 like this:
/// \code
///  driver.setTxPower(14);
/// \endcode
///
/// If you are connecting an RF69 to a ESP8266 board breakout board that exposes pins
/// 12, 13, 14, 15 (ie NOT an ESP-01) you can connect like this:
/// \code
///                   ESP8266      RFM69W
///                    GND-----------GND   (ground in)
///                    VIN-----------3.3V  (3.3V in)
/// interrupt D0 pin GPIO0-----------DIO0  (interrupt request out)
///           SS pin GPIO15----------NSS   (chip select in)
///      SCK SPI pin GPIO14----------SCK   (SPI clock in)
///     MOSI SPI pin GPIO13----------MOSI  (SPI Data in)
///     MISO SPI pin GPIO12----------MISO  (SPI Data out)
/// \endcode
/// and initialise with
/// \code
/// RH_RF69 driver(15, 0);
/// \endcode
/// If you are connecting an RF69 to a Sparkfun nRF52832 Breakout board 
/// with Arduino 1.8.9 with board:
/// "SparkFun nRF52 Boards by Sparkfun Electronics version 0.2.3",
/// you can connect like this:
/// \code
///                 nRF52832     RFM69W
///                 GND----------GND   (ground in)
///                 3V3----------3.3V  (3.3V in)
/// interrupt 0 pin 02-----------DIO0  (interrupt request out)
///          SS pin 08-----------NSS   (chip select in)
///     SCK SPI pin 13-----------SCK   (SPI clock in)
///    MOSI SPI pin 11-----------MOSI  (SPI Data in)
///    MISO SPI pin 12-----------MISO  (SPI Data out)
/// \endcode
/// and initialise with
/// \code
/// RHSoftwareSPI softwarespi;
/// RH_RF69 driver(8, 2, softwarespi);
/// and inside your setup() function:
///    softwarespi.setPins(12, 11, 13);
/// \endcode
///
/// It is possible to have 2 or more radios connected to one Arduino, provided
/// each radio has its own SS and interrupt line (SCK, SDI and SDO are common
/// to all radios)
///
/// Caution: on some Arduinos such as the Mega 2560, if you set the slave
/// select pin to be other than the usual SS pin (D53 on Mega 2560), you may
/// need to set the usual SS pin to be an output to force the Arduino into SPI
/// master mode.
///
/// Caution: Power supply requirements of the RF69 module may be relevant in some circumstances: 
/// RF69 modules are capable of pulling 45mA+ at full power, where Arduino's 3.3V line can
/// give 50mA. You may need to make provision for alternate power supply for
/// the RF69, especially if you wish to use full transmit power, and/or you have
/// other shields demanding power. Inadequate power for the RF69 is likely to cause symptoms such as:
/// -reset's/bootups terminate with "init failed" messages
/// -random termination of communication after 5-30 packets sent/received
/// -"fake ok" state, where initialization passes fluently, but communication doesn't happen
/// -shields hang Arduino boards, especially during the flashing
///
/// \par Encryption
///
/// This driver support the on-chip AES encryption provided by the RF69.
/// You can enable encryption by calling setEncryptionKey() after init() has been called.
/// If both transmitter and receiver have been configured with the same AES key,
/// then the receiver will recover the unencrypted message sent by the receiver.
/// However, you should note that there is no way for RF69 nor for the RadioHead
/// drivers to know whether the AES 
/// key for a message is 'correct' or not. This is because the RF69 CRC covers the 
/// _encrypted_ payload not the plaintext.
///
/// In RadioHead managers that support addressing,
/// the RF69 AES encryption includes the RadioHead payload and the TO and FROM addresses, so 
/// occasionally (average one in 256 messages), a message encrypted with the 
/// 'wrong' key will have the 'correct' destination address, and will therefore be 
/// accepted by RadioHead as a 'random' message content from a 'random' sender.
/// Its up to your code to figure out whether the message makes sense or not.
///
/// \par Interrupts
///
/// The RH_RF69 driver uses interrupts to react to events in the RF69 module,
/// such as the reception of a new packet, or the completion of transmission
/// of a packet.  The RH_RF69 driver interrupt service routine reads status from
/// and writes data to the the RF69 module via the SPI interface. It is very
/// important therefore, that if you are using the RH_RF69 driver with another
/// SPI based deviced, that you disable interrupts while you transfer data to
/// and from that other device.  Use cli() to disable interrupts and sei() to
/// reenable them.
///
/// \par Memory
///
/// The RH_RF69 driver requires non-trivial amounts of memory. The sample
/// programs above all compile to about 8kbytes each, which will fit in the
/// flash proram memory of most Arduinos. However, the RAM requirements are
/// more critical. Therefore, you should be vary sparing with RAM use in
/// programs that use the RH_RF69 driver.
///
/// It is often hard to accurately identify when you are hitting RAM limits on Arduino. 
/// The symptoms can include:
/// - Mysterious crashes and restarts
/// - Changes in behaviour when seemingly unrelated changes are made (such as adding print() statements)
/// - Hanging
/// - Output from Serial.print() not appearing
/// 
/// \par Automatic Frequency Control (AFC)
///
/// The RF69 module is configured by the RH_RF69 driver to always use AFC.
///
/// \par Transmitter Power
///
/// You can control the transmitter power on the RF69 transceiver
/// with the RH_RF69::setTxPower() function. The argument can be any of
/// -18 to +13 (for RF69W) or -14 to 20 (for RF69HW) 
/// The default is 13. Eg:
/// \code
/// driver.setTxPower(-5);
/// \endcode
///
/// We have made some actual power measurements against
/// programmed power for Moteino (with RF69W)
/// - Moteino (with RF69W), USB power
/// - 10cm RG58C/U soldered direct to RFM69 module ANT and GND
/// - bnc connecteor
/// - 12dB attenuator
/// - BNC-SMA adapter
/// - MiniKits AD8307 HF/VHF Power Head (calibrated against Rohde&Schwartz 806.2020 test set)
/// - Tektronix TDS220 scope to measure the Vout from power head
/// \code
/// Program power           Measured Power
///    dBm                         dBm
///    -18                         -17
///    -16                         -16
///    -14                         -14
///    -12                         -12
///    -10                         -9
///    -8                          -7
///    -6                          -4
///    -4                          -3
///    -2                          -2
///    0                           0.2
///    2                           3
///    4                           5
///    6                           7
///    8                           10
///    10                          13
///    12                          14
///    13                          15
///    14                         -51
///    20                         -51
/// \endcode
/// We have also made some actual power measurements against
/// programmed power for Anarduino MiniWireless with RFM69-HW
/// Anarduino MiniWireless (with RFM69-HW), USB power
/// - 10cm RG58C/U soldered direct to RFM69 module ANT and GND
/// - bnc connecteor
/// - 2x12dB attenuators
/// - BNC-SMA adapter
/// - MiniKits AD8307 HF/VHF Power Head (calibrated against Rohde&Schwartz 806.2020 test set)
/// - Tektronix TDS220 scope to measure the Vout from power head
/// \code
/// Program power           Measured Power
///    dBm                         dBm
///    -18                         no measurable output
///    0                           no measurable output
///    13                          no measurable output
///    14                          11
///    15                          12
///    16                          12.4
///    17                          14
///    18                          15
///    19                          15.8
///    20                          17
/// \endcode
/// (Caution: we dont claim laboratory accuracy for these measurements)
/// You would not expect to get anywhere near these powers to air with a simple 1/4 wavelength wire antenna.
/// Caution: although the RFM69 appears to have a PC antenna on board, you will get much better power and range even 
/// with just a 1/4 wave wire antenna.
///
/// \par Performance
///
/// Some simple speed performance tests have been conducted.
/// In general packet transmission rate will be limited by the modulation scheme.
/// Also, if your code does any slow operations like Serial printing it will also limit performance. 
/// We disabled any printing in the tests below.
/// We tested with RH_RF69::GFSK_Rb250Fd250, which is probably the fastest scheme available.
/// We tested with a 13 octet message length, over a very short distance of 10cm.
///
/// Transmission (no reply) tests with modulation RH_RF69::GFSK_Rb250Fd250 and a 
/// 13 octet message show about 152 messages per second transmitted and received.
///
/// Transmit-and-wait-for-a-reply tests with modulation RH_RF69::GFSK_Rb250Fd250 and a 
/// 13 octet message (send and receive) show about 68 round trips per second.
///
class RH_RF69 : public RHSPIDriver
{
public:

    /// \brief Defines register values for a set of modem configuration registers
    ///
    /// Defines register values for a set of modem configuration registers
    /// that can be passed to setModemRegisters() if none of the choices in
    /// ModemConfigChoice suit your need setModemRegisters() writes the
    /// register values from this structure to the appropriate RF69 registers
    /// to set the desired modulation type, data rate and deviation/bandwidth.
    typedef struct
    {
	uint8_t    reg_02;   ///< Value for register RH_RF69_REG_02_DATAMODUL
	uint8_t    reg_03;   ///< Value for register RH_RF69_REG_03_BITRATEMSB
	uint8_t    reg_04;   ///< Value for register RH_RF69_REG_04_BITRATELSB
	uint8_t    reg_05;   ///< Value for register RH_RF69_REG_05_FDEVMSB
	uint8_t    reg_06;   ///< Value for register RH_RF69_REG_06_FDEVLSB
	uint8_t    reg_19;   ///< Value for register RH_RF69_REG_19_RXBW
	uint8_t    reg_1a;   ///< Value for register RH_RF69_REG_1A_AFCBW
	uint8_t    reg_37;   ///< Value for register RH_RF69_REG_37_PACKETCONFIG1
    } ModemConfig;
  
    /// Choices for setModemConfig() for a selected subset of common
    /// modulation types, and data rates. If you need another configuration,
    /// use the register calculator.  and call setModemRegisters() with your
    /// desired settings.  
    /// These are indexes into MODEM_CONFIG_TABLE. We strongly recommend you use these symbolic
    /// definitions and not their integer equivalents: its possible that new values will be
    /// introduced in later versions (though we will try to avoid it).
    /// CAUTION: some of these configurations do not work corectly and are marked as such.
    typedef enum
    {
	FSK_Rb2Fd5 = 0,	   ///< FSK, Whitening, Rb = 2kbs,    Fd = 5kHz
	FSK_Rb2_4Fd4_8,    ///< FSK, Whitening, Rb = 2.4kbs,  Fd = 4.8kHz 
	FSK_Rb4_8Fd9_6,    ///< FSK, Whitening, Rb = 4.8kbs,  Fd = 9.6kHz 
	FSK_Rb9_6Fd19_2,   ///< FSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz
	FSK_Rb19_2Fd38_4,  ///< FSK, Whitening, Rb = 19.2kbs, Fd = 38.4kHz
	FSK_Rb38_4Fd76_8,  ///< FSK, Whitening, Rb = 38.4kbs, Fd = 76.8kHz
	FSK_Rb57_6Fd120,   ///< FSK, Whitening, Rb = 57.6kbs, Fd = 120kHz
	FSK_Rb125Fd125,    ///< FSK, Whitening, Rb = 125kbs,  Fd = 125kHz
	FSK_Rb250Fd250,    ///< FSK, Whitening, Rb = 250kbs,  Fd = 250kHz
	FSK_Rb55555Fd50,   ///< FSK, Whitening, Rb = 55555kbs,Fd = 50kHz for RFM69 lib compatibility

	GFSK_Rb2Fd5,	    ///< GFSK, Whitening, Rb = 2kbs,    Fd = 5kHz
	GFSK_Rb2_4Fd4_8,    ///< GFSK, Whitening, Rb = 2.4kbs,  Fd = 4.8kHz
	GFSK_Rb4_8Fd9_6,    ///< GFSK, Whitening, Rb = 4.8kbs,  Fd = 9.6kHz
	GFSK_Rb9_6Fd19_2,   ///< GFSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz
	GFSK_Rb19_2Fd38_4,  ///< GFSK, Whitening, Rb = 19.2kbs, Fd = 38.4kHz
	GFSK_Rb38_4Fd76_8,  ///< GFSK, Whitening, Rb = 38.4kbs, Fd = 76.8kHz
	GFSK_Rb57_6Fd120,   ///< GFSK, Whitening, Rb = 57.6kbs, Fd = 120kHz
	GFSK_Rb125Fd125,    ///< GFSK, Whitening, Rb = 125kbs,  Fd = 125kHz
	GFSK_Rb250Fd250,    ///< GFSK, Whitening, Rb = 250kbs,  Fd = 250kHz
	GFSK_Rb55555Fd50,   ///< GFSK, Whitening, Rb = 55555kbs,Fd = 50kHz

	OOK_Rb1Bw1,         ///< OOK, Whitening, Rb = 1kbs,    Rx Bandwidth = 1kHz. 
	OOK_Rb1_2Bw75,      ///< OOK, Whitening, Rb = 1.2kbs,  Rx Bandwidth = 75kHz. 
	OOK_Rb2_4Bw4_8,     ///< OOK, Whitening, Rb = 2.4kbs,  Rx Bandwidth = 4.8kHz. 
	OOK_Rb4_8Bw9_6,     ///< OOK, Whitening, Rb = 4.8kbs,  Rx Bandwidth = 9.6kHz. 
	OOK_Rb9_6Bw19_2,    ///< OOK, Whitening, Rb = 9.6kbs,  Rx Bandwidth = 19.2kHz. 
	OOK_Rb19_2Bw38_4,   ///< OOK, Whitening, Rb = 19.2kbs, Rx Bandwidth = 38.4kHz. 
	OOK_Rb32Bw64,       ///< OOK, Whitening, Rb = 32kbs,   Rx Bandwidth = 64kHz. 

//	Test,
    } ModemConfigChoice;

    /// Constructor. You can have multiple instances, but each instance must have its own
    /// interrupt and slave select pin. After constructing, you must call init() to initialise the interface
    /// and the radio module. A maximum of 3 instances can co-exist on one processor, provided there are sufficient
    /// distinct interrupt lines, one for each instance.
    /// \param[in] slaveSelectPin the Arduino pin number of the output to use to select the RF69 before
    /// accessing it. Defaults to the normal SS pin for your Arduino (D10 for Diecimila, Uno etc, D53 for Mega, D10 for Maple)
    /// \param[in] interruptPin The interrupt Pin number that is connected to the RF69 DIO0 interrupt line. 
    /// Defaults to pin 2.
    /// Caution: You must specify an interrupt capable pin.
    /// On many Arduino boards, there are limitations as to which pins may be used as interrupts.
    /// On Leonardo pins 0, 1, 2 or 3. On Mega2560 pins 2, 3, 18, 19, 20, 21. On Due and Teensy, any digital pin.
    /// On Arduino Zero from arduino.cc, any digital pin other than 4.
    /// On Arduino M0 Pro from arduino.org, any digital pin other than 2.
    /// On other Arduinos pins 2 or 3. 
    /// See http://arduino.cc/en/Reference/attachInterrupt for more details.
    /// On Chipkit Uno32, pins 38, 2, 7, 8, 35.
    /// On other boards, any digital pin may be used.
    /// \param[in] spi Pointer to the SPI interface object to use. 
    ///                Defaults to the standard Arduino hardware SPI interface
    RH_RF69(uint8_t slaveSelectPin = SS, uint8_t interruptPin = 2, RHGenericSPI& spi = hardware_spi);
  
    /// Initialises this instance and the radio module connected to it.
    /// The following steps are taken:
    /// - Initialise the slave select pin and the SPI interface library
    /// - Checks the connected RF69 module can be communicated
    /// - Attaches an interrupt handler
    /// - Configures the RF69 module
    /// - Sets the frequency to 434.0 MHz
    /// - Sets the modem data rate to FSK_Rb2Fd5
    /// \return  true if everything was successful
    bool        init();

    /// Reads the on-chip temperature sensor.
    /// The RF69 must be in Idle mode (= RF69 Standby) to measure temperature.
    /// The measurement is uncalibrated and without calibration, you can expect it to be far from
    /// correct.
    /// \return The measured temperature, in degrees C from -40 to 85 (uncalibrated)
    int8_t        temperatureRead();   

    /// Sets the transmitter and receiver 
    /// centre frequency
    /// \param[in] centre Frequency in MHz. 240.0 to 960.0. Caution, RF69 comes in several
    /// different frequency ranges, and setting a frequency outside that range of your radio will probably not work
    /// \param[in] afcPullInRange Not used
    /// \return true if the selected frquency centre is within range
    bool        setFrequency(float centre, float afcPullInRange = 0.05);

    /// Reads and returns the current RSSI value. 
    /// Causes the current signal strength to be measured and returned
    /// If you want to find the RSSI
    /// of the last received message, use lastRssi() instead.
    /// \return The current RSSI value on units of 0.5dB.
    int8_t        rssiRead();

    /// Sets the parameters for the RF69 OPMODE.
    /// This is a low level device access function, and should not normally ned to be used by user code. 
    /// Instead can use stModeRx(), setModeTx(), setModeIdle()
    /// \param[in] mode RF69 OPMODE to set, one of RH_RF69_OPMODE_MODE_*.
    void           setOpMode(uint8_t mode);

    /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
    void           setModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the RF69.
    void           setModeRx();

    /// If current mode is Rx or Idle, changes it to Rx. F
    /// Starts the transmitter in the RF69.
    void           setModeTx();

    /// Sets the transmitter power output level.
    /// Be a good neighbour and set the lowest power level you need.
    /// Caution: legal power limits may apply in certain countries.
    /// After init(), the power will be set to 13dBm for a low power module.
    /// If you are using a high p[ower modfule such as an RFM69HW, you MUST set the power level
    /// with the ishighpowermodule flag set to true. Else you wil get no measurable power output.
    /// Simlarly if you are not using a high power module, you must NOT set the ishighpowermodule
    /// (which is the default)
    /// \param[in] power Transmitter power level in dBm. For RF69W (ishighpowermodule = false),
    /// valid values are from -18 to +13.; Values outside this range are trimmed.
    /// For RF69HW (ishighpowermodule = true), valid values are from -2 to +20.
    /// Caution: at +20dBm, duty cycle is limited to 1% and a 
    /// maximum VSWR of 3:1 at the antenna port.
    /// \param ishighpowermodule Set to true if the connected module is a high power module RFM69HW
    void           setTxPower(int8_t power, bool ishighpowermodule = RH_RF69_DEFAULT_HIGHPOWER);

    /// Sets all the registers required to configure the data modem in the RF69, including the data rate, 
    /// bandwidths etc. You can use this to configure the modem with custom configurations if none of the 
    /// canned configurations in ModemConfigChoice suit you.
    /// \param[in] config A ModemConfig structure containing values for the modem configuration registers.
    void           setModemRegisters(const ModemConfig* config);

    /// Select one of the predefined modem configurations. If you need a modem configuration not provided 
    /// here, use setModemRegisters() with your own ModemConfig. The default after init() is RH_RF69::GFSK_Rb250Fd250.
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

    /// Sets the length of the preamble
    /// in bytes. 
    /// Caution: this should be set to the same 
    /// value on all nodes in your network. Default is 4.
    /// Sets the message preamble length in REG_0?_PREAMBLE?SB
    /// \param[in] bytes Preamble length in bytes.  
    void           setPreambleLength(uint16_t bytes);

    /// Sets the sync words for transmit and receive 
    /// Caution: SyncWords should be set to the same 
    /// value on all nodes in your network. Nodes with different SyncWords set will never receive
    /// each others messages, so different SyncWords can be used to isolate different
    /// networks from each other. Default is { 0x2d, 0xd4 }.
    /// Caution: tests here show that with a single sync word (ie where len == 1), 
    /// RFM69 reception can be unreliable.
    /// To disable sync word generation and detection, call with the defaults: setSyncWords();
    /// \param[in] syncWords Array of sync words, 1 to 4 octets long. NULL if no sync words to be used.
    /// \param[in] len Number of sync words to set, 1 to 4. 0 if no sync words to be used.
    void           setSyncWords(const uint8_t* syncWords = NULL, uint8_t len = 0);

    /// Enables AES encryption and sets the AES encryption key, used
    /// to encrypt and decrypt all messages. The default is disabled.
    /// \param[in] key The key to use. Must be 16 bytes long. The same key must be installed
    /// in other instances of RF69, otherwise communications will not work correctly. If key is NULL,
    /// encryption is disabled, which is the default.
    void           setEncryptionKey(uint8_t* key = NULL);

    /// Returns the time in millis since the most recent preamble was received, and when the most recent
    /// RSSI measurement was made.
    uint32_t getLastPreambleTime();

    /// The maximum message length supported by this driver
    /// \return The maximum message length supported by this driver
    uint8_t maxMessageLength();

    /// Prints the value of a single register
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// For debugging/testing only
    /// \return true if successful
    bool printRegister(uint8_t reg);

    /// Prints the value of all the RF69 registers
    /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform
    /// For debugging/testing only
    /// \return true if successful
    bool printRegisters();

    /// Sets the radio operating mode for the case when the driver is idle (ie not
    /// transmitting or receiving), allowing you to control the idle mode power requirements
    /// at the expense of slower transitions to transmit and receive modes.
    /// By default, the idle mode is RH_RF69_OPMODE_MODE_STDBY,
    /// but eg setIdleMode(RH_RF69_OPMODE_MODE_SLEEP) will provide a much lower
    /// idle current but slower transitions. Call this function after init().
    /// \param[in] idleMode The chip operating mode to use when the driver is idle. One of RH_RF69_OPMODE_*
    void setIdleMode(uint8_t idleMode);

    /// Sets the radio into low-power sleep mode.
    /// If successful, the transport will stay in sleep mode until woken by 
    /// changing mode it idle, transmit or receive (eg by calling send(), recv(), available() etc)
    /// Caution: there is a time penalty as the radio takes a finite time to wake from sleep mode.
    /// \return true if sleep mode was successfully entered.
    virtual bool    sleep();

    /// Return the integer value of the device type
    /// as read from the device in from RH_RF69_REG_10_VERSION.
    /// Expect 0x24, depending on the type of device actually
    /// connected.
    /// \return The integer device type
    uint16_t deviceType() {return _deviceType;};

protected:
    /// This is a low level function to handle the interrupts for one instance of RF69.
    /// Called automatically by isr*()
    /// Should not need to be called by user code.
    void           handleInterrupt();

    /// Low level function to read the FIFO and put the received data into the receive buffer
    /// Should not need to be called by user code.
    void           readFifo();

protected:
    /// Low level interrupt service routine for RF69 connected to interrupt 0
    static void         isr0();

    /// Low level interrupt service routine for RF69 connected to interrupt 1
    static void         isr1();

    /// Low level interrupt service routine for RF69 connected to interrupt 1
    static void         isr2();

    /// Array of instances connected to interrupts 0 and 1
    static RH_RF69*     _deviceForInterrupt[];

    /// Index of next interrupt number to use in _deviceForInterrupt
    static uint8_t      _interruptCount;

    /// The configured interrupt pin connected to this instance
    uint8_t             _interruptPin;

    /// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
    /// else 0xff
    uint8_t             _myInterruptIndex;

    /// The radio OP mode to use when mode is RHModeIdle
    uint8_t             _idleMode; 

    /// The reported device type
    uint8_t             _deviceType;

    /// The selected output power in dBm
    int8_t              _power;

    /// The message length in _buf
    volatile uint8_t    _bufLen;

    /// Array of octets of teh last received message or the next to transmit message
    uint8_t             _buf[RH_RF69_MAX_MESSAGE_LEN];

    /// True when there is a valid message in the Rx buffer
    volatile bool    _rxBufValid;

    /// Time in millis since the last preamble was received (and the last time the RSSI was measured)
    uint32_t            _lastPreambleTime;
};

/// @example rf69_client.ino
/// @example rf69_server.ino
/// @example rf69_reliable_datagram_client.ino
/// @example rf69_reliable_datagram_server.ino


#endif
