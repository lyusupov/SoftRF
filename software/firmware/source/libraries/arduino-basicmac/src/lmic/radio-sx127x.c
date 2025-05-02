// Copyright (C) 2020 Linar Yusupov. All rights reserved.
// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#include "board.h"
#include "hw.h"
#include "lmic.h"

#if defined(BRD_sx1272_radio) || defined(BRD_sx1276_radio)

// ----------------------------------------
// Registers Mapping
#define RegFifo                                    0x00 // common
#define RegOpMode                                  0x01 // common
#define FSKRegBitrateMsb                           0x02
#define FSKRegBitrateLsb                           0x03
#define FSKRegFdevMsb                              0x04
#define FSKRegFdevLsb                              0x05
#define RegFrfMsb                                  0x06 // common
#define RegFrfMid                                  0x07 // common
#define RegFrfLsb                                  0x08 // common
#define RegPaConfig                                0x09 // common
#define RegPaRamp                                  0x0A // common
#define RegOcp                                     0x0B // common
#define RegLna                                     0x0C // common
#define FSKRegRxConfig                             0x0D
#define LORARegFifoAddrPtr                         0x0D
#define FSKRegRssiConfig                           0x0E
#define LORARegFifoTxBaseAddr                      0x0E
#define FSKRegRssiCollision                        0x0F
#define LORARegFifoRxBaseAddr                      0x0F
#define FSKRegRssiThresh                           0x10
#define LORARegFifoRxCurrentAddr                   0x10
#define FSKRegRssiValue                            0x11
#define LORARegIrqFlagsMask                        0x11
#define FSKRegRxBw                                 0x12
#define LORARegIrqFlags                            0x12
#define FSKRegAfcBw                                0x13
#define LORARegRxNbBytes                           0x13
#define FSKRegOokPeak                              0x14
#define LORARegRxHeaderCntValueMsb                 0x14
#define FSKRegOokFix                               0x15
#define LORARegRxHeaderCntValueLsb                 0x15
#define FSKRegOokAvg                               0x16
#define LORARegRxPacketCntValueMsb                 0x16
#define LORARegRxpacketCntValueLsb                 0x17
#define LORARegModemStat                           0x18
#define LORARegPktSnrValue                         0x19
#define FSKRegAfcFei                               0x1A
#define LORARegPktRssiValue                        0x1A
#define FSKRegAfcMsb                               0x1B
#define LORARegRssiValue                           0x1B
#define FSKRegAfcLsb                               0x1C
#define LORARegHopChannel                          0x1C
#define FSKRegFeiMsb                               0x1D
#define LORARegModemConfig1                        0x1D
#define FSKRegFeiLsb                               0x1E
#define LORARegModemConfig2                        0x1E
#define FSKRegPreambleDetect                       0x1F
#define LORARegSymbTimeoutLsb                      0x1F
#define FSKRegRxTimeout1                           0x20
#define LORARegPreambleMsb                         0x20
#define FSKRegRxTimeout2                           0x21
#define LORARegPreambleLsb                         0x21
#define FSKRegRxTimeout3                           0x22
#define LORARegPayloadLength                       0x22
#define FSKRegRxDelay                              0x23
#define LORARegPayloadMaxLength                    0x23
#define FSKRegOsc                                  0x24
#define LORARegHopPeriod                           0x24
#define FSKRegPreambleMsb                          0x25
#define LORARegFifoRxByteAddr                      0x25
#define LORARegModemConfig3                        0x26
#define FSKRegPreambleLsb                          0x26
#define FSKRegSyncConfig                           0x27
#define LORARegFeiMsb                              0x28
#define FSKRegSyncValue1                           0x28
#define LORAFeiMib                                 0x29
#define FSKRegSyncValue2                           0x29
#define LORARegFeiLsb                              0x2A
#define FSKRegSyncValue3                           0x2A
#define FSKRegSyncValue4                           0x2B
#define LORARegRssiWideband                        0x2C
#define FSKRegSyncValue5                           0x2C
#define FSKRegSyncValue6                           0x2D
#define FSKRegSyncValue7                           0x2E
#define FSKRegSyncValue8                           0x2F
#define FSKRegPacketConfig1                        0x30
#define FSKRegPacketConfig2                        0x31
#define LORARegDetectOptimize                      0x31
#define FSKRegPayloadLength                        0x32
#define FSKRegNodeAdrs                             0x33
#define LORARegInvertIQ                            0x33
#define FSKRegBroadcastAdrs                        0x34
#define FSKRegFifoThresh                           0x35
#define FSKRegSeqConfig1                           0x36
#define FSKRegSeqConfig2                           0x37
#define LORARegDetectionThreshold                  0x37
#define FSKRegTimerResol                           0x38
#define FSKRegTimer1Coef                           0x39
#define LORARegSyncWord                            0x39
#define FSKRegTimer2Coef                           0x3A
#define FSKRegImageCal                             0x3B
#define LORARegInvertIQ2                           0x3B
#define FSKRegTemp                                 0x3C
#define FSKRegLowBat                               0x3D
#define FSKRegIrqFlags1                            0x3E
#define FSKRegIrqFlags2                            0x3F
#define RegDioMapping1                             0x40 // common
#define RegDioMapping2                             0x41 // common
#define RegVersion                                 0x42 // common
// #define RegAgcRef                                  0x43 // common
// #define RegAgcThresh1                              0x44 // common
// #define RegAgcThresh2                              0x45 // common
// #define RegAgcThresh3                              0x46 // common
// #define RegPllHop                                  0x4B // common
#define SX1272_RegTcxo                             0x58
#define SX1276_RegTcxo                             0x4B
#define SX1272_RegPaDac                            0x5A
#define SX1276_RegPaDac                            0x4D
// #define RegPll                                     0x5C // common
// #define RegPllLowPn                                0x5E // common
// #define RegFormerTemp                              0x6C // common
#define SX1272_RegBitRateFrac                      0x70
#define SX1276_RegBitRateFrac                      0x5D

// ----------------------------------------
// SX1272 RegModemConfig1 settings
#define SX1272_MC1_CR_4_5                    0x08
#define SX1272_MC1_CR_4_6                    0x10
#define SX1272_MC1_CR_4_7                    0x18
#define SX1272_MC1_CR_4_8                    0x20
#define SX1272_MC1_IMPLICIT_HEADER_MODE_ON   0x04 // required for receive
#define SX1272_MC1_RX_PAYLOAD_CRCON          0x02
#define SX1272_MC1_LOW_DATA_RATE_OPTIMIZE    0x01

// SX1272 RegModemConfig2 settings
#define SX1272_MC2_AGCAUTO                   0x04

// opmodes
#define OPMODE_MASK           0x07
#define OPMODE_SLEEP          0
#define OPMODE_STANDBY        1
#define OPMODE_FSTX           2
#define OPMODE_TX             3
#define OPMODE_FSRX           4
#define OPMODE_RX             5
#define OPMODE_RX_SINGLE      6
#define OPMODE_CAD            7

// LoRa opmode bits:
#define OPMODE_LORA           0x80
// SX1272: bit7=1 (LoRa), bit6=0 (AccessSharedReg=LoRa), bit5+4+3=000 (unused), bit2+1+0=mode
// SX1276: bit7=1 (LoRa), bit6=0 (AccessSharedReg=LoRa), bit5+4=00 (reserved), bit3=0 (access HF test regs), bit2+1+0=mode
#define OPMODE_LORA_SLEEP     (0b10000000+OPMODE_SLEEP)
#define OPMODE_LORA_STANDBY   (0b10000000+OPMODE_STANDBY)
#define OPMODE_LORA_FSTX      (0b10000000+OPMODE_FSTX)
#define OPMODE_LORA_TX        (0b10000000+OPMODE_TX)
#define OPMODE_LORA_FSRX      (0b10000000+OPMODE_FSRX)
#define OPMODE_LORA_RX        (0b10000000+OPMODE_RX)
#define OPMODE_LORA_RX_SINGLE (0b10000000+OPMODE_RX_SINGLE)
#define OPMODE_LORA_CAD       (0b10000000+OPMODE_CAD)

// FSK opmode bits:
#ifdef BRD_sx1272_radio
// SX1272: bit7=0 (FSK), bit6+5=00 (modulation=FSK), bit4+3=00 (no shaping), bits2+1+0=mode
#define OPMODE_FSK_SLEEP      (0b00000000+OPMODE_SLEEP)
#define OPMODE_FSK_STANDBY    (0b00000000+OPMODE_STANDBY) // (reset value of RegOpMode)
#define OPMODE_FSK_FSTX       (0b00000000+OPMODE_FSTX)
#define OPMODE_FSK_TX         (0b00000000+OPMODE_TX)
#define OPMODE_FSK_FSRX       (0b00000000+OPMODE_FSRX)
#define OPMODE_FSK_RX         (0b00000000+OPMODE_RX)
#endif

#ifdef BRD_sx1276_radio
// SX1276: bit7=0 (FSK), bit6+5=00 (modulation=FSK), bit4=0 (reserved), bit3=1 (access LF test regs), bits2+1+0=mode
#define OPMODE_FSK_SLEEP      (0b00001000+OPMODE_SLEEP)
#define OPMODE_FSK_STANDBY    (0b00001000+OPMODE_STANDBY) // (reset value of RegOpMode)
#define OPMODE_FSK_FSTX       (0b00001000+OPMODE_FSTX)
#define OPMODE_FSK_TX         (0b00001000+OPMODE_TX)
#define OPMODE_FSK_FSRX       (0b00001000+OPMODE_FSRX)
#define OPMODE_FSK_RX         (0b00001000+OPMODE_RX)
#endif

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK            0x80
#define IRQ_LORA_RXDONE_MASK            0x40
#define IRQ_LORA_CRCERR_MASK            0x20
#define IRQ_LORA_HEADER_MASK            0x10
#define IRQ_LORA_TXDONE_MASK            0x08
#define IRQ_LORA_CDDONE_MASK            0x04
#define IRQ_LORA_FHSSCH_MASK            0x02
#define IRQ_LORA_CDDETD_MASK            0x01

// interrupt flags when large packet successfully: rcvd sent
#define IRQ_FSK1_MODEREADY_MASK         0x80     // 1    1
#define IRQ_FSK1_RXREADY_MASK           0x40     // 1    0
#define IRQ_FSK1_TXREADY_MASK           0x20     // 0    1
#define IRQ_FSK1_PLLLOCK_MASK           0x10     // 1    1
#define IRQ_FSK1_RSSI_MASK              0x08     // 1    0
#define IRQ_FSK1_TIMEOUT_MASK           0x04     // 0    0
#define IRQ_FSK1_PREAMBLEDETECT_MASK    0x02     // 1    0
#define IRQ_FSK1_SYNCADDRESSMATCH_MASK  0x01     // 1    0
#define IRQ_FSK2_FIFOFULL_MASK          0x80     // 0    0
#define IRQ_FSK2_FIFOEMPTY_MASK         0x40     // 0    1
#define IRQ_FSK2_FIFOLEVEL_MASK         0x20     // 0    0
#define IRQ_FSK2_FIFOOVERRUN_MASK       0x10     // 0    0
#define IRQ_FSK2_PACKETSENT_MASK        0x08     // 0    1
#define IRQ_FSK2_PAYLOADREADY_MASK      0x04     // 1    0
#define IRQ_FSK2_CRCOK_MASK             0x02     // 1    0
#define IRQ_FSK2_LOWBAT_MASK            0x01     // 0    0

// ----------------------------------------
// DIO function mappings            MAP1:D0D1D2D3
#define MAP1_LORA_DIO0_RXDONE   0x00  // 00------
#define MAP1_LORA_DIO0_TXDONE   0x40  // 01------
#define MAP1_LORA_DIO0_NOP      0xC0  // 11------
#define MAP1_LORA_DIO1_RXTOUT   0x00  // --00----
#define MAP1_LORA_DIO1_NOP      0x30  // --11----
#define MAP1_LORA_DIO2_NOP      0x0C  // ----11--
#define MAP1_LORA_DIO3_NOP      0x03  // ------11
//                                  MAP2:D4D5XXXX
#define MAP2_LORA_DIO4_NOP      0xC0  // 11------
#define MAP2_LORA_DIO5_NOP      0x30  // --11----
#define MAP2_LORA_RFU           0x00  // ----000-
#define MAP2_LORA_IRQ_PREAMBLE  0x01  // -------1

//                                  MAP1:D0D1D2D3
#define MAP1_FSK_DIO0_RXDONE    0x00  // 00------
#define MAP1_FSK_DIO0_TXDONE    0x00  // 00------
#define MAP1_FSK_DIO1_LEVEL     0x00  // --00----
#define MAP1_FSK_DIO1_EMPTY     0x10  // --01----
#define MAP1_FSK_DIO1_FULL      0x20  // --10----
#define MAP1_FSK_DIO1_NOP       0x30  // --11----
#define MAP1_FSK_DIO2_TXNOP     0x04  // ----01--
#define MAP1_FSK_DIO2_RXTOUT    0x08  // ----10--

// FSK ImageCal defines
#define RF_IMAGECAL_IMAGECAL_START      0x40
#define RF_IMAGECAL_IMAGECAL_RUNNING    0x20

// IQ Inversion
#define IQRXNORMAL  0x27 // (see AN1200.24 SX1276 settings for LoRaWAN)
#define IQ2RXNORMAL 0x1D
#define IQRXINVERT  0x67
#define IQ2RXINVERT 0x19

// radio-specific settings
#if defined(BRD_sx1276_radio)
#define RADIO_VERSION               0x12
#define RST_PIN_RESET_STATE         0
#define RSSI_HF_CONST               157
#define RegPaDac                    SX1276_RegPaDac
#define RegTcxo                     SX1276_RegTcxo
#define RegBitRateFrac              SX1276_RegBitRateFrac
#define LORA_TXDONE_FIXUP           us2osticksRound(67)  // determined by timestamping DIO0 with SX1301 (mku/20190315)
#define LORA_RXSTART_FIXUP          us2osticksRound(101) // determined by osc measurement GPIO vs with DIO5 (mode-ready) (mku/20190315)
#define FSK_TXDONE_FIXUP            us2osticks(0) // XXX
#define FSK_RXDONE_FIXUP            us2osticks(0) // XXX

static const u2_t LORA_RXDONE_FIXUP_125[] = {
    [FSK]  =     us2osticks(0),
    [SF7]  =     us2osticks(0),
    [SF8]  =  us2osticks(1648),
    [SF9]  =  us2osticks(3265),
    [SF10] =  us2osticks(7049),
    [SF11] = us2osticks(13641),
    [SF12] = us2osticks(31189),
};

static const u2_t LORA_RXDONE_FIXUP_500[] = {
    [FSK]  = us2osticks(    0),
    [SF7]  = us2osticks(    0),
    [SF8]  = us2osticks(    0),
    [SF9]  = us2osticks(    0),
    [SF10] = us2osticks(    0),
    [SF11] = us2osticks(    0),
    [SF12] = us2osticks(    0),
};

#elif defined(BRD_sx1272_radio)
#define RADIO_VERSION               0x22
#define RST_PIN_RESET_STATE         1
#define RSSI_HF_CONST               139
#define RegPaDac                    SX1272_RegPaDac
#define RegTcxo                     SX1272_RegTcxo
#define RegBitRateFrac              SX1272_RegBitRateFrac
#define LORA_TXDONE_FIXUP           us2osticks(43) // XXX
#define LORA_RXSTART_FIXUP          us2osticks(0) // XXX
#define FSK_TXDONE_FIXUP            us2osticks(0) // XXX
#define FSK_RXDONE_FIXUP            us2osticks(0) // XXX

static const u2_t LORA_RXDONE_FIXUP_125[] = {
    [FSK]  = us2osticksRound(    0),
    [SF7]  = us2osticksRound(  749),
    [SF8]  = us2osticksRound( 1343),
    [SF9]  = us2osticksRound( 3265),
    [SF10] = us2osticksRound( 7049),
    [SF11] = us2osticksRound(13641),
    [SF12] = us2osticksRound(31189),
};

// Based Nucleo board regr tests rxlatency-regr
static const u2_t LORA_RXDONE_FIXUP_500[] = {
    [FSK]  = us2osticksRound(   0),
    [SF7]  = us2osticksRound( 193),
    [SF8]  = us2osticksRound( 344),
    [SF9]  = us2osticksRound( 737),
    [SF10] = us2osticksRound(1521),
    [SF11] = us2osticksRound(3240),
    [SF12] = us2osticksRound(6972),
};

#endif

#define FIFOTHRESH 32

#define RXLORA_REG_HOPCHANNEL_CRC_ON_PAYLOAD 0x40

//-----------------------------------------
// Parameters for RSSI monitoring
#define SX127X_FREQ_LF_MAX      525000000       // per datasheet 6.3

// per datasheet 5.5.3:
#define SX127X_RSSI_ADJUST_LF   -164            // add to rssi value to get dB (LF)
#define SX127X_RSSI_ADJUST_HF   -157            // add to rssi value to get dB (HF)

// state
static struct {
    // large packet handling
    unsigned char* fifoptr;
    int fifolen;
#ifdef BRD_sx1276_radio
    // one-time receiver chain calibration
    int calibrated;
#endif
} state;

// ----------------------------------------
static void writeReg (u1_t addr, u1_t data) {
    hal_spi_select(1);
    hal_spi(addr | 0x80);
    hal_spi(data);
    hal_spi_select(0);
}

#define FIFO_Push_Inv(x)      writeReg(RegFifo, (u1_t)~(x))

static u1_t readReg (u1_t addr) {
    hal_spi_select(1);
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
    hal_spi_select(0);
    return val;
}

// (used by perso)
static void radio_writeBuf (u1_t addr, u1_t* buf, u1_t len) {
    hal_spi_select(1);
    hal_spi(addr | 0x80);
    u1_t i;
    for (i = 0; i < len; i++) {
        hal_spi(buf[i]);
    }
    hal_spi_select(0);
}

static void radio_writeBuf_Inv (u1_t addr, u1_t* buf, u1_t len) {
    hal_spi_select(1);
    hal_spi(addr | 0x80);
    u1_t i;
    for (i = 0; i < len; i++) {
        hal_spi(~buf[i]);
    }
    hal_spi_select(0);
}

// (used by  perso)
static void radio_readBuf (u1_t addr, u1_t* buf, u1_t len) {
    hal_spi_select(1);
    hal_spi(addr & 0x7F);
    u1_t i;
    for (i = 0; i < len; i++) {
        buf[i] = hal_spi(0x00);
    }
    hal_spi_select(0);
}

static void radio_readBuf_Inv (u1_t addr, u1_t* buf, u1_t len) {
    hal_spi_select(1);
    hal_spi(addr & 0x7F);
    u1_t i;
    for (i = 0; i < len; i++) {
        buf[i] = ~(hal_spi(0x00));
    }
    hal_spi_select(0);
}

static void sx127x_radio_sleep (void) {
    writeReg(RegOpMode, OPMODE_LORA_SLEEP); // LoRa/FSK bit is ignored when not in SLEEP mode
}

// configure LoRa modem
static void configLoraModem (void) {
#if defined(BRD_sx1276_radio)
    // set ModemConfig1 'bbbbccch' (bw=xxxx, cr=xxx, implicitheader=x)
    writeReg(LORARegModemConfig1,
	     ((getBw(LMIC.rps) + 7) << 4) | // BW125=0 --> 7
	     ((getCr(LMIC.rps) + 1) << 1) | // CR4_5=0 --> 1
	     (getIh(LMIC.rps) != 0));       // implicit header

    // set ModemConfig2 'sssstcmm' (sf=xxxx, txcont=0, rxpayloadcrc=x, symtimeoutmsb=00)
    writeReg(LORARegModemConfig2,
	     ((getSf(LMIC.rps)-1+7) << 4) |     // SF7=1 --> 7
	     ((getNocrc(LMIC.rps) == 0) << 2)); // rxcrc

    // set ModemConfig3 'uuuuoarr' (unused=0000, lowdatarateoptimize=x, agcauto=1, reserved=00)
    writeReg(LORARegModemConfig3,
	     (enDro(LMIC.rps) << 3) | // symtime >= 16ms
	     (1 << 2));               // autoagc

    // SX1276 Errata: 2.1 Sensitivity Optimization with a 500kHz Bandwith
    if (getBw(LMIC.rps) == BW500) {
    	writeReg(0x36, 0x02);
    	writeReg(0x3A, 0x64);
    } else {
    	writeReg(0x36, 0x03);
    	// no need to reset register 0x3a
    }
#elif defined(BRD_sx1272_radio)
    // set ModemConfig1 'bbccchco' (bw=xx, cr=xxx, implicitheader=x, rxpayloadcrc=x, lowdatarateoptimize=x)
    writeReg(LORARegModemConfig1,
	     (getBw(LMIC.rps) << 6) |           // BW125=0 --> 0
	     ((getCr(LMIC.rps) + 1) << 3) |     // CR4_5=0 --> 1
	     ((getIh(LMIC.rps) != 0) << 2) |    // implicit header
	     ((getNocrc(LMIC.rps) == 0) << 1) | // rxcrc
	     enDro(LMIC.rps));                  // symtime >= 16ms

    // set ModemConfig2 'sssstamm' (sf=xxxx, txcont=0, agcauto=1 symtimeoutmsb=00)
    writeReg(LORARegModemConfig2,
	     ((getSf(LMIC.rps)-1+7) << 4) | // SF7=1 --> 7
	     (1 << 2));                     // autoagc
#endif // BRD_sx1272_radio

    if (getIh(LMIC.rps)) {
    	writeReg(LORARegPayloadLength, getIh(LMIC.rps)); // required length
    }
}

static void configChannel (void) {
    // set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
    u4_t frf = ((lmic_u8_t)LMIC.freq << 19) / 32000000;

    writeReg(RegFrfMsb, frf >> 16);
    writeReg(RegFrfMid, frf >> 8);
    writeReg(RegFrfLsb, frf >> 0);

#ifdef BRD_sx1276_radio
    // run one-time receiver chain calibration (in STANDBY mode!)
    if (!state.calibrated) {
    	state.calibrated = 1;
    	writeReg(FSKRegImageCal, RF_IMAGECAL_IMAGECAL_START);
    	while ( readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING );
    }
#endif
}

// by default PA_BOOST output pin is selected. make sure to set CFG_PA_RFO for boards which use RF out pins (HF+LF)!
#if !defined(CFG_PA_BOOST) && !defined(CFG_PA_RFO)
#define CFG_PA_BOOST
#endif

// PaDac 'rrrrrddd' (reserved=10000, dacdefault=100 dachigh=111)
// Ocp   'uuottttt' (unused=00, Ocp=x, trim=xxxxx)
//
// SX1276:
// PaCfg 'bmmmpppp' (PaSelect=x, MaxPower=xxx, OutputPower=xxxx)
//   OutputPower = pw-2    if PaSelect = 1 (PA_BOOST pin              2..17dBm or 5..20dBm)
//   OutputPower = pw      if PaSelect = 0 (RFO pin and MaxPower=111  0..15dBm)
//   OutputPower = pw+4    if PaSelect = 0 (RFO pin and MaxPower=000 -4..11dBm)
//
// SX1272:
// PaCfg 'buuupppp' (PaSelect=x, unused=000, OutputPower=xxxx)
//   OutputPower = pw-2    if PaSelect = 1 (PA_BOOST pin              2..17dBm or 5..20dBm)
//   OutputPower = pw+1    if PaSelect = 0 (RFO pin                  -1..14dBm)
//
// power-on:  PaDac PaCfg Ocp
//   SX1272:  0x84  0x0F  0x2B
//   SX1276:  0x84  0x4F  0x2B
static void configPower (s1_t pw) {
#if (defined(CFG_wailmer_board) || defined(CFG_wailord_board)) && defined(CFG_us915)
    // XXX - TODO - externalize this somehow
    // wailmer/wailord can only use 17dBm at DR4 (US)
    if (getBw(LMIC.rps) == BW500 && pw > 17) {
    	pw = 17;
    }
#endif

#if defined(CFG_PA_BOOST)
    if (pw > 17) { // use high-power +20dBm option
    	if (pw > 20) {
    	    pw = 20;
    	}
    	writeReg(RegPaDac, 0x87); // high power
    	writeReg(RegPaConfig, 0x80 | (pw - 5)); // BOOST (5..20dBm)
    } else {
    	if (pw < 2) {
    	    pw = 2;
    	}
    	writeReg(RegPaDac, 0x84); // normal power
    	writeReg(RegPaConfig, 0x80 | (pw - 2)); // BOOST (2..17dBm)
    }
#elif defined(CFG_PA_RFO)
#if defined(BRD_sx1276_radio)
    if (pw > 0) {
    	if (pw > 15) {
    	    pw = 15;
    	}
    	writeReg(RegPaConfig, 0x70 | pw); // RFO, maxpower=111 (0..15dBm)
    } else {
    	if (pw < -4) {
    	    pw = -4;
    	}
    	writeReg(RegPaConfig, pw + 4); // RFO, maxpower=000 (-4..11dBm)
    }
    writeReg(RegPaDac, 0x84); // normal power
#elif defined(BRD_sx1272_radio)
    if (pw < -1) {
    	pw = -1;
    } else if (pw > 14) {
    	pw = 14;
    }
    writeReg(RegPaConfig, pw + 1); // RFO (-1..14dBm)
    writeReg(RegPaDac, 0x84); // normal power
#endif
#else
#error "must define CFG_PA_BOOST or CFG_PA_RFO!"
#endif

    // set 50us PA ramp-up time
    writeReg(RegPaRamp, 0b00011000); // unused=000, LowPnTxPllOff=1, PaRamp=1000
}

static void power_tcxo (void) {
    // power-up TCXO and set tcxo as input
    if ( hal_pin_tcxo(1) ) {
    	writeReg(RegTcxo, 0b00011001); // reserved=000, tcxo=1, reserved=1001
    }
}

// continuous wave
static void txcw (void) {
    // select FSK modem (from sleep mode)
    writeReg(RegOpMode, OPMODE_FSK_SLEEP);
    ASSERT(readReg(RegOpMode) == OPMODE_FSK_SLEEP);

    // power-up tcxo
    power_tcxo();

    // enter standby mode (required for FIFO loading))
    writeReg(RegOpMode, OPMODE_FSK_STANDBY);

    // set frequency deviation
    writeReg(FSKRegFdevMsb, 0x00);
    writeReg(FSKRegFdevLsb, 0x00);

    // configure frequency
    configChannel();

    // configure output power
    configPower(LMIC.txpow + LMIC.txPowAdj + TX_ERP_ADJ);

    // set continuous mode
    writeReg(LORARegModemConfig2, readReg(LORARegModemConfig2) | 0x08);

    // initialize the payload size and address pointers
    writeReg(FSKRegPayloadLength, 1);
    writeReg(RegFifo, 0);

    // enable antenna switch for TX
    hal_pin_rxtx(1);

    // now we actually start the transmission
    writeReg(RegOpMode, OPMODE_FSK_TX);
}

#define RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY 0x80

static void txfsk (void) {
    // select FSK modem (from sleep mode)
    writeReg(RegOpMode, OPMODE_FSK_SLEEP);
    ASSERT(readReg(RegOpMode) == OPMODE_FSK_SLEEP);

    // power-up tcxo
    power_tcxo();

    writeReg(FSKRegIrqFlags2, IRQ_FSK2_FIFOOVERRUN_MASK);

    // enter standby mode (required for FIFO loading))
    writeReg(RegOpMode, OPMODE_FSK_STANDBY);

    ASSERT(LMIC.protocol);

    // set bitrate
    switch (LMIC.protocol->bitrate)
    {
    case RF_BITRATE_38400:
      writeReg(FSKRegBitrateMsb, 0x03); // 38400 bps
      writeReg(FSKRegBitrateLsb, 0x41);
      writeReg(RegBitRateFrac,   0x05);
      break;
    case RF_BITRATE_100KBPS:
    default:
      writeReg(FSKRegBitrateMsb, 0x01); // 100 kbps
      writeReg(FSKRegBitrateLsb, 0x40);
      writeReg(RegBitRateFrac,   0x00);
      break;
    }

    // set frequency deviation
    switch (LMIC.protocol->deviation)
    {
    case RF_FREQUENCY_DEVIATION_10KHZ:
      writeReg(FSKRegFdevMsb, 0x00); // +/- 10kHz
      writeReg(FSKRegFdevLsb, 0xa4);
      break;
    case RF_FREQUENCY_DEVIATION_19_2KHZ:
      writeReg(FSKRegFdevMsb, 0x01); // +/- 19.2kHz
      writeReg(FSKRegFdevLsb, 0x3b);
      break;
    case RF_FREQUENCY_DEVIATION_25KHZ:
      writeReg(FSKRegFdevMsb, 0x01); // +/- 25kHz
      writeReg(FSKRegFdevLsb, 0x9a);
      break;
    case RF_FREQUENCY_DEVIATION_50KHZ:
    default:
      writeReg(FSKRegFdevMsb, 0x03); // +/- 50kHz
      writeReg(FSKRegFdevLsb, 0x33);
      break;
    }

    // frame and packet handler settings
    writeReg(FSKRegPreambleMsb, 0x00);
    /* add extra preamble symbol at Tx to ease reception on partner's side */
    writeReg(FSKRegPreambleLsb, LMIC.protocol->preamble_size > 2 ?
      LMIC.protocol->preamble_size : LMIC.protocol->preamble_size + 1);

    // set preamble
    uint8_t SyncConfig = (LMIC.protocol->syncword_size - 1);
    switch (LMIC.protocol->preamble_type)
    {
    case RF_PREAMBLE_TYPE_AA:
      writeReg(FSKRegSyncConfig, (0x10 | SyncConfig));
      break;
    case RF_PREAMBLE_TYPE_55:
    default:
      writeReg(FSKRegSyncConfig, (0x30 | SyncConfig));
      break;
    }

    // set whitening
    switch (LMIC.protocol->whitening)
    {
    case RF_WHITENING_MANCHESTER:
      writeReg(FSKRegPacketConfig1, 0x20);
      break;
    case RF_WHITENING_NONE:
    case RF_WHITENING_NICERF:
    default:
      writeReg(FSKRegPacketConfig1, 0x00);
      break;
    }
    writeReg(FSKRegPacketConfig2, 0x40);

    // set syncword
    int i=0;
    for (i=0; i < LMIC.protocol->syncword_size; i++) {
      writeReg((FSKRegSyncValue1 + i), LMIC.protocol->syncword[i]);
    }

    // configure frequency
    configChannel();

    // configure output power
    configPower(LMIC.txpow + LMIC.txPowAdj + TX_ERP_ADJ);

    // set the IRQ mapping DIO0=PacketSent DIO1=NOP DIO2=NOP
    writeReg(RegDioMapping1, MAP1_FSK_DIO0_TXDONE | MAP1_FSK_DIO1_NOP | MAP1_FSK_DIO2_TXNOP);

    // setup FIFO
    writeReg(FSKRegFifoThresh, RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY );

    // initialize the payload size and address pointers
    writeReg(FSKRegPayloadLength, LMIC.dataLen); // (insert length byte into payload))

    switch (LMIC.protocol->payload_type)
    {
    case RF_PAYLOAD_INVERTED:
      radio_writeBuf_Inv(RegFifo, LMIC.frame, LMIC.dataLen);
      break;
    case RF_PAYLOAD_DIRECT:
    default:
      radio_writeBuf(RegFifo, LMIC.frame, LMIC.dataLen);
      break;
    }

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO0);

    // enable antenna switch for TX
    hal_pin_rxtx(1);

    // now we actually start the transmission
    writeReg(RegOpMode, OPMODE_FSK_TX);
}

static void txlora (void) {
    // select LoRa modem (from sleep mode)
    writeReg(RegOpMode, OPMODE_LORA_SLEEP);
    ASSERT(readReg(RegOpMode) == OPMODE_LORA_SLEEP);

    // power-up tcxo
    power_tcxo();

    // enter standby mode (required for FIFO loading)
    writeReg(RegOpMode, OPMODE_LORA_STANDBY);

    // configure LoRa modem
    configLoraModem();

    // configure frequency
    configChannel();

    // configure output power
    configPower(LMIC.txpow + LMIC.txPowAdj + TX_ERP_ADJ);

    // set sync word
    writeReg(LORARegSyncWord, LMIC.syncword);

    // set IQ inversion mode
    writeReg(LORARegInvertIQ,  IQRXNORMAL);
    writeReg(LORARegInvertIQ2, IQ2RXNORMAL);

    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP DIO3=NOP DIO4=NOP DIO5=NOP
    writeReg(RegDioMapping1, MAP1_LORA_DIO0_TXDONE | MAP1_LORA_DIO1_NOP | MAP1_LORA_DIO2_NOP | MAP1_LORA_DIO3_NOP);
    writeReg(RegDioMapping2, MAP2_LORA_DIO4_NOP | MAP2_LORA_DIO5_NOP);

    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);

    // mask all IRQs but TxDone
    writeReg(LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO0);

    // initialize the payload size and address pointers
    writeReg(LORARegFifoTxBaseAddr, 0x00);
    writeReg(LORARegFifoAddrPtr, 0x00);
    writeReg(LORARegPayloadLength, LMIC.dataLen);

    // download buffer to the radio FIFO
    radio_writeBuf(RegFifo, LMIC.frame, LMIC.dataLen);

    // enable antenna switch for TX
    hal_pin_rxtx(1);

    // now we actually start the transmission
    BACKTRACE();
    writeReg(RegOpMode, OPMODE_LORA_TX);
}

static void sx127x_radio_starttx (bool txcontinuous) {
    ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );
    if (txcontinuous) {
    	txcw();
        } else {
    	if (getSf(LMIC.rps) == FSK) { // FSK modem
    	    txfsk();
    	} else { // LoRa modem
    	    txlora();
    	}
    }
}

static void rxlora (bool rxcontinuous) {
    ostime_t t0 = os_getTime();
    // select LoRa modem (from sleep mode)
    writeReg(RegOpMode, OPMODE_LORA_SLEEP);
    ASSERT(readReg(RegOpMode) == OPMODE_LORA_SLEEP);

    // power-up tcxo
    power_tcxo();

    // enter standby mode (warm up)
    writeReg(RegOpMode, OPMODE_LORA_STANDBY);

    // configure LoRa modem (cfg1, cfg2, cfg3)
    configLoraModem();

    // configure frequency
    configChannel();

#if 1
    // set LNA gain 'gggbbrbb' (LnaGain=001 (max), LnaBoostLf=00 (default), reserved=0, LnaBoostHf=11 (150%))
    writeReg(RegLna, 0b00100011);
#else
#define LNA_RX_GAIN (0x20|0x1)
    // set LNA gain
    writeReg(RegLna, LNA_RX_GAIN);
#endif

    // set max payload size
    writeReg(LORARegPayloadMaxLength, 64);

    // set IQ inversion mode
    writeReg(LORARegInvertIQ,  (LMIC.noRXIQinversion) ? IQRXNORMAL  : IQRXINVERT);
    writeReg(LORARegInvertIQ2, (LMIC.noRXIQinversion) ? IQ2RXNORMAL : IQ2RXINVERT);

    // set max preamble length 8
    writeReg(LORARegPreambleMsb, 0x00);
    writeReg(LORARegPreambleLsb, 0x08);

    // set symbol timeout (for single rx)
    writeReg(LORARegSymbTimeoutLsb, LMIC.rxsyms);

    // set sync word
    writeReg(LORARegSyncWord, LMIC.syncword);

    // configure DIO mapping DIO0=RxDone DIO1=(Timeout or NOP) DIO2=NOP DIO3=NOP DIO4=NOP DIO5=NOP
    writeReg(RegDioMapping1, (rxcontinuous) ?
	     (MAP1_LORA_DIO0_RXDONE | MAP1_LORA_DIO1_NOP    | MAP1_LORA_DIO2_NOP | MAP1_LORA_DIO3_NOP) :
	     (MAP1_LORA_DIO0_RXDONE | MAP1_LORA_DIO1_RXTOUT | MAP1_LORA_DIO2_NOP | MAP1_LORA_DIO3_NOP));
    writeReg(RegDioMapping2, MAP2_LORA_DIO4_NOP | MAP2_LORA_DIO5_NOP);

    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);

    // enable required radio IRQs
    writeReg(LORARegIrqFlagsMask, (rxcontinuous) ?
                  ~(IRQ_LORA_RXDONE_MASK | IRQ_LORA_CRCERR_MASK) :
                  ~(IRQ_LORA_RXDONE_MASK | IRQ_LORA_RXTOUT_MASK));

    // enable IRQs in HAL
    hal_irqmask_set((rxcontinuous) ? HAL_IRQMASK_DIO0 : (HAL_IRQMASK_DIO0 | HAL_IRQMASK_DIO1));

    // now instruct the radio to receive
    // it takes 1068us from beginning of rxlora() up to here (keep RX_RAMPUP in sync!)
    // (lock interrupts only for final fine tuned rx timing...)
    hal_disableIRQs();
    if (rxcontinuous) { // continous rx
      	BACKTRACE();
      	// enable antenna switch for RX (and account power consumption)
      	hal_pin_rxtx(0);
      	// rx now...
          writeReg(RegOpMode, OPMODE_LORA_RX);
    } else { // single rx
      	BACKTRACE();
      	// busy wait until exact rx time
              ostime_t rxtime = LMIC.rxtime - LORA_RXSTART_FIXUP;
      	ostime_t now = os_getTime();
      	    debug_printf("WARNING: LoRa rxtime %u now %u\r\n", rxtime , now);
      	if( rxtime - now < 0 ) {
      	    debug_printf("WARNING: rxtime is %d ticks in the past! (ramp-up time %d ms / %d ticks)\r\n",
      			 now - LMIC.rxtime, osticks2ms(now - t0), now - t0);
      	}

      	// enable antenna switch for RX (and account power consumption)
      	hal_pin_rxtx(0);
      	// rx now...
        writeReg(RegOpMode, OPMODE_LORA_RX_SINGLE);
    }
    hal_enableIRQs();
}

static void rxfsk (bool rxcontinuous) {
    // configure radio (needs rampup time)
    ostime_t t0 = os_getTime();

    // select FSK modem (from sleep mode)
    writeReg(RegOpMode, OPMODE_FSK_SLEEP);
    ASSERT( readReg(RegOpMode) == OPMODE_FSK_SLEEP );

    // power-up tcxo
    power_tcxo();

    // enter standby mode (warm up)
    writeReg(RegOpMode, OPMODE_FSK_STANDBY);

    // configure frequency
    configChannel();

    // set bitrate
    switch (LMIC.protocol->bitrate)
    {
    case RF_BITRATE_38400:
      writeReg(FSKRegBitrateMsb, 0x03); // 38400 bps
      writeReg(FSKRegBitrateLsb, 0x41);
      break;
    case RF_BITRATE_100KBPS:
    default:
      writeReg(FSKRegBitrateMsb, 0x01); // 100kbps
      writeReg(FSKRegBitrateLsb, 0x40);
      break;
    }

    // set LNA gain
    //writeReg(RegLna, LNA_RX_GAIN);
    writeReg(RegLna, 0x20 | 0x03); // max gain, boost enable
    //writeReg(RegLna, 0x20 | 0x00); // max gain, default LNA current
    //writeReg(RegLna, 0x60 | 0x00); // -12dB gain, default LNA current
    //writeReg(RegLna, 0x80 | 0x00); // -24dB gain, default LNA current
    //writeReg(RegLna, 0xC0 | 0x00); // -48dB gain, default LNA current

    // configure receiver
    //writeReg(FSKRegRxConfig, 0x1E); // AFC auto, AGC, trigger on preamble?!?
    writeReg(FSKRegRxConfig, 0x0E); // AFC off, AGC on, trigger on preamble?!?
    //writeReg(FSKRegRxConfig, 0x06); // AFC off, AGC off, trigger on preamble?!?

    // set receiver bandwidth
    switch (LMIC.protocol->bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_50KHZ:
      writeReg(FSKRegRxBw, 0x0B); // 50kHz SSb
      break;
    case RF_RX_BANDWIDTH_SS_100KHZ:
      writeReg(FSKRegRxBw, 0x0A); // 100kHz SSb
      break;
    case RF_RX_BANDWIDTH_SS_166KHZ:
      writeReg(FSKRegRxBw, 0x11); // 166.6kHz SSB
      break;
    case RF_RX_BANDWIDTH_SS_200KHZ:
      writeReg(FSKRegRxBw, 0x09); // 200kHz SSB
      break;
    case RF_RX_BANDWIDTH_SS_250KHZ:
      writeReg(FSKRegRxBw, 0x01); // 250kHz SSB
      break;
    case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      writeReg(FSKRegRxBw, 0x02); // 125kHz SSb; BW >= (DR + 2 X FDEV)
      break;
    }

    // set AFC bandwidth
//    writeReg(FSKRegAfcBw, 0x0B); // 50kHz SSB  // PAW
//    writeReg(FSKRegAfcBw, 0x12); // 83.3kHz SSB
      writeReg(FSKRegAfcBw, 0x11); // 166.6kHz SSB
//    writeReg(FSKRegAfcBw, 0x09); // 200kHz SSB
//    writeReg(FSKRegAfcBw, 0x01); // 250kHz SSB

    // set preamble detection
    switch (LMIC.protocol->preamble_size)
    {
    case 0:
      //writeReg(FSKRegPreambleDetect, 0x05); // disable, 5 chip errors
    case 1:
      // Legacy, OGNTP
      writeReg(FSKRegPreambleDetect, 0x85); // enable, 1 bytes, 5 chip errors
      break;
    case 2:
      writeReg(FSKRegPreambleDetect, 0xAA); // enable, 2 bytes, 10 chip errors
      break;
    case 3:
    case 4:
    case 5:
    default:
      // PAW
      writeReg(FSKRegPreambleDetect, 0xCA); // enable, 3 bytes, 10 chip errors
      break;
    }

    // set sync config
    uint8_t SyncConfig = (LMIC.protocol->syncword_size - 1);
    switch (LMIC.protocol->preamble_type)
    {
    case RF_PREAMBLE_TYPE_AA:
      writeReg(FSKRegSyncConfig, (0x10 | SyncConfig));
      break;
    case RF_PREAMBLE_TYPE_55:
    default:
      writeReg(FSKRegSyncConfig, (0x30 | SyncConfig));
      break;
    }

    // set sync value
    int i=0;
    for (i=0; i < LMIC.protocol->syncword_size; i++) {
      writeReg((FSKRegSyncValue1 + i), LMIC.protocol->syncword[i]);
    }

    // set packet config
    switch (LMIC.protocol->whitening)
    {
    case RF_WHITENING_MANCHESTER:
      writeReg(FSKRegPacketConfig1, 0x20);
      break;
    case RF_WHITENING_NONE:
    case RF_WHITENING_NICERF:
    default:
      writeReg(FSKRegPacketConfig1, 0x00);
      break;
    }
    writeReg(FSKRegPacketConfig2, 0x40); // packet mode

    // set payload length
    writeReg(FSKRegPayloadLength,
      LMIC.protocol->payload_size +
      LMIC.protocol->payload_offset +
      LMIC.protocol->crc_size);

    // set frequency deviation
    switch (LMIC.protocol->deviation)
    {
    case RF_FREQUENCY_DEVIATION_10KHZ:
      writeReg(FSKRegFdevMsb, 0x00); // +/- 10kHz
      writeReg(FSKRegFdevLsb, 0xa4);
      break;
    case RF_FREQUENCY_DEVIATION_19_2KHZ:
      writeReg(FSKRegFdevMsb, 0x01); // +/- 19.2kHz
      writeReg(FSKRegFdevLsb, 0x3b);
      break;
    case RF_FREQUENCY_DEVIATION_25KHZ:
      writeReg(FSKRegFdevMsb, 0x01); // +/- 25kHz
      writeReg(FSKRegFdevLsb, 0x9a);
      break;
    case RF_FREQUENCY_DEVIATION_50KHZ:
    default:
      writeReg(FSKRegFdevMsb, 0x03); // +/- 50kHz
      writeReg(FSKRegFdevLsb, 0x33);
      break;
    }

    state.fifolen = -1;
    state.fifoptr = LMIC.frame;

    // I hope that 256 samples will cover full Rx packet
    // writeReg(FSKRegRssiConfig, 0x07);

    // configure DIO mapping DIO0=PayloadReady DIO1=NOP DIO2=TimeOut
    writeReg(RegDioMapping1, MAP1_FSK_DIO0_RXDONE | MAP1_FSK_DIO1_NOP | MAP1_FSK_DIO2_RXTOUT);

    // enable IRQs in HAL
    hal_irqmask_set(HAL_IRQMASK_DIO0 | HAL_IRQMASK_DIO2);

    // now instruct the radio to receive
    hal_disableIRQs();

    if (rxcontinuous) {
    	// XXX not suppported - receiver does not automatically restart
    	BACKTRACE();
    	radio_set_irq_timeout(os_getTime() + sec2osticks(5)); // time out after 5 sec
    } else {
    	BACKTRACE();
    	// busy wait until exact rx time
    	ostime_t now = os_getTime();
	    debug_printf("WARNING: FSK rxtime %u now %u\r\n", LMIC.rxtime , now);
    	if (LMIC.rxtime - now < 0) {
    	    debug_printf("WARNING: rxtime is %d ticks in the past! (ramp-up time %d ms / %d ticks)\r\n",
    			 now - LMIC.rxtime, osticks2ms(now - t0), now - t0);
    	}

    	// set preamble timeout
    	//writeReg(FSKRegRxTimeout2, (LMIC.rxsyms + 1) / 2); // (TimeoutRxPreamble * 16 * Tbit)

    }

    // enable antenna switch for RX (and account power consumption)
    hal_pin_rxtx(0);

    // rx
    writeReg(RegOpMode, OPMODE_FSK_RX);
    hal_enableIRQs();
}


#define RXMODE_RSSI 1 //XXX:FIXIT
static void sx127x_radio_cca (void) {
    // start receiver for selected channel/rps, don't receive frames
    radio_startrx(RXMODE_RSSI);
    // wait for opmode
    while ((readReg(RegOpMode) & OPMODE_MASK) != OPMODE_RX);
    // initialize threshold
    int rssi;
    int rssi_th = -RSSI_OFF + LMIC.rssi + RSSI_HF_CONST;
    int rssi_max = -RSSI_HF_CONST;
    ostime_t t0 = os_getTime();
    // delay 1ms
    t0 += ms2osticks(1);
    while ((os_getTime() - t0) < 0);
    // sample rssi values
    do {
	rssi = readReg(LORARegRssiValue);
	if (rssi > rssi_max) {
	    rssi_max = rssi;
	}
    } while (rssi < rssi_th && (os_getTime() - t0) < LMIC.rxtime);
    LMIC.rssi = -RSSI_HF_CONST + rssi_max + RSSI_OFF;
}

void sx127x_radio_startrx (bool rxcontinuous) {
    ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );

    if (getSf(LMIC.rps) == FSK) { // FSK modem
        rxfsk(rxcontinuous);
    } else { // LoRa modem
        rxlora(rxcontinuous);
    }
}

// reset radio
static void sx127x_radio_reset (void) {
    // power-up tcxo
    power_tcxo();

    // drive RST pin
    hal_pin_rst(RST_PIN_RESET_STATE);

    // wait > 100us
    hal_waitUntil(os_getTime() + ms2osticks(1));

    // configure RST pin floating
    hal_pin_rst(2);

    // wait > 5ms
    hal_waitUntil(os_getTime() + ms2osticks(10));

    // power-down TCXO
    hal_pin_tcxo(0);

    // check opmode
    // TODO: When the reset pin is not connected, the transceiver might
    // not be in standby mode now
    //ASSERT( readReg(RegOpMode) == OPMODE_FSK_STANDBY );
}

static void sx127x_radio_init (void) {
    hal_disableIRQs();

    // reset radio (FSK/STANDBY)
    sx127x_radio_reset();

    // go to SLEEP mode
    writeReg(RegOpMode, OPMODE_FSK_SLEEP);

    // sanity check, read version number
    u1_t r_ver = readReg(RegVersion);
    ASSERT( r_ver == RADIO_VERSION || r_ver == RADIO_VERSION+1 );

#ifdef BRD_sx1276_radio
    state.calibrated = 0;
#endif

    hal_enableIRQs();
}

// called by hal to check if we got one IRQ
// This trick directly read the Lora module IRQ register
// and thus avoid any IRQ line used to controler
static u1_t prev_LORARegIrqFlags = 0;
static u1_t prev_FSKRegIrqFlags2 = 0;
static u1_t prev_FSKRegIrqFlags1 = 0;

static u1_t sx127x_radio_has_irq (void) {
    u1_t flags ;
    if( (readReg(RegOpMode) & OPMODE_LORA) != 0) { // LORA modem
        flags = readReg(LORARegIrqFlags);
        flags &= ( IRQ_LORA_TXDONE_MASK | IRQ_LORA_RXDONE_MASK | IRQ_LORA_RXTOUT_MASK );
        if ( flags != prev_LORARegIrqFlags ) {
            prev_LORARegIrqFlags = flags;
            if (flags)
              return 1;
        }
    } else { // FSK modem
        flags = readReg(FSKRegIrqFlags2);
        flags &= ( IRQ_FSK2_PACKETSENT_MASK | IRQ_FSK2_PAYLOADREADY_MASK);
        if ( flags != prev_FSKRegIrqFlags2 ) {
            prev_FSKRegIrqFlags2 = flags;
            if (flags)
              return 1;
        }
        flags = readReg(FSKRegIrqFlags1);
        flags &= ( IRQ_FSK1_TIMEOUT_MASK | IRQ_FSK1_SYNCADDRESSMATCH_MASK );
        if ( flags != prev_FSKRegIrqFlags1 ) {
            prev_FSKRegIrqFlags1 = flags;
            if (flags)
              return 1;
        }
    }
    return 0;
}

// (run by irqjob)
static bool sx127x_radio_irq_process (ostime_t irqtime, u1_t diomask) {
    // dispatch modem
    if (getSf(LMIC.rps) == FSK) { // FSK modem
    	u1_t irqflags1 = readReg(FSKRegIrqFlags1);
    	u1_t irqflags2 = readReg(FSKRegIrqFlags2);

    	if (irqflags2 & IRQ_FSK2_PACKETSENT_MASK) { // TXDONE
    	    BACKTRACE();

          // save exact tx time
          LMIC.txend = irqtime - FSK_TXDONE_FIXUP;

    	} else if (irqflags2 & IRQ_FSK2_PAYLOADREADY_MASK) { // RXDONE
    	    BACKTRACE();

          // read the PDU and inform the MAC that we received something
          LMIC.dataLen = readReg(FSKRegPayloadLength);
          // now read the FIFO
          switch (LMIC.protocol->payload_type)
          {
          case RF_PAYLOAD_INVERTED:
            radio_readBuf_Inv(RegFifo, LMIC.frame, LMIC.dataLen);
            break;
          case RF_PAYLOAD_DIRECT:
          default:
            radio_readBuf(RegFifo, LMIC.frame, LMIC.dataLen);
            break;
          }

            // save exact rx timestamps
            LMIC.rxtime  = irqtime - FSK_RXDONE_FIXUP; // end of frame timestamp
	          LMIC.rxtime0 = LMIC.rxtime - calcAirTime(LMIC.rps, LMIC.dataLen); // beginning of frame timestamp
#ifdef DEBUG_RX
      	    debug_printf("RX[freq=%.1F,FSK,rssi=%d,len=%d]: %h\r\n",
      			 LMIC.freq, 6, LMIC.rssi - RSSI_OFF, LMIC.dataLen, LMIC.frame, LMIC.dataLen);
#endif

    	} else if (irqflags1 & IRQ_FSK1_TIMEOUT_MASK) { // TIMEOUT
    	    BACKTRACE();

          // indicate timeout
          LMIC.dataLen = 0;

      } else if (irqflags1 & IRQ_FSK1_SYNCADDRESSMATCH_MASK ) {
    	    BACKTRACE();

          // read rx quality parameters
          LMIC.snr  = 0; // determine snr
          // RSSI [dBm]
          LMIC.rssi = - readReg(FSKRegRssiValue) / 2;

          return false;

    	} else if( irqflags2 & IRQ_FSK2_FIFOEMPTY_MASK ) { // FIFOEMPTY (TX)
    	    BACKTRACE();

    	    // keep waiting for FifoEmpty or PacketSent interrupt
    	    return false;

    	} else if( irqflags2 & IRQ_FSK2_FIFOLEVEL_MASK ) { // FIFOLEVEL (RX)
    	    BACKTRACE();

    	    // keep waiting for FifoLevel or PayloadReady interrupt
    	    return false;

    	} else {
    	    // unexpected irq
    	    debug_printf("UNEXPECTED FSK IRQ %02x %02x\r\n", irqflags1, irqflags2);
//	    ASSERT(0);
    	}

    	// clear FSK IRQ flags
    	writeReg(FSKRegIrqFlags1, 0xFF);
    	writeReg(FSKRegIrqFlags2, 0xFF);

    } else { // LORA modem
    	u1_t irqflags = readReg(LORARegIrqFlags);

        if (irqflags & IRQ_LORA_TXDONE_MASK) { // TXDONE
    	    BACKTRACE();

          // save exact tx time
          LMIC.txend = irqtime - LORA_TXDONE_FIXUP;

        } else if (irqflags & IRQ_LORA_RXDONE_MASK) { // RXDONE (rx or scan)
            /*
             * RF noise may trigger LoRa packets receiver
             * to capture some garbage into the FIFO.
             * CRC checker helps to filter them out.
             */
            u1_t crc_on = readReg(LORARegHopChannel) & RXLORA_REG_HOPCHANNEL_CRC_ON_PAYLOAD;

            if (crc_on && !(irqflags & IRQ_LORA_CRCERR_MASK)) {
        	    BACKTRACE();

              // read rx quality parameters
              uint8_t value = readReg(LORARegPktSnrValue);

              if( value & 0x80 ) {                      // The SNR sign bit is 1
                value = ( ( ~value + 1 ) & 0xFF ) >> 2; // Invert and divide by 4
                LMIC.snr = -value;
              } else {
                // Divide by 4
                LMIC.snr = ( value & 0xFF ) >> 2;
              }

              int rssiAdjust = LMIC.freq > SX127X_FREQ_LF_MAX ?
                SX127X_RSSI_ADJUST_HF : SX127X_RSSI_ADJUST_LF;

              if (LMIC.snr < 0) {
                LMIC.rssi = rssiAdjust + readReg(LORARegPktRssiValue) + LMIC.snr;
              } else {
                LMIC.rssi = rssiAdjust + (16 * readReg(LORARegPktRssiValue)) / 15;
              }

              // get PDU length
              LMIC.dataLen = readReg(LORARegRxNbBytes);

              // save exact rx timestamps
              LMIC.rxtime = irqtime; // end of frame timestamp
              if (getBw(LMIC.rps) == BW125) {
                  LMIC.rxtime -= LORA_RXDONE_FIXUP_125[getSf(LMIC.rps)];
              }
              else if (getBw(LMIC.rps) == BW500) {
                  LMIC.rxtime -= LORA_RXDONE_FIXUP_500[getSf(LMIC.rps)];
              }
        	    LMIC.rxtime0 = LMIC.rxtime - calcAirTime(LMIC.rps, LMIC.dataLen); // beginning of frame timestamp

        	    // set FIFO read address pointer (to address of last packet received)
        	    writeReg(LORARegFifoAddrPtr, readReg(LORARegFifoRxCurrentAddr));

        	    // read FIFO
        	    radio_readBuf(RegFifo, LMIC.frame, LMIC.dataLen);
#ifdef DEBUG_RX
        	    debug_printf("RX[freq=%.1F,sf=%d,bw=%s,rssi=%d,snr=%.2F,len=%d]: %h\r\n",
        			 LMIC.freq, 6,
        			 getSf(LMIC.rps) + 6, ("125\0" "250\0" "500\0" "rfu") + (4 * getBw(LMIC.rps)),
        			 LMIC.rssi - RSSI_OFF, LMIC.snr * 100 / SNR_SCALEUP, 2,
        			 LMIC.dataLen,
        			 LMIC.frame, LMIC.dataLen);
#endif
            } else {
              // indicate CRC error
              LMIC.dataLen = 0;
            }
      	} else if (irqflags & IRQ_LORA_RXTOUT_MASK) { // RXTOUT
      	    BACKTRACE();

            // indicate timeout
            LMIC.dataLen = 0;

        } else {
      	    // unexpected irq
//	        ASSERT(0);
      	}

    	// mask all LoRa IRQs
    	writeReg(LORARegIrqFlagsMask, 0xFF);

    	// clear LoRa IRQ flags
    	writeReg(LORARegIrqFlags, 0xFF);
    }

    // radio operation completed
    return true;
}

const SX12XX_ops_t sx127x_ll_ops = {
    sx127x_radio_init,
    sx127x_radio_sleep,
    sx127x_radio_starttx,
    sx127x_radio_startrx,
    sx127x_radio_has_irq,
    sx127x_radio_irq_process,
    sx127x_radio_cca
};

#endif
