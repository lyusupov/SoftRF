/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "lmic.h"
#include <protocol.h>

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
// #define RegTcxo                                    0x58 // common
#define SX1272_RegTcxo                             0x58
#define SX1276_RegTcxo                             0x4B
#define SX1272_RegPaDac                            0x5A
#define SX1276_RegPaDac                            0x4D
// #define RegPll                                     0x5C // common
// #define RegPllLowPn                                0x5E // common
// #define RegFormerTemp                              0x6C // common
#define SX1272_RegBitRateFrac                      0x70
#define SX1276_RegBitRateFrac                      0x5D

#ifdef CFG_sx1276_radio
#define RADIO_VERSION               0x12
#define RegPaDac                    SX1276_RegPaDac
#define RegTcxo                     SX1276_RegTcxo
#define RegBitRateFrac              SX1276_RegBitRateFrac
#elif CFG_sx1272_radio
#define RADIO_VERSION               0x22
#define RegPaDac                    SX1272_RegPaDac
#define RegTcxo                     SX1272_RegTcxo
#define RegBitRateFrac              SX1272_RegBitRateFrac
#endif

// ----------------------------------------
// spread factors and mode for RegModemConfig2
#define SX1272_MC2_FSK  0x00
#define SX1272_MC2_SF7  0x70
#define SX1272_MC2_SF8  0x80
#define SX1272_MC2_SF9  0x90
#define SX1272_MC2_SF10 0xA0
#define SX1272_MC2_SF11 0xB0
#define SX1272_MC2_SF12 0xC0
// bandwidth for RegModemConfig1
#define SX1272_MC1_BW_125  0x00
#define SX1272_MC1_BW_250  0x40
#define SX1272_MC1_BW_500  0x80
// coding rate for RegModemConfig1
#define SX1272_MC1_CR_4_5 0x08
#define SX1272_MC1_CR_4_6 0x10
#define SX1272_MC1_CR_4_7 0x18
#define SX1272_MC1_CR_4_8 0x20
#define SX1272_MC1_IMPLICIT_HEADER_MODE_ON 0x04 // required for receive
#define SX1272_MC1_RX_PAYLOAD_CRCON        0x02
#define SX1272_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12
// transmit power configuration for RegPaConfig
#define SX1272_PAC_PA_SELECT_PA_BOOST 0x80
#define SX1272_PAC_PA_SELECT_RFIO_PIN 0x00


// sx1276 RegModemConfig1
#define SX1276_MC1_BW_125                0x70
#define SX1276_MC1_BW_250                0x80
#define SX1276_MC1_BW_500                0x90
#define SX1276_MC1_CR_4_5            0x02
#define SX1276_MC1_CR_4_6            0x04
#define SX1276_MC1_CR_4_7            0x06
#define SX1276_MC1_CR_4_8            0x08

#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON    0x01

// sx1276 RegModemConfig2
#define SX1276_MC2_RX_PAYLOAD_CRCON        0x04

// sx1276 RegModemConfig3
#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE  0x08
#define SX1276_MC3_AGCAUTO                 0x04

// preamble for lora networks (nibbles swapped)
#define LORA_MAC_PREAMBLE                  0x34

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
#ifdef CFG_sx1276_radio
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70
#elif CFG_sx1272_radio
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x74
#endif


//-----------------------------------------
// Parameters for RSSI monitoring
#define SX127X_FREQ_LF_MAX      525000000       // per datasheet 6.3

// per datasheet 5.5.3:
#define SX127X_RSSI_ADJUST_LF   -164            // add to rssi value to get dB (LF)
#define SX127X_RSSI_ADJUST_HF   -157            // add to rssi value to get dB (HF)

// ----------------------------------------
// Constants for radio registers
#define OPMODE_LORA      0x80
#define OPMODE_MASK      0x07
#define OPMODE_SLEEP     0x00
#define OPMODE_STANDBY   0x01
#define OPMODE_FSTX      0x02
#define OPMODE_TX        0x03
#define OPMODE_FSRX      0x04
#define OPMODE_RX        0x05
#define OPMODE_RX_SINGLE 0x06
#define OPMODE_CAD       0x07

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK 0x80
#define IRQ_LORA_RXDONE_MASK 0x40
#define IRQ_LORA_CRCERR_MASK 0x20
#define IRQ_LORA_HEADER_MASK 0x10
#define IRQ_LORA_TXDONE_MASK 0x08
#define IRQ_LORA_CDDONE_MASK 0x04
#define IRQ_LORA_FHSSCH_MASK 0x02
#define IRQ_LORA_CDDETD_MASK 0x01

#define IRQ_FSK1_MODEREADY_MASK         0x80
#define IRQ_FSK1_RXREADY_MASK           0x40
#define IRQ_FSK1_TXREADY_MASK           0x20
#define IRQ_FSK1_PLLLOCK_MASK           0x10
#define IRQ_FSK1_RSSI_MASK              0x08
#define IRQ_FSK1_TIMEOUT_MASK           0x04
#define IRQ_FSK1_PREAMBLEDETECT_MASK    0x02
#define IRQ_FSK1_SYNCADDRESSMATCH_MASK  0x01
#define IRQ_FSK2_FIFOFULL_MASK          0x80
#define IRQ_FSK2_FIFOEMPTY_MASK         0x40
#define IRQ_FSK2_FIFOLEVEL_MASK         0x20
#define IRQ_FSK2_FIFOOVERRUN_MASK       0x10
#define IRQ_FSK2_PACKETSENT_MASK        0x08
#define IRQ_FSK2_PAYLOADREADY_MASK      0x04
#define IRQ_FSK2_CRCOK_MASK             0x02
#define IRQ_FSK2_LOWBAT_MASK            0x01

// ----------------------------------------
// DIO function mappings                D0D1D2D3
#define MAP_DIO0_LORA_RXDONE   0x00  // 00------
#define MAP_DIO0_LORA_TXDONE   0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT   0x00  // --00----
#define MAP_DIO1_LORA_NOP      0x30  // --11----
#define MAP_DIO2_LORA_NOP      0xC0  // ----11--

#define MAP_DIO0_FSK_READY     0x00  // 00------ (packet sent / payload ready)
#define MAP_DIO1_FSK_NOP       0x30  // --11----
#define MAP_DIO2_FSK_TXNOP     0x04  // ----01--
#define MAP_DIO2_FSK_TIMEOUT   0x08  // ----10--


// FSK IMAGECAL defines
#define RF_IMAGECAL_AUTOIMAGECAL_MASK               0x7F
#define RF_IMAGECAL_AUTOIMAGECAL_ON                 0x80
#define RF_IMAGECAL_AUTOIMAGECAL_OFF                0x00  // Default

#define RF_IMAGECAL_IMAGECAL_MASK                   0xBF
#define RF_IMAGECAL_IMAGECAL_START                  0x40

#define RF_IMAGECAL_IMAGECAL_RUNNING                0x20
#define RF_IMAGECAL_IMAGECAL_DONE                   0x00  // Default


// RADIO STATE
// (initialized by radio_init(), used by radio_rand1())
static u1_t randbuf[16];


#ifdef CFG_sx1276_radio
#define LNA_RX_GAIN (0x20|0x1)
// PA config
#define PA_BOOST            0x80
#define DAC_HIGH_POWER      0x07    // use for +20 dBm power output
#define DAC_DEFAULT_POWER   0x04    // use for +17 dBm or lower output
// over current config
#define DEFAULT_CURRENT     0x2b    // default over current setting (100 mA)
#define HIGHPOWER_CURRENT   0x31    // setting for +20 dBm output (140 mA)

#define OPMODE_FSK_SX1276_LowFrequencyModeOn        (1u << 3)
#define OPMODE_LORA_SX1276_LowFrequencyModeOn       (1u << 3)
#elif CFG_sx1272_radio
#define LNA_RX_GAIN (0x20|0x03)
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif

#define readReg(addr)                 hal_spi_read_reg (addr)
#define writeReg(addr, data)          hal_spi_write_reg(addr, data)
#define readBuf(addr, buf, len)       hal_spi_read_buf (addr, buf, len, 0)
#define readBuf_Inv(addr, buf, len)   hal_spi_read_buf (addr, buf, len, 1)
#define writeBuf(addr, buf, len)      hal_spi_write_buf(addr, buf, len, 0)
#define writeBuf_Inv(addr, buf, len)  hal_spi_write_buf(addr, buf, len, 1)

//#define FIFO_Push_Inv(x)              writeReg(RegFifo, (u1_t)~(x))

#if defined(CONFIG_IDF_TARGET_ESP32C5)
#if CONFIG_ARDUINO_ISR_IRAM
#define ARDUINO_ISR_ATTR IRAM_ATTR
#else
#define ARDUINO_ISR_ATTR
#endif
extern void ARDUINO_ISR_ATTR delayMicroseconds(uint32_t);
#endif /* CONFIG_IDF_TARGET_ESP32C5 */

static void opmode (u1_t mode) {
#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2) || defined(RASPBERRY_PI)
    delay(1);
#endif
    writeReg(RegOpMode, (readReg(RegOpMode) & ~OPMODE_MASK) | mode);
#if defined(CONFIG_IDF_TARGET_ESP32C5)
    delayMicroseconds(1500);
#else
    delay(1);
#endif
}

static void opmodeLora() {
    u1_t u = OPMODE_LORA;
#ifdef CFG_sx1276_radio
    if (LMIC.freq <= SX127X_FREQ_LF_MAX) {
        u |= OPMODE_LORA_SX1276_LowFrequencyModeOn;
    }
#endif
    writeReg(RegOpMode, u);
}

static void opmodeFSK() {
    u1_t u = 0;
#ifdef CFG_sx1276_radio
    if (LMIC.freq <= SX127X_FREQ_LF_MAX) {
        u |= OPMODE_FSK_SX1276_LowFrequencyModeOn;
    }
#endif
    writeReg(RegOpMode, u);
}

// configure LoRa modem (cfg1, cfg2)
static void configLoraModem () {
    sf_t sf = getSf(LMIC.rps);

#ifdef CFG_sx1276_radio
        u1_t mc1 = 0, mc2 = 0, mc3 = 0;

        switch (getBw(LMIC.rps)) {
        case BW125: mc1 |= SX1276_MC1_BW_125; break;
        case BW250: mc1 |= SX1276_MC1_BW_250; break;
        case BW500: mc1 |= SX1276_MC1_BW_500; break;
        default:
            ASSERT(0);
        }
        switch( getCr(LMIC.rps) ) {
        case CR_4_5: mc1 |= SX1276_MC1_CR_4_5; break;
        case CR_4_6: mc1 |= SX1276_MC1_CR_4_6; break;
        case CR_4_7: mc1 |= SX1276_MC1_CR_4_7; break;
        case CR_4_8: mc1 |= SX1276_MC1_CR_4_8; break;
        default:
            ASSERT(0);
        }

        if (getIh(LMIC.rps)) {
            mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;
            writeReg(LORARegPayloadLength, getIh(LMIC.rps)); // required length
        }
        // set ModemConfig1
        writeReg(LORARegModemConfig1, mc1);

        mc2 = (SX1272_MC2_SF7 + ((sf-1)<<4));
        if (getNocrc(LMIC.rps) == 0) {
            mc2 |= SX1276_MC2_RX_PAYLOAD_CRCON;
        }
        writeReg(LORARegModemConfig2, mc2);

        mc3 = SX1276_MC3_AGCAUTO;
        if ((sf == SF11 || sf == SF12) && getBw(LMIC.rps) == BW125) {
            mc3 |= SX1276_MC3_LOW_DATA_RATE_OPTIMIZE;
        }
        writeReg(LORARegModemConfig3, mc3);
#elif CFG_sx1272_radio
        u1_t mc1 = (getBw(LMIC.rps)<<6);

        switch( getCr(LMIC.rps) ) {
        case CR_4_5: mc1 |= SX1272_MC1_CR_4_5; break;
        case CR_4_6: mc1 |= SX1272_MC1_CR_4_6; break;
        case CR_4_7: mc1 |= SX1272_MC1_CR_4_7; break;
        case CR_4_8: mc1 |= SX1272_MC1_CR_4_8; break;
        }

        if ((sf == SF11 || sf == SF12) && getBw(LMIC.rps) == BW125) {
            mc1 |= SX1272_MC1_LOW_DATA_RATE_OPTIMIZE;
        }

        if (getNocrc(LMIC.rps) == 0) {
            mc1 |= SX1272_MC1_RX_PAYLOAD_CRCON;
        }

        if (getIh(LMIC.rps)) {
            mc1 |= SX1272_MC1_IMPLICIT_HEADER_MODE_ON;
            writeReg(LORARegPayloadLength, getIh(LMIC.rps)); // required length
        }
        // set ModemConfig1
        writeReg(LORARegModemConfig1, mc1);

        // set ModemConfig2 (sf, AgcAutoOn=1 SymbTimeoutHi=00)
        writeReg(LORARegModemConfig2, (SX1272_MC2_SF7 + ((sf-1)<<4)) | 0x04);

#if CFG_TxContinuousMode
        // Only for testing
        // set ModemConfig2 (sf, TxContinuousMode=1, AgcAutoOn=1 SymbTimeoutHi=00)
        writeReg(LORARegModemConfig2, (SX1272_MC2_SF7 + ((sf-1)<<4)) | 0x06);
#endif

#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif /* CFG_sx1272_radio */
}

static void configChannel () {
    // set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
    uint64_t frf = ((uint64_t)LMIC.freq << 19) / 32000000;
    writeReg(RegFrfMsb, (u1_t)(frf>>16));
    writeReg(RegFrfMid, (u1_t)(frf>> 8));
    writeReg(RegFrfLsb, (u1_t)(frf>> 0));
}


static void configPower () {
#ifdef CFG_sx1276_radio
    // set PA config (2-17 dBm using PA_BOOST)
    s1_t pw = (s1_t) LMIC.txpow;

    if (pw < 2) {
      pw = 2;
    }

    u1_t dac = (readReg(RegPaDac) & 0xF8) | \
                (pw > 17 ? DAC_HIGH_POWER : DAC_DEFAULT_POWER);

    /*
     * set current limit to:
     * 140 mA for >17 dBm
     * 100 mA - by default
     */
    u1_t ocp = (pw > 17 ? HIGHPOWER_CURRENT : DEFAULT_CURRENT);

    if (pw > 17) {
      pw = 17;
    }

    // check board type for BOOST pin
    writeReg(RegOcp, ocp);
    writeReg(RegPaDac, dac);
    writeReg(RegPaConfig, (u1_t)(PA_BOOST|((pw-2)&0xf)));

#elif CFG_sx1272_radio
    // set PA config (2-17 dBm using PA_BOOST)
    s1_t pw = (s1_t)LMIC.txpow;
    if(pw > 17) {
        pw = 17;
    } else if(pw < 2) {
        pw = 2;
    }
    writeReg(RegPaConfig, (u1_t)(0x80|(pw-2)));
#else
#error Missing CFG_sx1272_radio/CFG_sx1276_radio
#endif /* CFG_sx1272_radio */
}

static void power_tcxo (void) {
    // power-up TCXO and set tcxo as input
    if ( hal_pin_tcxo(1) ) {
      writeReg(RegTcxo, 0b00011001); // reserved=000, tcxo=1, reserved=1001
    }
}

#define RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY 0x80

static void txfsk () {
    // select FSK modem (from sleep mode)
    writeReg(RegOpMode, 0x0 /* 0x10 */ ); // FSK, BT=0.5
    ASSERT(readReg(RegOpMode) == 0x0 /* 0x10 */ );

    // power-up tcxo
    power_tcxo();

    writeReg(FSKRegIrqFlags2, IRQ_FSK2_FIFOOVERRUN_MASK);

    // enter standby mode (required for FIFO loading))
    opmode(OPMODE_STANDBY);

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

    int i=0;
    for (i=0; i < LMIC.protocol->syncword_size; i++) {
      writeReg((FSKRegSyncValue1 + i), LMIC.protocol->syncword[i]);
    }

    // configure frequency
    configChannel();
    // configure output power
    writeReg(RegPaRamp, 0x49); 
    configPower();

    // set the IRQ mapping DIO0=PacketSent DIO1=NOP DIO2=NOP
    writeReg(RegDioMapping1, MAP_DIO0_FSK_READY|MAP_DIO1_FSK_NOP|MAP_DIO2_FSK_TXNOP);

    // initialize the payload size and address pointers
    writeReg(FSKRegPayloadLength, LMIC.dataLen); // (insert length byte into payload))


    writeReg(FSKRegFifoThresh, RF_FIFOTHRESH_TXSTARTCONDITION_FIFONOTEMPTY  );

    switch (LMIC.protocol->payload_type)
    {
    case RF_PAYLOAD_INVERTED:
      writeBuf_Inv(RegFifo, LMIC.frame, LMIC.dataLen);
      break;
    case RF_PAYLOAD_DIRECT:
    default:
      writeBuf(RegFifo, LMIC.frame, LMIC.dataLen);
      break;
    }

    // enable antenna switch for TX
    hal_pin_rxtx(1);

    // now we actually start the transmission
    opmode(OPMODE_TX);
}

static void txlora () {
    // select LoRa modem (from sleep mode)
    //writeReg(RegOpMode, OPMODE_LORA);
    //Serial.println("+++ txlora +++");
    opmodeLora();
    ASSERT((readReg(RegOpMode) & OPMODE_LORA) != 0);

    // power-up tcxo
    power_tcxo();

    // enter standby mode (required for FIFO loading))
    opmode(OPMODE_STANDBY);
    // configure LoRa modem (cfg1, cfg2)
    configLoraModem();
    // configure frequency
    configChannel();
    // configure output power
    writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
    configPower();

    // set sync word
    writeReg(LORARegSyncWord, LMIC.syncword);

    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeReg(RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);
    // mask all IRQs but TxDone
    writeReg(LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);

    // initialize the payload size and address pointers
    writeReg(LORARegFifoTxBaseAddr, 0x00);
    writeReg(LORARegFifoAddrPtr, 0x00);
    writeReg(LORARegPayloadLength, LMIC.dataLen);

    // download buffer to the radio FIFO
    writeBuf(RegFifo, LMIC.frame, LMIC.dataLen);

    // enable antenna switch for TX
    hal_pin_rxtx(1);

    // now we actually start the transmission
    opmode(OPMODE_TX);

#if LMIC_DEBUG_LEVEL > 0
    u1_t sf = getSf(LMIC.rps) + 6; // 1 == SF7
    u1_t bw = getBw(LMIC.rps);
    u1_t cr = getCr(LMIC.rps);
    printf("%lu: TXMODE, freq=%lu, len=%d, SF=%d, BW=%d, CR=4/%d, IH=%d\n",
           os_getTime(), LMIC.freq, LMIC.dataLen, sf,
           bw == BW125 ? 125 : (bw == BW250 ? 250 : 500),
           cr == CR_4_5 ? 5 : (cr == CR_4_6 ? 6 : (cr == CR_4_7 ? 7 : 8)),
           getIh(LMIC.rps)
   );
#endif
}

// start transmitter (buf=LMIC.frame, len=LMIC.dataLen)
static void starttx () {
    ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );
    if(getSf(LMIC.rps) == FSK) { // FSK modem
        txfsk();
    } else { // LoRa modem
        txlora();
    }
    // the radio will go back to STANDBY mode as soon as the TX is finished
    // the corresponding IRQ will inform us about completion.
}

enum { RXMODE_SINGLE, RXMODE_SCAN, RXMODE_RSSI };

static CONST_TABLE(u1_t, rxlorairqmask)[] = {
    [RXMODE_SINGLE] = IRQ_LORA_RXDONE_MASK|IRQ_LORA_RXTOUT_MASK,
    [RXMODE_SCAN]   = IRQ_LORA_RXDONE_MASK|IRQ_LORA_CRCERR_MASK,
    [RXMODE_RSSI]   = 0x00,
};

// start LoRa receiver (time=LMIC.rxtime, timeout=LMIC.rxsyms, result=LMIC.frame[LMIC.dataLen])
static void rxlora (u1_t rxmode) {
    //Serial.println("*** rxlora ***");
    // select LoRa modem (from sleep mode)
    opmodeLora();
    ASSERT((readReg(RegOpMode) & OPMODE_LORA) != 0);

    // power-up tcxo
    power_tcxo();

    // enter standby mode (warm up))
    opmode(OPMODE_STANDBY);
    // don't use MAC settings at startup
    if(rxmode == RXMODE_RSSI) { // use fixed settings for rssi scan
        writeReg(LORARegModemConfig1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1);
        writeReg(LORARegModemConfig2, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2);
    } else { // single or continuous rx mode
        // configure LoRa modem (cfg1, cfg2)
        configLoraModem();
        // configure frequency
        configChannel();
    }
    // set LNA gain
    writeReg(RegLna, LNA_RX_GAIN);
    // set max payload size
    writeReg(LORARegPayloadMaxLength, 64);
#if !defined(DISABLE_INVERT_IQ_ON_RX)
    // use inverted I/Q signal (prevent mote-to-mote communication)

    // XXX: use flag to switch on/off inversion
    if (LMIC.noRXIQinversion) {
        writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ) & ~(1<<6));
    } else {
        writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ)|(1<<6));
    }
#endif
    // set symbol timeout (for single rx)
    writeReg(LORARegSymbTimeoutLsb, LMIC.rxsyms);

    // set sync word
    writeReg(LORARegSyncWord, LMIC.syncword);

    // configure DIO mapping DIO0=RxDone DIO1=RxTout DIO2=NOP
    writeReg(RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_RXTOUT|MAP_DIO2_LORA_NOP);
    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);
    // enable required radio IRQs
    writeReg(LORARegIrqFlagsMask, ~TABLE_GET_U1(rxlorairqmask, rxmode));

    // enable antenna switch for RX
    hal_pin_rxtx(0);

    // now instruct the radio to receive
    if (rxmode == RXMODE_SINGLE) { // single rx
        hal_waitUntil(LMIC.rxtime); // busy wait until exact rx time
        opmode(OPMODE_RX_SINGLE);
    } else { // continous rx (scan or rssi)
        opmode(OPMODE_RX);
    }

#if LMIC_DEBUG_LEVEL > 0
    if (rxmode == RXMODE_RSSI) {
        printf("RXMODE_RSSI\n");
    } else {
        u1_t sf = getSf(LMIC.rps) + 6; // 1 == SF7
        u1_t bw = getBw(LMIC.rps);
        u1_t cr = getCr(LMIC.rps);
        printf("%lu: %s, freq=%lu, SF=%d, BW=%d, CR=4/%d, IH=%d\n",
               os_getTime(),
               rxmode == RXMODE_SINGLE ? "RXMODE_SINGLE" : (rxmode == RXMODE_SCAN ? "RXMODE_SCAN" : "UNKNOWN_RX"),
               LMIC.freq, sf,
               bw == BW125 ? 125 : (bw == BW250 ? 250 : 500),
               cr == CR_4_5 ? 5 : (cr == CR_4_6 ? 6 : (cr == CR_4_7 ? 7 : 8)),
               getIh(LMIC.rps)
       );
    }
#endif
}

static void rxfsk (u1_t rxmode) {
    // only single rx (no continuous scanning, no noise sampling)
    ASSERT( rxmode == RXMODE_SINGLE );
    // select FSK modem (from sleep mode)
    //writeReg(RegOpMode, 0x00); // (not LoRa)
    opmodeFSK();
    ASSERT((readReg(RegOpMode) & OPMODE_LORA) == 0);

    // power-up tcxo
    power_tcxo();

    // enter standby mode (warm up))
    opmode(OPMODE_STANDBY);

    // configure frequency
    configChannel();

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

    writeReg(FSKRegPayloadLength,
      LMIC.protocol->payload_size +
      LMIC.protocol->payload_offset +
      LMIC.protocol->crc_size);

    // set sync value
    int i=0;
    for (i=0; i < LMIC.protocol->syncword_size; i++) {
      writeReg((FSKRegSyncValue1 + i), LMIC.protocol->syncword[i]);
    }

    // set preamble timeout
    //writeReg(FSKRegRxTimeout2, 0xFF);

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

    // I hope that 256 samples will cover full Rx packet
    // writeReg(FSKRegRssiConfig, 0x07);

    // configure DIO mapping DIO0=PayloadReady DIO1=NOP DIO2=TimeOut
    writeReg(RegDioMapping1, MAP_DIO0_FSK_READY|MAP_DIO1_FSK_NOP|MAP_DIO2_FSK_TIMEOUT);

    // enable antenna switch for RX
    hal_pin_rxtx(0);
#if 0  /* SoftRF: there is no need to wait for anything here */
    // now instruct the radio to receive
    hal_waitUntil(LMIC.rxtime); // busy wait until exact rx time
#endif
    opmode(OPMODE_RX); // no single rx mode available in FSK
}

static void startrx (u1_t rxmode) {
    ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );
    if(getSf(LMIC.rps) == FSK) { // FSK modem
        rxfsk(rxmode);
    } else { // LoRa modem
        rxlora(rxmode);
    }
    // the radio will go back to STANDBY mode as soon as the RX is finished
    // or timed out, and the corresponding IRQ will inform us about completion.
}

// get random seed from wideband noise rssi
void radio_init () {
    hal_disableIRQs();

    // power-up tcxo
    power_tcxo();

    // manually reset radio
#ifdef CFG_sx1276_radio
    hal_pin_rst(0); // drive RST pin low
#else
    hal_pin_rst(1); // drive RST pin high
#endif
    hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us
    hal_pin_rst(2); // configure RST pin floating!
    hal_waitUntil(os_getTime()+ms2osticks(5)); // wait 5ms

    // power-down TCXO
    hal_pin_tcxo(0);

    opmode(OPMODE_SLEEP);

    // sanity check, read version number
    u1_t r_ver = readReg(RegVersion);
    ASSERT( r_ver == RADIO_VERSION || r_ver == RADIO_VERSION+1 );

    // seed 15-byte randomness via noise rssi
    rxlora(RXMODE_RSSI);
    while( (readReg(RegOpMode) & OPMODE_MASK) != OPMODE_RX ); // continuous rx
#if !defined(CONFIG_IDF_TARGET_ESP32C5)
    int i=1;
    for(i=1; i<16; i++) {
        int j=0;
        for(j=0; j<8; j++) {
            u1_t b; // wait for two non-identical subsequent least-significant bits
            while( (b = readReg(LORARegRssiWideband) & 0x01) == (readReg(LORARegRssiWideband) & 0x01) );
            randbuf[i] = (randbuf[i] << 1) | b;
        }
    }
    randbuf[0] = 16; // set initial index
#endif /* CONFIG_IDF_TARGET_ESP32C5 */
#ifdef CFG_sx1276mb1_board
    // chain calibration
    writeReg(RegPaConfig, 0);

    // Launch Rx chain calibration for LF band
    writeReg(FSKRegImageCal, (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK)|RF_IMAGECAL_IMAGECAL_START);
    while((readReg(FSKRegImageCal)&RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING){ ; }

    // Sets a Frequency in HF band
    u4_t frf = 868000000;
    writeReg(RegFrfMsb, (u1_t)(frf>>16));
    writeReg(RegFrfMid, (u1_t)(frf>> 8));
    writeReg(RegFrfLsb, (u1_t)(frf>> 0));

    // Launch Rx chain calibration for HF band
    writeReg(FSKRegImageCal, (readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK)|RF_IMAGECAL_IMAGECAL_START);
    while((readReg(FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING) { ; }
#endif /* CFG_sx1276mb1_board */

    opmode(OPMODE_SLEEP);

    hal_enableIRQs();
}

// return next random byte derived from seed buffer
// (buf[0] holds index of next byte to be returned)
u1_t radio_rand1 () {
#if !defined(CONFIG_IDF_TARGET_ESP32C5)
    u1_t i = randbuf[0];
    ASSERT( i != 0 );
    if( i==16 ) {
        os_aes(AES_ENC, randbuf, 16); // encrypt seed with any key
        i = 0;
    }
    u1_t v = randbuf[i++];
    randbuf[0] = i;
    return v;
#else
    return 0;
#endif /* CONFIG_IDF_TARGET_ESP32C5 */
}

u1_t radio_rssi () {
    hal_disableIRQs();
    u1_t r = readReg(LORARegRssiValue);
    hal_enableIRQs();
    return r;
}

static CONST_TABLE(u2_t, LORA_RXDONE_FIXUP)[] = {
    [FSK]  =     us2osticks(0), // (   0 ticks)
    [SF7]  =     us2osticks(0), // (   0 ticks)
    [SF8]  =  us2osticks(1648), // (  54 ticks)
    [SF9]  =  us2osticks(3265), // ( 107 ticks)
    [SF10] =  us2osticks(7049), // ( 231 ticks)
    [SF11] = us2osticks(13641), // ( 447 ticks)
    [SF12] = us2osticks(31189), // (1022 ticks)
};


// called by hal to check if we got one IRQ
// This trick directly read the Lora module IRQ register
// and thus avoid any IRQ line used to controler
u1_t radio_has_irq (void) {
    u1_t flags ;
    if( (readReg(RegOpMode) & OPMODE_LORA) != 0) { // LORA modem
        flags = readReg(LORARegIrqFlags);
        if( flags & ( IRQ_LORA_TXDONE_MASK | IRQ_LORA_RXDONE_MASK | IRQ_LORA_RXTOUT_MASK ) ) 
            return 1;
    } else { // FSK modem
        flags = readReg(FSKRegIrqFlags2);
        if ( flags & ( IRQ_FSK2_PACKETSENT_MASK | IRQ_FSK2_PAYLOADREADY_MASK) ) 
            return 1;
        flags = readReg(FSKRegIrqFlags1);
        if ( flags & ( IRQ_FSK1_TIMEOUT_MASK | IRQ_FSK1_SYNCADDRESSMATCH_MASK ) )
            return 1;
    }
    return 0;
}

#define RXLORA_REG_HOPCHANNEL_CRC_ON_PAYLOAD 0x40

// called by hal ext IRQ handler
// (radio goes to stanby mode after tx/rx operations)
void radio_irq_handler (u1_t dio) {
#if CFG_TxContinuousMode
    // clear radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);
    u1_t p = readReg(LORARegFifoAddrPtr);
    writeReg(LORARegFifoAddrPtr, 0x00);
    u1_t s = readReg(RegOpMode);
    u1_t c = readReg(LORARegModemConfig2);
    opmode(OPMODE_TX);
    return;
#else /* ! CFG_TxContinuousMode */
    ostime_t now = os_getTime();
    if( (readReg(RegOpMode) & OPMODE_LORA) != 0) { // LORA modem
        u1_t flags = readReg(LORARegIrqFlags);
        if( flags & IRQ_LORA_TXDONE_MASK ) {
            // save exact tx time
            LMIC.txend = now - us2osticks(43); // TXDONE FIXUP
        } else if( flags & IRQ_LORA_RXDONE_MASK ) {
            /*
             * RF noise may trigger LoRa packets receiver
             * to capture some garbage into the FIFO.
             * CRC checker helps to filter them out.
             */
            u1_t crc_on = readReg(LORARegHopChannel) & RXLORA_REG_HOPCHANNEL_CRC_ON_PAYLOAD;

            if (crc_on && !(flags & IRQ_LORA_CRCERR_MASK)) {
              // save exact rx time
              if(getBw(LMIC.rps) == BW125) {
                  now -= TABLE_GET_U2(LORA_RXDONE_FIXUP, getSf(LMIC.rps));
              }
              LMIC.rxtime = now;
              // read the PDU and inform the MAC that we received something
              LMIC.dataLen = (readReg(LORARegModemConfig1) & SX1272_MC1_IMPLICIT_HEADER_MODE_ON) ?
                  readReg(LORARegPayloadLength) : readReg(LORARegRxNbBytes);
              // set FIFO read address pointer
              writeReg(LORARegFifoAddrPtr, readReg(LORARegFifoRxCurrentAddr));
              // now read the FIFO
              readBuf(RegFifo, LMIC.frame, LMIC.dataLen);
              // read rx quality parameters
              //LMIC.snr  = readReg(LORARegPktSnrValue); // SNR [dB] * 4
              //LMIC.rssi = readReg(LORARegPktRssiValue) - 125 + 64; // RSSI [dBm] (-196...+63)
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
            } else {
              // indicate CRC error
              LMIC.dataLen = 0;
            }
        } else if( flags & IRQ_LORA_RXTOUT_MASK ) {
            // indicate timeout
            LMIC.dataLen = 0;
        }
        // mask all radio IRQs
        writeReg(LORARegIrqFlagsMask, 0xFF);
        // clear radio IRQ flags
        writeReg(LORARegIrqFlags, 0xFF);
    } else { // FSK modem
        u1_t flags1 = readReg(FSKRegIrqFlags1);
        u1_t flags2 = readReg(FSKRegIrqFlags2);
        if( flags2 & IRQ_FSK2_PACKETSENT_MASK ) {
            // save exact tx time
            LMIC.txend = now;
        } else if( flags2 & IRQ_FSK2_PAYLOADREADY_MASK ) {
            // save exact rx time
            LMIC.rxtime = now;
            // read the PDU and inform the MAC that we received something
            LMIC.dataLen = readReg(FSKRegPayloadLength);
            // now read the FIFO
            switch (LMIC.protocol->payload_type)
            {
            case RF_PAYLOAD_INVERTED:
              readBuf_Inv(RegFifo, LMIC.frame, LMIC.dataLen);
              break;
            case RF_PAYLOAD_DIRECT:
            default:
              readBuf(RegFifo, LMIC.frame, LMIC.dataLen);
              break;
            }
            // read rx quality parameters
            // LMIC.snr  = 0; // determine snr
            // Average RSSI [dBm] for prev. 256 samples
            // LMIC.rssi = - readReg(FSKRegRssiValue) / 2;
        } else if( flags1 & IRQ_FSK1_TIMEOUT_MASK ) {
            // indicate timeout
            LMIC.dataLen = 0;
        } else if( flags1 & IRQ_FSK1_SYNCADDRESSMATCH_MASK ) {
            // read rx quality parameters
            LMIC.snr  = 0; // determine snr
            // RSSI [dBm]
            LMIC.rssi = - readReg(FSKRegRssiValue) / 2;
            return;
        } else {
          if( ! (flags2 & IRQ_FSK2_FIFOEMPTY_MASK) ) {
            ASSERT(0);
          }
        }
    }
    // go from stanby to sleep
    opmode(OPMODE_SLEEP);
    // run os job (use preset func ptr)
    os_setCallback(&LMIC.osjob, LMIC.osjob.func);
#endif /* ! CFG_TxContinuousMode */
}

void os_radio (u1_t mode) {
    hal_disableIRQs();
    switch (mode) {
      case RADIO_RST:
        // put radio to sleep
        opmode(OPMODE_SLEEP);
        // power-down TCXO
        hal_pin_tcxo(0);
        break;

      case RADIO_TX:
        if ((readReg(RegOpMode) & OPMODE_MASK) != OPMODE_SLEEP) opmode(OPMODE_SLEEP);
        // transmit frame now
        starttx(); // buf=LMIC.frame, len=LMIC.dataLen
        break;

      case RADIO_RX:
        if ((readReg(RegOpMode) & OPMODE_MASK) != OPMODE_SLEEP) opmode(OPMODE_SLEEP);
        // receive frame now (exactly at rxtime)
        startrx(RXMODE_SINGLE); // buf=LMIC.frame, time=LMIC.rxtime, timeout=LMIC.rxsyms
        break;

      case RADIO_RXON:
        // start scanning for beacon now
        startrx(RXMODE_SCAN); // buf=LMIC.frame
        break;
    }
    hal_enableIRQs();
}
