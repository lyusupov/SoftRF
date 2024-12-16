/*
 * almic.cpp
 * Copyright (C) 2017-2025 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#if defined(ARDUINO)
#include <SPI.h>
#endif /* ARDUINO */

#include "../RF.h"

#if !defined(EXCLUDE_SX12XX)
/*
 * Arduino-LMIC/BASICMAC
 * SX12XX-specific code
 */

#include "../EEPROM.h"
#include "../Battery.h"

#ifndef RadioSPI
#define RadioSPI        SPI
#endif

extern size_t RF_tx_size;

static bool sx1276_probe(void);
static bool sx1262_probe(void);
static void sx12xx_setup(void);
static void sx12xx_channel(int8_t);
static bool sx12xx_receive(void);
static bool sx12xx_transmit(void);
static void sx1276_shutdown(void);
static void sx1262_shutdown(void);

const rfchip_ops_t sx1276_ops = {
  RF_IC_SX1276,
  "SX127x",
  sx1276_probe,
  sx12xx_setup,
  sx12xx_channel,
  sx12xx_receive,
  sx12xx_transmit,
  sx1276_shutdown
};

#if defined(USE_BASICMAC)
const rfchip_ops_t sx1262_ops = {
  RF_IC_SX1262,
  "SX126x",
  sx1262_probe,
  sx12xx_setup,
  sx12xx_channel,
  sx12xx_receive,
  sx12xx_transmit,
  sx1262_shutdown
};
#endif /* USE_BASICMAC */

osjob_t sx12xx_txjob;
osjob_t sx12xx_timeoutjob;

static void sx12xx_tx_func (osjob_t* job);
static void sx12xx_rx_func (osjob_t* job);
static void sx12xx_rx(osjobcb_t func);

static bool sx12xx_receive_complete = false;
bool sx12xx_receive_active = false;
static bool sx12xx_transmit_complete = false;

static int8_t sx12xx_channel_prev = (int8_t) -1;

#if defined(USE_BASICMAC)
void os_getDevEui (u1_t* buf) { }
u1_t os_getRegion (void) { return REGCODE_EU868; }
#else
#if !defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be set. Update \
       config.h in the lmic library to set it.
#endif
#endif

#define SX1276_RegVersion          0x42 // common

static u1_t sx1276_readReg (u1_t addr) {
#if defined(USE_BASICMAC)
    hal_spi_select(1);
#else
    hal_pin_nss(0);
#endif
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
#if defined(USE_BASICMAC)
    hal_spi_select(0);
#else
    hal_pin_nss(1);
#endif
    return val;
}

static bool sx1276_probe()
{
  u1_t v, v_reset;

  SoC->SPI_begin();

  lmic_hal_init (nullptr);

  // manually reset radio
  hal_pin_rst(0); // drive RST pin low
  hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us

  v_reset = sx1276_readReg(SX1276_RegVersion);

  hal_pin_rst(2); // configure RST pin floating!
  hal_waitUntil(os_getTime()+ms2osticks(5)); // wait 5ms

  v = sx1276_readReg(SX1276_RegVersion);

  pinMode(lmic_pins.nss, INPUT);
  RadioSPI.end();

  if (v == 0x12 || v == 0x13) {

    if (v_reset == 0x12 || v_reset == 0x13) {
      RF_SX12XX_RST_is_connected = false;
    }

    return true;
  } else {
    return false;
  }
}

#if defined(USE_BASICMAC)

#define CMD_READREGISTER            0x1D
#define REG_LORASYNCWORDLSB         0x0741
#define SX126X_DEF_LORASYNCWORDLSB  0x24

static void sx1262_ReadRegs (uint16_t addr, uint8_t* data, uint8_t len) {
    hal_spi_select(1);
    hal_pin_busy_wait();
    hal_spi(CMD_READREGISTER);
    hal_spi(addr >> 8);
    hal_spi(addr);
    hal_spi(0x00); // NOP
    for (uint8_t i = 0; i < len; i++) {
        data[i] = hal_spi(0x00);
    }
    hal_spi_select(0);
}

static uint8_t sx1262_ReadReg (uint16_t addr) {
    uint8_t val;
    sx1262_ReadRegs(addr, &val, 1);
    return val;
}

static bool sx1262_probe()
{
  u1_t v, v_reset;

  SoC->SPI_begin();

  lmic_hal_init (nullptr);

  // manually reset radio
  hal_pin_rst(0); // drive RST pin low
  hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us

  v_reset = sx1262_ReadReg(REG_LORASYNCWORDLSB);

  hal_pin_rst(2); // configure RST pin floating!
  hal_waitUntil(os_getTime()+ms2osticks(5)); // wait 5ms

  v = sx1262_ReadReg(REG_LORASYNCWORDLSB);

  pinMode(lmic_pins.nss, INPUT);
  RadioSPI.end();

  u1_t fanet_sw_lsb = ((fanet_proto_desc.syncword[0]  & 0x0F) << 4) | 0x04;
  if (v == SX126X_DEF_LORASYNCWORDLSB || v == fanet_sw_lsb) {

    if (v_reset == SX126X_DEF_LORASYNCWORDLSB || v == fanet_sw_lsb) {
      RF_SX12XX_RST_is_connected = false;
    }

    return true;
  } else {
    return false;
  }
}
#endif

static void sx12xx_channel(int8_t channel)
{
  if (channel != -1 && channel != sx12xx_channel_prev) {
    uint32_t frequency = RF_FreqPlan.getChanFrequency((uint8_t) channel);
    int8_t fc = settings->freq_corr;

    if (LMIC.protocol->type == RF_PROTOCOL_LEGACY) {
      nRF905_band_t nrf_band;
      uint32_t nrf_freq_resolution;

      nrf_band = (frequency >= 844800000UL ? NRF905_BAND_868 : NRF905_BAND_433);
      nrf_freq_resolution = (nrf_band == NRF905_BAND_433 ? 100000UL : 200000UL);
      frequency -= (frequency % nrf_freq_resolution);
    }

    //Serial.print("frequency: "); Serial.println(frequency);

    if (sx12xx_receive_active) {
      os_radio(RADIO_RST);
      sx12xx_receive_active = false;
    }

    if (rf_chip->type == RF_IC_SX1276) {
      /* correction of not more than 30 kHz is allowed */
      if (fc > 30) {
        fc = 30;
      } else if (fc < -30) {
        fc = -30;
      };
    } else {
      /* Most of SX1262 designs use TCXO */
      fc = 0;
    }

#if defined(ENABLE_PROL)
    if (LMIC.protocol->type == RF_PROTOCOL_APRS) {
      frequency = 433775000UL;
    }
#endif /* ENABLE_PROL */

    /* Actual RF chip's channel registers will be updated before each Tx or Rx session */
    LMIC.freq = frequency + (fc * 1000);
    //LMIC.freq = 868200000UL;

    sx12xx_channel_prev = channel;
  }
}

static void sx12xx_setup()
{
  SoC->SPI_begin();

  // initialize runtime env
  os_init (nullptr);

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  switch (settings->rf_protocol)
  {
  case RF_PROTOCOL_OGNTP:
    LMIC.protocol   = &ogntp_proto_desc;
    protocol_encode = &ogntp_encode;
    protocol_decode = &ogntp_decode;
    break;
  case RF_PROTOCOL_P3I:
    LMIC.protocol   = &p3i_proto_desc;
    protocol_encode = &p3i_encode;
    protocol_decode = &p3i_decode;
    break;
  case RF_PROTOCOL_FANET:
    LMIC.protocol   = &fanet_proto_desc;
    protocol_encode = &fanet_encode;
    protocol_decode = &fanet_decode;
    break;
#if defined(ENABLE_PROL)
  case RF_PROTOCOL_APRS:
    LMIC.protocol   = &prol_proto_desc;
    protocol_encode = &aprs_encode;
    protocol_decode = &aprs_decode;
    break;
#endif /* ENABLE_PROL */
#if defined(ENABLE_ADSL)
  case RF_PROTOCOL_ADSL_860:
    LMIC.protocol   = &adsl_proto_desc;
    protocol_encode = &adsl_encode;
    protocol_decode = &adsl_decode;
    break;
#endif /* ENABLE_ADSL */
  case RF_PROTOCOL_LEGACY:
  default:
    LMIC.protocol   = &legacy_proto_desc;
    protocol_encode = &legacy_encode;
    protocol_decode = &legacy_decode;
    /*
     * Enforce legacy protocol setting for SX1276
     * if other value (UAT) left in EEPROM from other (UATM) radio
     */
    settings->rf_protocol = RF_PROTOCOL_LEGACY;
    break;
  }

  RF_FreqPlan.setPlan(settings->band, settings->rf_protocol);

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    /* Load regional max. EIRP at first */
    LMIC.txpow = RF_FreqPlan.MaxTxPower;

    if (rf_chip->type == RF_IC_SX1262) {
      /* SX1262 is unable to give more than 22 dBm */
      if (LMIC.txpow > 22)
        LMIC.txpow = 22;
    } else {
      /* SX1276 is unable to give more than 20 dBm */
      if (LMIC.txpow > 20)
        LMIC.txpow = 20;
    }

#if 1
    /*
     * Enforce Tx power limit until confirmation
     * that RFM95W is doing well
     * when antenna is not connected
     */
    if (LMIC.txpow > 17)
      LMIC.txpow = 17;
#endif

    /* Enforce Tx power limit for a limb-worn (handheld or wristband) device */
#if 0
    if (hw_info.model == SOFTRF_MODEL_BADGE ||
        hw_info.model == SOFTRF_MODEL_BRACELET) {
      if (settings->band == RF_BAND_US &&
          LMIC.txpow > 13) {
        LMIC.txpow = 13; /* d = 0.5 cm */
      }
    }
#endif
    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    LMIC.txpow = 2; /* 2 dBm is minimum for RFM95W on PA_BOOST pin */
    break;
  }
}

static void sx12xx_setvars()
{
  if (LMIC.protocol && LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA) {
    LMIC.datarate = LMIC.protocol->bitrate;
    LMIC.syncword = LMIC.protocol->syncword[0];
  } else {
    LMIC.datarate = DR_FSK;
  }

#if defined(USE_BASICMAC)

#define updr2rps  LMIC_updr2rps

  // LMIC.rps = MAKERPS(sf, BW250, CR_4_5, 0, 0);

  LMIC.noRXIQinversion = true;
  LMIC.rxsyms = 100;

#endif /* USE_BASICMAC */

  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  if (LMIC.protocol && LMIC.protocol->type == RF_PROTOCOL_FANET) {
    /* for only a few nodes around, increase the coding rate to ensure a more robust transmission */
    LMIC.rps = setCr(LMIC.rps, CR_4_8);

    if (RF_FreqPlan.Bandwidth == RF_RX_BANDWIDTH_SS_250KHZ) {
      LMIC.rps = setBw(LMIC.rps, BW500);
    } else if (RF_FreqPlan.Bandwidth == RF_RX_BANDWIDTH_SS_62KHZ) {
      LMIC.rps = setBw(LMIC.rps, BW125);
    }

#if defined(ENABLE_PROL)
  } else if (LMIC.protocol && LMIC.protocol->type == RF_PROTOCOL_APRS) {
    LMIC.rps = setCr(LMIC.rps, CR_4_5);
    LMIC.rps = setBw(LMIC.rps, BW125);
#endif /* ENABLE_PROL */
  }
}

static bool sx12xx_receive()
{
  bool success = false;

  sx12xx_receive_complete = false;

  if (!sx12xx_receive_active) {
    if (settings->power_save & POWER_SAVE_NORECEIVE) {
      LMIC_shutdown();
    } else {
      sx12xx_setvars();
      sx12xx_rx(sx12xx_rx_func);
    }
    sx12xx_receive_active = true;
  }

  if (sx12xx_receive_complete == false) {
    // execute scheduled jobs and events
    os_runstep();
  };

  if (sx12xx_receive_complete == true) {

    u1_t size = LMIC.dataLen - LMIC.protocol->payload_offset - LMIC.protocol->crc_size;

    if (size > sizeof(RxBuffer)) {
      size = sizeof(RxBuffer);
    }

#if defined(ENABLE_PROL)
    if (settings->rf_protocol == RF_PROTOCOL_APRS) {
      /*
       * APRS-over-LoRa header
       */
      if (LMIC.frame[0] == '<'  &&
          LMIC.frame[1] == 0xFF &&
          LMIC.frame[2] == 0x01) {
        if (size < sizeof(RxBuffer)) {
          RxBuffer[size  ] = 0;
        } else {
          RxBuffer[size-1] = 0;
        }
      } else {
        // Serial.println("RX: invalid PRoL header");
        return success;
      }
    }
#endif /* ENABLE_PROL */

    for (u1_t i=0; i < size; i++) {
        RxBuffer[i] = LMIC.frame[i + LMIC.protocol->payload_offset];
    }

    RF_last_rssi = LMIC.rssi;
    rx_packets_counter++;
    success = true;
  }

  return success;
}

static bool sx12xx_transmit()
{
    bool success = true;

    sx12xx_transmit_complete = false;
    sx12xx_receive_active = false;

    sx12xx_setvars();
    os_setCallback(&sx12xx_txjob, sx12xx_tx_func);

    unsigned long tx_timeout = LMIC.protocol ? (LMIC.protocol->air_time + 25) : 60;

#if defined(USE_SOFTSPI)
    tx_timeout += tx_timeout;
#endif

#if defined(USE_BASICMAC)
    if (rf_chip->type == RF_IC_SX1262 && lmic_pins.busy == LMIC_UNUSED_PIN) {
      tx_timeout += 200;
    }
#endif

    unsigned long tx_start   = millis();

    while (sx12xx_transmit_complete == false) {
      if ((millis() - tx_start) > tx_timeout) {
        os_radio(RADIO_RST);
        success = false;
        //Serial.println("TX timeout");
        break;
      }

      // execute scheduled jobs and events
      os_runstep();

      yield();
    };

  return success;
}

static void sx1276_shutdown()
{
  LMIC_shutdown();

  RadioSPI.end();
}

#if defined(USE_BASICMAC)
static void sx1262_shutdown()
{
  os_init (nullptr);
  sx126x_ll_ops.radio_sleep();
  delay(1);

  RadioSPI.end();
}
#endif /* USE_BASICMAC */

// Enable rx mode and call func when a packet is received
static void sx12xx_rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX for LoRa only (e.g. without a timeout,
  // still stops after receiving a packet)
  os_radio(LMIC.protocol &&
           LMIC.protocol->modulation_type == RF_MODULATION_TYPE_LORA ?
           RADIO_RXON : RADIO_RX);
  //Serial.println("RX");
}

static void sx12xx_rx_func (osjob_t* job) {

  u1_t crc8, pkt_crc8;
  u2_t crc16, pkt_crc16;
  u1_t i;

  // SX1276 is in SLEEP after IRQ handler, Force it to enter RX mode
  sx12xx_receive_active = false;

  /* FANET (LoRa) LMIC IRQ handler may deliver empty packets here when CRC is invalid. */
  if (LMIC.dataLen == 0) {
    return;
  }

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_CRC_MODES:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }

  //Serial.print("Got ");
  //Serial.print(LMIC.dataLen);
  //Serial.println(" bytes");

  switch (LMIC.protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
  case RF_PROTOCOL_OGNTP:
  case RF_PROTOCOL_ADSL_860:
  default:
    break;
  }

  for (i = LMIC.protocol->payload_offset;
       i < (LMIC.dataLen - LMIC.protocol->crc_size);
       i++)
  {

    switch (LMIC.protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_CRC_MODES:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)(LMIC.frame[i]));
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[i]));
      break;
    }

    switch (LMIC.protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      LMIC.frame[i] ^= pgm_read_byte(&whitening_pattern[i - LMIC.protocol->payload_offset]);
      break;
    case RF_WHITENING_MANCHESTER:
    case RF_WHITENING_NONE:
    default:
      break;
    }

#if DEBUG
    Serial.printf("%02x", (u1_t)(LMIC.frame[i]));
#endif
  }

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_NONE:
    sx12xx_receive_complete = true;
    break;
  case RF_CHECKSUM_TYPE_GALLAGER:
    if (LDPC_Check((uint8_t  *) &LMIC.frame[0])) {
#if DEBUG
      Serial.printf(" %02x%02x%02x%02x%02x%02x is wrong FEC",
        LMIC.frame[i], LMIC.frame[i+1], LMIC.frame[i+2],
        LMIC.frame[i+3], LMIC.frame[i+4], LMIC.frame[i+5]);
#endif
      sx12xx_receive_complete = false;
    } else {
      sx12xx_receive_complete = true;
    }
    break;
  case RF_CHECKSUM_TYPE_CRC_MODES:
#if defined(ENABLE_ADSL)
    if (ADSL_Packet::checkPI((uint8_t  *) &LMIC.frame[0], LMIC.dataLen)) {
      sx12xx_receive_complete = false;
    } else
#endif /* ENABLE_ADSL */
    {
      sx12xx_receive_complete = true;
    }
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    pkt_crc8 = LMIC.frame[i];
#if DEBUG
    if (crc8 == pkt_crc8 ) {
      Serial.printf(" %02x is valid crc", pkt_crc8);
    } else {
      Serial.printf(" %02x is wrong crc", pkt_crc8);
    }
#endif
    if (crc8 == pkt_crc8) {
      sx12xx_receive_complete = true;
    } else {
      sx12xx_receive_complete = false;
    }
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    pkt_crc16 = (LMIC.frame[i] << 8 | LMIC.frame[i+1]);
#if DEBUG
    if (crc16 == pkt_crc16 ) {
      Serial.printf(" %04x is valid crc", pkt_crc16);
    } else {
      Serial.printf(" %04x is wrong crc", pkt_crc16);
    }
#endif
    if (crc16 == pkt_crc16) {
      sx12xx_receive_complete = true;
    } else {
      sx12xx_receive_complete = false;
    }
    break;
  }

#if DEBUG
  Serial.println();
#endif

}

// Transmit the given string and call the given function afterwards
static void sx12xx_tx(unsigned char *buf, size_t size, osjobcb_t func) {

  u1_t crc8;
  u2_t crc16;

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_CRC_MODES:
  case RF_CHECKSUM_TYPE_NONE:
     /* crc16 left not initialized */
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    crc8 = 0x71;     /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_0000:
    crc16 = 0x0000;  /* seed value */
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  default:
    crc16 = 0xffff;  /* seed value */
    break;
  }
  
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet

  LMIC.dataLen = 0;

  switch (LMIC.protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
    /* insert Net ID */
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 24) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >> 16) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  8) & 0x000000FF);
    LMIC.frame[LMIC.dataLen++] = (u1_t) ((LMIC.protocol->net_id >>  0) & 0x000000FF);
    /* insert byte with payload size */
    LMIC.frame[LMIC.dataLen++] = LMIC.protocol->payload_size;

    /* insert byte with CRC-8 seed value when necessary */
    if (LMIC.protocol->crc_type == RF_CHECKSUM_TYPE_CRC8_107) {
      LMIC.frame[LMIC.dataLen++] = crc8;
    }

    break;
#if 1
  case RF_PROTOCOL_APRS:
    /*
     * APRS-over-LoRa header
     */
    LMIC.frame[LMIC.dataLen++] = (u1_t) '<';
    LMIC.frame[LMIC.dataLen++] = (u1_t) 0xFF;
    LMIC.frame[LMIC.dataLen++] = (u1_t) 0x01;
    break;
#endif
  case RF_PROTOCOL_OGNTP:
  case RF_PROTOCOL_ADSL_860:
  default:
    break;
  }

  for (u1_t i=0; i < size; i++) {

    switch (LMIC.protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      LMIC.frame[LMIC.dataLen] = buf[i] ^ pgm_read_byte(&whitening_pattern[i]);
      break;
    case RF_WHITENING_MANCHESTER:
    case RF_WHITENING_NONE:
    default:
      LMIC.frame[LMIC.dataLen] = buf[i];
      break;
    }

    switch (LMIC.protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_CRC_MODES:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)(LMIC.frame[LMIC.dataLen]));
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      crc16 = update_crc_ccitt(crc16, (u1_t)(LMIC.frame[LMIC.dataLen]));
      break;
    }

    LMIC.dataLen++;
  }

  switch (LMIC.protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_CRC_MODES:
  case RF_CHECKSUM_TYPE_NONE:
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    LMIC.frame[LMIC.dataLen++] = crc8;
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    LMIC.frame[LMIC.dataLen++] = (crc16 >>  8) & 0xFF;
    LMIC.frame[LMIC.dataLen++] = (crc16      ) & 0xFF;
    break;
  }

  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
  //Serial.println("TX");
}

static void sx12xx_txdone_func (osjob_t* job) {
  sx12xx_transmit_complete = true;
}

static void sx12xx_tx_func (osjob_t* job) {

  if (RF_tx_size > 0) {
    sx12xx_tx((unsigned char *) &TxBuffer[0], RF_tx_size, sx12xx_txdone_func);
  }
}
#endif /* EXCLUDE_SX12XX */
