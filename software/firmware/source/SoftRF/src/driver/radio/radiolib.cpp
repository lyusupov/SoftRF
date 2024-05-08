/*
 * radiolib.cpp
 * Copyright (C) 2024 Linar Yusupov
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

#include "../RF.h"

#if defined(USE_RADIOLIB)

#include "../EEPROM.h"
#include "../Battery.h"

#include <manchester.h>

extern size_t RF_tx_size;

static bool lr112x_probe(void);
static void lr112x_setup(void);
static void lr112x_channel(int8_t);
static bool lr112x_receive(void);
static bool lr112x_transmit(void);
static void lr112x_shutdown(void);

const rfchip_ops_t lr112x_ops = {
  RF_IC_LR112X,
  "LR112x",
  lr112x_probe,
  lr112x_setup,
  lr112x_channel,
  lr112x_receive,
  lr112x_transmit,
  lr112x_shutdown
};

#define USE_SX1262      1
#define USE_LR1121      0

#if USE_SX1262
#define RADIO_TYPE      SX1262
#elif USE_LR1121
#define RADIO_TYPE      LR1121
#endif

Module     *mod;
RADIO_TYPE *radio;

const rf_proto_desc_t  *rl_protocol = &ogntp_proto_desc;

static int8_t lr112x_channel_prev    = (int8_t) -1;

static volatile bool lr112x_receive_complete = false;

static bool lr112x_receive_active    = false;
static bool lr112x_transmit_complete = false;

#if USE_SX1262 && !defined(USE_BASICMAC)

#define CMD_READREGISTER            0x1D
#define REG_LORASYNCWORDLSB         0x0741
#define SX126X_DEF_LORASYNCWORDLSB  0x24

#define RADIOLIB_MAX_DATA_LENGTH    128

typedef struct
{
  uint8_t len;
  uint8_t payload[RADIOLIB_MAX_DATA_LENGTH];
} RadioLib_DataPacket;

RadioLib_DataPacket txPacket;
RadioLib_DataPacket rxPacket;

static const SPISettings probe_settings(1000000UL, MSBFIRST, SPI_MODE0);

static void hal_spi_select (int on) {

#if defined(SPI_HAS_TRANSACTION)
    if (on)
        SPI.beginTransaction(probe_settings);
    else
        SPI.endTransaction();
#endif

    //Serial.println(val?">>":"<<");
    digitalWrite(lmic_pins.nss, !on ? HIGH : LOW);
}

// Datasheet defins typical times until busy goes low. Most are < 200us,
// except when waking up from sleep, which typically takes 3500us. Since
// we cannot know here if we are in sleep, we'll have to assume we are.
// Since 3500 is typical, not maximum, wait a bit more than that.
static unsigned long MAX_BUSY_TIME = 5000;

static void hal_pin_busy_wait (void) {
    if (lmic_pins.busy == LMIC_UNUSED_PIN) {
        // TODO: We could probably keep some state so we know the chip
        // is in sleep, since otherwise the delay can be much shorter.
        // Also, all delays after commands (rather than waking up from
        // sleep) are measured from the *end* of the previous SPI
        // transaction, so we could wait shorter if we remember when
        // that was.
        delayMicroseconds(MAX_BUSY_TIME);
    } else {
        unsigned long start = micros();

        while((micros() - start) < MAX_BUSY_TIME && digitalRead(lmic_pins.busy)) /* wait */;
    }
}

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
#endif

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void lr112x_receive_handler(void) {
  lr112x_receive_complete = true;
}

static bool lr112x_probe()
{
#if USE_SX1262
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
  SPI.end();

  u1_t fanet_sw_lsb = ((fanet_proto_desc.syncword[0]  & 0x0F) << 4) | 0x04;
  if (v == SX126X_DEF_LORASYNCWORDLSB || v == fanet_sw_lsb) {

    if (v_reset == SX126X_DEF_LORASYNCWORDLSB || v == fanet_sw_lsb) {
      RF_SX12XX_RST_is_connected = false;
    }

    return true;
  } else {
    return false;
  }
#endif

#if USE_LR1121
  bool success = false;

  /* TBD */

  return success;
#endif
}

static void lr112x_channel(int8_t channel)
{
  if (channel != -1 && channel != lr112x_channel_prev) {
    uint32_t frequency = RF_FreqPlan.getChanFrequency((uint8_t) channel);

    if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {
      nRF905_band_t nrf_band;
      uint32_t nrf_freq_resolution;

      nrf_band = (frequency >= 844800000UL ? NRF905_BAND_868 : NRF905_BAND_433);
      nrf_freq_resolution = (nrf_band == NRF905_BAND_433 ? 100000UL : 200000UL);
      frequency -= (frequency % nrf_freq_resolution);
    }

    int state = radio->setFrequency(frequency / 1000000.0);

    lr112x_channel_prev = channel;
    /* restart Rx upon a channel switch */
    lr112x_receive_active = false;
  }
}

static void lr112x_setup()
{
  int state;

  SoC->SPI_begin();

  mod   = new Module(lmic_pins.nss, lmic_pins.dio[0],
                     lmic_pins.rst, lmic_pins.busy,
                     SPI);
  radio = new RADIO_TYPE(mod);

  switch (settings->rf_protocol)
  {
  case RF_PROTOCOL_OGNTP:
    rl_protocol     = &ogntp_proto_desc;
    protocol_encode = &ogntp_encode;
    protocol_decode = &ogntp_decode;
    break;
  case RF_PROTOCOL_P3I:
    rl_protocol     = &p3i_proto_desc;
    protocol_encode = &p3i_encode;
    protocol_decode = &p3i_decode;
    break;
  case RF_PROTOCOL_FANET:
    rl_protocol     = &fanet_proto_desc;
    protocol_encode = &fanet_encode;
    protocol_decode = &fanet_decode;
    break;
#if defined(ENABLE_PROL)
  case RF_PROTOCOL_APRS:
    rl_protocol     = &prol_proto_desc;
    protocol_encode = &aprs_encode;
    protocol_decode = &aprs_decode;
    break;
#endif /* ENABLE_PROL */
#if defined(ENABLE_ADSL)
  case RF_PROTOCOL_ADSL_860:
    rl_protocol     = &adsl_proto_desc;
    protocol_encode = &adsl_encode;
    protocol_decode = &adsl_decode;
    break;
#endif /* ENABLE_ADSL */
  case RF_PROTOCOL_LEGACY:
  default:
    rl_protocol     = &legacy_proto_desc;
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

  float br, fdev, bw;
  switch (rl_protocol->modulation_type)
  {
  case RF_MODULATION_TYPE_LORA:
    state = radio->begin();    // start LoRa mode (and disable FSK)

    switch (RF_FreqPlan.Bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_250KHZ:
      bw = 500.0; /* BW_500 */
      break;
    case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      bw = 250.0; /* BW_250 */
      break;
    }
    state = radio->setBandwidth(bw);

    switch (rl_protocol->type)
    {
    case RF_PROTOCOL_FANET:
    default:
      state = radio->setSpreadingFactor(7); /* SF_7 */
      state = radio->setCodingRate(5);      /* CR_5 */
      break;
    }

    state = radio->setSyncWord((uint8_t) rl_protocol->syncword[0]);

    state = radio->setPreambleLength(8);
    state = radio->explicitHeader();
    state = radio->setCRC(true);

    break;
  case RF_MODULATION_TYPE_2FSK:
  case RF_MODULATION_TYPE_PPM: /* TBD */
  default:

#if USE_SX1262
    state = radio->beginFSK(); // start FSK mode (and disable LoRa)
#endif
#if USE_LR1121
    state = radio->beginGFSK();
#endif

    switch (rl_protocol->bitrate)
    {
    case RF_BITRATE_38400:
      br = 38.4;
      break;
    case RF_BITRATE_100KBPS:
    default:
      br = 100.0;
      break;
    }
    state = radio->setBitRate(br);

    switch (rl_protocol->deviation)
    {
    case RF_FREQUENCY_DEVIATION_9_6KHZ:
      fdev = 9.6;
      break;
    case RF_FREQUENCY_DEVIATION_19_2KHZ:
      fdev = 19.2;
      break;
    case RF_FREQUENCY_DEVIATION_25KHZ:
      fdev = 25.0;
      break;
    case RF_FREQUENCY_DEVIATION_50KHZ:
    case RF_FREQUENCY_DEVIATION_NONE:
    default:
      fdev = 50.0;
      break;
    }
    state = radio->setFrequencyDeviation(fdev);

    switch (rl_protocol->bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_50KHZ:
      bw = 117.3;
      break;
    case RF_RX_BANDWIDTH_SS_100KHZ:
      bw = 234.3;
      break;
    case RF_RX_BANDWIDTH_SS_166KHZ:
      bw = 312.0;
      break;
    case RF_RX_BANDWIDTH_SS_200KHZ:
    case RF_RX_BANDWIDTH_SS_250KHZ:
    case RF_RX_BANDWIDTH_SS_1567KHZ:
      bw = 467.0;
      break;
    case RF_RX_BANDWIDTH_SS_125KHZ:
    default:
      bw = 234.3;
      break;
    }
    state = radio->setRxBandwidth(bw);

    state = radio->setEncoding(RADIOLIB_ENCODING_NRZ);
    state = radio->setPreambleLength(rl_protocol->preamble_size * 8);
    state = radio->setDataShaping(RADIOLIB_SHAPING_0_5);

    switch (rl_protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    case RF_CHECKSUM_TYPE_CCITT_1D02:
    case RF_CHECKSUM_TYPE_CRC8_107:
    case RF_CHECKSUM_TYPE_RS:
      /* TBD */
      break;
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_CRC_MODES:
    case RF_CHECKSUM_TYPE_NONE:
    default:
      state = radio->setCRC(0, 0);
      break;
    }

    size_t pkt_size = rl_protocol->payload_offset + rl_protocol->payload_size +
                      rl_protocol->crc_size;

    switch (rl_protocol->whitening)
    {
    case RF_WHITENING_MANCHESTER:
      pkt_size += pkt_size;
      break;
    case RF_WHITENING_PN9:
    case RF_CHECKSUM_TYPE_CRC_MODES:
    case RF_WHITENING_NICERF:
    default:
      break;
    }
    state = radio->fixedPacketLengthMode(pkt_size);

    state = radio->disableAddressFiltering();
    state = radio->setSyncWord((uint8_t *) rl_protocol->syncword,
                               (size_t)    rl_protocol->syncword_size);
    break;
  }

  float txpow;

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    /* Load regional max. EIRP at first */
    txpow = RF_FreqPlan.MaxTxPower;

    if (txpow > 22)
      txpow = 22;

#if 1
    /*
     * Enforce Tx power limit until confirmation
     * that LR112x is doing well
     * when antenna is not connected
     */
    if (txpow > 17)
      txpow = 17;
#endif
    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    txpow = 2;
    break;
  }

  state = radio->setOutputPower(txpow);

#if USE_SX1262
  state = radio->setDio2AsRfSwitch();
  state = radio->setCurrentLimit(100.0);
  state = radio->setRxBoostedGainMode(true);
#endif

  radio->setPacketReceivedAction(lr112x_receive_handler);
}

static bool lr112x_receive()
{
  bool success = false;
  int state;

  if (settings->power_save & POWER_SAVE_NORECEIVE) {
    return success;
  }

  if (!lr112x_receive_active) {

    state = radio->startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      lr112x_receive_active = true;
    }
  }

  if (lr112x_receive_complete == true) {

    size_t size = 0;
    uint8_t offset;

    u1_t crc8, pkt_crc8;
    u2_t crc16, pkt_crc16;

    rxPacket.len = radio->getPacketLength();
    state = radio->readData(rxPacket.payload, rxPacket.len);

    RadioLib_DataPacket *rxPacket_ptr = &rxPacket;

    switch (rl_protocol->crc_type)
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

    switch (rl_protocol->type)
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

    switch (rl_protocol->type)
    {
    case RF_PROTOCOL_P3I:
      uint8_t i;
      offset = rl_protocol->payload_offset;
      for (i = 0; i < rl_protocol->payload_size; i++)
      {
        update_crc8(&crc8, (u1_t)(rxPacket_ptr->payload[i + offset]));
        if (i < sizeof(RxBuffer)) {
          RxBuffer[i] = rxPacket_ptr->payload[i + offset] ^
                        pgm_read_byte(&whitening_pattern[i]);
        }
      }

      pkt_crc8 = rxPacket_ptr->payload[i + offset];

      if (crc8 == pkt_crc8) {

        success = true;
      }
      break;
    case RF_PROTOCOL_OGNTP:
    case RF_PROTOCOL_ADSL_860:
    case RF_PROTOCOL_LEGACY:
    default:
      offset = 0;
      size   = rl_protocol->payload_offset +
               rl_protocol->payload_size +
               rl_protocol->payload_size +
               rl_protocol->crc_size +
               rl_protocol->crc_size;
      if (rxPacket_ptr->len >= (size + offset)) {
        uint8_t i, val1, val2;
        for (i = 0; i < size; i++) {
          val1 = pgm_read_byte(&ManchesterDecode[rxPacket_ptr->payload[i + offset]]);
          i++;
          val2 = pgm_read_byte(&ManchesterDecode[rxPacket_ptr->payload[i + offset]]);
          if ((i>>1) < sizeof(RxBuffer)) {
            RxBuffer[i>>1] = ((val1 & 0x0F) << 4) | (val2 & 0x0F);

            if (i < size - (rl_protocol->crc_size + rl_protocol->crc_size)) {
              switch (rl_protocol->crc_type)
              {
              case RF_CHECKSUM_TYPE_GALLAGER:
              case RF_CHECKSUM_TYPE_CRC_MODES:
              case RF_CHECKSUM_TYPE_NONE:
                break;
              case RF_CHECKSUM_TYPE_CCITT_FFFF:
              case RF_CHECKSUM_TYPE_CCITT_0000:
              default:
                crc16 = update_crc_ccitt(crc16, (u1_t)(RxBuffer[i>>1]));
                break;
              }
            }
          }
        }

        size = size>>1;

        switch (rl_protocol->crc_type)
        {
        case RF_CHECKSUM_TYPE_GALLAGER:
          if (LDPC_Check((uint8_t  *) &RxBuffer[0]) == 0) {
            success = true;
          }
          break;
        case RF_CHECKSUM_TYPE_CRC_MODES:
#if defined(ENABLE_ADSL)
          if (ADSL_Packet::checkPI((uint8_t  *) &RxBuffer[0], size) == 0) {
            success = true;
          }
#endif /* ENABLE_ADSL */
          break;
        case RF_CHECKSUM_TYPE_CCITT_FFFF:
        case RF_CHECKSUM_TYPE_CCITT_0000:
          offset = rl_protocol->payload_offset + rl_protocol->payload_size;
          if (offset + 1 < sizeof(RxBuffer)) {
            pkt_crc16 = (RxBuffer[offset] << 8 | RxBuffer[offset+1]);
            if (crc16 == pkt_crc16) {

              success = true;
            }
          }
          break;
        default:
          break;
        }
      }
      break;
    }

    if (success) {
//      RF_last_rssi = rxPacket_ptr->rssi;
      rx_packets_counter++;
    }

    lr112x_receive_complete = false;
  }

  return success;
}

static bool lr112x_transmit()
{
  u1_t crc8;
  u2_t crc16;
  u1_t i;

  bool success = false;

  if (RF_tx_size <= 0) {
    return success;
  }

  lr112x_receive_active = false;
  lr112x_transmit_complete = false;

  size_t PayloadLen = 0;

  switch (rl_protocol->crc_type)
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

  switch (rl_protocol->type)
  {
  case RF_PROTOCOL_LEGACY:
    /* take in account NRF905/FLARM "address" bytes */
    crc16 = update_crc_ccitt(crc16, 0x31);
    crc16 = update_crc_ccitt(crc16, 0xFA);
    crc16 = update_crc_ccitt(crc16, 0xB6);
    break;
  case RF_PROTOCOL_P3I:
    /* insert Net ID */
    txPacket.payload[PayloadLen++] = (u1_t) ((rl_protocol->net_id >> 24) & 0x000000FF);
    txPacket.payload[PayloadLen++] = (u1_t) ((rl_protocol->net_id >> 16) & 0x000000FF);
    txPacket.payload[PayloadLen++] = (u1_t) ((rl_protocol->net_id >>  8) & 0x000000FF);
    txPacket.payload[PayloadLen++] = (u1_t) ((rl_protocol->net_id >>  0) & 0x000000FF);
    /* insert byte with payload size */
    txPacket.payload[PayloadLen++] = rl_protocol->payload_size;

    /* insert byte with CRC-8 seed value when necessary */
    if (rl_protocol->crc_type == RF_CHECKSUM_TYPE_CRC8_107) {
      txPacket.payload[PayloadLen++] = crc8;
    }

    break;
  case RF_PROTOCOL_OGNTP:
  case RF_PROTOCOL_ADSL_860:
  default:
    break;
  }

  for (i=0; i < RF_tx_size; i++) {

    switch (rl_protocol->whitening)
    {
    case RF_WHITENING_NICERF:
      txPacket.payload[PayloadLen] = TxBuffer[i] ^ pgm_read_byte(&whitening_pattern[i]);
      break;
    case RF_WHITENING_MANCHESTER:
      txPacket.payload[PayloadLen] = pgm_read_byte(&ManchesterEncode[(TxBuffer[i] >> 4) & 0x0F]);
      PayloadLen++;
      txPacket.payload[PayloadLen] = pgm_read_byte(&ManchesterEncode[(TxBuffer[i]     ) & 0x0F]);
      break;
    case RF_WHITENING_NONE:
    default:
      txPacket.payload[PayloadLen] = TxBuffer[i];
      break;
    }

    switch (rl_protocol->crc_type)
    {
    case RF_CHECKSUM_TYPE_GALLAGER:
    case RF_CHECKSUM_TYPE_CRC_MODES:
    case RF_CHECKSUM_TYPE_NONE:
      break;
    case RF_CHECKSUM_TYPE_CRC8_107:
      update_crc8(&crc8, (u1_t)(txPacket.payload[PayloadLen]));
      break;
    case RF_CHECKSUM_TYPE_CCITT_FFFF:
    case RF_CHECKSUM_TYPE_CCITT_0000:
    default:
      if (rl_protocol->whitening == RF_WHITENING_MANCHESTER) {
        crc16 = update_crc_ccitt(crc16, (u1_t)(TxBuffer[i]));
      } else {
        crc16 = update_crc_ccitt(crc16, (u1_t)(txPacket.payload[PayloadLen]));
      }
      break;
    }

    PayloadLen++;
  }

  switch (rl_protocol->crc_type)
  {
  case RF_CHECKSUM_TYPE_GALLAGER:
  case RF_CHECKSUM_TYPE_CRC_MODES:
  case RF_CHECKSUM_TYPE_NONE:
    break;
  case RF_CHECKSUM_TYPE_CRC8_107:
    txPacket.payload[PayloadLen++] = crc8;
    break;
  case RF_CHECKSUM_TYPE_CCITT_FFFF:
  case RF_CHECKSUM_TYPE_CCITT_0000:
  default:
    if (rl_protocol->whitening == RF_WHITENING_MANCHESTER) {
      txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16 >>  8) & 0xFF) >> 4) & 0x0F]);
      txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16 >>  8) & 0xFF)     ) & 0x0F]);
      txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16      ) & 0xFF) >> 4) & 0x0F]);
      txPacket.payload[PayloadLen++] = pgm_read_byte(&ManchesterEncode[(((crc16      ) & 0xFF)     ) & 0x0F]);
      PayloadLen++;
    } else {
      txPacket.payload[PayloadLen++] = (crc16 >>  8) & 0xFF;
      txPacket.payload[PayloadLen++] = (crc16      ) & 0xFF;
    }
    break;
  }

  txPacket.len = PayloadLen;

  int state = radio->transmit((uint8_t *) &txPacket.payload, (size_t) txPacket.len);

  if (state == RADIOLIB_ERR_NONE) {

    success = true;

#if 0
    // the packet was successfully transmitted
    Serial.println(F("success!"));

    // print measured data rate
    Serial.print(F("[SX1262] Datarate:\t"));
    Serial.print(radio->getDataRate());
    Serial.println(F(" bps"));

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));

  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    // timeout occured while transmitting packet
    Serial.println(F("timeout!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
#endif
  }

  return success;
}

static void lr112x_shutdown()
{
  int state = radio->sleep(false);

  SPI.end();
}

#endif /* USE_RADIOLIB */
