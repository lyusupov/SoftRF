/*
 * radiolib.cpp
 * Copyright (C) 2024-2025 Linar Yusupov
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

static bool lr1110_probe(void);
static bool lr1121_probe(void);
static void lr11xx_setup(void);
static void lr11xx_channel(int8_t);
static bool lr11xx_receive(void);
static bool lr11xx_transmit(void);
static void lr11xx_shutdown(void);

const rfchip_ops_t lr1110_ops = {
  RF_IC_LR1110,
  "LR1110",
  lr1110_probe,
  lr11xx_setup,
  lr11xx_channel,
  lr11xx_receive,
  lr11xx_transmit,
  lr11xx_shutdown
};

const rfchip_ops_t lr1121_ops = {
  RF_IC_LR1121,
  "LR1121",
  lr1121_probe,
  lr11xx_setup,
  lr11xx_channel,
  lr11xx_receive,
  lr11xx_transmit,
  lr11xx_shutdown
};

#define USE_SX1262      0
#define USE_LR11XX      1

#ifndef RadioSPI
#define RadioSPI        SPI
#endif

Module  *mod;
#if USE_SX1262
SX1262  *radio;
#elif USE_LR11XX
LR11x0  *radio;
#endif

const rf_proto_desc_t  *rl_protocol  = &ogntp_proto_desc;

static int8_t lr112x_channel_prev    = (int8_t) -1;

static volatile bool lr112x_receive_complete = false;

static bool lr112x_receive_active    = false;
static bool lr112x_transmit_complete = false;

#define RADIOLIB_MAX_DATA_LENGTH    128

typedef struct
{
  uint8_t len;
  uint8_t payload[RADIOLIB_MAX_DATA_LENGTH];
} RadioLib_DataPacket;

RadioLib_DataPacket txPacket;
RadioLib_DataPacket rxPacket;

#if !defined(USE_BASICMAC)
static const SPISettings probe_settings(1000000UL, MSBFIRST, SPI_MODE0);

static void hal_spi_select (int on) {

#if defined(SPI_HAS_TRANSACTION)
    if (on)
        RadioSPI.beginTransaction(probe_settings);
    else
        RadioSPI.endTransaction();
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
#endif /* USE_BASICMAC */

#if USE_SX1262

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
#endif /* USE_SX1262 */

#if USE_LR11XX
static void lr11xx_GetVersion (uint8_t* hw, uint8_t* device,
                               uint8_t* major, uint8_t* minor) {
    uint8_t buf[4] = { 0 };

    hal_pin_busy_wait();
    hal_spi_select(1);

    hal_spi((uint8_t)((RADIOLIB_LR11X0_CMD_GET_VERSION & 0xFF00) >> 8));
    hal_spi((uint8_t) (RADIOLIB_LR11X0_CMD_GET_VERSION & 0x00FF));
    hal_spi_select(0);

    hal_pin_busy_wait();
    hal_spi_select(1);

    hal_spi(RADIOLIB_LR11X0_CMD_NOP);
    for (uint8_t i = 0; i < sizeof(buf); i++) {
        buf[i] = hal_spi(0x00);
    }
    hal_spi_select(0);

    if (hw)     { *hw     = buf[0]; }
    if (device) { *device = buf[1]; }
    if (major)  { *major  = buf[2]; }
    if (minor)  { *minor  = buf[3]; }
#if 0
    Serial.print("hw     = "); Serial.println(*hw, HEX);
    Serial.print("device = "); Serial.println(*device, HEX);
    Serial.print("major  = "); Serial.println(*major, HEX);
    Serial.print("minor  = "); Serial.println(*minor, HEX);
#endif
}

static const uint32_t rfswitch_dio_pins_hpdtek[] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
    RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table_hpdtek[] = {
    // mode                  DIO5  DIO6
    { LR11x0::MODE_STBY,   { LOW,  LOW  } },
    { LR11x0::MODE_RX,     { HIGH, LOW  } },
    { LR11x0::MODE_TX,     { LOW,  HIGH } },
    { LR11x0::MODE_TX_HP,  { LOW,  HIGH } },
    { LR11x0::MODE_TX_HF,  { LOW,  LOW  } },
    { LR11x0::MODE_GNSS,   { LOW,  LOW  } },
    { LR11x0::MODE_WIFI,   { LOW,  LOW  } },
    END_OF_MODE_TABLE,
};

static const uint32_t rfswitch_dio_pins_seeed[] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
    RADIOLIB_LR11X0_DIO7, RADIOLIB_LR11X0_DIO8,
    RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table_seeed[] = {
    // mode                  DIO5  DIO6  DIO7  DIO8
    { LR11x0::MODE_STBY,   { LOW,  LOW,  LOW,  LOW  } },
    // SKY13373
    { LR11x0::MODE_RX,     { HIGH, LOW,  LOW,  HIGH } },
    { LR11x0::MODE_TX,     { HIGH, HIGH, LOW,  HIGH } },
    { LR11x0::MODE_TX_HP,  { LOW,  HIGH, LOW,  HIGH } },
    { LR11x0::MODE_TX_HF,  { LOW,  LOW,  LOW,  LOW  } },
    // BGA524N6
    { LR11x0::MODE_GNSS,   { LOW,  LOW,  HIGH, LOW  } },
    // LC
    { LR11x0::MODE_WIFI,   { LOW,  LOW,  LOW,  LOW  } },
    END_OF_MODE_TABLE,
};

static const uint32_t rfswitch_dio_pins_ebyte[] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6, RADIOLIB_LR11X0_DIO7,
    RADIOLIB_NC, RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table_ebyte[] = {
    // mode                  DIO5  DIO6  DIO7
    { LR11x0::MODE_STBY,   { LOW,  LOW,  LOW  } },
    { LR11x0::MODE_RX,     { LOW,  HIGH, LOW  } },
    { LR11x0::MODE_TX,     { HIGH, HIGH, LOW  } },
    { LR11x0::MODE_TX_HP,  { HIGH, LOW,  LOW  } },
    { LR11x0::MODE_TX_HF,  { LOW,  LOW,  LOW  } },
    { LR11x0::MODE_GNSS,   { LOW,  LOW,  HIGH } },
    { LR11x0::MODE_WIFI,   { LOW,  LOW,  LOW  } },
    END_OF_MODE_TABLE,
};
#endif /* USE_LR11XX */

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

static bool lr1110_probe()
{
#if USE_SX1262
  return false;
#endif
#if USE_LR11XX
  u1_t device, device_reset;
  u1_t hw, major, minor;

  SoC->SPI_begin();

  lmic_hal_init (nullptr);

  // manually reset radio
  hal_pin_rst(0); // drive RST pin low
  hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us

  lr11xx_GetVersion(&hw, &device_reset, &major, &minor);

  hal_pin_rst(2); // configure RST pin floating!
  hal_waitUntil(os_getTime()+ms2osticks(300)); // wait 300 ms

  lr11xx_GetVersion(&hw, &device, &major, &minor);

  pinMode(lmic_pins.nss, INPUT);
  RadioSPI.end();

  if (device == RADIOLIB_LR11X0_DEVICE_LR1110) {

    if (device_reset == RADIOLIB_LR11X0_DEVICE_LR1110) {
      RF_SX12XX_RST_is_connected = false;
    }
#if 0
    char buf[8];
    snprintf(buf, sizeof(buf), "%d.%d", major, minor);
    Serial.print("INFO: LR1110 base FW version ");
    Serial.println(buf);
#endif
    return true;
  } else {
    return false;
  }
#endif
}

static bool lr1121_probe()
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
#endif

#if USE_LR11XX
  u1_t device, device_reset;
  u1_t hw, major, minor;

  SoC->SPI_begin();

  lmic_hal_init (nullptr);

  // manually reset radio
  hal_pin_rst(0); // drive RST pin low
  hal_waitUntil(os_getTime()+ms2osticks(1)); // wait >100us

  lr11xx_GetVersion(&hw, &device_reset, &major, &minor);

  hal_pin_rst(2); // configure RST pin floating!
  hal_waitUntil(os_getTime()+ms2osticks(300)); // wait 300 ms

  lr11xx_GetVersion(&hw, &device, &major, &minor);

  pinMode(lmic_pins.nss, INPUT);
  RadioSPI.end();

  if (device == RADIOLIB_LR11X0_DEVICE_LR1121) {

    if (device_reset == RADIOLIB_LR11X0_DEVICE_LR1121) {
      RF_SX12XX_RST_is_connected = false;
    }
#if 0
    char buf[8];
    snprintf(buf, sizeof(buf), "%d.%d", major, minor);
    Serial.print("INFO: LR1121 base FW version ");
    Serial.println(buf);
#endif
    return true;
  } else {
    return false;
  }
#endif
}

static void lr11xx_channel(int8_t channel)
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

static void lr11xx_setup()
{
  int state;

  SoC->SPI_begin();

  uint32_t irq  = lmic_pins.dio[0] == LMIC_UNUSED_PIN ?
                  RADIOLIB_NC : lmic_pins.dio[0];
  uint32_t busy = lmic_pins.busy == LMIC_UNUSED_PIN ?
                  RADIOLIB_NC : lmic_pins.busy;

  mod   = new Module(lmic_pins.nss, irq, lmic_pins.rst, busy, RadioSPI);
#if USE_SX1262
  radio = new SX1262(mod);
#endif
#if USE_LR11XX
  switch (rf_chip->type)
  {
  case RF_IC_LR1110:
    radio = new LR1110(mod);
    break;
  case RF_IC_LR1121:
  default:
    radio = new LR1121(mod);
    break;
  }
#endif

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

#if USE_LR11XX
  float Vtcxo;

  switch (hw_info.model)
  {
  case SOFTRF_MODEL_STANDALONE:
  case SOFTRF_MODEL_ACADEMY:
    // Ebyte E80-900M2213S
    // LR1121 TCXO Voltage
    Vtcxo = 1.8;
    break;

  case SOFTRF_MODEL_NEO:
  case SOFTRF_MODEL_BADGE:
  case SOFTRF_MODEL_PRIME_MK3:
    // HPDTeK HPD-16E
    // LR1121 TCXO Voltage 2.85~3.15V
    Vtcxo = 3.0;
    break;

  case SOFTRF_MODEL_CARD:
    // Seeed
    // LR1110 TCXO Voltage
  default:
    Vtcxo = 1.6;
    break;
  }
#endif

  float br, fdev, bw;
  switch (rl_protocol->modulation_type)
  {
  case RF_MODULATION_TYPE_LORA:
#if USE_SX1262
    state = radio->begin();    // start LoRa mode (and disable FSK)
#endif
#if USE_LR11XX
#if RADIOLIB_VERSION_MAJOR == 6
    state = radio->begin(125.0, 9, 7, RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, 10, 8, Vtcxo);
#else
    state = radio->begin(125.0, 9, 7, RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, 8, Vtcxo);
#endif /* RADIOLIB_VERSION_MAJOR */
#endif

    switch (RF_FreqPlan.Bandwidth)
    {
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = 125.0; /* BW_125 */
      break;
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
#if USE_LR11XX
#if RADIOLIB_VERSION_MAJOR == 6
    state = radio->beginGFSK(4.8, 5.0, 156.2, 10, 16, Vtcxo);
#else
    state = radio->beginGFSK(4.8, 5.0, 156.2, 16, Vtcxo);
#endif /* RADIOLIB_VERSION_MAJOR */
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
    case RF_RX_BANDWIDTH_SS_62KHZ:
      bw = 156.2;
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
      /* CRC is driven by software */
      state = radio->setCRC(0, 0);
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
    case RF_WHITENING_NONE:
    case RF_WHITENING_NICERF:
    default:
      break;
    }
    state = radio->fixedPacketLengthMode(pkt_size);

    state = radio->disableAddressFiltering();

    /* Work around premature P3I syncword detection */
    if (rl_protocol->syncword_size == 2) {
      uint8_t preamble = rl_protocol->preamble_type == RF_PREAMBLE_TYPE_AA ?
                         0xAA : 0x55;
      uint8_t sword[4] = { preamble,
                           preamble,
                           rl_protocol->syncword[0],
                           rl_protocol->syncword[1]
                         };
      state = radio->setSyncWord(sword, 4);
    } else {
      state = radio->setSyncWord((uint8_t *) rl_protocol->syncword,
                                 (size_t)    rl_protocol->syncword_size);
    }
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
  uint32_t rxe = lmic_pins.rxe == LMIC_UNUSED_PIN ? RADIOLIB_NC : lmic_pins.rxe;
  uint32_t txe = lmic_pins.txe == LMIC_UNUSED_PIN ? RADIOLIB_NC : lmic_pins.txe;
  if (rxe == RADIOLIB_NC && txe == RADIOLIB_NC) {
    state = radio->setDio2AsRfSwitch();
  } else {
    radio->setRfSwitchPins(rxe, txe);
  }

  state = radio->setCurrentLimit(100.0);
#endif

#if USE_LR11XX
  switch (hw_info.model)
  {
  case SOFTRF_MODEL_CARD:
#if 1
    radio->setDioAsRfSwitch(0x0f, 0x0, 0x09, 0x0B, 0x0A, 0x0, 0x4, 0x0);
#else
    radio->setRfSwitchTable(rfswitch_dio_pins_seeed, rfswitch_table_seeed);
#endif
    break;

  case SOFTRF_MODEL_STANDALONE:
  case SOFTRF_MODEL_ACADEMY:
    /* Ebyte E80-900M2213S */
#if 1
    radio->setDioAsRfSwitch(0x07, 0x0, 0x02, 0x03, 0x01, 0x0, 0x4, 0x0);
#else
    radio->setRfSwitchTable(rfswitch_dio_pins_ebyte, rfswitch_table_ebyte);
#endif
    break;

  case SOFTRF_MODEL_NEO:
  case SOFTRF_MODEL_BADGE:
  case SOFTRF_MODEL_PRIME_MK3:
  default:
    radio->setRfSwitchTable(rfswitch_dio_pins_hpdtek, rfswitch_table_hpdtek);
    break;
  }
#endif

  state = radio->setRxBoostedGainMode(true);

  radio->setPacketReceivedAction(lr112x_receive_handler);
}

static bool memeqzero(const uint8_t *data, size_t length)
{
	const uint8_t *p = data;
	size_t len;

	/* Check first 16 bytes manually */
	for (len = 0; len < 16; len++) {
		if (!length)
			return true;
		if (*p)
			return false;
		p++;
		length--;
	}

	/* Now we know that's zero, memcmp with self. */
	return memcmp((void *) data, (void *) p, length) == 0;
}

static bool lr11xx_receive()
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

    rxPacket.len = radio->getPacketLength();

    if (rxPacket.len > 0) {

      if (rxPacket.len > sizeof(rxPacket.payload)) {
        rxPacket.len = sizeof(rxPacket.payload);
      }

      state = radio->readData(rxPacket.payload, rxPacket.len);
      lr112x_receive_active = false;

      if (state == RADIOLIB_ERR_NONE &&
         !memeqzero(rxPacket.payload, rxPacket.len)) {
        size_t size = 0;
        uint8_t offset;

        u1_t crc8, pkt_crc8;
        u2_t crc16, pkt_crc16;

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

        uint8_t i;

        switch (rl_protocol->type)
        {
        case RF_PROTOCOL_P3I:
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
        case RF_PROTOCOL_FANET:
          offset = rl_protocol->payload_offset;
          size   = rl_protocol->payload_size + rl_protocol->crc_size;
          for (i = 0; i < size; i++)
          {
            if (i < sizeof(RxBuffer)) {
              RxBuffer[i] = rxPacket_ptr->payload[i + offset];
            }
          }
          success = true;
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
            uint8_t val1, val2;
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
          RF_last_rssi = radio->getRSSI();
          rx_packets_counter++;
        }
      }

      memset(rxPacket.payload, 0, sizeof(rxPacket.payload));
#if USE_SX1262 && (RADIOLIB_GODMODE || RADIOLIB_LOW_LEVEL)
      radio->writeBuffer(rxPacket.payload, rxPacket.len);
      radio->setBufferBaseAddress();
#endif
      rxPacket.len = 0;
    }

    lr112x_receive_complete = false;
  }

  return success;
}

static bool lr11xx_transmit()
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

    memset(txPacket.payload, 0, sizeof(txPacket.payload));
#if USE_SX1262 && (RADIOLIB_GODMODE || RADIOLIB_LOW_LEVEL)
    radio->setBufferBaseAddress();
    radio->writeBuffer(txPacket.payload, txPacket.len);
    radio->setBufferBaseAddress();
#endif

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

static void lr11xx_shutdown()
{
#if USE_SX1262
  int state = radio->sleep(false);
#endif

#if USE_LR11XX
  int state = radio->standby(RADIOLIB_LR11X0_STANDBY_RC);
  state = radio->setTCXO(0);
  state = radio->sleep(false, 0);
#endif

  RadioSPI.end();
}

#endif /* USE_RADIOLIB */
