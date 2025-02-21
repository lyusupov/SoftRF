/*
 * radiohead.cpp
 * Copyright (C) 2025 Linar Yusupov
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

#if defined(USE_RADIOHEAD) && !defined(USE_RADIOLIB)

#include "../EEPROM.h"
#include "../Battery.h"

#include <manchester.h>

#ifndef RadioSPI
#define RadioSPI        SPI
#endif

const rf_proto_desc_t *rh_protocol = &ogntp_proto_desc;

#if !defined(USE_BASICMAC)
#include <SPI.h>

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

#if !defined(EXCLUDE_SI443X)

#include <RH_RF22.h>

#define RADIOHEAD_SI443X_DEVICE_VERSION     0x06 //  4 0  chip version register

static bool si4432_probe(void);
static void si4432_setup(void);
static void si4432_channel(int8_t);
static bool si4432_receive(void);
static bool si4432_transmit(void);
static void si4432_shutdown(void);

const rfchip_ops_t si4432_ops = {
  RF_IC_SI4432,
  "SI4432",
  si4432_probe,
  si4432_setup,
  si4432_channel,
  si4432_receive,
  si4432_transmit,
  si4432_shutdown
};

RH_RF22 *radio_silabs;

static int8_t si4432_channel_prev    = (int8_t) -1;

static bool si4432_receive_active    = false;

static u1_t si4432_readReg (u1_t addr) {
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

static bool si4432_probe()
{
  SoC->SPI_begin();

  lmic_hal_init (nullptr);

  hal_pin_rst(1); // drive SDN pin high
  delay(1);

  hal_pin_rst(0); // drive SDN pin low
  delay(100);

  u1_t v = si4432_readReg(RH_RF22_REG_01_VERSION_CODE);

#if 0
  Serial.print("si4432 version = "); Serial.println(v, HEX);
#endif

  pinMode(lmic_pins.nss, INPUT);
  RadioSPI.end();

  hal_pin_rst(2); // configure SDN pin floating!

  if (v == RADIOHEAD_SI443X_DEVICE_VERSION) {
    return true;
  } else {
    return false;
  }
}

static void si4432_channel(int8_t channel)
{
  if (channel != -1 && channel != si4432_channel_prev) {
    uint32_t frequency = RF_FreqPlan.getChanFrequency((uint8_t) channel);

    if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {
      nRF905_band_t nrf_band;
      uint32_t nrf_freq_resolution;

      nrf_band = (frequency >= 844800000UL ? NRF905_BAND_868 : NRF905_BAND_433);
      nrf_freq_resolution = (nrf_band == NRF905_BAND_433 ? 100000UL : 200000UL);
      frequency -= (frequency % nrf_freq_resolution);
    }

    bool state = radio_silabs->setFrequency(frequency / 1000000.0);

#if 0
    if (state == false) {
      Serial.println(F("[Si4432] Selected frequency is invalid for this module!"));
      while (true) { delay(10); }
    }
#endif

    si4432_channel_prev = channel;
    /* restart Rx upon a channel switch */
    si4432_receive_active = false;
  }
}

static void si4432_setup()
{
  SoC->SPI_begin();

  uint32_t irq  = lmic_pins.busy == LMIC_UNUSED_PIN ?
                  RH_INVALID_PIN : lmic_pins.busy;

  radio_silabs = new RH_RF22(lmic_pins.nss, irq /*, RadioSPI */);

  switch (settings->rf_protocol)
  {
  case RF_PROTOCOL_OGNTP:
    rh_protocol     = &ogntp_proto_desc;
    protocol_encode = &ogntp_encode;
    protocol_decode = &ogntp_decode;
    break;
  case RF_PROTOCOL_P3I:
    rh_protocol     = &p3i_proto_desc;
    protocol_encode = &p3i_encode;
    protocol_decode = &p3i_decode;
    break;
#if defined(ENABLE_ADSL)
  case RF_PROTOCOL_ADSL_860:
    rh_protocol     = &adsl_proto_desc;
    protocol_encode = &adsl_encode;
    protocol_decode = &adsl_decode;
    break;
#endif /* ENABLE_ADSL */
  case RF_PROTOCOL_LEGACY:
  default:
    rh_protocol     = &legacy_proto_desc;
    protocol_encode = &legacy_encode;
    protocol_decode = &legacy_decode;
    /*
     * Enforce legacy protocol setting for Si4432
     * if other value (UAT) left in EEPROM from other (UATM) radio
     */
    settings->rf_protocol = RF_PROTOCOL_LEGACY;
    break;
  }

  RF_FreqPlan.setPlan(settings->band, settings->rf_protocol);

  float br, fdev, bw;

#if 0
  Serial.print(F("[Si4432] Initializing ... "));
#endif

  bool success = radio_silabs->init();

#if 0
  if (success == true) {
    Serial.println(F("success!"));
  } else {
    Serial.println(F("failed."));
    while (true) { delay(10); }
  }
#endif

#if 0
  /* Work around 0xAA preamble in use by OGNTP */
  uint8_t preambleLen = (rh_protocol->preamble_type == RF_PREAMBLE_TYPE_AA) ?
                        1 : rh_protocol->preamble_size * 2;
#else
  uint8_t preambleLen = rh_protocol->preamble_size * 2;
  preambleLen += (rh_protocol->preamble_type == RF_PREAMBLE_TYPE_AA) ? 1 : 0;
#endif
  radio_silabs->setPreambleLength(preambleLen); // in 4-bit nibbles

  /* Work around premature P3I syncword detection */
  if (rh_protocol->syncword_size == 2) {
    uint8_t preamble = rh_protocol->preamble_type == RF_PREAMBLE_TYPE_AA ?
                       0xAA : 0x55;
    uint8_t sword[4] = { preamble,
                         preamble,
                         rh_protocol->syncword[0],
                         rh_protocol->syncword[1]
                       };
    radio_silabs->setSyncWords(sword, 4);
#if 0
  /* Work around 0xAA preamble in use by OGNTP */
  } else if (rh_protocol->preamble_type == RF_PREAMBLE_TYPE_AA &&
             rh_protocol->preamble_size == 1) {
    uint8_t sword[4] = { 0xAA,
                         rh_protocol->syncword[0],
                         rh_protocol->syncword[1],
                         rh_protocol->syncword[2]
                       };
    radio_silabs->setSyncWords(sword, 4);
    if (rh_protocol->syncword_size > 3) {
//      pkt_size += rh_protocol->syncword_size - 3;
    }
#endif
  } else {
    radio_silabs->setSyncWords((uint8_t *) rh_protocol->syncword,
                               rh_protocol->syncword_size > 4 ? 4 :
                               (size_t) rh_protocol->syncword_size);
    if (rh_protocol->syncword_size > 4) {
//      pkt_size += rh_protocol->syncword_size - 4;
    }
  }

  radio_silabs->setPromiscuous(false);
  radio_silabs->setGpioReversed(false);

  const RH_RF22::ModemConfig cfg_38k = {
    0x02, 0x03, 0x68, 0x01, 0x3a, 0x93, 0x04, 0xd5, 0x40, 0x0a, 0x1e, 0x80,
    0x60, 0x09, 0xd5, 0x0c, 0x23, 0x1f }; // 38.4, 19.6 TBD

  const RH_RF22::ModemConfig cfg_100k = {
    0x8a, 0x03, 0x60, 0x01, 0x55, 0x55, 0x02, 0xad, 0x40, 0x0a, 0x50, 0x80,
    0x60, 0x20, 0x00, 0x0c, 0x23, 0xc8 }; // 125, 125 TBD

  radio_silabs->setModemRegisters(&cfg_100k);

  float txpow;

  switch(settings->txpower)
  {
  case RF_TX_POWER_FULL:

    /* Load regional max. EIRP at first */
    txpow = RF_FreqPlan.MaxTxPower;

    if (txpow > 20)
      txpow = 20;

#if 1
    /*
     * Enforce Tx power limit until confirmation
     * that Si4432 is doing well
     * when antenna is not connected
     */
    if (txpow > 17)
      txpow = 17;
#endif
    radio_silabs->setTxPower(RH_RF22_TXPOW_14DBM); /* TBD */
    break;
  case RF_TX_POWER_OFF:
  case RF_TX_POWER_LOW:
  default:
    radio_silabs->setTxPower(RH_RF22_TXPOW_2DBM);
    break;
  }
}

static bool si4432_receive()
{
  return false; /* TBD */
}

static bool si4432_transmit()
{
  return false; /* TBD */
}

static void si4432_shutdown()
{
  int state = radio_silabs->sleep();
}
#endif /* EXCLUDE_SI443X */

#endif /* USE_RADIOHEAD */
