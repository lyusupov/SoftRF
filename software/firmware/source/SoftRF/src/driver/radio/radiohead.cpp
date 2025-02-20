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

#if 1
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
  /* TBD */
}

static void si4432_setup()
{
  /* TBD */
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
  /* TBD */
}
#endif /* EXCLUDE_SI443X */

#endif /* USE_RADIOHEAD */
