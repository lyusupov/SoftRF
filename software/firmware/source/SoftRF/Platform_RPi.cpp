/*
 * Platform_RPi.cpp
 * Copyright (C) 2018 Linar Yusupov
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

#if defined(RASPBERRY_PI)

#include "Platform_RPi.h"
#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include <TinyGPS++.h>
#include <aircraft.h>

// Dragino LoRa/GPS HAT or compatible SX1276 pin mapping
lmic_pinmap lmic_pins = {
    .nss = SOC_GPIO_PIN_SS,
    .rxtx = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
    .rst = SOC_GPIO_PIN_RST,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void onEvent (ev_t ev) { }

settings_t *settings;
ufo_t ThisAircraft;
TinyGPSPlus gnss;  // Create an Instance of the TinyGPS++ object called gnss
aircraft the_aircraft;

String Bin2Hex(byte *buffer)
{
  String str = "";
  for (int i=0; i < PKT_SIZE; i++) {
    byte c = buffer[i];
    str += (c < 0x10 ? "0" : "") + String(c, HEX);
  }
  return str;
}

static void RPi_setup()
{

}

static uint32_t RPi_getChipId()
{
  return 0;
}

static long RPi_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

static bool RPi_EEPROM_begin(size_t size)
{
  return true;
}

static void RPi_SPI_begin()
{

}

SoC_ops_t RPi_ops = {
  SOC_RPi,
  "RPi",
  RPi_setup,
  RPi_getChipId,
  NULL,
  NULL,
  NULL,
  RPi_random,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  RPi_EEPROM_begin,
  RPi_SPI_begin,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL
};

int main()
{

}

#endif /* RASPBERRY_PI */
