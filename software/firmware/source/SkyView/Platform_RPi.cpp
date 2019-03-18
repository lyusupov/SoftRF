/*
 * Platform_RPi.cpp
 * Copyright (C) 2019 Linar Yusupov
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

/*
 * Usage example:
 *
 *  pi@raspberrypi $ make -f Makefile.RPi
 *
 *     < ... skipped ... >
 *
 *
 */

#if defined(RASPBERRY_PI)

#include <stdio.h>

#include "SoCHelper.h"
#include "NMEAHelper.h"
#include "EPDHelper.h"
#include "TrafficHelper.h"

TTYSerial SerialInput("/dev/ttyUSB1");

static const uint8_t SS    = 8; // pin 24

GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> display(GxEPD2_270(/*CS=5*/ SS,
                                       /*DC=*/ 25, /*RST=*/ 17, /*BUSY=*/ 24));

lmic_pinmap lmic_pins = {
    .nss = LMIC_UNUSED_PIN,
    .rxtx = { LMIC_UNUSED_PIN, LMIC_UNUSED_PIN },
    .rst = LMIC_UNUSED_PIN,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void radio_irq_handler (u1_t dio) {
}
u1_t radio_has_irq (void) {
    return 0;
}

static void RPi_setup()
{

}

const SoC_ops_t RPi_ops = {
  SOC_RPi,
  "RPi",
  RPi_setup
};

int main()
{
  // Init GPIO bcm
  if (!bcm2835_init()) {
      fprintf( stderr, "bcm2835_init() Failed\n\n" );
      exit(EXIT_FAILURE);
  }

  Serial.begin(38400);

  EPD_setup();
  NMEA_setup();

  while (true) {
    NMEA_loop();
    EPD_loop();
    ClearExpired();
  }

  return 0;
}

#endif /* RASPBERRY_PI */
