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
#include "RFHelper.h"
#include "LEDHelper.h"
#include "SoundHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"
#include "D1090Helper.h"

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

eeprom_t eeprom_block;
settings_t *settings = &eeprom_block.field.settings;
ufo_t ThisAircraft, fo;
TinyGPSPlus gnss;  // Create an Instance of the TinyGPS++ object called gnss
aircraft the_aircraft;
volatile unsigned long PPS_TimeMarker = 0;

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
  eeprom_block.field.magic = SOFTRF_EEPROM_MAGIC;
  eeprom_block.field.version = SOFTRF_EEPROM_VERSION;
  eeprom_block.field.settings.mode = SOFTRF_MODE_NORMAL;
  eeprom_block.field.settings.rf_protocol = RF_PROTOCOL_OGNTP;
  eeprom_block.field.settings.band = RF_BAND_EU;
  eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_GLIDER;
  eeprom_block.field.settings.txpower = RF_TX_POWER_FULL;
  eeprom_block.field.settings.volume = BUZZER_VOLUME_FULL;
  eeprom_block.field.settings.pointer = DIRECTION_NORTH_UP;
  eeprom_block.field.settings.bluetooth = BLUETOOTH_OFF;
  eeprom_block.field.settings.alarm = TRAFFIC_ALARM_DISTANCE;

  eeprom_block.field.settings.nmea_g   = true;
  eeprom_block.field.settings.nmea_p   = false;
  eeprom_block.field.settings.nmea_l   = true;
  eeprom_block.field.settings.nmea_s   = true;
  eeprom_block.field.settings.nmea_out = NMEA_UART;
  eeprom_block.field.settings.gdl90    = GDL90_OFF;
  eeprom_block.field.settings.d1090    = D1090_OFF;
  eeprom_block.field.settings.stealth  = false;
  eeprom_block.field.settings.no_track = false;
}

static uint32_t RPi_getChipId()
{
  return gethostid();
}

static long RPi_random(long howsmall, long howBig)
{
  return random(howsmall, howBig);
}

static void RPi_SPI_begin()
{

}

void RPi_GNSS_PPS_Interrupt_handler() {
  PPS_TimeMarker = millis();
}

static unsigned long RPi_get_PPS_TimeMarker() {
  return PPS_TimeMarker;
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
  NULL,
  RPi_SPI_begin,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  RPi_get_PPS_TimeMarker,
  NULL
};

int main()
{
  // Init GPIO bcm
  if (!bcm2835_init()) {
      fprintf( stderr, "bcm2835_init() Failed\n\n" );
      return 1;
  }

  Serial.begin(38400);

  byte soc_id = SoC_setup(); // Has to be very first procedure in the execution order

  byte rf_chip_id = RF_setup();

  ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;
  ThisAircraft.aircraft_type = settings->aircraft_type;
  ThisAircraft.protocol = settings->rf_protocol;
  ThisAircraft.stealth  = settings->stealth;
  ThisAircraft.no_track = settings->no_track;

  ThisAircraft.latitude = gnss.location.lat();
  ThisAircraft.longitude = gnss.location.lng();
  ThisAircraft.altitude = gnss.altitude.meters();
  ThisAircraft.course = gnss.course.deg();
  ThisAircraft.speed = gnss.speed.knots();
  ThisAircraft.hdop = (uint16_t) gnss.hdop.value();
  ThisAircraft.geoid_separation = gnss.separation.meters();

  while (true) {
    // Do common RF stuff first
    RF_loop();

    RF_Transmit(RF_Encode());

    bool success = RF_Receive();
    if (success) {
      fo.raw = Bin2Hex(RxBuffer);
      if (protocol_decode && (*protocol_decode)((void *) RxBuffer, &ThisAircraft, &fo)) {
        fo.rssi = RF_last_rssi;

        printf("%X %d %f %f %f %f %f\n", fo.addr, fo.aircraft_type,
                                         fo.latitude, fo.longitude,
                                         fo.altitude, fo.speed, fo.course);
      }
    }
  }

  return 0;
}

#endif /* RASPBERRY_PI */
