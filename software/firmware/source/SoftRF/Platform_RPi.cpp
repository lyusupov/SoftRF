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

/*
 * Usage example:
 *
 *  pi@raspberrypi $ make -f Makefile.RPi
 *
 *     < ... skipped ... >
 *
 *  pi@raspberrypi $ cat /dev/ttyUSB0 | sudo ./SoftRF
 *  SX1276 RFIC is detected.
 *  $GPGSA,A,3,02,30,05,06,07,09,,,,,,,5.09,3.19,3.97*04
 *  $GPRMC,145750.00,A,5XXX.XXX68,N,03XXX.XXX33,E,0.701,,051118,,,A*7E
 *  $GPGGA,145750.00,5XXX.XXX68,N,03XXX.XXX33,E,1,06,3.19,179.2,M,12.5,M,,*5E
 *  $PFLAA,3,0,0,0,2,C5D804!OGN_C5D804,0,,0,00000.0,1*60
 *  $PFLAU,1,1,2,1,3,-30,2,0,0*4E
 *
 *     < ... skipped ... >
 *
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
#include "BaroHelper.h"
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
ufo_t ThisAircraft;
aircraft the_aircraft;

char UDPpacketBuffer[256]; // buffer to hold incoming and outgoing packets

hardware_info_t hw_info = {
  .model    = SOFTRF_MODEL_STANDALONE,
  .revision = 0,
  .soc      = SOC_NONE,
  .rf       = RF_IC_NONE,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = BARO_MODULE_NONE,
  .display  = DISPLAY_NONE
};

#define isTimeToExport() (millis() - ExportTimeMarker > 1000)
unsigned long ExportTimeMarker = 0;

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

static void RPi_WiFi_transmit_UDP(int port, byte *buf, size_t size)
{
  /* TBD */
}

static void RPi_SPI_begin()
{
  /* TBD */
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
  RPi_WiFi_transmit_UDP,
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

#include <stdio.h>
#include <sys/select.h>

bool inputAvailable()
{
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return (FD_ISSET(0, &fds));
}

#include <iostream>
#include <string>

std::string input_line;

static void RPi_PickGNSSFix()
{
  if (inputAvailable()) {
    std::getline(std::cin, input_line);
    const char *str = input_line.c_str();
    int len = input_line.length();
    for (int i=0; i < len; i++) {
      gnss.encode(str[i]);
    }
    if (settings->nmea_g) {
      NMEA_Out((byte *) str, len, true);
    }
  }
}

int main()
{
  // Init GPIO bcm
  if (!bcm2835_init()) {
      fprintf( stderr, "bcm2835_init() Failed\n\n" );
      return 1;
  }

  Serial.begin(38400);

  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  hw_info.rf = RF_setup();

  ThisAircraft.addr = SoC->getChipId() & 0x00FFFFFF;
  ThisAircraft.aircraft_type = settings->aircraft_type;
  ThisAircraft.protocol = settings->rf_protocol;
  ThisAircraft.stealth  = settings->stealth;
  ThisAircraft.no_track = settings->no_track;

  Traffic_setup();
  NMEA_setup();

  while (true) {
    // Do common RF stuff first
    RF_loop();

    RPi_PickGNSSFix();

    GNSSTimeSync();

    ThisAircraft.timestamp = now();
    if (isValidFix()) {
      ThisAircraft.latitude = gnss.location.lat();
      ThisAircraft.longitude = gnss.location.lng();
      ThisAircraft.altitude = gnss.altitude.meters();
      ThisAircraft.course = gnss.course.deg();
      ThisAircraft.speed = gnss.speed.knots();
      ThisAircraft.hdop = (uint16_t) gnss.hdop.value();
      ThisAircraft.geoid_separation = gnss.separation.meters();

      /*
       * When geoidal separation is zero or not available - use approx. EGM96 value
       */
      if (ThisAircraft.geoid_separation == 0.0) {
        ThisAircraft.geoid_separation = (float) LookupSeparation(
                                                  ThisAircraft.latitude,
                                                  ThisAircraft.longitude
                                                );
        /* we can assume the GPS unit is giving ellipsoid height */
        ThisAircraft.altitude -= ThisAircraft.geoid_separation;
      }

      RF_Transmit(RF_Encode());
    }

    bool success = RF_Receive();

#if 0
    if (success) {
      fo.raw = Bin2Hex(RxBuffer);
      if (protocol_decode && (*protocol_decode)((void *) RxBuffer, &ThisAircraft, &fo)) {
        fo.rssi = RF_last_rssi;

        printf("%X %d %f %f %f %f %f\n", fo.addr, fo.aircraft_type,
                                         fo.latitude, fo.longitude,
                                         fo.altitude, fo.speed, fo.course);
      }
    }
#else
    if (success && isValidFix()) ParseData();

    if (isValidFix()) {
      Traffic_loop();
    }

    if (isTimeToExport() && isValidFix()) {
      NMEA_Export();
      GDL90_Export();
      D1090_Export();
      ExportTimeMarker = millis();
    }

    // Handle Air Connect
    NMEA_loop();

    ClearExpired();
#endif
  }

  return 0;
}

#endif /* RASPBERRY_PI */
