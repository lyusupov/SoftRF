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
 *  pi@raspberrypi $ { echo "{\"class\":\"SOFTRF\",\"protocol\":\"OGNTP\"}" ; cat /dev/ttyUSB0 ; } | sudo ./SoftRF
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
#include <sstream>
#include <string>
#include <locale>
#include <iomanip>

#include <ArduinoJson.h>

std::string input_line;
StaticJsonBuffer<4096> jsonBuffer;

bool hasValidGPSDFix = false;

#define isValidGPSDFix() (hasValidGPSDFix)
#define isValidFix() (isValidGNSSFix() || isValidGPSDFix())

static void RPi_PickGNSSFix()
{
  if (inputAvailable()) {
    std::getline(std::cin, input_line);
    const char *str = input_line.c_str();
    int len = input_line.length();

    if (str[0] == '$' && str[1] == 'G') {
      // NMEA input
      for (int i=0; i < len; i++) {
        gnss.encode(str[i]);
      }
      if (settings->nmea_g) {
        NMEA_Out((byte *) str, len, true);
      }

      GNSSTimeSync();

      if (isValidGNSSFix()) {
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
      }

    } else if (str[0] == '{') {
      // JSON input

      JsonObject& root = jsonBuffer.parseObject(str);

      JsonVariant msg_class = root["class"]; // "TPV"

      if (msg_class.success()) {
        const char *msg_class_s = msg_class.as<char*>();

        if (!strcmp(msg_class_s,"TPV")) {

          int mode = 0;
          bool hasmode = root.containsKey("mode");
          if (hasmode) {
            mode = root["mode"];
          }

          if (mode == 3) { // 3D fix

            std::tm t = {};

            bool hastime = root.containsKey("time");
            time_t epoch = 0;
            if (hastime) {

              const char *time_s = root["time"]; // "2018-11-06T09:16:39.196Z"
              std::istringstream ss(time_s);
              ss.imbue(std::locale("en_US.UTF-8"));
              ss >> std::get_time(&t, "%Y-%m-%dT%H:%M:%S");
              if (ss.fail()) {
                  std::cout << "Parse failed\n";
              }

              epoch = mktime(&t);

              setTime(t.tm_hour, t.tm_min, t.tm_sec, t.tm_mday,
                      t.tm_mon + 1, t.tm_year + 1900);

              hasValidGPSDFix = true;
            }

            float lat = root["lat"];
            float lon = root["lon"];
            float alt = root["alt"];

            bool hastrack = root.containsKey("track");
            int track = 0;
            if (hastrack) {
               track = root["track"];
            }

            int speed = root["speed"];

            ThisAircraft.latitude = lat;
            ThisAircraft.longitude = lon;
            ThisAircraft.altitude = alt;
            if (hastrack) {
              ThisAircraft.course = track;
            }
            ThisAircraft.speed = speed / _GPS_MPS_PER_KNOT;
            //ThisAircraft.hdop = (uint16_t) gnss.hdop.value();
            //ThisAircraft.geoid_separation = gnss.separation.meters();

          }
        } else if (!strcmp(msg_class_s,"SOFTRF")) {

          JsonVariant mode = root["mode"];
          if (mode.success()) {
            const char * mode_s = mode.as<char*>();
            if (!strcmp(mode_s,"NORMAL")) {
              eeprom_block.field.settings.mode = SOFTRF_MODE_NORMAL;
            } else if (!strcmp(mode_s,"BRIDGE")) {
              eeprom_block.field.settings.mode = SOFTRF_MODE_BRIDGE;
            } else if (!strcmp(mode_s,"UAV")) {
              eeprom_block.field.settings.mode = SOFTRF_MODE_UAV;
            }
          }

          JsonVariant protocol = root["protocol"];
          if (protocol.success()) {
            const char * protocol_s = protocol.as<char*>();
            if (!strcmp(protocol_s,"LEGACY")) {
              eeprom_block.field.settings.rf_protocol = RF_PROTOCOL_LEGACY;
            } else if (!strcmp(protocol_s,"OGNTP")) {
              eeprom_block.field.settings.rf_protocol = RF_PROTOCOL_OGNTP;
            } else if (!strcmp(protocol_s,"P3I")) {
              eeprom_block.field.settings.rf_protocol = RF_PROTOCOL_P3I;
            } else if (!strcmp(protocol_s,"FANET")) {
              eeprom_block.field.settings.rf_protocol = RF_PROTOCOL_FANET;
            }
          }

          JsonVariant band = root["band"];
          if (band.success()) {
            const char * band_s = band.as<char*>();
            if (!strcmp(band_s,"AUTO")) {
              eeprom_block.field.settings.band = RF_BAND_AUTO;
            } else if (!strcmp(band_s,"EU")) {
              eeprom_block.field.settings.band = RF_BAND_EU;
            } else if (!strcmp(band_s,"US")) {
              eeprom_block.field.settings.band = RF_BAND_US;
            } else if (!strcmp(band_s,"AU")) {
              eeprom_block.field.settings.band = RF_BAND_AU;
            } else if (!strcmp(band_s,"NZ")) {
              eeprom_block.field.settings.band = RF_BAND_NZ;
            } else if (!strcmp(band_s,"RU")) {
              eeprom_block.field.settings.band = RF_BAND_RU;
            } else if (!strcmp(band_s,"CN")) {
              eeprom_block.field.settings.band = RF_BAND_CN;
            } else if (!strcmp(band_s,"UK")) {
              eeprom_block.field.settings.band = RF_BAND_UK;
            } else if (!strcmp(band_s,"IN")) {
              eeprom_block.field.settings.band = RF_BAND_IN;
            }
          }

          JsonVariant aircraft_type = root["aircraft_type"];
          if (aircraft_type.success()) {
            const char * aircraft_type_s = aircraft_type.as<char*>();
            if (!strcmp(aircraft_type_s,"GLDER")) {
              eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_GLIDER;
            } else if (!strcmp(aircraft_type_s,"TOWPLANE")) {
              eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_TOWPLANE;
            } else if (!strcmp(aircraft_type_s,"POWERED")) {
              eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_POWERED;
            } else if (!strcmp(aircraft_type_s,"HELICOPTER")) {
              eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_HELICOPTER;
            } else if (!strcmp(aircraft_type_s,"UAV")) {
              eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_UAV;
            } else if (!strcmp(aircraft_type_s,"HANGGLIDER")) {
              eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_HANGGLIDER;
            } else if (!strcmp(aircraft_type_s,"PARAGLIDER")) {
              eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_PARAGLIDER;
            } else if (!strcmp(aircraft_type_s,"BALLOON")) {
              eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_BALLOON;
            }
          }

          JsonVariant alarm = root["alarm"];
          if (alarm.success()) {
            const char * alarm_s = alarm.as<char*>();
            if (!strcmp(alarm_s,"NONE")) {
              eeprom_block.field.settings.alarm = TRAFFIC_ALARM_NONE;
            } else if (!strcmp(alarm_s,"DISTANCE")) {
              eeprom_block.field.settings.alarm = TRAFFIC_ALARM_DISTANCE;
            } else if (!strcmp(alarm_s,"VECTOR")) {
              eeprom_block.field.settings.alarm = TRAFFIC_ALARM_VECTOR;
            }
          }

          JsonVariant txpower = root["txpower"];
          if (txpower.success()) {
            const char * txpower_s = txpower.as<char*>();
            if (!strcmp(txpower_s,"FULL")) {
              eeprom_block.field.settings.txpower = RF_TX_POWER_FULL;
            } else if (!strcmp(txpower_s,"LOW")) {
              eeprom_block.field.settings.txpower = RF_TX_POWER_LOW;
            } else if (!strcmp(txpower_s,"OFF")) {
              eeprom_block.field.settings.txpower = RF_TX_POWER_OFF;
            }
          }

          JsonVariant volume = root["volume"];
          if (volume.success()) {
            const char * volume_s = volume.as<char*>();
            if (!strcmp(volume_s,"FULL")) {
              eeprom_block.field.settings.volume = BUZZER_VOLUME_FULL;
            } else if (!strcmp(volume_s,"LOW")) {
              eeprom_block.field.settings.volume = BUZZER_VOLUME_LOW;
            } else if (!strcmp(volume_s,"OFF")) {
              eeprom_block.field.settings.volume = BUZZER_OFF;
            }
          }

          JsonVariant pointer = root["pointer"];
          if (pointer.success()) {
            const char * pointer_s = pointer.as<char*>();
            if (!strcmp(pointer_s,"UP")) {
              eeprom_block.field.settings.pointer = DIRECTION_TRACK_UP;
            } else if (!strcmp(pointer_s,"NORTH")) {
              eeprom_block.field.settings.pointer = DIRECTION_NORTH_UP;
            } else if (!strcmp(pointer_s,"OFF")) {
              eeprom_block.field.settings.pointer = LED_OFF;
            }
          }

          JsonVariant nmea_g = root["nmea"]["gnss"];
          if (nmea_g.success()) {
            eeprom_block.field.settings.nmea_g = nmea_g.as<bool>();
          }

          JsonVariant nmea_p = root["nmea"]["private"];
          if (nmea_p.success()) {
            eeprom_block.field.settings.nmea_p = nmea_p.as<bool>();
          }

          JsonVariant nmea_l = root["nmea"]["legacy"];
          if (nmea_l.success()) {
            eeprom_block.field.settings.nmea_l = nmea_l.as<bool>();
          }

          JsonVariant nmea_s = root["nmea"]["sensors"];
          if (nmea_s.success()) {
            eeprom_block.field.settings.nmea_s = nmea_s.as<bool>();
          }

          JsonVariant nmea_out = root["nmea"]["output"];
          if (nmea_out.success()) {
            const char * nmea_out_s = nmea_out.as<char*>();
            if (!strcmp(nmea_out_s,"OFF")) {
              eeprom_block.field.settings.nmea_out = NMEA_OFF;
            } else if (!strcmp(nmea_out_s,"UART")) {
              eeprom_block.field.settings.nmea_out = NMEA_UART;
            } else if (!strcmp(nmea_out_s,"UDP")) {
              eeprom_block.field.settings.nmea_out = NMEA_UDP;
            }
          }

          JsonVariant gdl90 = root["gdl90"];
          if (gdl90.success()) {
            const char * gdl90_s = gdl90.as<char*>();
            if (!strcmp(gdl90_s,"OFF")) {
              eeprom_block.field.settings.gdl90 = GDL90_OFF;
            } else if (!strcmp(gdl90_s,"UART")) {
              eeprom_block.field.settings.gdl90 = GDL90_UART;
            } else if (!strcmp(gdl90_s,"UDP")) {
              eeprom_block.field.settings.gdl90 = GDL90_UDP;
            }
          }

          JsonVariant d1090 = root["d1090"];
          if (d1090.success()) {
            const char * d1090_s = d1090.as<char*>();
            if (!strcmp(d1090_s,"OFF")) {
              eeprom_block.field.settings.d1090 = D1090_OFF;
            } else if (!strcmp(d1090_s,"UART")) {
              eeprom_block.field.settings.d1090 = D1090_UART;
            }
          }

          JsonVariant stealth = root["stealth"];
          if (stealth.success()) {
            eeprom_block.field.settings.stealth = stealth.as<bool>();
          }

          JsonVariant no_track = root["no_track"];
          if (no_track.success()) {
            eeprom_block.field.settings.no_track = no_track.as<bool>();
          }

        }
      }

      jsonBuffer.clear();

      if ((time(NULL) - now()) > 3) {
        hasValidGPSDFix = false;
      }
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

    RPi_PickGNSSFix();

    RF_loop();

    ThisAircraft.timestamp = now();

    if (isValidFix()) {
      RF_Transmit(RF_Encode());
    }

    bool success = RF_Receive();

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
  }

  return 0;
}

#endif /* RASPBERRY_PI */
