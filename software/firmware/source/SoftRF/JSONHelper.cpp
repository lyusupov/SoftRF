/*
 * JSONHelper.cpp
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

#include <ArduinoJson.h>

#include "SoCHelper.h"
#include <TinyGPS++.h>
#include "EEPROMHelper.h"
#include "RFHelper.h"
#include "LEDHelper.h"
#include "SoundHelper.h"
#include "BaroHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"
#include "D1090Helper.h"
#include "JSONHelper.h"

#include <iostream>
#include <sstream>
#include <string>
#include <locale>
#include <iomanip>

StaticJsonBuffer<4096> jsonBuffer;

bool hasValidGPSDFix = false;

extern eeprom_t eeprom_block;
extern settings_t *settings;

void JSON_Export()
{
  if (settings->json != JSON_PING) {
    return;
  }

  float distance;
  time_t this_moment = now();
  char buffer[3 * 80 * MAX_TRACKING_OBJECTS];
  bool has_aircraft = false;

  JsonObject& root = jsonBuffer.createObject();
  JsonArray& aircraft_array = root.createNestedArray("aircraft");

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr && (this_moment - Container[i].timestamp) <= EXPORT_EXPIRATION_TIME) {

      distance = Container[i].distance;

      if (distance < ALARM_ZONE_NONE) {

        char hexbuf[8];
        char callsign[8+1];

        snprintf(hexbuf, sizeof(hexbuf), "%06X", Container[i].addr);

        JsonObject& aircraft = aircraft_array.createNestedObject();

        aircraft["icaoAddress"] = hexbuf; // ICAO of the aircraft
        aircraft["trafficSource"] = 2; // 0 = 1090ES , 1 = UAT
        aircraft["latDD"] = Container[i].latitude;  // Latitude expressed as decimal degrees
        aircraft["lonDD"] = Container[i].longitude; // Longitude expressed as decimal degrees
        /* Geometric altitude or barometric pressure altitude in millimeters */
        aircraft["altitudeMM"] = (long) (Container[i].altitude * 1000);
        /* Course over ground in centi-degrees */
        aircraft["headingDE2"] = (int) (Container[i].course * 100);
        /* Horizontal velocity in centimeters/sec */
        aircraft["horVelocityCMS"] = (unsigned long) (Container[i].speed * _GPS_MPS_PER_KNOT * 100);
        /* Vertical velocity in centimeters/sec with positive being up */
        aircraft["verVelocityCMS"] = (long) (Container[i].vs * 100 / (_GPS_FEET_PER_METER * 60.0));
        aircraft["squawk"] = (settings->band == RF_BAND_US ? 1200 : 7000); // VFR Squawk code
        aircraft["altitudeType"] = 1; // Altitude Source: 0 = Pressure 1 = Geometric
        memcpy(callsign, GDL90_CallSign_Prefix[Container[i].protocol],
          strlen(GDL90_CallSign_Prefix[Container[i].protocol]));
        memcpy(callsign + strlen(GDL90_CallSign_Prefix[Container[i].protocol]),
          hexbuf, strlen(hexbuf) + 1);
        aircraft["Callsign"] = callsign; // Callsign
        aircraft["emitterType"] = AT_TO_GDL90(Container[i].aircraft_type); // Category type of the emitter
        aircraft["utcSync"] = 1; // UTC time flag
        /* Time packet was received at the pingStation ISO 8601 format: YYYY-MM-DDTHH:mm:ss:ffffffffZ */
        aircraft["timeStamp"] = "NONE"; // TBD

        has_aircraft = true;
      }
    }
  }

  if (has_aircraft) {
    root.printTo(buffer);
    Serial.println(buffer);
  }

  jsonBuffer.clear();
}

void parseTPV(JsonObject& root)
{
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
}

void parseSettings(JsonObject& root)
{
  JsonVariant mode = root["mode"];
  if (mode.success()) {
    const char * mode_s = mode.as<char*>();
    if (!strcmp(mode_s,"NORMAL")) {
      eeprom_block.field.settings.mode = SOFTRF_MODE_NORMAL;
    } else if (!strcmp(mode_s,"BRIDGE")) {
      eeprom_block.field.settings.mode = SOFTRF_MODE_BRIDGE;
    } else if (!strcmp(mode_s,"RELAY")) {
      eeprom_block.field.settings.mode = SOFTRF_MODE_RELAY;
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
    if (!strcmp(pointer_s,"TRACK")) {
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

  JsonVariant json = root["json"];
  if (json.success()) {
    const char * json_s = json.as<char*>();
    if (!strcmp(json_s,"OFF")) {
      eeprom_block.field.settings.json = JSON_OFF;
    } else if (!strcmp(json_s,"PING")) {
      eeprom_block.field.settings.json = JSON_PING;
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

#endif /* RASPBERRY_PI */
