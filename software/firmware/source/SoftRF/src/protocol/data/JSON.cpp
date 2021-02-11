/*
 * JSONHelper.cpp
 * Copyright (C) 2018-2021 Linar Yusupov
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

#if defined(RASPBERRY_PI) || defined(ARDUINO_ARCH_NRF52)

#include <ArduinoJson.h>

#include "../../system/SoC.h"
#include <TinyGPS++.h>
#include "../../driver/EEPROM.h"
#include "../../driver/RF.h"
#include "../../driver/LED.h"
#include "../../driver/Sound.h"
#include "../../driver/Baro.h"
#include "../../driver/EPD.h"
#include "../../TrafficHelper.h"
#include "NMEA.h"
#include "GDL90.h"
#include "D1090.h"
#include "JSON.h"

extern eeprom_t eeprom_block;
extern settings_t *settings;

#endif /* RASPBERRY_PI || ARDUINO_ARCH_NRF52 */

#if defined(RASPBERRY_PI)

#include <iostream>
#include <sstream>
#include <string>
#include <locale>
#include <iomanip>

StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;

bool hasValidGPSDFix = false;

byte getVal(char c)
{
   if(c >= '0' && c <= '9')
     return (byte)(c - '0');
   else
     return (byte)(toupper(c)-'A'+10);
}

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
        char timebuf[32];
        time_t timestamp = now(); /* GNSS date&time */

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
        strftime(timebuf, sizeof(timebuf), "%FT%T:00000000Z", gmtime(&timestamp));
        aircraft["timeStamp"] = timebuf;

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

void parsePING(JsonObject& root)
{
  ping_aircraft_t *aircraft_array;

  JsonArray& aircraft = root["aircraft"];

  int size = aircraft.size();
  time_t timestamp = now();

  if (size > 0) {
    aircraft_array = (ping_aircraft_t *)
                      malloc(sizeof(ping_aircraft_t) * size);

    if (aircraft_array == NULL) {
      return;
    }

    for (int i=0; i < size; i++) {
      JsonObject& aircraft_obj = aircraft[i];

      aircraft_array[i].icaoAddress = aircraft_obj["icaoAddress"];
      aircraft_array[i].trafficSource = aircraft_obj["trafficSource"];
      aircraft_array[i].latDD = aircraft_obj["latDD"];
      aircraft_array[i].lonDD = aircraft_obj["lonDD"];
      aircraft_array[i].altitudeMM = aircraft_obj["altitudeMM"];
      aircraft_array[i].headingDE2 = aircraft_obj["headingDE2"];
      aircraft_array[i].horVelocityCMS = aircraft_obj["horVelocityCMS"];
      aircraft_array[i].verVelocityCMS = aircraft_obj["verVelocityCMS"];
      aircraft_array[i].squawk = aircraft_obj["squawk"];
      aircraft_array[i].altitudeType = aircraft_obj["altitudeType"];
      aircraft_array[i].Callsign = aircraft_obj["Callsign"];
      aircraft_array[i].emitterType = aircraft_obj["emitterType"];
      aircraft_array[i].utcSync = aircraft_obj["utcSync"];
      aircraft_array[i].timeStamp = aircraft_obj["timeStamp"];
    }

    for (int i=0; i < size; i++) {

      if (aircraft_array[i].icaoAddress &&
          aircraft_array[i].latDD != 0.0 &&
          aircraft_array[i].lonDD != 0.0 &&
          aircraft_array[i].altitudeMM != 0) {

        fo = EmptyFO;
        memset(fo.raw, 0, sizeof(fo.raw));

#if 0
        std::tm t = {};
        std::istringstream ss(aircraft_array[i].timeStamp);
        ss.imbue(std::locale("en_US.UTF-8"));
        ss >> std::get_time(&t, "%Y-%m-%dT%H:%M:%S");
        if (ss.fail()) {
            std::cout << "Parse failed\n";
        }

        if (aircraft_array[i].utcSync) {
          fo.timestamp = mktime(&t);
        } else {
          fo.timestamp = timestamp;
        }
#else
        fo.timestamp = timestamp;
#endif
        fo.protocol = RF_PROTOCOL_ADSB_1090;

        fo.addr = strtoul (&aircraft_array[i].icaoAddress[0], NULL, 16);
        fo.addr_type = ADDR_TYPE_ICAO;

        fo.latitude = aircraft_array[i].latDD;
        fo.longitude = aircraft_array[i].lonDD;

        if (aircraft_array[i].altitudeType == 0) {
          fo.pressure_altitude = aircraft_array[i].altitudeMM / 1000.0;

          /* TBD */
          fo.altitude = fo.pressure_altitude;
        } else if (aircraft_array[i].altitudeType == 1) {
          fo.altitude = aircraft_array[i].altitudeMM / 1000.0;
        }

        fo.course = (float) aircraft_array[i].headingDE2 / 100.0;
        fo.speed = (float) aircraft_array[i].horVelocityCMS / (_GPS_MPS_PER_KNOT * 100);
        fo.aircraft_type = GDL90_TO_AT(aircraft_array[i].emitterType);
        fo.vs = (float) aircraft_array[i].verVelocityCMS * (_GPS_FEET_PER_METER * 60.0) / 100;
        fo.stealth = false;
        fo.no_track = false;
        fo.rssi = 0;

        Traffic_Update(&fo);

        int j;

        /* Try to find and update an entry with the same aircraft ID */
        for (j=0; j < MAX_TRACKING_OBJECTS; j++) {
          if (Container[j].addr == fo.addr && Container[j].protocol == fo.protocol) {
            Container[j] = fo;
            break;
          }
        }

        if (j < MAX_TRACKING_OBJECTS) {
            continue;
        }

        /* Fill a free entry if able */
        for (j=0; j < MAX_TRACKING_OBJECTS; j++) {
          if (Container[j].addr == 0 &&
             memcmp(Container[j].raw, EmptyFO.raw, sizeof(EmptyFO.raw)) == 0) {
            Container[j] = fo;
            break;
          }
        }

        if (j < MAX_TRACKING_OBJECTS) {
            continue;
        }

        /* Overwrite expired entry */
        for (j=0; j < MAX_TRACKING_OBJECTS; j++) {
          if (timestamp - Container[j].timestamp > ENTRY_EXPIRATION_TIME) {
            Container[j] = fo;
            break;
          }
        }
      }
    }

#if 0
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].addr &&
          Container[i].latitude  != 0.0 &&
          Container[i].longitude != 0.0 &&
          Container[i].altitude  != 0.0) {

        printf("%06X %f %f %f %d %d %d\n",
            Container[i].addr,
            Container[i].latitude,
            Container[i].longitude,
            Container[i].altitude,
            Container[i].addr_type,
            (int) Container[i].vs,
            Container[i].aircraft_type);
      }
    }
#endif

    free(aircraft_array);
  }
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

void parseD1090(JsonObject& root)
{
  dump1090_aircraft_t *aircraft_array;

  float var_now = root["now"];
  int var_messages = root["messages"];

  JsonArray& aircraft = root["aircraft"];

  int size = aircraft.size();
  time_t timestamp = now();

  if (size > 0) {
    aircraft_array = (dump1090_aircraft_t *)
                      malloc(sizeof(dump1090_aircraft_t) * size);

    if (aircraft_array == NULL) {
      return;
    }

    for (int i=0; i < size; i++) {
      JsonObject& aircraft_obj = aircraft[i];

      aircraft_array[i].hex = aircraft_obj["hex"];
      aircraft_array[i].squawk = aircraft_obj["squawk"];
      aircraft_array[i].flight = aircraft_obj["flight"];
      aircraft_array[i].lat = aircraft_obj["lat"];
      aircraft_array[i].lon = aircraft_obj["lon"];
      aircraft_array[i].nucp = aircraft_obj["nucp"];
      aircraft_array[i].seen_pos = aircraft_obj["seen_pos"];
      aircraft_array[i].altitude = aircraft_obj["altitude"];
      aircraft_array[i].vert_rate = aircraft_obj["vert_rate"];
      aircraft_array[i].track = aircraft_obj["track"];
      aircraft_array[i].speed = aircraft_obj["speed"];
      aircraft_array[i].messages = aircraft_obj["messages"];
      aircraft_array[i].seen = aircraft_obj["seen"];
      aircraft_array[i].rssi = aircraft_obj["rssi"];
    }

    for (int i=0; i < size; i++) {

      if (aircraft_array[i].hex &&
          aircraft_array[i].lat != 0.0 &&
          aircraft_array[i].lon != 0.0 &&
          aircraft_array[i].altitude != 0.0) {

        fo = EmptyFO;
        memset(fo.raw, 0, sizeof(fo.raw));
#if 0
        fo.timestamp = (time_t) (var_now - aircraft_array[i].seen_pos);
#else
        fo.timestamp = timestamp;
#endif
        fo.protocol = RF_PROTOCOL_ADSB_1090;

        if (aircraft_array[i].hex[0] == '~') {
          fo.addr = strtoul (&aircraft_array[i].hex[1], NULL, 16);
          fo.addr_type = ADDR_TYPE_ANONYMOUS;
        } else {
          fo.addr = strtoul (&aircraft_array[i].hex[0], NULL, 16);
          fo.addr_type = ADDR_TYPE_ICAO;
        }

        fo.latitude = aircraft_array[i].lat;
        fo.longitude = aircraft_array[i].lon;
        fo.pressure_altitude = aircraft_array[i].altitude / _GPS_FEET_PER_METER;

        /* TBD */
        fo.altitude = fo.pressure_altitude;

        fo.course = aircraft_array[i].track;
        fo.speed = aircraft_array[i].speed;
        fo.aircraft_type = AIRCRAFT_TYPE_JET;
        fo.vs = aircraft_array[i].vert_rate;
        fo.stealth = false;
        fo.no_track = false;
        fo.rssi = aircraft_array[i].rssi;

        Traffic_Update(&fo);

        int j;

        /* Try to find and update an entry with the same aircraft ID */
        for (j=0; j < MAX_TRACKING_OBJECTS; j++) {
          if (Container[j].addr == fo.addr && Container[j].protocol == fo.protocol) {
            Container[j] = fo;
            break;
          }
        }

        if (j < MAX_TRACKING_OBJECTS) {
            continue;
        }

        /* Fill a free entry if able */
        for (j=0; j < MAX_TRACKING_OBJECTS; j++) {
          if (Container[j].addr == 0 &&
             memcmp(Container[j].raw, EmptyFO.raw, sizeof(EmptyFO.raw)) == 0) {
            Container[j] = fo;
            break;
          }
        }

        if (j < MAX_TRACKING_OBJECTS) {
            continue;
        }

        /* Overwrite expired entry */
        for (j=0; j < MAX_TRACKING_OBJECTS; j++) {
          if (timestamp - Container[j].timestamp > ENTRY_EXPIRATION_TIME) {
            Container[j] = fo;
            break;
          }
        }
      }
    }

#if 0
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].addr &&
          Container[i].latitude  != 0.0 &&
          Container[i].longitude != 0.0 &&
          Container[i].altitude  != 0.0) {

        printf("%06X %f %f %f %d %d %d\n",
            Container[i].addr,
            Container[i].latitude,
            Container[i].longitude,
            Container[i].altitude,
            Container[i].addr_type,
            (int) Container[i].vs,
            Container[i].aircraft_type);
      }
    }
#endif

    free(aircraft_array);
  }
}

void parseRAW(JsonObject& root)
{

  JsonArray& rawdata = root["rawdata"];

  int size = rawdata.size();
  time_t timestamp = now();

  if (size > 0) {

    for (int i=0; i < size; i++) {
      const char* data = rawdata[i];
      size_t data_len = strlen(data);
      if (data_len > 0) {

        fo = EmptyFO;

        if (data_len > 2 * MAX_PKT_SIZE) {
          data_len = 2 * MAX_PKT_SIZE;
        }

        if (data_len > 2 * sizeof(fo.raw)) {
          data_len = 2 * sizeof(fo.raw);
        }

        for(int j = 0; j < data_len ; j+=2)
        {
          fo.raw[j>>1] = getVal(data[j+1]) + (getVal(data[j]) << 4);
        }

        fo.timestamp = timestamp;
        fo.protocol = RF_PROTOCOL_ADSB_1090;

        int j;

        /* Fill a free entry if able */
        for (j=0; j < MAX_TRACKING_OBJECTS; j++) {
          if (Container[j].addr == 0 &&
             memcmp(Container[j].raw, EmptyFO.raw, sizeof(EmptyFO.raw)) == 0) {
            Container[j] = fo;
            break;
          }
        }

        if (j < MAX_TRACKING_OBJECTS) {
            continue;
        }

        /* Overwrite expired entry */
        for (j=0; j < MAX_TRACKING_OBJECTS; j++) {
          if (timestamp - Container[j].timestamp > ENTRY_EXPIRATION_TIME) {
            Container[j] = fo;
            break;
          }
        }
      }
    }

#if 0
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (memcmp(Container[i].raw, EmptyFO.raw, sizeof(EmptyFO.raw)) != 0) {
        size_t size = RF_Payload_Size(settings->rf_protocol);
        size = size > sizeof(Container[i].raw) ? sizeof(Container[i].raw) : size;
        String str = Bin2Hex(Container[i].raw, size);
        printf("%s\n", str.c_str());
      }
    }
#endif

  }
}
#endif /* RASPBERRY_PI */

#if defined(RASPBERRY_PI) || defined(ARDUINO_ARCH_NRF52)

void parseUISettings(JsonObject& root)
{
  JsonVariant units = root["units"];
  if (units.success()) {
    const char * units_s = units.as<char*>();
    if (!strcmp(units_s,"METRIC")) {
      ui_settings.units = UNITS_METRIC;
    } else if (!strcmp(units_s,"IMPERIAL")) {
      ui_settings.units = UNITS_IMPERIAL;
    } else if (!strcmp(units_s,"MIXED")) {
      ui_settings.units = UNITS_MIXED;
    }
  }

  JsonVariant zoom = root["zoom"];
  if (zoom.success()) {
    const char * zoom_s = zoom.as<char*>();
    if (!strcmp(zoom_s,"LOWEST")) {
      ui_settings.zoom = ZOOM_LOWEST;
    } else if (!strcmp(zoom_s,"LOW")) {
      ui_settings.zoom = ZOOM_LOW;
    } else if (!strcmp(zoom_s,"MEDIUM")) {
      ui_settings.zoom = ZOOM_MEDIUM;
    } else if (!strcmp(zoom_s,"HIGH")) {
      ui_settings.zoom = ZOOM_HIGH;
    }
  }
#if 0
  JsonVariant protocol = root["data"];
  if (protocol.success()) {
    const char * protocol_s = protocol.as<char*>();
    if (!strcmp(protocol_s,"NMEA")) {
      ui_settings.protocol = PROTOCOL_NMEA;
    } else if (!strcmp(protocol_s,"GDL90")) {
      ui_settings.protocol = PROTOCOL_GDL90;
    } else if (!strcmp(protocol_s,"MAV1")) {
      ui_settings.protocol = PROTOCOL_MAVLINK_1;
    } else if (!strcmp(protocol_s,"MAV2")) {
      ui_settings.protocol = PROTOCOL_MAVLINK_2;
    } else if (!strcmp(protocol_s,"D1090")) {
      ui_settings.protocol = PROTOCOL_D1090;
    }
  }
#endif
  JsonVariant orientation = root["orientation"];
  if (orientation.success()) {
    const char * orientation_s = orientation.as<char*>();
    if (!strcmp(orientation_s,"TRACK")) {
      ui_settings.orientation = DIRECTION_TRACK_UP;
    } else if (!strcmp(orientation_s,"NORTH")) {
      ui_settings.orientation = DIRECTION_NORTH_UP;
    }
  }

  JsonVariant vmode = root["vmode"];
  if (vmode.success()) {
    const char * vmode_s = vmode.as<char*>();
    if (!strcmp(vmode_s,"STATUS")) {
      ui_settings.vmode = VIEW_MODE_STATUS;
    } else if (!strcmp(vmode_s,"RADAR")) {
      ui_settings.vmode = VIEW_MODE_RADAR;
    } else if (!strcmp(vmode_s,"TEXT")) {
      ui_settings.vmode = VIEW_MODE_TEXT;
    } else if (!strcmp(vmode_s,"BARO")) {
      ui_settings.vmode = VIEW_MODE_BARO;
    } else if (!strcmp(vmode_s,"TIME")) {
      ui_settings.vmode = VIEW_MODE_TIME;
    }
  }

  JsonVariant aghost = root["aghost"];
  if (aghost.success()) {
    const char * aghost_s = aghost.as<char*>();
    if (!strcmp(aghost_s,"OFF")) {
      ui_settings.aghost = ANTI_GHOSTING_OFF;
    } else if (!strcmp(aghost_s,"2MIN")) {
      ui_settings.aghost = ANTI_GHOSTING_2MIN;
    } else if (!strcmp(aghost_s,"5MIN")) {
      ui_settings.aghost = ANTI_GHOSTING_5MIN;
    } else if (!strcmp(aghost_s,"10MIN")) {
      ui_settings.aghost = ANTI_GHOSTING_10MIN;
    }
  }

  JsonVariant filter = root["filter"];
  if (filter.success()) {
    const char * filter_s = filter.as<char*>();
    if (!strcmp(filter_s,"OFF")) {
      ui_settings.filter = TRAFFIC_FILTER_OFF;
    } else if (!strcmp(filter_s,"500M")) {
      ui_settings.filter = TRAFFIC_FILTER_500M;
    } else if (!strcmp(filter_s,"1500M")) {
      ui_settings.filter = TRAFFIC_FILTER_1500M;
    }
  }

  JsonVariant team = root["team"];
  if (team.success()) {
    uint32_t team_32 = team.as<unsigned int>();
    ui_settings.team = team_32;
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
    } else if (!strcmp(mode_s,"TEST")) {
      eeprom_block.field.settings.mode = SOFTRF_MODE_TXRX_TEST;
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
    } else if (!strcmp(protocol_s,"UAT")) {
      eeprom_block.field.settings.rf_protocol = RF_PROTOCOL_ADSB_UAT;
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
    } else if (!strcmp(band_s,"IL")) {
      eeprom_block.field.settings.band = RF_BAND_IL;
    } else if (!strcmp(band_s,"KR")) {
      eeprom_block.field.settings.band = RF_BAND_KR;
    }
  }

  JsonVariant aircraft_type = root["aircraft_type"];
  if (aircraft_type.success()) {
    const char * aircraft_type_s = aircraft_type.as<char*>();
    if (!strcmp(aircraft_type_s,"GLIDER")) {
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
    } else if (!strcmp(aircraft_type_s,"STATIC")) {
      eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_STATIC;
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

  JsonVariant fcor = root["fcor"];
  if (fcor.success()) {
    int fc = fcor.as<signed int>();
    if (fc > 30) {
      fc = 30;
    } else if (fc < -30) {
      fc = -30;
    };
    eeprom_block.field.settings.freq_corr = fc;
  }
}

#endif /* RASPBERRY_PI || ARDUINO_ARCH_NRF52 */
