/*
 * JSONHelper.h
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

#ifndef JSONHELPER_H
#define JSONHELPER_H

#include <ArduinoJson.h>

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#if defined(RASPBERRY_PI)
#include <raspi/raspi.h>
#endif /* RASPBERRY_PI */

#define JSON_BUFFER_SIZE  65536
#define isValidGPSDFix() (hasValidGPSDFix)

enum
{
	JSON_OFF,
	JSON_PING
};

struct dump1090_aircraft_struct {
  const char* hex;
  const char* squawk;
  const char* flight;
  float       lat;
  float       lon;
  int         nucp;
  float       seen_pos;
  int         altitude;   // in feet
  int         vert_rate;  // in feet/minute
  int         track;      // degrees, 0-360
  int         speed;      // in knots
  int         messages;
  float       seen;
  float       rssi;
};

struct ping_aircraft_struct {
  const char* icaoAddress;
  int         trafficSource;
  float       latDD;
  float       lonDD;
  long        altitudeMM;
  int         headingDE2;
  int         horVelocityCMS;
  int         verVelocityCMS;
  int         squawk;
  int         altitudeType;
  const char* Callsign;
  int         emitterType;
  int         utcSync;
  const char* timeStamp;
};

typedef  struct dump1090_aircraft_struct dump1090_aircraft_t;
typedef  struct ping_aircraft_struct ping_aircraft_t;

extern StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
extern bool hasValidGPSDFix;

extern void JSON_Export();
extern void parseTPV(JsonObject&);
extern void parseSettings(JsonObject&);
extern void parseUISettings(JsonObject&);
extern void parseD1090(JsonObject&);
extern void parsePING(JsonObject&);
extern void parseRAW(JsonObject&);
extern byte getVal(char);

#endif /* JSONHELPER_H */