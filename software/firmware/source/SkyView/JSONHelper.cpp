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

#if defined(RASPBERRY_PI)

#include <ArduinoJson.h>

#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "JSONHelper.h"

StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;

extern eeprom_t eeprom_block;
extern settings_t *settings;

void parseSettings(JsonObject& root)
{
  JsonVariant adapter = root["adapter"];
  if (adapter.success()) {
    const char * adapter_s = adapter.as<char*>();
    if (!strcmp(adapter_s,"HAT27")) {
      eeprom_block.field.settings.adapter = ADAPTER_WAVESHARE_PI_HAT_2_7;
    } else if (!strcmp(adapter_s,"OLED")) {
      eeprom_block.field.settings.adapter = ADAPTER_OLED;
    }
  }

  JsonVariant connection = root["connection"];
  if (connection.success()) {
    const char * connection_s = connection.as<char*>();
    if (!strcmp(connection_s,"SERIAL")) {
      eeprom_block.field.settings.connection = CON_SERIAL;
    } else if (!strcmp(connection_s,"UDP")) {
      eeprom_block.field.settings.connection = CON_WIFI_UDP;
    } else if (!strcmp(connection_s,"TCP")) {
      eeprom_block.field.settings.connection = CON_WIFI_TCP;
    } else if (!strcmp(connection_s,"SPP")) {
      eeprom_block.field.settings.connection = CON_BLUETOOTH_SPP;
    } else if (!strcmp(connection_s,"LE")) {
      eeprom_block.field.settings.connection = CON_BLUETOOTH_LE;
    }
  }

  JsonVariant baudrate = root["baudrate"];
  if (baudrate.success()) {
    const char * baudrate_s = baudrate.as<char*>();
    if (!strcmp(baudrate_s,"4800")) {
      eeprom_block.field.settings.baudrate = B4800;
    } else if (!strcmp(baudrate_s,"9600")) {
      eeprom_block.field.settings.baudrate = B9600;
    } else if (!strcmp(baudrate_s,"19200")) {
      eeprom_block.field.settings.baudrate = B19200;
    } else if (!strcmp(baudrate_s,"38400")) {
      eeprom_block.field.settings.baudrate = B38400;
    } else if (!strcmp(baudrate_s,"57600")) {
      eeprom_block.field.settings.baudrate = B57600;
    } else if (!strcmp(baudrate_s,"115200")) {
      eeprom_block.field.settings.baudrate = B115200;
    } else if (!strcmp(baudrate_s,"2000000")) {
      eeprom_block.field.settings.baudrate = B2000000;
    }
  }

  JsonVariant protocol = root["protocol"];
  if (protocol.success()) {
    const char * protocol_s = protocol.as<char*>();
    if (!strcmp(protocol_s,"NMEA")) {
      eeprom_block.field.settings.protocol = PROTOCOL_NMEA;
    } else if (!strcmp(protocol_s,"GDL90")) {
      eeprom_block.field.settings.protocol = PROTOCOL_GDL90;
    } else if (!strcmp(protocol_s,"MAV1")) {
      eeprom_block.field.settings.protocol = PROTOCOL_MAVLINK_1;
    } else if (!strcmp(protocol_s,"MAV2")) {
      eeprom_block.field.settings.protocol = PROTOCOL_MAVLINK_2;
    } else if (!strcmp(protocol_s,"D1090")) {
      eeprom_block.field.settings.protocol = PROTOCOL_D1090;
    } else if (!strcmp(protocol_s,"UATRADIO")) {
      eeprom_block.field.settings.protocol = PROTOCOL_UATRADIO;
    }
  }

  JsonVariant orientation = root["orientation"];
  if (orientation.success()) {
    const char * orientation_s = orientation.as<char*>();
    if (!strcmp(orientation_s,"NORTH")) {
      eeprom_block.field.settings.orientation = DIRECTION_NORTH_UP;
    } else if (!strcmp(orientation_s,"TRACK")) {
      eeprom_block.field.settings.orientation = DIRECTION_TRACK_UP;
    }
  }

  JsonVariant units = root["units"];
  if (units.success()) {
    const char * units_s = units.as<char*>();
    if (!strcmp(units_s,"METRIC")) {
      eeprom_block.field.settings.units = UNITS_METRIC;
    } else if (!strcmp(units_s,"IMPERIAL")) {
      eeprom_block.field.settings.units = UNITS_IMPERIAL;
    } else if (!strcmp(units_s,"MIXED")) {
      eeprom_block.field.settings.units = UNITS_MIXED;
    }
  }

  JsonVariant vmode = root["vmode"];
  if (vmode.success()) {
    const char * vmode_s = vmode.as<char*>();
    if (!strcmp(vmode_s,"RADAR")) {
      eeprom_block.field.settings.vmode = VIEW_MODE_RADAR;
    } else if (!strcmp(vmode_s,"TABLE")) {
      eeprom_block.field.settings.vmode = VIEW_MODE_TABLE;
    } else if (!strcmp(vmode_s,"TEXT")) {
      eeprom_block.field.settings.vmode = VIEW_MODE_TEXT;
    }
  }

  JsonVariant zoom = root["zoom"];
  if (zoom.success()) {
    const char * zoom_s = zoom.as<char*>();
    if (!strcmp(zoom_s,"MEDIUM")) {
      eeprom_block.field.settings.zoom = ZOOM_MEDIUM;
    } else if (!strcmp(zoom_s,"HIGH")) {
      eeprom_block.field.settings.zoom = ZOOM_HIGH;
    } else if (!strcmp(zoom_s,"LOW")) {
      eeprom_block.field.settings.zoom = ZOOM_LOW;
    } else if (!strcmp(zoom_s,"LOWEST")) {
      eeprom_block.field.settings.zoom = ZOOM_LOWEST;
    }
  }

  JsonVariant adb = root["adb"];
  if (adb.success()) {
    const char * adb_s = adb.as<char*>();
    if (!strcmp(adb_s,"AUTO")) {
      eeprom_block.field.settings.adb = DB_AUTO;
    } else if (!strcmp(adb_s,"NONE")) {
      eeprom_block.field.settings.adb = DB_NONE;
    } else if (!strcmp(adb_s,"FLN")) {
      eeprom_block.field.settings.adb = DB_FLN;
    } else if (!strcmp(adb_s,"OGN")) {
      eeprom_block.field.settings.adb = DB_OGN;
    } else if (!strcmp(adb_s,"ICAO")) {
      eeprom_block.field.settings.adb = DB_ICAO;
    }
  }

  JsonVariant idpref = root["idpref"];
  if (idpref.success()) {
    const char * idpref_s = idpref.as<char*>();
    if (!strcmp(idpref_s,"REG")) {
      eeprom_block.field.settings.idpref = ID_REG;
    } else if (!strcmp(idpref_s,"TAIL")) {
      eeprom_block.field.settings.idpref = ID_TAIL;
    } else if (!strcmp(idpref_s,"MAM")) {
      eeprom_block.field.settings.idpref = ID_MAM;
    }
  }

  JsonVariant voice = root["voice"];
  if (voice.success()) {
    const char * voice_s = voice.as<char*>();
    if (!strcmp(voice_s,"OFF")) {
      eeprom_block.field.settings.voice = VOICE_OFF;
    } else if (!strcmp(voice_s,"VOICE1")) {
      eeprom_block.field.settings.voice = VOICE_1;
    } else if (!strcmp(voice_s,"VOICE2")) {
      eeprom_block.field.settings.voice = VOICE_2;
    } else if (!strcmp(voice_s,"VOICE3")) {
      eeprom_block.field.settings.voice = VOICE_3;
    }
  }

  JsonVariant aghost = root["aghost"];
  if (aghost.success()) {
    const char * aghost_s = aghost.as<char*>();
    if (!strcmp(aghost_s,"OFF")) {
      eeprom_block.field.settings.aghost = ANTI_GHOSTING_OFF;
    } else if (!strcmp(aghost_s,"AUTO")) {
      eeprom_block.field.settings.aghost = ANTI_GHOSTING_AUTO;
    } else if (!strcmp(aghost_s,"2MIN")) {
      eeprom_block.field.settings.aghost = ANTI_GHOSTING_2MIN;
    } else if (!strcmp(aghost_s,"5MIN")) {
      eeprom_block.field.settings.aghost = ANTI_GHOSTING_5MIN;
    } else if (!strcmp(aghost_s,"10MIN")) {
      eeprom_block.field.settings.aghost = ANTI_GHOSTING_10MIN;
    }
  }

  JsonVariant filter = root["filter"];
  if (filter.success()) {
    const char * filter_s = filter.as<char*>();
    if (!strcmp(filter_s,"OFF")) {
      eeprom_block.field.settings.filter = TRAFFIC_FILTER_OFF;
    } else if (!strcmp(filter_s,"500M")) {
      eeprom_block.field.settings.filter = TRAFFIC_FILTER_500M;
    } else if (!strcmp(filter_s,"1500M")) {
      eeprom_block.field.settings.filter = TRAFFIC_FILTER_1500M;
    }
  }

  JsonVariant team = root["team"];
  if (team.success()) {
    const char * team_s = team.as<char*>();
    eeprom_block.field.settings.team = strtol(team_s, NULL, 0); /* try to autodetect base */
  }
}

#endif /* RASPBERRY_PI */
