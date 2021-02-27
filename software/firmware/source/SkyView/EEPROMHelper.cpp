/*
 * EEPROMHelper.cpp
 * Copyright (C) 2019-2021 Linar Yusupov
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

#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "EPDHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"

#include "SkyView.h"

// start reading from the first byte (address 0) of the EEPROM

eeprom_t eeprom_block;
settings_t *settings;

void EEPROM_setup()
{
  if (!SoC->EEPROM_begin(sizeof(eeprom_t)))
  {
    Serial.print(F("ERROR: Failed to initialize "));
    Serial.print(sizeof(eeprom_t));
    Serial.println(F(" bytes of EEPROM!"));
    Serial.flush();
    delay(1000000);
  }

  for (int i=0; i<sizeof(eeprom_t); i++) {
    eeprom_block.raw[i] = EEPROM.read(i);  
  }

  if (eeprom_block.field.magic != SKYVIEW_EEPROM_MAGIC) {
    Serial.println(F("WARNING! User defined settings are not initialized yet. Loading defaults..."));

    EEPROM_defaults();
  } else {
    Serial.print(F("EEPROM version: "));
    Serial.println(eeprom_block.field.version);

    if (eeprom_block.field.version != SKYVIEW_EEPROM_VERSION) {
      Serial.println(F("WARNING! Version mismatch of user defined settings. Loading defaults..."));

      EEPROM_defaults();
    }
  }
  settings = &eeprom_block.field.settings;
}

void EEPROM_defaults()
{
  eeprom_block.field.magic                    = SKYVIEW_EEPROM_MAGIC;
  eeprom_block.field.version                  = SKYVIEW_EEPROM_VERSION;

  if (SoC->id == SOC_ESP8266)
    eeprom_block.field.settings.adapter       = ADAPTER_NODEMCU;
  else
#if defined(BUILD_SKYVIEW_HD)
    eeprom_block.field.settings.adapter       = ADAPTER_TTGO_T5_4_7;
#else
    eeprom_block.field.settings.adapter       = ADAPTER_TTGO_T5S;
#endif

  eeprom_block.field.settings.connection      = CON_SERIAL;
  eeprom_block.field.settings.baudrate        = B38400;
  eeprom_block.field.settings.protocol        = PROTOCOL_NMEA;
  eeprom_block.field.settings.orientation     = DIRECTION_NORTH_UP;

  strcpy(eeprom_block.field.settings.server,    DEFAULT_AP_SSID);
  strcpy(eeprom_block.field.settings.key,       DEFAULT_AP_PSK);

  eeprom_block.field.settings.units           = UNITS_METRIC;
  eeprom_block.field.settings.vmode           = VIEW_MODE_RADAR;
  eeprom_block.field.settings.zoom            = ZOOM_MEDIUM;
  eeprom_block.field.settings.adb             = DB_NONE;
  eeprom_block.field.settings.idpref          = ID_REG;
  eeprom_block.field.settings.voice           = VOICE_1;
  eeprom_block.field.settings.aghost          = ANTI_GHOSTING_OFF;

  eeprom_block.field.settings.filter          = TRAFFIC_FILTER_OFF;
  eeprom_block.field.settings.power_save      = POWER_SAVE_NONE;
  eeprom_block.field.settings.team            = 0;
}

void EEPROM_store()
{
  for (int i=0; i<sizeof(eeprom_t); i++) {
    EEPROM.write(i, eeprom_block.raw[i]);  
  }

  EEPROM.commit();
}