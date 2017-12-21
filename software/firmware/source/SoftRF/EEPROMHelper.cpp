/*
 * EEPROMHelper.cpp
 * Copyright (C) 2016-2017 Linar Yusupov
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

#include <EEPROM.h>

#include "EEPROMHelper.h"
#include "RFHelper.h"

// start reading from the first byte (address 0) of the EEPROM

eeprom_t eeprom_block;
settings_t *settings;

void EEPROM_setup()
{
  EEPROM.begin(sizeof(eeprom_t));

  for (int i=0; i<sizeof(eeprom_t); i++) {
    eeprom_block.raw[i] = EEPROM.read(i);  
  }

  if (eeprom_block.field.magic != SOFTRF_EEPROM_MAGIC) {
    Serial.println(F("Warning! EEPROM magic mismatch! Loading defaults..."));

    EEPROM_defaults();
  }
  Serial.print(F("EEPROM version: "));
  Serial.println(eeprom_block.field.version);

  settings = &eeprom_block.field.settings;
}

void EEPROM_defaults()
{
  eeprom_block.field.magic = SOFTRF_EEPROM_MAGIC;
  eeprom_block.field.version = SOFTRF_EEPROM_VERSION;
  eeprom_block.field.settings.mode = 1;
  eeprom_block.field.settings.rf_protocol = RF_PROTOCOL_LEGACY;
  eeprom_block.field.settings.band = NRF905_BAND_868;
  eeprom_block.field.settings.txpower = NRF905_PWR_10;
  eeprom_block.field.settings.volume = 2;
  eeprom_block.field.settings.nmea_g = 1;
  eeprom_block.field.settings.nmea_p = 0;
  eeprom_block.field.settings.nmea_l = 0;
  eeprom_block.field.settings.gdl90  = 0;
  eeprom_block.field.settings.d1090  = 0;
  eeprom_block.field.settings.pointer = DIRECTION_NORTH_UP;
}


void EEPROM_store()
{
  for (int i=0; i<sizeof(eeprom_t); i++) {
    EEPROM.write(i, eeprom_block.raw[i]);  
  }

  EEPROM.commit();
}