/*
 * EEPROMHelper.cpp
 * Copyright (C) 2016-2020 Linar Yusupov
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

#if defined(EXCLUDE_EEPROM)
void EEPROM_setup()    {}
void EEPROM_store()    {}
#else

#include "EEPROMHelper.h"
#include "RFHelper.h"
#include "LEDHelper.h"
#include "SoundHelper.h"
#include "BluetoothHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"
#include "D1090Helper.h"
#include "JSONHelper.h"
#include "BatteryHelper.h"

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

  if (eeprom_block.field.magic != SOFTRF_EEPROM_MAGIC) {
    Serial.println(F("WARNING! User defined settings are not initialized yet. Loading defaults..."));

    EEPROM_defaults();
  } else {
    Serial.print(F("EEPROM version: "));
    Serial.println(eeprom_block.field.version);

    if (eeprom_block.field.version != SOFTRF_EEPROM_VERSION) {
      Serial.println(F("WARNING! Version mismatch of user defined settings. Loading defaults..."));

      EEPROM_defaults();
    }
  }
  settings = &eeprom_block.field.settings;
}

void EEPROM_defaults()
{
  eeprom_block.field.magic                  = SOFTRF_EEPROM_MAGIC;
  eeprom_block.field.version                = SOFTRF_EEPROM_VERSION;
  eeprom_block.field.settings.mode          = SOFTRF_MODE_NORMAL;
  eeprom_block.field.settings.rf_protocol   = RF_PROTOCOL_OGNTP;
  eeprom_block.field.settings.band          = RF_BAND_EU;
  eeprom_block.field.settings.aircraft_type = AIRCRAFT_TYPE_GLIDER;
  eeprom_block.field.settings.txpower       = RF_TX_POWER_FULL;
  eeprom_block.field.settings.bluetooth     = BLUETOOTH_OFF;
  eeprom_block.field.settings.alarm         = TRAFFIC_ALARM_DISTANCE;

  /* This will speed up 'factory' boot sequence on Editions other than Standalone */
  if (hw_info.model == SOFTRF_MODEL_STANDALONE) {
    eeprom_block.field.settings.volume      = BUZZER_VOLUME_FULL;
    eeprom_block.field.settings.pointer     = DIRECTION_NORTH_UP;
  } else {
    eeprom_block.field.settings.volume      = BUZZER_OFF;
    eeprom_block.field.settings.pointer     = LED_OFF;
  }

  eeprom_block.field.settings.nmea_g     = true;
  eeprom_block.field.settings.nmea_p     = false;
  eeprom_block.field.settings.nmea_l     = true;
  eeprom_block.field.settings.nmea_s     = true;

#if defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB)
  eeprom_block.field.settings.nmea_out   = NMEA_BLUETOOTH;  /* STM32 USB */
#else
  eeprom_block.field.settings.nmea_out   = NMEA_UART;
#endif

  eeprom_block.field.settings.gdl90      = GDL90_OFF;
  eeprom_block.field.settings.d1090      = D1090_OFF;
  eeprom_block.field.settings.json       = JSON_OFF;
  eeprom_block.field.settings.stealth    = false;
  eeprom_block.field.settings.no_track   = false;
  eeprom_block.field.settings.power_save = POWER_SAVE_NONE;
  eeprom_block.field.settings.freq_corr  = 0;
}

void EEPROM_store()
{
  for (int i=0; i<sizeof(eeprom_t); i++) {
    EEPROM.write(i, eeprom_block.raw[i]);  
  }

  EEPROM_commit();
}

#endif /* EXCLUDE_EEPROM */
