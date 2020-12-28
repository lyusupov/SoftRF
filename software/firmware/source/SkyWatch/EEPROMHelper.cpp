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
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "BatteryHelper.h"

#include <protocol.h>
#include <freqplan.h>

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

  if (eeprom_block.field.magic != SKYWATCH_EEPROM_MAGIC) {
    Serial.println(F("WARNING! User defined settings are not initialized yet. Loading defaults..."));

    EEPROM_defaults();
  } else {
    Serial.print(F("EEPROM version: "));
    Serial.println(eeprom_block.field.version);

    if (eeprom_block.field.version != SKYWATCH_EEPROM_VERSION) {
      Serial.println(F("WARNING! Version mismatch of user defined settings. Loading defaults..."));

      EEPROM_defaults();
    }
  }
  settings = &eeprom_block.field.settings;
}

void EEPROM_defaults()
{
  eeprom_block.field.magic                      = SKYWATCH_EEPROM_MAGIC;
  eeprom_block.field.version                    = SKYWATCH_EEPROM_VERSION;

  /* SoftRF defaults */
  eeprom_block.field.settings.s.mode            = SOFTRF_MODE_NORMAL;
  eeprom_block.field.settings.s.rf_protocol     = RF_PROTOCOL_OGNTP;
  eeprom_block.field.settings.s.band            = RF_BAND_EU;
  eeprom_block.field.settings.s.aircraft_type   = AIRCRAFT_TYPE_GLIDER;
  eeprom_block.field.settings.s.txpower         = RF_TX_POWER_FULL;
  eeprom_block.field.settings.s.bluetooth       = BLUETOOTH_OFF;
  eeprom_block.field.settings.s.alarm           = TRAFFIC_ALARM_DISTANCE;
  eeprom_block.field.settings.s.volume          = BUZZER_OFF;
  eeprom_block.field.settings.s.pointer         = LED_OFF;
  eeprom_block.field.settings.s.nmea_g          = true;
  eeprom_block.field.settings.s.nmea_p          = false;
  eeprom_block.field.settings.s.nmea_l          = true;
  eeprom_block.field.settings.s.nmea_s          = true;
  eeprom_block.field.settings.s.nmea_out        = NMEA_UART;
  eeprom_block.field.settings.s.gdl90           = GDL90_OFF;
  eeprom_block.field.settings.s.d1090           = D1090_OFF;
  eeprom_block.field.settings.s.json            = JSON_OFF;
  eeprom_block.field.settings.s.stealth         = false;
  eeprom_block.field.settings.s.no_track        = false;
  eeprom_block.field.settings.s.power_save      = POWER_SAVE_NONE;

  /* SkyView defaults */
  eeprom_block.field.settings.m.adapter         = ADAPTER_NONE;

  eeprom_block.field.settings.m.connection      = CON_SERIAL_MAIN;
  if (hw_info.model == SOFTRF_MODEL_SKYWATCH) {
    eeprom_block.field.settings.m.baudrate      = B115200; /* S76G AN3155 BR */
  } else {
    eeprom_block.field.settings.m.baudrate      = B38400;
  }
  eeprom_block.field.settings.m.protocol        = PROTOCOL_NMEA;
  eeprom_block.field.settings.m.orientation     = DIRECTION_NORTH_UP;

  strcpy(eeprom_block.field.settings.m.ssid,      DEFAULT_AP_SSID);
  strcpy(eeprom_block.field.settings.m.psk,       DEFAULT_AP_PSK);

  eeprom_block.field.settings.m.bluetooth       = BLUETOOTH_OFF;

  strcpy(eeprom_block.field.settings.m.bt_name,   DEFAULT_BT_NAME);
  strcpy(eeprom_block.field.settings.m.bt_key,    DEFAULT_BT_KEY);

  eeprom_block.field.settings.m.units           = UNITS_METRIC;
  eeprom_block.field.settings.m.vmode           = VIEW_MODE_STATUS;
  eeprom_block.field.settings.m.zoom            = ZOOM_MEDIUM;
  eeprom_block.field.settings.m.adb             = DB_AUTO;
  eeprom_block.field.settings.m.idpref          = ID_REG;
  eeprom_block.field.settings.m.voice           = VOICE_OFF;
  eeprom_block.field.settings.m.aghost          = ANTI_GHOSTING_OFF;
}

void EEPROM_store()
{
  for (int i=0; i<sizeof(eeprom_t); i++) {
    EEPROM.write(i, eeprom_block.raw[i]);  
  }

  EEPROM.commit();
}
