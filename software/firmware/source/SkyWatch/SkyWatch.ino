/*
 * SkyWatch(.ino) firmware
 * Copyright (C) 2019-2021 Linar Yusupov
 *
 * This firmware is essential part of the SoftRF project.
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
 * Credits:
 *   Arduino Time Library is developed by Paul Stoffregen, http://github.com/PaulStoffregen
 *   TinyGPS++ and PString Libraries are developed by Mikal Hart
 *   Arduino core for ESP32 is developed/supported by Hristo Gochkov
 *   jQuery library is developed by JS Foundation
 *   Adafruit GFX library is developed by Adafruit Industries
 *   Sqlite3 Arduino library for ESP32 is developed by Arundale Ramanathan
 *   FLN/OGN aircrafts data is courtesy of FlarmNet/GliderNet
 *   Adafruit SSD1306 library is developed by Adafruit Industries
 *   AceButton library is developed by Brian Park
 *   Flashrom library is part of the flashrom.org project
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
#include "NMEAHelper.h"
#include "TrafficHelper.h"
#include "WiFiHelper.h"
#include "WebHelper.h"
#include "BatteryHelper.h"
#include "GDL90Helper.h"
#include "TFTHelper.h"
#include "BaroHelper.h"

ufo_t ThisDevice;
hardware_info_t hw_info = {
  .model    = SOFTRF_MODEL_WEBTOP,
  .revision = HW_REV_UNKNOWN,
  .soc      = SOC_NONE,
  .rf       = RF_IC_NONE,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = BARO_MODULE_NONE,
  .display  = DISPLAY_NONE,
  .storage  = STORAGE_NONE
};

#if DEBUG_POWER
#include <axp20x.h>
extern AXP20X_Class axp;

void print_current(const char *s, bool d)
{
  char buf[128];

  float vbus_cur = axp.getVbusCurrent();
  float batt_cur = axp.isChargeing() ?
    axp.getBattChargeCurrent() : axp.getBattDischargeCurrent();

  if (d) { delay(1000); }

  Serial.println();
  Serial.println(s);
  snprintf(buf, sizeof(buf), "%.2f", vbus_cur);
  Serial.print(F("Vbus current: ")); Serial.println(buf);
  snprintf(buf, sizeof(buf), "%.2f", batt_cur);
  Serial.print(F("Battery current: ")); Serial.println(buf);
  Serial.println();
}
#endif

bool inServiceMode = false;

void setup()
{
  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  delay(300);
  Serial.begin(SERIAL_OUT_BR, SERIAL_OUT_BITS);
  Serial.println();

  Serial.println();
  Serial.print(hw_info.model == SOFTRF_MODEL_SKYWATCH ?
                                F(SKYWATCH_IDENT "-") : F(SOFTRF_IDENT "-"));
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SKYWATCH_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(F("Copyright (C) 2019-2021 Linar Yusupov. All rights reserved."));
  Serial.flush();

  EEPROM_setup();

  ThisDevice.addr = 0 /* SoC->getChipId() & 0x00FFFFFF */;
  ThisDevice.aircraft_type = settings->s.aircraft_type;
  ThisDevice.protocol = settings->s.rf_protocol;
  ThisDevice.stealth  = settings->s.stealth;
  ThisDevice.no_track = settings->s.no_track;

  Battery_setup();
  SoC->Button_setup();

  hw_info.rf = RF_IC_SX1276;
  hw_info.baro = Baro_setup();
  hw_info.display = TFT_setup();

  WiFi_setup();

  if (SoC->Bluetooth) {
     SoC->Bluetooth->setup();
  }

  if (SoC->DB_init()) {
    hw_info.storage = STORAGE_uSD;
  }

  Web_setup();
  Traffic_setup();

  Serial.flush();

  switch (settings->m.protocol)
  {
  case PROTOCOL_GDL90:
    GDL90_setup();
    break;
  case PROTOCOL_NMEA:
  default:
    NMEA_setup();
    break;
  }

  /* If a Dongle is connected - try to wake it up */
  if (settings->m.protocol == PROTOCOL_NMEA) {
    const char *on_msg = "$PSRFC,?*47\r\n";

    switch (settings->m.connection)
    {
    case CON_SERIAL_MAIN:
      SerialInput.write(on_msg);
      SerialInput.flush();
      break;
    case CON_SERIAL_AUX:
      Serial.write(on_msg);
      Serial.flush();
      break;
    case CON_NONE:
    default:
      break;
    }
  }

  SoC->WDT_setup();
}

void loop()
{
#if defined(EXPERIMENTAL)
  if (inServiceMode) {
    service_loop();
  } else
#endif /* EXPERIMENTAL */
  {
    normal_loop();
  }
}

void normal_loop()
{
  Baro_loop();

  switch (settings->m.protocol)
  {
  case PROTOCOL_GDL90:
    GDL90_loop();
    break;
  case PROTOCOL_NMEA:
  default:
    NMEA_loop();
    break;
  }

  if (SoC->Bluetooth) {
    SoC->Bluetooth->loop();
  }

  Traffic_loop();

  TFT_loop();

  Traffic_ClearExpired();

  WiFi_loop();

  // Handle Web
  Web_loop();

  SoC->Button_loop();

  SoC->loop();

  Battery_loop();

  yield();
}

#if defined(EXPERIMENTAL)
void service_loop()
{
  bool bypass_inactive = true;

  while (Serial.available() > 0) {
    SerialInput.write(Serial.read());
    bypass_inactive = false;
  }
  while (SerialInput.available() > 0) {
    Serial.write(SerialInput.read());
    bypass_inactive = false;
  }
  if (bypass_inactive) {
//    TFT_loop();

//    Traffic_ClearExpired();

//    WiFi_loop();

    // Handle Web
//    Web_loop();

    SoC->Button_loop();

    SoC->loop();

//    Battery_loop();
  }
}
#endif /* EXPERIMENTAL */

void shutdown(const char *msg)
{
  SoC->WDT_fini();

  /* If a Dongle is connected - try to shut it down */
  if (settings->m.protocol == PROTOCOL_NMEA) {
    const char *off_msg = "$PSRFC,OFF*37\r\n";

    switch (settings->m.connection)
    {
    case CON_SERIAL_MAIN:
      SerialInput.write(off_msg);
      SerialInput.flush();
      break;
    case CON_SERIAL_AUX:
      Serial.write(off_msg);
      Serial.flush();
      break;
    case CON_NONE:
    default:
      break;
    }
  }

  NMEA_fini();

  Web_fini();

  SoC->DB_fini();

  WiFi_fini();

  TFT_fini(msg);

  SoC->Button_fini();

  SoC_fini();
}
