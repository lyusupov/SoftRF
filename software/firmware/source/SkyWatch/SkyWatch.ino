/*
 * SkyWatch(.ino) firmware
 * Copyright (C) 2019-2020 Linar Yusupov
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

#include "SkyWatch.h"

ufo_t ThisDevice;
hardware_info_t hw_info = {
  .model    = SOFTRF_MODEL_SKYWATCH,
  .revision = HW_REV_UNKNOWN,
  .soc      = SOC_NONE,
  .rf       = RF_IC_NONE,
  .gnss     = GNSS_MODULE_NONE,
  .baro     = BARO_MODULE_NONE,
  .display  = DISPLAY_NONE
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

void setup()
{
  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  delay(300);
  Serial.begin(SERIAL_OUT_BR);
  Serial.println();

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

  SoC->DB_init();

  Web_setup();
  Traffic_setup();

  SoC->WDT_setup();
}

void loop()
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

void shutdown(const char *msg)
{
  SoC->WDT_fini();

  NMEA_fini();

  Web_fini();

  SoC->DB_fini();

  WiFi_fini();

  TFT_fini(msg);

  SoC->Button_fini();

  SoC_fini();
}
