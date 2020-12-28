/*
 * SkyView(.ino) firmware
 * Copyright (C) 2019-2021 Linar Yusupov
 *
 * This firmware is essential part of the SoftRF project.
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
 * Credits:
 *   Arduino core for ESP8266 is developed/supported by ESP8266 Community (support-esp8266@esp8266.com)
 *   Arduino Time Library is developed by Paul Stoffregen, http://github.com/PaulStoffregen
 *   TinyGPS++ and PString Libraries are developed by Mikal Hart
 *   Arduino core for ESP32 is developed/supported by Hristo Gochkov
 *   jQuery library is developed by JS Foundation
 *   BCM2835 C library is developed by Mike McCauley
 *   GxEPD2 library is developed by Jean-Marc Zingg
 *   Adafruit GFX library is developed by Adafruit Industries
 *   GDL90 decoder is developed by Ryan David
 *   Sqlite3 Arduino library for ESP32 is developed by Arundale Ramanathan
 *   FLN/OGN aircrafts data is courtesy of FlarmNet/GliderNet
 *   Adafruit SSD1306 library is developed by Adafruit Industries
 *   ESP32 I2S WAV player example is developed by Tuan Nha
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
#include "EPDHelper.h"
#include "TrafficHelper.h"
#include "WiFiHelper.h"
#include "WebHelper.h"
#include "BatteryHelper.h"
#include "GDL90Helper.h"

#include "SkyView.h"

hardware_info_t hw_info = {
  .model    = SOFTRF_MODEL_SKYVIEW,
  .revision = HW_REV_UNKNOWN,
  .soc      = SOC_NONE,
  .display  = DISPLAY_NONE
};

/* Poll input source(s) */
void Input_loop() {
  switch (settings->protocol)
  {
  case PROTOCOL_GDL90:
    GDL90_loop();
    break;
  case PROTOCOL_NMEA:
  default:
    NMEA_loop();
    break;
  }
}

void setup()
{
  hw_info.soc = SoC_setup(); // Has to be very first procedure in the execution order

  delay(300);
  Serial.begin(SERIAL_OUT_BR); Serial.println();

  Serial.println();
  Serial.print(F(SKYVIEW_IDENT));
  Serial.print(SoC->name);
  Serial.print(F(" FW.REV: " SKYVIEW_FIRMWARE_VERSION " DEV.ID: "));
  Serial.println(String(SoC->getChipId(), HEX));
  Serial.println(F("Copyright (C) 2019-2021 Linar Yusupov. All rights reserved."));
  Serial.flush();

  EEPROM_setup();
  Battery_setup();
  SoC->Button_setup();

  switch (settings->protocol)
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
  if (settings->connection == CON_SERIAL &&
      settings->protocol   == PROTOCOL_NMEA) {
    SerialInput.write("$PSRFC,?*47\r\n");
    SerialInput.flush();
  }

  Serial.println();
  Serial.print(F("Intializing E-ink display module (may take up to 10 seconds)... "));
  Serial.flush();
  hw_info.display = EPD_setup(true);
  if (hw_info.display != DISPLAY_NONE) {
    Serial.println(F(" done."));
  } else {
    Serial.println(F(" failed!"));
  }

  WiFi_setup();

  SoC->DB_init();

  SoC->TTS("POST");

  Web_setup();
  Traffic_setup();

  SoC->WDT_setup();
}

void loop()
{
  if (SoC->Bluetooth) {
    SoC->Bluetooth->loop();
  }

  Input_loop();

  Traffic_loop();

  EPD_loop();

  Traffic_ClearExpired();

  WiFi_loop();

  // Handle Web
  Web_loop();

  SoC->Button_loop();

  Battery_loop();
}

void shutdown(const char *msg)
{
  SoC->WDT_fini();

  /* If a Dongle is connected - try to shut it down */
  if (settings->connection == CON_SERIAL &&
      settings->protocol   == PROTOCOL_NMEA) {
    SerialInput.write("$PSRFC,OFF*37\r\n");
    SerialInput.flush();
  }

  if (SoC->Bluetooth) {
    SoC->Bluetooth->fini();
  }

  Web_fini();

  SoC->DB_fini();

  WiFi_fini();

  EPD_fini(msg);

  SoC->Button_fini();

  SoC_fini();
}
