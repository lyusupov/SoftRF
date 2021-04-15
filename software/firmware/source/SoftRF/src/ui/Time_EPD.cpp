/*
 * View_Time_EPD.cpp
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

#include "../system/SoC.h"

#if defined(USE_EPAPER)

#include "../driver/EPD.h"
#include "../driver/Battery.h"

#if defined(ARDUINO_ARCH_NRF52)
#include <pcf8563.h>
#include <bluefruit.h>

extern RTC_Date fw_build_date_time;
#endif /* ARDUINO_ARCH_NRF52 */

#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMono18pt7b.h>
#include "U8g2_for_Adafruit_GFX.h"

static const char TZ_text[] = "UTC";

U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;

static const uint8_t bt_icon[] = {
  0x0F, 0xF0, 0x1D, 0x38, 0x31, 0x98, 0x31, 0xCC,
  0x6D, 0xEC, 0x6F, 0x6C, 0x67, 0xC4, 0x63, 0x84,
  0x63, 0xC4, 0x67, 0xE4, 0x6D, 0x6C, 0x61, 0xEC,
  0x31, 0xCC, 0x31, 0x98, 0x1D, 0x38, 0x0F, 0xF0,
};

void EPD_time_setup()
{
  u8g2Fonts.begin(*display); // connect u8g2 procedures to Adafruit GFX
  u8g2Fonts.setFontDirection(3);
  u8g2Fonts.setForegroundColor(GxEPD_BLACK);
  u8g2Fonts.setBackgroundColor(GxEPD_WHITE);
  u8g2Fonts.setFont(u8g2_font_battery19_tn);
}

void EPD_time_loop()
{
  char buf_hm[8];
  char buf_sec[4];

  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  if (EPD_vmode_updated) {
    EPD_Clear_Screen();

    yield();

    EPD_vmode_updated = false;
  }

  if (!EPD_ready_to_display) {

    bool ble_has_client = false;

    strcpy(buf_hm, "--:--");
    strcpy(buf_sec, "  ");

#if defined(ARDUINO_ARCH_NRF52)

    if (rtc && rtc->isVaild()) {
      RTC_Date now = rtc->getDateTime();

      if (now.year >= fw_build_date_time.year &&
          now.year <  fw_build_date_time.year + 15) {

        snprintf(buf_hm,  sizeof(buf_hm),  "%2d:%02d", now.hour, now.minute);
        snprintf(buf_sec, sizeof(buf_sec), "%02d"    , now.second);
      }
    }

    ble_has_client = Bluefruit.connected();

#endif /* ARDUINO_ARCH_NRF52 */

    display->fillScreen(GxEPD_WHITE);

    display->setFont(&FreeMonoBold12pt7b);
    display->getTextBounds(TZ_text, 0, 0, &tbx, &tby, &tbw, &tbh);

    display->setCursor((display->width() - tbw) / 2, tbh + tbh / 2);
    display->print(TZ_text);

    if (ble_has_client) {
      display->drawBitmap(display->width() - 48, 3, bt_icon, 16, 16, GxEPD_BLACK);
    }

    u8g2Fonts.setCursor(display->width() - 5, 15);
    u8g2Fonts.print(Battery_charge() / 20);

    display->setFont(&FreeMonoBold24pt7b);
    display->getTextBounds(buf_hm, 0, 0, &tbx, &tby, &tbw, &tbh);

    display->setCursor((display->width() - tbw) / 2, display->height() / 2);
    display->print(buf_hm);

    display->setFont(&FreeMono18pt7b);
    display->getTextBounds(buf_sec, 0, 0, &tbx, &tby, &tbw, &tbh);

    display->setCursor((display->width() - tbw) / 2, display->height() / 2 + tbh + tbh);
    display->print(buf_sec);

    /* a signal to background EPD update task */
    EPD_ready_to_display = true;
  }
}

void EPD_time_next()
{

}

void EPD_time_prev()
{

}

#endif /* USE_EPAPER */
