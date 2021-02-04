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
#include "../TrafficHelper.h"
#include "../driver/Battery.h"
#include <protocol.h>

#if defined(ARDUINO_ARCH_NRF52)
#include <pcf8563.h>
#endif /* ARDUINO_ARCH_NRF52 */

#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>

const char NOTIME_text[] = "--:--:--";
const char TZ_text[]     = "UTC";

void EPD_time_setup()
{

}

void EPD_time_loop()
{
  char buf[16];

  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  if (EPD_vmode_updated) {
    EPD_Clear_Screen();

    yield();

    EPD_vmode_updated = false;
  }

  if (!EPD_ready_to_display) {

#if defined(ARDUINO_ARCH_NRF52)
    RTC_Date now;

    if (rtc) {
      now = rtc->getDateTime();
    }

    if (now.year < 2019 || now.year > 2029) {
      strcpy(buf, NOTIME_text);
    } else {
      snprintf(buf, sizeof(buf), "%2d:%02d:%02d",
               now.hour, now.minute, now.second);
    }
#else
    strcpy(buf, NOTIME_text);
#endif /* ARDUINO_ARCH_NRF52 */

    display->fillScreen(GxEPD_WHITE);

    display->setFont(&FreeMonoBold18pt7b);
    display->getTextBounds(buf, 0, 0, &tbx, &tby, &tbw, &tbh);

    display->setCursor((display->width() - tbw) / 2, (display->height() - tbh) / 2);
    display->print(buf);

    display->setFont(&FreeMonoBold12pt7b);
    display->getTextBounds(TZ_text, 0, 0, &tbx, &tby, &tbw, &tbh);

    display->setCursor((display->width() - tbw) / 2, (3 * display->height()) / 4);
    display->print(TZ_text);

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
