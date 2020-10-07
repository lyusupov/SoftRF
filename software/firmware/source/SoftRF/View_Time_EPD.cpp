/*
 * View_Time_EPD.cpp
 * Copyright (C) 2019-2020 Linar Yusupov
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

#if defined(USE_EPAPER)

#if defined(ARDUINO_ARCH_NRF52)

#include "EPDHelper.h"
#include "TrafficHelper.h"
#include "BatteryHelper.h"
#include <protocol.h>
#include <pcf8563.h>

#include <Fonts/FreeMonoBold12pt7b.h>

const char NOTIME_text[] = "-- : -- : --";
const char TZ_text[]     = "UTC";

void EPD_time_setup()
{

}

void EPD_time_loop()
{
  char buf[16];

  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  RTC_Date now;

  if (rtc) {
    now = rtc->getDateTime();
  }

  if (now.year < 2019 || now.year > 2029) {
    strcpy(buf, NOTIME_text);
  } else {
    snprintf(buf, sizeof(buf), "%2d : %02d : %02d",
             now.hour, now.minute, now.second);
  }

  display->setPartialWindow(0, 0, display->width(), display->height());
  display->setFont(&FreeMonoBold12pt7b);
  display->getTextBounds(buf, 0, 0, &tbx, &tby, &tbw, &tbh);

  display->firstPage();
  do
  {
    display->fillScreen(GxEPD_WHITE);

    display->setCursor((display->width() - tbw) / 2, (display->height() - tbh) / 2);
    display->print(buf);

    display->getTextBounds(TZ_text, 0, 0, &tbx, &tby, &tbw, &tbh);

    display->setCursor((display->width() - tbw) / 2, (2 * display->height()) / 3);
    display->print(TZ_text);
  }
  while (display->nextPage());

  EPD_HIBERNATE;
}

void EPD_time_next()
{

}

void EPD_time_prev()
{

}

#endif /* ARDUINO_ARCH_NRF52 */

#endif /* USE_EPAPER */
