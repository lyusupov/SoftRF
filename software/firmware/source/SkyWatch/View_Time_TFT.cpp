/*
 * View_Time_TFT.cpp
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

#if !defined(EXCLUDE_TFT)

#include "TFTHelper.h"
#include "TrafficHelper.h"
#include "BatteryHelper.h"
#include <protocol.h>


const char NOTIME_text[] = "-- : -- : --";
const char TZ_text[]     = "UTC";

void TFT_time_setup()
{

}

void TFT_time_loop()
{
  char buf[16];
  uint32_t disp_value;

  uint16_t tbw;
  uint16_t tbh;

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

  sprite->createSprite(tft->width(), tft->height());

  sprite->fillSprite(TFT_BLACK);
  sprite->setTextColor(TFT_WHITE);

  sprite->setTextFont(4);
  sprite->setTextSize(2);

  tbw = sprite->textWidth(buf);
  tbh = sprite->fontHeight();

  sprite->setCursor((sprite->width() - tbw) / 2, (sprite->height() - tbh) / 2);
  sprite->print(buf);

  sprite->setTextSize(1);

  tbw = sprite->textWidth(TZ_text);
  tbh = sprite->fontHeight();

  sprite->setCursor((sprite->width() - tbw) / 2, (2 * sprite->height()) / 3);
  sprite->print(TZ_text);

  tft->setBitmapColor(TFT_WHITE, TFT_NAVY);
  sprite->pushSprite(0, 0);
  sprite->deleteSprite();
}

void TFT_time_next()
{

}

void TFT_time_prev()
{

}
#endif /* EXCLUDE_TFT */
