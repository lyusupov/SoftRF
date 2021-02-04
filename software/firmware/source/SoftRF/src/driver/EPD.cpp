/*
 * EPDHelper.cpp
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

#include "EPD.h"
#include "LED.h"
#include "RF.h"
#include "Baro.h"

#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>

const char EPD_SoftRF_text1[] = "SoftRF";
const char EPD_SoftRF_text2[] = "and";
const char EPD_SoftRF_text3[] = "LilyGO";

const char EPD_Radio_text[]   = "RADIO   ";
const char EPD_GNSS_text[]    = "GNSS    ";
const char EPD_Display_text[] = "DISPLAY ";
const char EPD_RTC_text[]     = "RTC     ";
const char EPD_Flash_text[]   = "FLASH   ";
const char EPD_Baro_text[]    = "BARO    ";

unsigned long EPDTimeMarker = 0;

static int EPD_view_mode = 0;
bool EPD_vmode_updated = true;

volatile bool EPD_ready_to_display = false;

void EPD_Clear_Screen()
{
  while (EPD_ready_to_display) delay(100);

  display->setFullWindow();
  display->fillScreen(GxEPD_WHITE);
  display->display(false);

  EPD_POWEROFF;
}

bool EPD_setup(bool splash_screen)
{
  bool rval = false;

  int16_t  tbx1, tby1;
  uint16_t tbw1, tbh1;
  int16_t  tbx2, tby2;
  uint16_t tbw2, tbh2;
  int16_t  tbx3, tby3;
  uint16_t tbw3, tbh3;
  uint16_t x, y;

  display->init( /* 38400 */ );

  // first update should be full refresh
  display->setRotation(1);
  display->setFont(&FreeMonoBold24pt7b);
  display->setTextColor(GxEPD_BLACK);
  display->setTextWrap(false);

  display->getTextBounds(EPD_SoftRF_text1, 0, 0, &tbx1, &tby1, &tbw1, &tbh1);
  display->getTextBounds(EPD_SoftRF_text2, 0, 0, &tbx2, &tby2, &tbw2, &tbh2);
  display->getTextBounds(EPD_SoftRF_text3, 0, 0, &tbx3, &tby3, &tbw3, &tbh3);

  display->setFullWindow();

  {
    display->fillScreen(GxEPD_WHITE);

    if (hw_info.model == SOFTRF_MODEL_BADGE) {
      x = (display->width() - tbw1) / 2;
      y = (display->height() + tbh1) / 2 - tbh3;
      display->setCursor(x, y);
      display->print(EPD_SoftRF_text1);
      x = (display->width() - tbw2) / 2;
      y = (display->height() + tbh2) / 2;
      display->setCursor(x, y);
      display->print(EPD_SoftRF_text2);
      x = (display->width() - tbw3) / 2;
      y = (display->height() + tbh3) / 2 + tbh3;
      display->setCursor(x, y);
      display->print(EPD_SoftRF_text3);
    } else {
      x = (display->width() - tbw1) / 2;
      y = (display->height() + tbh1) / 2;
      display->setCursor(x, y);
      display->print(EPD_SoftRF_text1);
    }
  }

  display->display(false);

  EPD_POWEROFF;

  rval = display->epd2.probe();

  EPD_view_mode = ui->vmode;;

  EPD_status_setup();
  EPD_radar_setup();
  EPD_text_setup();
  EPD_time_setup();

  EPDTimeMarker = millis();

  return rval;
}

void EPD_info1(bool rtc, bool spiflash)
{
  switch (hw_info.display)
  {
  case DISPLAY_EPD_1_54:
  case DISPLAY_EPD_2_7:
    int16_t  tbx, tby;
    uint16_t tbw, tbh;

    uint16_t x, y;

    display->setFont(&FreeMonoBold18pt7b);
    display->getTextBounds(EPD_Radio_text, 0, 0, &tbx, &tby, &tbw, &tbh);

    display->setFullWindow();

    {
      display->fillScreen(GxEPD_WHITE);

      x = 0;
      y = (tbh + INFO_1_LINE_SPACING);

      display->setCursor(x, y);
      display->print(EPD_Radio_text);
      display->print(hw_info.rf != RF_IC_NONE ? "+" : "-");

      y += (tbh + INFO_1_LINE_SPACING);

      display->setCursor(x, y);
      display->print(EPD_GNSS_text);
      display->print(hw_info.gnss != GNSS_MODULE_NONE ? "+" : "-");

      y += (tbh + INFO_1_LINE_SPACING);

      display->setCursor(x, y);
      display->print(EPD_Display_text);
      display->print(hw_info.display != DISPLAY_NONE ? "+" : "-");

      if (hw_info.model == SOFTRF_MODEL_BADGE) {
        y += (tbh + INFO_1_LINE_SPACING);

        display->setCursor(x, y);
        display->print(EPD_RTC_text);
        display->print(rtc ? "+" : "-");

        y += (tbh + INFO_1_LINE_SPACING);

        display->setCursor(x, y);
        display->print(EPD_Flash_text);
        display->print(spiflash ? "+" : "-");

        y += (tbh + INFO_1_LINE_SPACING);

        display->setCursor(x, y);
        display->print(EPD_Baro_text);
        display->print(hw_info.baro != BARO_MODULE_NONE ? "+" : "-");
      }
    }

    display->display(false);

    EPD_POWEROFF;

    delay(4000);

  case DISPLAY_NONE:
  default:
    break;
  }
}

void EPD_loop()
{
  switch (hw_info.display)
  {
  case DISPLAY_EPD_1_54:

    if (isTimeToEPD()) {
      switch (EPD_view_mode)
      {
      case VIEW_MODE_RADAR:
        EPD_radar_loop();
        break;
      case VIEW_MODE_TEXT:
        EPD_text_loop();
        break;
      case VIEW_MODE_TIME:
        EPD_time_loop();
        break;
      case VIEW_MODE_STATUS:
      default:
        EPD_status_loop();
        break;
      }

      EPDTimeMarker = millis();
    }
    break;

  case DISPLAY_NONE:
  default:
    break;
  }
}

void EPD_fini(int reason)
{
  switch (hw_info.display)
  {
  case DISPLAY_EPD_1_54:
    while (EPD_ready_to_display) delay(100);

    EPD_Message(reason == SOFTRF_SHUTDOWN_LOWBAT ? "LOW"     : "OFF",
                reason == SOFTRF_SHUTDOWN_LOWBAT ? "BATTERY" : NULL);

    while (EPD_ready_to_display) delay(100);

    EPD_HIBERNATE;
    break;

  case DISPLAY_NONE:
  default:
    break;
  }
}

void EPD_Mode()
{
  if (hw_info.display == DISPLAY_EPD_1_54) {
    if (EPD_view_mode == VIEW_MODE_STATUS) {
      EPD_view_mode = VIEW_MODE_RADAR;
      EPD_vmode_updated = true;
    }  else if (EPD_view_mode == VIEW_MODE_RADAR) {
      EPD_view_mode = VIEW_MODE_TEXT;
      EPD_vmode_updated = true;
    }  else if (EPD_view_mode == VIEW_MODE_TEXT) {
      EPD_view_mode = VIEW_MODE_TIME;
      EPD_vmode_updated = true;
    }  else if (EPD_view_mode == VIEW_MODE_TIME) {
      EPD_view_mode = VIEW_MODE_STATUS;
      EPD_vmode_updated = true;
    }
  }
}

void EPD_Up()
{
  if (hw_info.display == DISPLAY_EPD_1_54) {
    switch (EPD_view_mode)
    {
    case VIEW_MODE_RADAR:
      EPD_radar_unzoom();
      break;
    case VIEW_MODE_TEXT:
      EPD_text_prev();
      break;
    case VIEW_MODE_TIME:
      EPD_time_prev();
      break;
    case VIEW_MODE_STATUS:
    default:
      EPD_status_prev();
      break;
    }
  }
}

void EPD_Down()
{
  if (hw_info.display == DISPLAY_EPD_1_54) {
    switch (EPD_view_mode)
    {
    case VIEW_MODE_RADAR:
      EPD_radar_zoom();
      break;
    case VIEW_MODE_TEXT:
      EPD_text_next();
      break;
    case VIEW_MODE_TIME:
      EPD_time_next();
      break;
    case VIEW_MODE_STATUS:
    default:
      EPD_status_next();
      break;
    }
  }
}

void EPD_Message(const char *msg1, const char *msg2)
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x, y;

  if (msg1 != NULL && strlen(msg1) != 0 && !EPD_ready_to_display) {

    display->setFont(&FreeMonoBold18pt7b);

    {
      display->fillScreen(GxEPD_WHITE);

      if (msg2 == NULL) {

        display->getTextBounds(msg1, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = (display->width() - tbw) / 2;
        y = (display->height() + tbh) / 2;
        display->setCursor(x, y);
        display->print(msg1);

      } else {

        display->getTextBounds(msg1, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = (display->width() - tbw) / 2;
        y = display->height() / 2 - tbh;
        display->setCursor(x, y);
        display->print(msg1);

        display->getTextBounds(msg2, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = (display->width() - tbw) / 2;
        y = display->height() / 2 + tbh;
        display->setCursor(x, y);
        display->print(msg2);
      }
    }

    /* a signal to background EPD update task */
    EPD_ready_to_display = true;
  }
}

EPD_Task_t EPD_Task( void * pvParameters )
{
  for( ;; )
  {
    if (EPD_ready_to_display) {

      display->display(true);

      yield();

      EPD_POWEROFF;

      EPD_ready_to_display = false;
    }

    yield();
  }
}

#endif /* USE_EPAPER */
