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

#include "EPDHelper.h"

#include <Fonts/FreeMonoOblique9pt7b.h>
#include <Fonts/FreeMonoBoldOblique9pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeSerif9pt7b.h>

#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "TrafficHelper.h"

#include "SkyView.h"

GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> *display;

const char EPD_SkyView_text1[] = "Sky";
const char EPD_SkyView_text2[] = "View";
const char EPD_SkyView_text3[] = "Presented by";
const char EPD_SkyView_text4[] = "SoftRF project";
const char EPD_SkyView_text5[] = "and LilyGO";
const char EPD_SkyView_text6[] = "SkyView";
const char EPD_SkyView_text7[] = "Author:";
const char EPD_SkyView_text8[] = "Linar Yusupov";
const char EPD_SkyView_text9[] = "(C) 2019-2021";

unsigned long EPDTimeMarker = 0;
bool EPD_display_frontpage = false;

static int EPD_view_mode = 0;
static unsigned long EPD_anti_ghosting_timer = 0;

volatile int EPD_task_command = EPD_UPDATE_NONE;

byte EPD_setup(bool splash_screen)
{
  byte rval = DISPLAY_NONE;
  int16_t  tbx1, tby1;
  uint16_t tbw1, tbh1;
  int16_t  tbx2, tby2;
  uint16_t tbw2, tbh2;
  int16_t  tbx3, tby3;
  uint16_t tbw3, tbh3;
  int16_t  tbx4, tby4;
  uint16_t tbw4, tbh4;
  int16_t  tbx5, tby5;
  uint16_t tbw5, tbh5;

  EPD_view_mode = settings->vmode;

  SoC->EPD_setup();

  if (!display)
    return rval;

  display->init();

  display->setRotation(0);
  display->setTextColor(GxEPD_BLACK);
  display->setFullWindow();
  display->fillScreen(GxEPD_WHITE);

  // first update should be full refresh
  if (splash_screen) {
    display->setFont(&FreeMonoBold24pt7b);

    display->getTextBounds(EPD_SkyView_text1, 0, 0, &tbx1, &tby1, &tbw1, &tbh1);
    display->getTextBounds(EPD_SkyView_text2, 0, 0, &tbx2, &tby2, &tbw2, &tbh2);

    {
      uint16_t x = (display->width() - tbw1) / 2;
      uint16_t y = (display->height() + tbh1) / 2;
      display->setCursor(x - (tbw1 / 3), y - tbh1);
      display->print(EPD_SkyView_text1);
      x = (display->width() - tbw2) / 2;
      y = (display->height() + tbh2) / 2;
      display->setCursor(x + (tbw2 / 7), y - (tbh2 - tbh1) );
      display->print(EPD_SkyView_text2);

      display->setFont(&FreeMonoOblique9pt7b);
      display->getTextBounds(EPD_SkyView_text3, 0, 0, &tbx3, &tby3, &tbw3, &tbh3);
      x = (display->width() - tbw3) / 2;
      y = (display->height() + tbh3) * 3 / 4;
      display->setCursor(x, y);
      display->print(EPD_SkyView_text3);
      display->setFont(&FreeMonoBoldOblique9pt7b);
      display->getTextBounds(EPD_SkyView_text4, 0, 0, &tbx4, &tby4, &tbw4, &tbh4);
      x = (display->width() - tbw4) / 2;
      y += tbh3;
      y += 3;
      display->setCursor(x, y);
      display->print(EPD_SkyView_text4);

      if (hw_info.revision == HW_REV_T5S_1_9 || hw_info.revision == HW_REV_T5S_2_8) {
        display->getTextBounds(EPD_SkyView_text5, 0, 0, &tbx5, &tby5, &tbw5, &tbh5);
        x = (display->width() - tbw5) / 2;
        y += tbh4;
        y += 3;
        display->setCursor(x, y);
        display->print(EPD_SkyView_text5);
      }
    }
  }

  display->display(false);

  if (display->epd2.probe()) {
    rval = DISPLAY_EPD_2_7;
  }

  EPD_radar_setup();
  EPD_text_setup();

  EPDTimeMarker = millis();
  EPD_anti_ghosting_timer = millis();

  return rval;
}

void EPD_loop()
{
  if (hw_info.display == DISPLAY_EPD_2_7) {

    switch (settings->aghost)
    {
    case ANTI_GHOSTING_2MIN:
      if (millis() - EPD_anti_ghosting_timer > 2 * 60000UL) {
        EPD_display_frontpage   = false;
        EPD_anti_ghosting_timer = millis();
      }
      break;
    case ANTI_GHOSTING_5MIN:
      if (millis() - EPD_anti_ghosting_timer > 5 * 60000UL) {
        EPD_display_frontpage   = false;
        EPD_anti_ghosting_timer = millis();
      }
      break;
    case ANTI_GHOSTING_10MIN:
      if (millis() - EPD_anti_ghosting_timer > 10 * 60000UL) {
        EPD_display_frontpage   = false;
        EPD_anti_ghosting_timer = millis();
      }
      break;
    case ANTI_GHOSTING_AUTO:
      if (millis() - EPD_anti_ghosting_timer > 5 * 60000UL &&
          Traffic_Count() == 0) {
        EPD_display_frontpage   = false;
        EPD_anti_ghosting_timer = millis();
      }
      break;
    case ANTI_GHOSTING_OFF:
    default:
      break;
    }

    if (!EPD_display_frontpage) {
      if (SoC->EPD_is_ready()) {
        display->fillScreen(GxEPD_WHITE);
        SoC->EPD_update(EPD_UPDATE_SLOW);

        EPD_display_frontpage = true;
      }
    } else {
      switch (EPD_view_mode)
      {
      case VIEW_MODE_RADAR:
        EPD_radar_loop();
        break;
      case VIEW_MODE_TEXT:
        EPD_text_loop();
        break;
      default:
        break;
      }
    }
  }
}

void EPD_fini(const char *msg)
{
  SoC->EPD_fini();

  if (hw_info.display == DISPLAY_EPD_2_7) {
    int16_t  tbx, tby;
    uint16_t tbw, tbh;

    {
      display->fillScreen(GxEPD_WHITE);
      uint16_t x = (display->width()  - 128) / 2;
      uint16_t y = (display->height() - 128) / 6;
      display->drawBitmap(x, y, sleep_icon_128x128, 128, 128, GxEPD_BLACK);

      display->setFont(&FreeMonoOblique9pt7b);
      display->getTextBounds(msg, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      display->setCursor(x, y - tbh);
      display->print(msg);

      display->setFont(&FreeMonoBold18pt7b);
      display->getTextBounds(EPD_SkyView_text6, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      y += (128 + 35);
      display->setCursor(x, y);
      display->print(EPD_SkyView_text6);

      display->setFont(&FreeMonoOblique9pt7b);
      display->getTextBounds(EPD_SkyView_text7, 0, 0, &tbx, &tby, &tbw, &tbh);
      x =  15;
      y += 32;
      display->setCursor(x, y);
      display->print(EPD_SkyView_text7);

      display->setFont(&FreeMonoBoldOblique9pt7b);
      display->getTextBounds(EPD_SkyView_text8, 0, 0, &tbx, &tby, &tbw, &tbh);
      x =  15;
      y += 15;
      display->setCursor(x, y);
      display->print(EPD_SkyView_text8);

      display->setFont(&FreeSerif9pt7b);
      display->getTextBounds(EPD_SkyView_text9, 0, 0, &tbx, &tby, &tbw, &tbh);
      x = (display->width() - tbw) / 2;
      y += 25;
      display->setCursor(x, y);
      display->print(EPD_SkyView_text9);
    }
    display->display(false);

    display->hibernate();
  }
}

void EPD_Mode()
{
  if (hw_info.display == DISPLAY_EPD_2_7) {

    if (EPD_view_mode == VIEW_MODE_RADAR) {
      EPD_view_mode = VIEW_MODE_TEXT;
      EPD_display_frontpage = false;
    }  else if (EPD_view_mode == VIEW_MODE_TEXT) {
      EPD_view_mode = VIEW_MODE_RADAR;
      EPD_display_frontpage = false;
    }
  }
}

void EPD_Up()
{
  if (hw_info.display == DISPLAY_EPD_2_7) {
    switch (EPD_view_mode)
    {
    case VIEW_MODE_RADAR:
      EPD_radar_unzoom();
      break;
    case VIEW_MODE_TEXT:
      EPD_text_prev();
      break;
    default:
      break;
    }
  }
}

void EPD_Down()
{
  if (hw_info.display == DISPLAY_EPD_2_7) {
    switch (EPD_view_mode)
    {
    case VIEW_MODE_RADAR:
      EPD_radar_zoom();
      break;
    case VIEW_MODE_TEXT:
      EPD_text_next();
      break;
    default:
      break;
    }
  }
}

void EPD_Message(const char *msg1, const char *msg2)
{
  if (hw_info.display == DISPLAY_EPD_2_7) {
    switch (EPD_view_mode)
    {
    case VIEW_MODE_RADAR:
      EPD_radar_Draw_Message(msg1, msg2);
      break;
    case VIEW_MODE_TEXT:
      EPD_text_Draw_Message(msg1, msg2);
      break;
    default:
      break;
    }
  }
}

void EPD_Update_Sync(int cmd)
{
  switch (cmd)
  {
  case EPD_UPDATE_SLOW:
    display->display(false);
    EPD_task_command = EPD_UPDATE_NONE;
    break;
  case EPD_UPDATE_FAST:
    display->display(true);
    yield();
    display->powerOff();
    EPD_task_command = EPD_UPDATE_NONE;
    break;
  case EPD_UPDATE_NONE:
  default:
    break;
  }
}

void EPD_Task( void * pvParameters )
{
  for( ;; )
  {
    if (hw_info.display == DISPLAY_EPD_2_7) {
      EPD_Update_Sync(EPD_task_command);
    }
    yield();
  }
}
