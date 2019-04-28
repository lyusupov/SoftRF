/*
 * EPDHelper.cpp
 * Copyright (C) 2019 Linar Yusupov
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
#include <Fonts/FreeMonoBold24pt7b.h>

#include "SoCHelper.h"
#include "EEPROMHelper.h"

#include "SkyView.h"

GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> *display;

const char SkyView_text1[] = "Sky";
const char SkyView_text2[] = "View";
const char SkyView_text3[] = "Presented by";
const char SkyView_text4[] = "SoftRF project";

unsigned long EPDTimeMarker = 0;
bool EPD_display_frontpage = false;

static int  EPD_view_mode = 0;

void EPD_Clear_Screen()
{
  display->setFullWindow();

  display->firstPage();
  do
  {
    display->fillScreen(GxEPD_WHITE);
  }
  while (display->nextPage());
}

byte EPD_setup()
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

  EPD_view_mode = settings->vmode;

  SoC->EPD_setup();

  if (!display)
    return rval;

  display->init();

  // first update should be full refresh
  display->setRotation(0);
  display->setFont(&FreeMonoBold24pt7b);
  display->setTextColor(GxEPD_BLACK);

  display->getTextBounds(SkyView_text1, 0, 0, &tbx1, &tby1, &tbw1, &tbh1);
  display->getTextBounds(SkyView_text2, 0, 0, &tbx2, &tby2, &tbw2, &tbh2);
  display->setFullWindow();
  display->firstPage();
  do
  {
    display->fillScreen(GxEPD_WHITE);
    uint16_t x = (display->width() - tbw1) / 2;
    uint16_t y = (display->height() + tbh1) / 2;
    display->setCursor(x - (tbw1 / 3), y - tbh1);
    display->print(SkyView_text1);
    x = (display->width() - tbw2) / 2;
    y = (display->height() + tbh2) / 2;
    display->setCursor(x + (tbw2 / 7), y - (tbh2 - tbh1) );
    display->print(SkyView_text2);

    display->setFont(&FreeMonoOblique9pt7b);
    display->getTextBounds(SkyView_text3, 0, 0, &tbx3, &tby3, &tbw3, &tbh3);
    x = (display->width() - tbw3) / 2;
    y = (display->height() + tbh3) * 3 / 4;
    display->setCursor(x, y);
    display->print(SkyView_text3);
    display->setFont(&FreeMonoBoldOblique9pt7b);
    display->getTextBounds(SkyView_text4, 0, 0, &tbx4, &tby4, &tbw4, &tbh4);
    x = (display->width() - tbw4) / 2;
    y = ((display->height() + tbh4) * 3 / 4) + tbh3;
    display->setCursor(x, y);
    display->print(SkyView_text4);
  }
  while (display->nextPage());

//  display->powerOff();
//  display->hibernate();

  if (display->epd2.probe()) {
    rval = DISPLAY_EPD_2_7;
  }

  switch (EPD_view_mode)
  {
  case VIEW_MODE_RADAR:
    EPD_do_radar_setup();
    break;
  case VIEW_MODE_TEXT:
//      EPD_do_text_setup();
    break;
  default:
    break;
  }

  if (rval == DISPLAY_EPD_2_7) delay(5000); /* display SkyView logo for 5 seconds */

  EPDTimeMarker = millis();

  return rval;
}

void EPD_loop()
{
  if (hw_info.display == DISPLAY_EPD_2_7) {

    switch (EPD_view_mode)
    {
    case VIEW_MODE_RADAR:
      EPD_do_radar_loop();
      break;
    case VIEW_MODE_TEXT:
//      EPD_do_text_loop();
      break;
    default:
      break;
    }
  }
}

void EPD_fini()
{
  if (hw_info.display == DISPLAY_EPD_2_7) {
    display->setFullWindow();
    display->firstPage();
    do
    {
      display->fillScreen(GxEPD_WHITE);
      uint16_t x = (display->width()  - 128) / 2;
      uint16_t y = (display->height() - 128) / 2;
      display->drawBitmap(x, y, sleep_icon_128x128, 128, 128, GxEPD_BLACK);
    }
    while (display->nextPage());

    display->powerOff();
  }
}
