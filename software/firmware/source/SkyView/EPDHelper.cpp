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

#include <TimeLib.h>

#include "SoCHelper.h"
#include "TrafficHelper.h"
#include "SkyView.h"

GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> *display;

const char SkyView_text1[] = "Sky";
const char SkyView_text2[] = "View";
const char SkyView_text3[] = "Presented by";
const char SkyView_text4[] = "SoftRF project";

#define isTimeToDisplay() (millis() - EPDTimeMarker > 2000)
unsigned long EPDTimeMarker = 0;

static bool EPD_display_frontpage = false;

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

  if (rval == DISPLAY_EPD_2_7) delay(5000); /* display SkyView logo for 5 seconds */

  return rval;
}

void EPD_loop()
{
  if (hw_info.display == DISPLAY_EPD_2_7) {

    if (!EPD_display_frontpage) {
      int16_t  tbx, tby;
      uint16_t tbw, tbh;

      display->setFullWindow();;

      display->firstPage();
      do
      {
        display->fillScreen(GxEPD_WHITE);
      }
      while (display->nextPage());

      uint16_t radar_x = 0;
      uint16_t radar_y = (display->height() - display->width()) / 2;
      uint16_t radar_w = display->width();

      uint16_t top_navboxes_x = 0;
      uint16_t top_navboxes_y = 0;
      uint16_t top_navboxes_w = display->width();
      uint16_t top_navboxes_h = radar_y;

      display->setPartialWindow(top_navboxes_x, top_navboxes_y,
                                top_navboxes_w, top_navboxes_h);

      display->firstPage();
      do
      {
        display->drawRoundRect( top_navboxes_x + 2, top_navboxes_y + 2,
                                top_navboxes_w / 2 - 4, top_navboxes_h - 4,
                                4, GxEPD_BLACK);
        display->drawRoundRect( top_navboxes_x + top_navboxes_w / 2 + 2,
                                top_navboxes_y + 2,
                                top_navboxes_w / 2 - 4, top_navboxes_h - 4,
                                4, GxEPD_BLACK);
      }
      while (display->nextPage());

      uint16_t bottom_navboxes_x = 0;
      uint16_t bottom_navboxes_y = radar_y + radar_w;
      uint16_t bottom_navboxes_w = display->width();
      uint16_t bottom_navboxes_h = radar_y;


      display->setPartialWindow(bottom_navboxes_x, bottom_navboxes_y,
                                bottom_navboxes_w, bottom_navboxes_h);

      display->firstPage();
      do
      {
        display->drawRoundRect( bottom_navboxes_x + 2, bottom_navboxes_y + 2,
                                bottom_navboxes_w / 2 - 4, bottom_navboxes_h - 4,
                                4, GxEPD_BLACK);
        display->drawRoundRect( bottom_navboxes_x + bottom_navboxes_w / 2 + 2,
                                bottom_navboxes_y + 2,
                                bottom_navboxes_w / 2 - 4, bottom_navboxes_h - 4,
                                4, GxEPD_BLACK);
      }
      while (display->nextPage());

//    display->powerOff();
      display->hibernate();

      EPD_display_frontpage = true;

    } else {

      if (isTimeToDisplay()) {

        uint16_t radar_x = 0;
        uint16_t radar_y = (display->height() - display->width()) / 2;
        uint16_t radar_w = display->width();

        display->setPartialWindow(radar_x, radar_y, radar_w, radar_w);

        uint16_t radar_center_x = radar_w / 2;
        uint16_t radar_center_y = radar_y + radar_w / 2;
        uint16_t radius = radar_w / 2 - 2;

        display->firstPage();
        do
        {
          for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
            if (Container[i].ID && (now() - Container[i].timestamp) <= EPD_EXPIRATION_TIME) {
#if 0
              Serial.print(F(" ID="));
              Serial.print((Container[i].ID >> 16) & 0xFF, HEX);
              Serial.print((Container[i].ID >>  8) & 0xFF, HEX);
              Serial.print((Container[i].ID      ) & 0xFF, HEX);
              Serial.println();

              Serial.print(F(" RelativeNorth=")); Serial.println(Container[i].RelativeNorth);
              Serial.print(F(" RelativeEast="));  Serial.println(Container[i].RelativeEast);
#endif
              int16_t x = ((int32_t) Container[i].RelativeEast  * (int32_t) radius) / 2000;
              int16_t y = ((int32_t) Container[i].RelativeNorth * (int32_t) radius) / 2000;
              display->fillCircle(radar_center_x + x,
                                  radar_center_y - y,
                                  5, GxEPD_BLACK);
            }
          }

          display->drawCircle(  radar_center_x, radar_center_y,
                                radius, GxEPD_BLACK);
          display->drawCircle(  radar_center_x, radar_center_y,
                                radius / 2, GxEPD_BLACK);
          display->fillTriangle(radar_center_x - 7, radar_center_y + 5,
                                radar_center_x    , radar_center_y - 5,
                                radar_center_x + 7, radar_center_y + 5,
                                GxEPD_BLACK);
          display->fillTriangle(radar_center_x - 7, radar_center_y + 5,
                                radar_center_x    , radar_center_y + 2,
                                radar_center_x + 7, radar_center_y + 5,
                                GxEPD_WHITE);
        }
        while (display->nextPage());

        display->hibernate();

        EPDTimeMarker = millis();
      }
    }
  }
}
