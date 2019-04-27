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

#include <Fonts/Picopixel.h>

#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoOblique9pt7b.h>
#include <Fonts/FreeMonoBoldOblique9pt7b.h>
#include <Fonts/FreeSerifBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>

#include <TimeLib.h>

#include "SoCHelper.h"
#include "TrafficHelper.h"
#include "BatteryHelper.h"
#include "EEPROMHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"

#include "SkyView.h"

GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> *display;

const char SkyView_text1[] = "Sky";
const char SkyView_text2[] = "View";
const char SkyView_text3[] = "Presented by";
const char SkyView_text4[] = "SoftRF project";

#define isTimeToDisplay() (millis() - EPDTimeMarker > 2000)
unsigned long EPDTimeMarker = 0;

static navbox_t navbox1;
static navbox_t navbox2;
static navbox_t navbox3;
static navbox_t navbox4;

static bool EPD_display_frontpage = false;

static void EPD_Clear_Screen()
{
  display->setFullWindow();

  display->firstPage();
  do
  {
    display->fillScreen(GxEPD_WHITE);
  }
  while (display->nextPage());
}

static void EPD_Draw_NavBoxes()
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  uint16_t top_navboxes_x = navbox1.x;
  uint16_t top_navboxes_y = navbox1.y;
  uint16_t top_navboxes_w = navbox1.width + navbox2.width;
  uint16_t top_navboxes_h = maxof2(navbox1.height, navbox2.height);

  display->setPartialWindow(top_navboxes_x, top_navboxes_y,
                            top_navboxes_w, top_navboxes_h);

  display->firstPage();
  do
  {
    display->drawRoundRect( navbox1.x + 1, navbox1.y + 1,
                            navbox1.width - 2, navbox1.height - 2,
                            4, GxEPD_BLACK);

    display->drawRoundRect( navbox2.x + 1, navbox2.y + 1,
                            navbox2.width - 2, navbox2.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&Picopixel);

    display->getTextBounds(navbox1.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox1.x + 5, navbox1.y + 5 + tbh);
    display->print(navbox1.title);

    display->getTextBounds(navbox2.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox2.x + 5, navbox2.y + 5 + tbh);
    display->print(navbox2.title);

    display->setFont(&FreeMonoBold18pt7b);

    display->setCursor(navbox1.x + 40, navbox1.y + 32);
    display->print(navbox1.value);

    display->setFont(&FreeSerifBold12pt7b);

    display->setCursor(navbox2.x + 8, navbox2.y + 30);
    display->print(navbox2.value == PROTOCOL_NMEA  ? "NMEA" :
                   navbox2.value == PROTOCOL_GDL90 ? " GDL" : " UNK" );
  }
  while (display->nextPage());

  uint16_t bottom_navboxes_x = navbox3.x;
  uint16_t bottom_navboxes_y = navbox3.y;
  uint16_t bottom_navboxes_w = navbox3.width + navbox4.width;
  uint16_t bottom_navboxes_h = maxof2(navbox3.height, navbox3.height);


  display->setPartialWindow(bottom_navboxes_x, bottom_navboxes_y,
                            bottom_navboxes_w, bottom_navboxes_h);

  display->firstPage();
  do
  {
    display->drawRoundRect( navbox3.x + 1, navbox3.y + 1,
                            navbox3.width - 2, navbox3.height - 2,
                            4, GxEPD_BLACK);
    display->drawRoundRect( navbox4.x + 1, navbox4.y + 1,
                            navbox4.width - 2, navbox4.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&Picopixel);

    display->getTextBounds(navbox3.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox3.x + 5, navbox3.y + 5 + tbh);
    display->print(navbox3.title);

    display->getTextBounds(navbox4.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox4.x + 5, navbox4.y + 5 + tbh);
    display->print(navbox4.title);

    display->setFont(&FreeSerifBold12pt7b);

    display->setCursor(navbox3.x + 15, navbox3.y + 30);
    display->print(navbox3.value == EPD_SCALE_2KM ? "2 KM" :
                   navbox3.value == EPD_SCALE_4KM ? "4 KM" : "8 KM" );

    display->setFont(&FreeMonoBold18pt7b);

    display->setCursor(navbox4.x + 15, navbox4.y + 32);
    display->print((float) navbox4.value / 10);
  }
  while (display->nextPage());

//display->powerOff();
  display->hibernate();
}

static void EPD_Draw_Message(const char *msg)
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  uint16_t radar_x = 0;
  uint16_t radar_y = (display->height() - display->width()) / 2;
  uint16_t radar_w = display->width();

  display->setPartialWindow(radar_x, radar_y, radar_w, radar_w);

  display->setFont(&FreeMonoBold18pt7b);

  display->getTextBounds(msg, 0, 0, &tbx, &tby, &tbw, &tbh);

  display->firstPage();
  do
  {
    display->fillScreen(GxEPD_WHITE);
    uint16_t x = (radar_w - tbw) / 2;
    uint16_t y = radar_y + (radar_w + tbh) / 2;
    display->setCursor(x, y);
    display->print(msg);
  }
  while (display->nextPage());

  display->hibernate();
}

static void EPD_Draw_Radar()
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x;
  uint16_t y;

  display->setFont(&FreeMono9pt7b);
  display->getTextBounds("N", 0, 0, &tbx, &tby, &tbw, &tbh);

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

    switch (settings->orientation)
    {
    case DIRECTION_NORTH_UP:
      x = radar_x + radar_w / 2 - radius + tbw/2;
      y = radar_y + (radar_w + tbh) / 2;
      display->setCursor(x , y);
      display->print("W");
      x = radar_x + radar_w / 2 + radius - (3 * tbw)/2;
      y = radar_y + (radar_w + tbh) / 2;
      display->setCursor(x , y);
      display->print("E");
      x = radar_x + (radar_w - tbw) / 2;
      y = radar_y + radar_w/2 - radius + (3 * tbh)/2;
      display->setCursor(x , y);
      display->print("N");
      x = radar_x + (radar_w - tbw) / 2;
      y = radar_y + radar_w/2 + radius - tbh/2;
      display->setCursor(x , y);
      display->print("S");
      break;
    case DIRECTION_TRACK_UP:
    default:
      /* TBD */
      break;
    }

  }
  while (display->nextPage());

  display->hibernate();
}

static void EPD_Update_NavBoxes()
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  if (navbox1.value != navbox1.prev_value) {

    display->setFont(&FreeMonoBold18pt7b);
    display->getTextBounds("00", 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setPartialWindow(navbox1.x + 40, navbox1.y + 33 - tbh,
                              tbw, tbh + 1);
    display->firstPage();
    do
    {
      display->fillRect(navbox1.x + 40, navbox1.y + 33 - tbh,
                        tbw, tbh + 1, GxEPD_WHITE);
      display->setCursor(navbox1.x + 40, navbox1.y + 32);
      display->print(navbox1.value);
    }
    while (display->nextPage());

    navbox1.prev_value = navbox1.value;
  }

  if (navbox2.value != navbox2.prev_value) {

    display->setFont(&FreeSerifBold12pt7b);
    display->getTextBounds("NMEA", 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setPartialWindow(navbox2.x + 8, navbox2.y + 31 - tbh,
                              tbw, tbh + 1);
    display->firstPage();
    do
    {
      display->fillRect(navbox2.x + 8, navbox2.y + 31 - tbh,
                        tbw, tbh + 1, GxEPD_WHITE);
      display->setCursor(navbox2.x + 8, navbox2.y + 30);
      display->print(navbox2.value == PROTOCOL_NMEA  ? "NMEA" :
                     navbox2.value == PROTOCOL_GDL90 ? " GDL" : " UNK" );
    }
    while (display->nextPage());

    navbox2.prev_value = navbox2.value;
  }

  if (navbox4.value != navbox4.prev_value) {

    display->setFont(&FreeMonoBold18pt7b);
    display->getTextBounds("0.0", 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setPartialWindow(navbox4.x + 16, navbox4.y + 33 - tbh,
                              tbw, tbh + 1);
    display->firstPage();
    do
    {
      display->fillRect (navbox4.x + 15, navbox4.y + 33 - tbh,
                         tbw, tbh + 1, GxEPD_WHITE);
      display->setCursor(navbox4.x + 15, navbox4.y + 32);
      display->print((float) navbox4.value / 10);
    }
    while (display->nextPage());

    navbox4.prev_value = navbox4.value;
  }

  display->hibernate();
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

  uint16_t radar_x = 0;
  uint16_t radar_y = (display->height() - display->width()) / 2;
  uint16_t radar_w = display->width();

  memcpy(navbox1.title, NAVBOX1_TITLE, strlen(NAVBOX1_TITLE));
  navbox1.x = 0;
  navbox1.y = 0;
  navbox1.width  = display->width() / 2;
  navbox1.height = (display->height() - display->width()) / 2;
  navbox1.value      = 0;
  navbox1.prev_value = navbox1.value;
  navbox1.timestamp  = millis();

  memcpy(navbox2.title, NAVBOX2_TITLE, strlen(NAVBOX2_TITLE));
  navbox2.x = navbox1.width;
  navbox2.y = navbox1.y;
  navbox2.width  = navbox1.width;
  navbox2.height = navbox1.height;
  navbox2.value      = PROTOCOL_NONE;
  navbox2.prev_value = navbox2.value;
  navbox2.timestamp  = millis();

  memcpy(navbox3.title, NAVBOX3_TITLE, strlen(NAVBOX3_TITLE));
  navbox3.x = 0;
  navbox3.y = radar_y + radar_w;
  navbox3.width  = navbox1.width;
  navbox3.height = navbox1.height;
  navbox3.value      = EPD_SCALE_4KM;
  navbox3.prev_value = navbox3.value;
  navbox3.timestamp  = millis();

  memcpy(navbox4.title, NAVBOX4_TITLE, strlen(NAVBOX4_TITLE));
  navbox4.x = navbox3.width;
  navbox4.y = navbox3.y;
  navbox4.width  = navbox3.width;
  navbox4.height = navbox3.height;
  navbox4.value      = (int) (Battery_voltage() * 10.0);
  navbox4.prev_value = navbox4.value;
  navbox4.timestamp  = millis();

  if (rval == DISPLAY_EPD_2_7) delay(5000); /* display SkyView logo for 5 seconds */

  EPDTimeMarker = millis();

  return rval;
}

void EPD_loop()
{
  if (hw_info.display == DISPLAY_EPD_2_7) {

    if (!EPD_display_frontpage) {

      EPD_Clear_Screen();
      EPD_Draw_NavBoxes();

      EPD_display_frontpage = true;

    } else {

      if (isTimeToDisplay()) {

        bool hasData = settings->protocol == PROTOCOL_NMEA  ? NMEA_isConnected()  :
                       settings->protocol == PROTOCOL_GDL90 ? GDL90_isConnected() :
                       false;

        if (hasData) {

          bool hasFix = settings->protocol == PROTOCOL_NMEA  ? isValidGNSSFix()   :
                        settings->protocol == PROTOCOL_GDL90 ? GDL90_hasOwnShip() :
                        false;

          if (hasFix) {
            EPD_Draw_Radar();
          } else {
            EPD_Draw_Message(NO_FIX_TEXT);
          }
        } else {
          EPD_Draw_Message(NO_DATA_TEXT);
        }

        navbox1.value = Traffic_Count();

        switch (settings->protocol)
        {
        case PROTOCOL_GDL90:
          navbox2.value = GDL90_hasHeartBeat() ?
                          PROTOCOL_GDL90 : PROTOCOL_NONE;
          break;
        case PROTOCOL_NMEA:
        default:
          navbox2.value = (NMEA_hasFLARM() || NMEA_hasGNSS()) ?
                          PROTOCOL_NMEA  : PROTOCOL_NONE;
          break;
        }

        navbox3.value = EPD_SCALE_4KM;
        navbox4.value = (int) (Battery_voltage() * 10.0);

        EPD_Update_NavBoxes();

        EPDTimeMarker = millis();
      }
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
