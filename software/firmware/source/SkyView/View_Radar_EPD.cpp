/*
 * View_Radar_EPD.cpp
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

#include <Fonts/Picopixel.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSerifBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>

#include <TimeLib.h>

#include "TrafficHelper.h"
#include "BatteryHelper.h"
#include "EEPROMHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"

#include "SkyView.h"

static navbox_t navbox1;
static navbox_t navbox2;
static navbox_t navbox3;
static navbox_t navbox4;

static int EPD_zoom = ZOOM_MEDIUM;

static void EPD_Draw_NavBoxes()
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  uint16_t top_navboxes_x = navbox1.x;
  uint16_t top_navboxes_y = navbox1.y;
  uint16_t top_navboxes_w = navbox1.width + navbox2.width;
  uint16_t top_navboxes_h = maxof2(navbox1.height, navbox2.height);

  {
    display->fillRect(top_navboxes_x, top_navboxes_y,
                      top_navboxes_w, top_navboxes_h,
                      GxEPD_WHITE);

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

  uint16_t bottom_navboxes_x = navbox3.x;
  uint16_t bottom_navboxes_y = navbox3.y;
  uint16_t bottom_navboxes_w = navbox3.width + navbox4.width;
  uint16_t bottom_navboxes_h = maxof2(navbox3.height, navbox4.height);

  {
    display->fillRect(bottom_navboxes_x, bottom_navboxes_y,
                      bottom_navboxes_w, bottom_navboxes_h,
                      GxEPD_WHITE);

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

    display->setCursor(navbox3.x + 10, navbox3.y + 30);

    if (settings->units == UNITS_METRIC || settings->units == UNITS_MIXED) {
      display->print(navbox3.value == ZOOM_LOWEST ? "20 KM" :
                     navbox3.value == ZOOM_LOW    ? "10 KM" :
                     navbox3.value == ZOOM_MEDIUM ? " 4 KM" :
                     navbox3.value == ZOOM_HIGH   ? " 2 KM" : "");
    } else {
      display->print(navbox3.value == ZOOM_LOWEST ? "10 NM" :
                     navbox3.value == ZOOM_LOW    ? " 5 NM" :
                     navbox3.value == ZOOM_MEDIUM ? " 2 NM" :
                     navbox3.value == ZOOM_HIGH   ? " 1 NM" : "");
    }

    display->setFont(&FreeMonoBold18pt7b);

    display->setCursor(navbox4.x + 15, navbox4.y + 32);
    display->print((float) navbox4.value / 10);
  }
}

void EPD_radar_Draw_Message(const char *msg1, const char *msg2)
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x, y;

  if (msg1 != NULL && strlen(msg1) != 0) {
    uint16_t radar_x = 0;
    uint16_t radar_y = (display->height() - display->width()) / 2;
    uint16_t radar_w = display->width();

    display->setFont(&FreeMonoBold18pt7b);

    {
      display->fillRect(radar_x, radar_y, radar_w, radar_w, GxEPD_WHITE);

      if (msg2 == NULL) {
        display->getTextBounds(msg1, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = (radar_w - tbw) / 2;
        y = radar_y + (radar_w + tbh) / 2;
        display->setCursor(x, y);
        display->print(msg1);
      } else {
        display->getTextBounds(msg1, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = (radar_w - tbw) / 2;
        y = radar_y + radar_w / 2 - tbh;
        display->setCursor(x, y);
        display->print(msg1);

        display->getTextBounds(msg2, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = (radar_w - tbw) / 2;
        y = radar_y + radar_w / 2 + tbh;
        display->setCursor(x, y);
        display->print(msg2);
      }
    }
  }
}

static void EPD_Draw_Radar()
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x;
  uint16_t y;
  char cog_text[6];

  /* divider is a half of full scale */
  int32_t divider = 2000; 

  display->setFont(&FreeMono9pt7b);
  display->getTextBounds("N", 0, 0, &tbx, &tby, &tbw, &tbh);

  uint16_t radar_x = 0;
  uint16_t radar_y = (display->height() - display->width()) / 2;
  uint16_t radar_w = display->width();

  display->fillRect(radar_x, radar_y, radar_w, radar_w, GxEPD_WHITE);

  uint16_t radar_center_x = radar_w / 2;
  uint16_t radar_center_y = radar_y + radar_w / 2;
  uint16_t radius = radar_w / 2 - 2;

  if (settings->units == UNITS_METRIC || settings->units == UNITS_MIXED) {
    switch(EPD_zoom)
    {
    case ZOOM_LOWEST:
      divider = 10000; /* 20 KM */
      break;
    case ZOOM_LOW:
      divider =  5000; /* 10 KM */
      break;
    case ZOOM_HIGH:
      divider =  1000; /*  2 KM */
      break;
    case ZOOM_MEDIUM:
    default:
      divider =  2000;  /* 4 KM */
      break;
    }
  } else {
    switch(EPD_zoom)
    {
    case ZOOM_LOWEST:
      divider = 9260;  /* 10 NM */
      break;
    case ZOOM_LOW:
      divider = 4630;  /*  5 NM */
      break;
    case ZOOM_HIGH:
      divider =  926;  /*  1 NM */
      break;
    case ZOOM_MEDIUM:  /*  2 NM */
    default:
      divider = 1852;
      break;
    }
  }

  {
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].ID && (now() - Container[i].timestamp) <= EPD_EXPIRATION_TIME) {

        float rel_x;
        float rel_y;
        float distance;
        float bearing;

        bool isTeam = (Container[i].ID == settings->team) ;

#if 0
        Serial.print(F(" ID="));
        Serial.print((Container[i].ID >> 16) & 0xFF, HEX);
        Serial.print((Container[i].ID >>  8) & 0xFF, HEX);
        Serial.print((Container[i].ID      ) & 0xFF, HEX);
        Serial.println();

        Serial.print(F(" RelativeNorth=")); Serial.println(Container[i].RelativeNorth);
        Serial.print(F(" RelativeEast="));  Serial.println(Container[i].RelativeEast);
#endif
        switch (settings->orientation)
        {
        case DIRECTION_NORTH_UP:
          rel_x = Container[i].RelativeEast;
          rel_y = Container[i].RelativeNorth;
          break;
        case DIRECTION_TRACK_UP:
          distance = sqrtf(Container[i].RelativeNorth * Container[i].RelativeNorth +
                           Container[i].RelativeEast  * Container[i].RelativeEast);

          bearing = atan2f(Container[i].RelativeNorth,
                           Container[i].RelativeEast) * 180.0 / PI;  /* -180 ... 180 */

          /* convert from math angle into course relative to north */
          bearing = (bearing <= 90.0 ? 90.0 - bearing :
                                      450.0 - bearing);

          bearing -= ThisAircraft.Track;

          rel_x = distance * sin(radians(bearing));
          rel_y = distance * cos(radians(bearing));

          break;
        default:
          /* TBD */
          break;
        }

        int16_t x = constrain((rel_x * radius) / divider, -32768, 32767);
        int16_t y = constrain((rel_y * radius) / divider, -32768, 32767);

        if        (Container[i].RelativeVertical >   EPD_RADAR_V_THRESHOLD) {
          if (isTeam) {
            display->drawTriangle(radar_center_x + x - 5, radar_center_y - y + 4,
                                  radar_center_x + x    , radar_center_y - y - 6,
                                  radar_center_x + x + 5, radar_center_y - y + 4,
                                  GxEPD_BLACK);
            display->drawTriangle(radar_center_x + x - 6, radar_center_y - y + 5,
                                  radar_center_x + x    , radar_center_y - y - 7,
                                  radar_center_x + x + 6, radar_center_y - y + 5,
                                  GxEPD_BLACK);
          } else {
            display->fillTriangle(radar_center_x + x - 4, radar_center_y - y + 3,
                                  radar_center_x + x    , radar_center_y - y - 5,
                                  radar_center_x + x + 4, radar_center_y - y + 3,
                                  GxEPD_BLACK);
          }
        } else if (Container[i].RelativeVertical < - EPD_RADAR_V_THRESHOLD) {
          if (isTeam) {
            display->drawTriangle(radar_center_x + x - 5, radar_center_y - y - 4,
                                  radar_center_x + x    , radar_center_y - y + 6,
                                  radar_center_x + x + 5, radar_center_y - y - 4,
                                  GxEPD_BLACK);
            display->drawTriangle(radar_center_x + x - 6, radar_center_y - y - 5,
                                  radar_center_x + x    , radar_center_y - y + 7,
                                  radar_center_x + x + 6, radar_center_y - y - 5,
                                  GxEPD_BLACK);
          } else {
            display->fillTriangle(radar_center_x + x - 4, radar_center_y - y - 3,
                                  radar_center_x + x    , radar_center_y - y + 5,
                                  radar_center_x + x + 4, radar_center_y - y - 3,
                                  GxEPD_BLACK);
          }
        } else {
          if (isTeam) {
            display->drawCircle(radar_center_x + x,
                                radar_center_y - y,
                                6, GxEPD_BLACK);
            display->drawCircle(radar_center_x + x,
                                radar_center_y - y,
                                7, GxEPD_BLACK);
           } else {
            display->fillCircle(radar_center_x + x,
                                radar_center_y - y,
                                5, GxEPD_BLACK);
          }
        }
      }
    }

    display->drawCircle(  radar_center_x, radar_center_y,
                          radius, GxEPD_BLACK);
    display->drawCircle(  radar_center_x, radar_center_y,
                          radius / 2, GxEPD_BLACK);

#if 0
    /* arrow tip */
    display->fillTriangle(radar_center_x - 7, radar_center_y + 5,
                          radar_center_x    , radar_center_y - 5,
                          radar_center_x + 7, radar_center_y + 5,
                          GxEPD_BLACK);
    display->fillTriangle(radar_center_x - 7, radar_center_y + 5,
                          radar_center_x    , radar_center_y + 2,
                          radar_center_x + 7, radar_center_y + 5,
                          GxEPD_WHITE);
#else
    /* little airplane */
    display->drawFastVLine(radar_center_x,      radar_center_y - 4, 14, GxEPD_BLACK);
    display->drawFastVLine(radar_center_x + 1,  radar_center_y - 4, 14, GxEPD_BLACK);

    display->drawFastHLine(radar_center_x - 8,  radar_center_y,     18, GxEPD_BLACK);
    display->drawFastHLine(radar_center_x - 10, radar_center_y + 1, 22, GxEPD_BLACK);

    display->drawFastHLine(radar_center_x - 3,  radar_center_y + 8,  8, GxEPD_BLACK);
    display->drawFastHLine(radar_center_x - 2,  radar_center_y + 9,  6, GxEPD_BLACK);
#endif

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
      x = radar_x + radar_w / 2 - radius + tbw/2;
      y = radar_y + (radar_w + tbh) / 2;
      display->setCursor(x , y);
      display->print("L");
      x = radar_x + radar_w / 2 + radius - (3 * tbw)/2;
      y = radar_y + (radar_w + tbh) / 2;
      display->setCursor(x , y);
      display->print("R");
      x = radar_x + (radar_w - tbw) / 2;
      y = radar_y + radar_w/2 + radius - tbh/2;
      display->setCursor(x , y);
      display->print("B");

      display->setFont(&FreeMonoBold9pt7b);
      snprintf(cog_text, sizeof(cog_text), "%03d", ThisAircraft.Track);
      display->getTextBounds(cog_text, 0, 0, &tbx, &tby, &tbw, &tbh);

      x = radar_x + (radar_w - tbw) / 2;
      y = radar_y + radar_w/2 - radius + (3 * tbh)/2;
      display->setCursor(x , y);
      display->print(cog_text);
      display->drawRoundRect( x - 2, y - tbh - 2,
                              tbw + 8, tbh + 6,
                              4, GxEPD_BLACK);
      break;
    default:
      /* TBD */
      break;
    }
  }
}

void EPD_radar_setup()
{
  EPD_zoom = settings->zoom;

  uint16_t radar_x = 0;
  uint16_t radar_y = (display->height() - display->width()) / 2;
  uint16_t radar_w = display->width();

  memcpy(navbox1.title, NAVBOX1_TITLE, strlen(NAVBOX1_TITLE));
  navbox1.x = 0;
  navbox1.y = 0;
  navbox1.width  = display->width() / 2;
  navbox1.height = (display->height() - display->width()) / 2;
  navbox1.value      = 0;
  navbox1.timestamp  = millis();

  memcpy(navbox2.title, NAVBOX2_TITLE, strlen(NAVBOX2_TITLE));
  navbox2.x = navbox1.width;
  navbox2.y = navbox1.y;
  navbox2.width  = navbox1.width;
  navbox2.height = navbox1.height;
  navbox2.value      = PROTOCOL_NONE;
  navbox2.timestamp  = millis();

  memcpy(navbox3.title, NAVBOX3_TITLE, strlen(NAVBOX3_TITLE));
  navbox3.x = 0;
  navbox3.y = radar_y + radar_w;
  navbox3.width  = navbox1.width;
  navbox3.height = navbox1.height;
  navbox3.value      = EPD_zoom;
  navbox3.timestamp  = millis();

  memcpy(navbox4.title, NAVBOX4_TITLE, strlen(NAVBOX4_TITLE));
  navbox4.x = navbox3.width;
  navbox4.y = navbox3.y;
  navbox4.width  = navbox3.width;
  navbox4.height = navbox3.height;
  navbox4.value      = (int) (Battery_voltage() * 10.0);
  navbox4.timestamp  = millis();
}

void EPD_radar_loop()
{
  if (isTimeToDisplay() && SoC->EPD_is_ready()) {

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
        EPD_radar_Draw_Message(NO_FIX_TEXT, NULL);
      }
    } else {
      EPD_radar_Draw_Message(NO_DATA_TEXT, NULL);
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

    navbox3.value = EPD_zoom;
    navbox4.value = (int) (Battery_voltage() * 10.0);

    EPD_Draw_NavBoxes();

    SoC->EPD_update(EPD_UPDATE_FAST);

    EPDTimeMarker = millis();
  }
}

void EPD_radar_zoom()
{
  if (EPD_zoom < ZOOM_HIGH) EPD_zoom++;
}

void EPD_radar_unzoom()
{
  if (EPD_zoom > ZOOM_LOWEST) EPD_zoom--;
}
