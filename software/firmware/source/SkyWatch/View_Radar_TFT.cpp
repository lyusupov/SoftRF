/*
 * View_Radar_TFT.cpp
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

#include "TFTHelper.h"

#include <TimeLib.h>

#include "TrafficHelper.h"
#include "BatteryHelper.h"
#include "EEPROMHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"

#include "SkyWatch.h"

static int TFT_zoom = ZOOM_MEDIUM;

void TFT_radar_Draw_Message(const char *msg1, const char *msg2)
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x, y;

  if (msg1 != NULL && strlen(msg1) != 0) {
    uint16_t radar_x = 0;
    uint16_t radar_y = (tft->height() - tft->width()) / 2;
    uint16_t radar_w = tft->width();

    tft->setTextFont(4);
    tft->setTextSize(2);

    {
      tft->fillScreen(TFT_NAVY);

      if (msg2 == NULL) {
        tbw = tft->textWidth(msg1);
        tbh = tft->fontHeight();
        x = (radar_w - tbw) / 2;
        y = radar_y + (radar_w - tbh) / 2;
        tft->setCursor(x, y);
        tft->print(msg1);
      } else {
        tbw = tft->textWidth(msg1);
        tbh = tft->fontHeight();
        x = (radar_w - tbw) / 2;
        y = radar_y + radar_w / 2 - tbh;
        tft->setCursor(x, y);
        tft->print(msg1);

        tbw = tft->textWidth(msg2);
        tbh = tft->fontHeight();
        x = (radar_w - tbw) / 2;
        y = radar_y + radar_w / 2;
        tft->setCursor(x, y);
        tft->print(msg2);
      }
    }
  }
}

static void TFT_Draw_Radar()
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x;
  uint16_t y;
  char cog_text[6];

  /* divider is a half of full scale */
  int32_t divider = 2000; 

  tft->setTextFont(4);
  tft->setTextSize(1);

  tbw = tft->textWidth("N");
  tbh = tft->fontHeight();

  uint16_t radar_x = 0;
  uint16_t radar_y = (tft->height() - tft->width()) / 2;
  uint16_t radar_w = tft->width();

  uint16_t radar_center_x = radar_w / 2;
  uint16_t radar_center_y = radar_y + radar_w / 2;
  uint16_t radius = radar_w / 2 - 1;

  if (settings->m.units == UNITS_METRIC || settings->m.units == UNITS_MIXED) {
    switch(TFT_zoom)
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
    switch(TFT_zoom)
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
      if (Container[i].ID && (now() - Container[i].timestamp) <= TFT_EXPIRATION_TIME) {

        int16_t rel_x;
        int16_t rel_y;
        float distance;
        float bearing;
#if 0
        Serial.print(F(" ID="));
        Serial.print((Container[i].ID >> 16) & 0xFF, HEX);
        Serial.print((Container[i].ID >>  8) & 0xFF, HEX);
        Serial.print((Container[i].ID      ) & 0xFF, HEX);
        Serial.println();

        Serial.print(F(" RelativeNorth=")); Serial.println(Container[i].RelativeNorth);
        Serial.print(F(" RelativeEast="));  Serial.println(Container[i].RelativeEast);
#endif
        switch (settings->m.orientation)
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

          rel_x = constrain(distance * sin(radians(bearing)),
                                       -32768, 32767);
          rel_y = constrain(distance * cos(radians(bearing)),
                                       -32768, 32767);
          break;
        default:
          /* TBD */
          break;
        }

        int16_t x = ((int32_t) rel_x * (int32_t) radius) / divider;
        int16_t y = ((int32_t) rel_y * (int32_t) radius) / divider;

        if        (Container[i].RelativeVertical >   TFT_RADAR_V_THRESHOLD) {
          tft->fillTriangle(radar_center_x + x - 4, radar_center_y - y + 3,
                                radar_center_x + x    , radar_center_y - y - 5,
                                radar_center_x + x + 4, radar_center_y - y + 3,
                                TFT_WHITE);
        } else if (Container[i].RelativeVertical < - TFT_RADAR_V_THRESHOLD) {
          tft->fillTriangle(radar_center_x + x - 4, radar_center_y - y - 3,
                                radar_center_x + x    , radar_center_y - y + 5,
                                radar_center_x + x + 4, radar_center_y - y - 3,
                                TFT_WHITE);
        } else {
          tft->fillCircle(radar_center_x + x,
                              radar_center_y - y,
                              5, TFT_WHITE);
        }
      }
    }

    tft->drawCircle(  radar_center_x, radar_center_y,
                          radius, TFT_WHITE);
    tft->drawCircle(  radar_center_x, radar_center_y,
                          radius / 2, TFT_WHITE);

#if 0
    /* arrow tip */
    tft->fillTriangle(radar_center_x - 7, radar_center_y + 5,
                          radar_center_x    , radar_center_y - 5,
                          radar_center_x + 7, radar_center_y + 5,
                          TFT_WHITE);
    tft->fillTriangle(radar_center_x - 7, radar_center_y + 5,
                          radar_center_x    , radar_center_y + 2,
                          radar_center_x + 7, radar_center_y + 5,
                          TFT_NAVY);
#else
    /* little airplane */
    tft->drawFastVLine(radar_center_x,      radar_center_y - 4, 14, TFT_WHITE);
    tft->drawFastVLine(radar_center_x + 1,  radar_center_y - 4, 14, TFT_WHITE);

    tft->drawFastHLine(radar_center_x - 8,  radar_center_y,     18, TFT_WHITE);
    tft->drawFastHLine(radar_center_x - 10, radar_center_y + 1, 22, TFT_WHITE);

    tft->drawFastHLine(radar_center_x - 3,  radar_center_y + 8,  8, TFT_WHITE);
    tft->drawFastHLine(radar_center_x - 2,  radar_center_y + 9,  6, TFT_WHITE);
#endif

    switch (settings->m.orientation)
    {
    case DIRECTION_NORTH_UP:
      x = radar_x + radar_w / 2 - radius + tbw/2;
      y = radar_y + (radar_w - tbh) / 2;
      tft->setCursor(x , y);
      tft->print("W");
      x = radar_x + radar_w / 2 + radius - (3 * tbw)/2;
      y = radar_y + (radar_w - tbh) / 2;
      tft->setCursor(x , y);
      tft->print("E");
      x = radar_x + (radar_w - tbw) / 2;
      y = radar_y + radar_w/2 - radius + tbh/2;
      tft->setCursor(x , y);
      tft->print("N");
      x = radar_x + (radar_w - tbw) / 2;
      y = radar_y + radar_w/2 + radius - tbh;
      tft->setCursor(x , y);
      tft->print("S");
      break;
    case DIRECTION_TRACK_UP:
      x = radar_x + radar_w / 2 - radius + tbw/2;
      y = radar_y + (radar_w - tbh) / 2;
      tft->setCursor(x , y);
      tft->print("L");
      x = radar_x + radar_w / 2 + radius - (3 * tbw)/2;
      y = radar_y + (radar_w - tbh) / 2;
      tft->setCursor(x , y);
      tft->print("R");
      x = radar_x + (radar_w - tbw) / 2;
      y = radar_y + radar_w/2 + radius - tbh;
      tft->setCursor(x , y);
      tft->print("B");

      snprintf(cog_text, sizeof(cog_text), "%03d", ThisAircraft.Track);
      tbw = tft->textWidth(cog_text);
      tbh = tft->fontHeight();
      x = radar_x + (radar_w - tbw) / 2;
      y = radar_y + radar_w/2 - radius + tbh/2;
      tft->setCursor(x , y);
      tft->print(cog_text);
#if 0
      tft->drawRoundRect( x - 2, y - tbh - 2,
                              tbw + 8, tbh + 6,
                              4, TFT_WHITE);
#endif
      break;
    default:
      /* TBD */
      break;
    }

  }
}

void TFT_radar_setup()
{
  TFT_zoom = settings->m.zoom;

  uint16_t radar_x = 0;
  uint16_t radar_y = 0;
  uint16_t radar_w = tft->width();
}

void TFT_radar_loop()
{
  if (!TFT_display_frontpage) {

    TFT_Clear_Screen();

    yield();

    TFT_display_frontpage = true;

  } else {

    if (isTimeToDisplay()) {

//      TFT_Clear_Screen();

      bool hasData = settings->m.protocol == PROTOCOL_NMEA  ? NMEA_isConnected()  :
                     settings->m.protocol == PROTOCOL_GDL90 ? GDL90_isConnected() :
                     false;

      if (hasData) {

        bool hasFix = settings->m.protocol == PROTOCOL_NMEA  ? isValidGNSSFix()   :
                      settings->m.protocol == PROTOCOL_GDL90 ? GDL90_hasOwnShip() :
                      false;

        if (hasFix) {
          TFT_Draw_Radar();
        } else {
          TFT_radar_Draw_Message(NO_FIX_TEXT, NULL);
        }
      } else {
        TFT_radar_Draw_Message(NO_DATA_TEXT, NULL);
      }

      yield();

      TFTTimeMarker = millis();
    }
  }
}

void TFT_radar_zoom()
{
  if (TFT_zoom < ZOOM_HIGH) TFT_zoom++;
}

void TFT_radar_unzoom()
{
  if (TFT_zoom > ZOOM_LOWEST) TFT_zoom--;
}
