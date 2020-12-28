/*
 * View_Radar_TFT.cpp
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

#include <TimeLib.h>

#include "TrafficHelper.h"
#include "BatteryHelper.h"
#include "EEPROMHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"

static int TFT_zoom = ZOOM_MEDIUM;

enum {
   STATE_RVIEW_NONE,
   STATE_RVIEW_RADAR,
   STATE_RVIEW_NOFIX,
   STATE_RVIEW_NODATA
};

static int view_state_curr = STATE_RVIEW_NONE;
static int view_state_prev = STATE_RVIEW_NONE;

static void TFT_Draw_Radar()
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x;
  uint16_t y;
  char cog_text[6];

  /* divider is a half of full scale */
  int32_t divider = 2000; 

  sprite->createSprite(tft->width(), tft->height());

  sprite->fillSprite(TFT_BLACK);
  sprite->setTextColor(TFT_WHITE);

  sprite->setTextFont(4);
  sprite->setTextSize(1);

  tbw = sprite->textWidth("N");
  tbh = sprite->fontHeight();

  uint16_t radar_x = 0;
  uint16_t radar_y = 0;
  uint16_t radar_w = sprite->width();

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

  sprite->drawCircle(  radar_center_x, radar_center_y,
                        radius, TFT_WHITE);
  sprite->drawCircle(  radar_center_x, radar_center_y,
                        radius / 2, TFT_WHITE);

#if 0
  /* arrow tip */
  sprite->fillTriangle(radar_center_x - 7, radar_center_y + 5,
                        radar_center_x    , radar_center_y - 5,
                        radar_center_x + 7, radar_center_y + 5,
                        TFT_WHITE);
  sprite->fillTriangle(radar_center_x - 7, radar_center_y + 5,
                        radar_center_x    , radar_center_y + 2,
                        radar_center_x + 7, radar_center_y + 5,
                        TFT_NAVY);
#else
  /* little airplane */
  sprite->drawFastVLine(radar_center_x,      radar_center_y - 4, 14, TFT_WHITE);
  sprite->drawFastVLine(radar_center_x + 1,  radar_center_y - 4, 14, TFT_WHITE);

  sprite->drawFastHLine(radar_center_x - 8,  radar_center_y,     18, TFT_WHITE);
  sprite->drawFastHLine(radar_center_x - 10, radar_center_y + 1, 22, TFT_WHITE);

  sprite->drawFastHLine(radar_center_x - 3,  radar_center_y + 8,  8, TFT_WHITE);
  sprite->drawFastHLine(radar_center_x - 2,  radar_center_y + 9,  6, TFT_WHITE);
#endif

  switch (settings->m.orientation)
  {
  case DIRECTION_NORTH_UP:
    x = radar_x + radar_w / 2 - radius + tbw/2;
    y = radar_y + (radar_w - tbh) / 2;
    sprite->setCursor(x , y);
    sprite->print("W");
    x = radar_x + radar_w / 2 + radius - (3 * tbw)/2;
    y = radar_y + (radar_w - tbh) / 2;
    sprite->setCursor(x , y);
    sprite->print("E");
    x = radar_x + (radar_w - tbw) / 2;
    y = radar_y + radar_w/2 - radius + tbh/2;
    sprite->setCursor(x , y);
    sprite->print("N");
    x = radar_x + (radar_w - tbw) / 2;
    y = radar_y + radar_w/2 + radius - tbh;
    sprite->setCursor(x , y);
    sprite->print("S");
    break;
  case DIRECTION_TRACK_UP:
    x = radar_x + radar_w / 2 - radius + tbw/2;
    y = radar_y + (radar_w - tbh) / 2;
    sprite->setCursor(x , y);
    sprite->print("L");
    x = radar_x + radar_w / 2 + radius - (3 * tbw)/2;
    y = radar_y + (radar_w - tbh) / 2;
    sprite->setCursor(x , y);
    sprite->print("R");
    x = radar_x + (radar_w - tbw) / 2;
    y = radar_y + radar_w/2 + radius - tbh;
    sprite->setCursor(x , y);
    sprite->print("B");

    snprintf(cog_text, sizeof(cog_text), "%03d", ThisAircraft.Track);
    tbw = sprite->textWidth(cog_text);
    tbh = sprite->fontHeight();
    x = radar_x + (radar_w - tbw) / 2;
    y = radar_y + radar_w/2 - radius + tbh/2;
    sprite->setCursor(x , y);
    sprite->print(cog_text);
#if 0
    sprite->drawRoundRect( x - 2, y - tbh - 2,
                            tbw + 8, tbh + 6,
                            4, TFT_WHITE);
#endif
    break;
  default:
    /* TBD */
    break;
  }

  sprite->setTextColor(TFT_WHITE, TFT_BLACK);
  x = radar_x;
  y = radar_y + radar_w - tbh;
  sprite->setCursor(x, y);

  if (settings->m.units == UNITS_METRIC || settings->m.units == UNITS_MIXED) {
    sprite->print(TFT_zoom == ZOOM_LOWEST ? "20 KM" :
                  TFT_zoom == ZOOM_LOW    ? "10 KM" :
                  TFT_zoom == ZOOM_MEDIUM ? " 4 KM" :
                  TFT_zoom == ZOOM_HIGH   ? " 2 KM" : "");
  } else {
    sprite->print(TFT_zoom == ZOOM_LOWEST ? "10 NM" :
                  TFT_zoom == ZOOM_LOW    ? " 5 NM" :
                  TFT_zoom == ZOOM_MEDIUM ? " 2 NM" :
                  TFT_zoom == ZOOM_HIGH   ? " 1 NM" : "");
  }

  tft->setBitmapColor(TFT_WHITE, TFT_NAVY);
  sprite->pushSprite(0, 0);
  sprite->deleteSprite();

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID && (now() - Container[i].timestamp) <= TFT_EXPIRATION_TIME) {

      int16_t rel_x;
      int16_t rel_y;
      float distance;
      float bearing;

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

      uint32_t color = Container[i].AlarmLevel == ALARM_LEVEL_URGENT ? TFT_RED :
                      (Container[i].AlarmLevel == ALARM_LEVEL_IMPORTANT ?
                       TFT_YELLOW : TFT_GREEN);

      if        (Container[i].RelativeVertical >   TFT_RADAR_V_THRESHOLD) {
        tft->fillTriangle(radar_center_x + x - 4, radar_center_y - y + 3,
                          radar_center_x + x    , radar_center_y - y - 5,
                          radar_center_x + x + 4, radar_center_y - y + 3,
                          color);
      } else if (Container[i].RelativeVertical < - TFT_RADAR_V_THRESHOLD) {
        tft->fillTriangle(radar_center_x + x - 4, radar_center_y - y - 3,
                          radar_center_x + x    , radar_center_y - y + 5,
                          radar_center_x + x + 4, radar_center_y - y - 3,
                          color);
      } else {
        tft->fillCircle(radar_center_x + x,
                        radar_center_y - y,
                        5, color);
      }
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
  bool hasData = settings->m.protocol == PROTOCOL_NMEA  ? NMEA_isConnected()  :
                 settings->m.protocol == PROTOCOL_GDL90 ? GDL90_isConnected() :
                 false;

  if (hasData) {

    bool hasFix = settings->m.protocol == PROTOCOL_NMEA  ? isValidGNSSFix()   :
                  settings->m.protocol == PROTOCOL_GDL90 ? GDL90_hasOwnShip() :
                  false;

    if (hasFix) {
      view_state_curr = STATE_RVIEW_RADAR;
    } else {
      view_state_curr = STATE_RVIEW_NOFIX;
    }
  } else {
    view_state_curr = STATE_RVIEW_NODATA;
  }

  if (TFT_vmode_updated) {
    view_state_prev = STATE_RVIEW_NONE;
    TFT_vmode_updated = false;
  }

  if (view_state_curr != view_state_prev &&
      view_state_curr == STATE_RVIEW_NOFIX) {
    TFT_Clear_Screen();
    TFT_Message(NO_FIX_TEXT, NULL);
    view_state_prev = view_state_curr;
  }

  if (view_state_curr != view_state_prev &&
      view_state_curr == STATE_RVIEW_NODATA) {
    TFT_Clear_Screen();
    TFT_Message(NO_DATA_TEXT, NULL);
    view_state_prev = view_state_curr;
  }

  if (view_state_curr == STATE_RVIEW_RADAR) {
    if (view_state_curr != view_state_prev) {
       TFT_Clear_Screen();
       view_state_prev = view_state_curr;
    }
    TFT_Draw_Radar();
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
#endif /* EXCLUDE_TFT */
