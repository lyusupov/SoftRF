/*
 * View_Text_TFT.cpp
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
#include "EEPROMHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"

#include "SkyWatch.h"

static int TFT_current = 1;

static void TFT_Draw_Text()
{
  int j=0;
  int bearing;
  char info_line [TEXT_VIEW_LINE_LENGTH];
  char id_text   [TEXT_VIEW_LINE_LENGTH];

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID && (now() - Container[i].timestamp) <= TFT_EXPIRATION_TIME) {

      traffic[j].fop = &Container[i];
      traffic[j].distance = sqrtf(Container[i].RelativeNorth * Container[i].RelativeNorth +
                                  Container[i].RelativeEast  * Container[i].RelativeEast);
      j++;
    }
  }

  if (j > 0) {

    uint8_t db;
    const char *u_dist, *u_alt, *u_spd;
    float disp_dist;
    int   disp_alt, disp_spd;

    qsort(traffic, j, sizeof(traffic_by_dist_t), traffic_cmp_by_distance);

    if (TFT_current > j) {
      TFT_current = j;
    }

    bearing = (int) atan2f(traffic[TFT_current - 1].fop->RelativeNorth,
                           traffic[TFT_current - 1].fop->RelativeEast) * 180.0 / PI;  /* -180 ... 180 */

    /* convert from math angle into course relative to north */
    bearing = (bearing <= 90 ? 90 - bearing :
                              450 - bearing);

    /* This bearing is always relative to current ground track */
//  if (settings->orientation == DIRECTION_TRACK_UP) {
      bearing -= ThisAircraft.Track;
//  }

    if (bearing < 0) {
      bearing += 360;
    }

    int oclock = ((bearing + 15) % 360) / 30;

    if (settings->m.adb == DB_AUTO) {
      switch (traffic[TFT_current - 1].fop->IDType)
      {
      case ADDR_TYPE_RANDOM:
        db = DB_OGN;
        break;
      case ADDR_TYPE_ICAO:
        db = DB_ICAO;
        break;
      case ADDR_TYPE_FLARM:
        db = DB_FLN;
        break;
      case ADDR_TYPE_ANONYMOUS:
        db = DB_OGN;
        break;
      case ADDR_TYPE_P3I:
        db = DB_ICAO;
        break;
      case ADDR_TYPE_FANET:
        db = DB_OGN;
        break;
      default:
        if (settings->m.protocol == PROTOCOL_GDL90) {
          db = DB_ICAO;
        } else {
          db = DB_FLN;
        }
        break;
      }
    } else {
      db = settings->m.adb;
    }

    switch (settings->m.units)
    {
    case UNITS_IMPERIAL:
      u_dist = "nm";
      u_alt  = "f";
      u_spd  = "kts";
      disp_dist = (traffic[TFT_current - 1].distance * _GPS_MILES_PER_METER) /
                  _GPS_MPH_PER_KNOT;
      disp_alt  = abs((int) (traffic[TFT_current - 1].fop->RelativeVertical *
                  _GPS_FEET_PER_METER));
      disp_spd  = traffic[TFT_current - 1].fop->GroundSpeed / _GPS_MPS_PER_KNOT;
      break;
    case UNITS_MIXED:
      u_dist = "km";
      u_alt  = "f";
      u_spd  = "kph";
      disp_dist = traffic[TFT_current - 1].distance / 1000.0;
      disp_alt  = abs((int) (traffic[TFT_current - 1].fop->RelativeVertical *
                  _GPS_FEET_PER_METER));
      disp_spd  = traffic[TFT_current - 1].fop->GroundSpeed * 3.6;
      break;
    case UNITS_METRIC:
    default:
      u_dist = "km";
      u_alt  = "m";
      u_spd  = "kph";
      disp_dist = traffic[TFT_current - 1].distance / 1000.0;
      disp_alt  = abs((int) traffic[TFT_current - 1].fop->RelativeVertical);
      disp_spd  = traffic[TFT_current - 1].fop->GroundSpeed * 3.6;
      break;
    }

    uint32_t id = traffic[TFT_current - 1].fop->ID;

    long start = micros();
    if (SoC->DB_query(db, id, id_text, sizeof(id_text))) {
#if 0
      Serial.print(F("Registration of "));
      Serial.print(id);
      Serial.print(F(" is "));
      Serial.println(id_text);
#endif
    } else {
      snprintf(id_text, sizeof(id_text), "ID: %06X", id);
    }
#if 0
     Serial.print(F("Time taken: "));
     Serial.println(micros()-start);
#endif

    tft->setTextFont(4);
    tft->setTextSize(1);

    {
      uint16_t x = 0;
      uint16_t y = 0;

      int16_t  tbx, tby;
      uint16_t tbw, tbh;

      tft->fillScreen(TFT_NAVY);

      snprintf(info_line, sizeof(info_line), "Traffic %d/%d", TFT_current, j);
      tbw = tft->textWidth(info_line);
      tbh = tft->fontHeight();
      x = (tft->width() - tbw) / 2;
      y += tbh / 3;
      tft->setCursor(x, y);
      tft->print(info_line);

      y += TEXT_VIEW_LINE_SPACING;

      if (oclock == 0) {
        strcpy(info_line, "   ahead");
      } else {
        snprintf(info_line, sizeof(info_line), " %2d o'clock", oclock);
      }
      tbw = tft->textWidth(info_line);
      tbh = tft->fontHeight();
      x = (tft->width() - tbw) / 2;
      y += tbh;
      tft->setCursor(x, y);
      tft->print(info_line);

      y += TEXT_VIEW_LINE_SPACING;

      snprintf(info_line, sizeof(info_line), "%4.1f %s out", disp_dist, u_dist);
      tbw = tft->textWidth(info_line);
      tbh = tft->fontHeight();
      x = (tft->width() - tbw) / 2;
      y += tbh;
      tft->setCursor(x, y);
      tft->print(info_line);

      y += TEXT_VIEW_LINE_SPACING;

      snprintf(info_line, sizeof(info_line), "%4d %s ", disp_alt, u_alt);

      if (traffic[TFT_current - 1].fop->RelativeVertical > 50) {
        strcat(info_line, "above");
      } else if (traffic[TFT_current - 1].fop->RelativeVertical < -50) {
        strcat(info_line, "below");
      } else {
        strcpy(info_line, "  same alt."); 
      }

      tbw = tft->textWidth(info_line);
      tbh = tft->fontHeight();
      x = (tft->width() - tbw) / 2;
      y += tbh;
      tft->setCursor(x, y);
      tft->print(info_line);

      y += TEXT_VIEW_LINE_SPACING;

      snprintf(info_line, sizeof(info_line), "CoG %3d deg",
               traffic[TFT_current - 1].fop->Track);
      tbw = tft->textWidth(info_line);
      tbh = tft->fontHeight();
      x = (tft->width() - tbw) / 2;
      y += tbh;
      tft->setCursor(x, y);
      tft->print(info_line);

      y += TEXT_VIEW_LINE_SPACING;

      snprintf(info_line, sizeof(info_line), "GS  %3d %s", disp_spd, u_spd);
      tbw = tft->textWidth(info_line);
      tbh = tft->fontHeight();
      x = (tft->width() - tbw) / 2;
      y += tbh;
      tft->setCursor(x, y);
      tft->print(info_line);

      y += TEXT_VIEW_LINE_SPACING;

      tbw = tft->textWidth(info_line);
      tbh = tft->fontHeight();
      x = (tft->width() - tbw) / 2;
      y += tbh;
      tft->setCursor(x, y);
      tft->print(id_text);
    }
  }
}

void TFT_text_Draw_Message(const char *msg1, const char *msg2)
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t x, y;

  if (msg1 != NULL && strlen(msg1) != 0) {

    tft->setTextFont(4);
    tft->setTextSize(2);

    {
      tft->fillScreen(TFT_NAVY);

      if (msg2 == NULL) {
        tbw = tft->textWidth(msg1);
        tbh = tft->fontHeight();
        x = (tft->width() - tbw) / 2;
        y = (tft->height() - tbh) / 2;
        tft->setCursor(x, y);
        tft->print(msg1);

      } else {
        tbw = tft->textWidth(msg1);
        tbh = tft->fontHeight();
        x = (tft->width() - tbw) / 2;
        y = tft->height() / 2 - tbh;
        tft->setCursor(x, y);
        tft->print(msg1);

        tbw = tft->textWidth(msg2);
        tbh = tft->fontHeight();
        x = (tft->width() - tbw) / 2;
        y = tft->height() / 2;
        tft->setCursor(x, y);
        tft->print(msg2);
      }
    }
  }
}

void TFT_text_setup()
{

}

void TFT_text_loop()
{
  if (!TFT_display_frontpage) {

    TFT_Clear_Screen();

    TFT_display_frontpage = true;

  } else {

    if (isTimeToDisplay()) {

      bool hasData = settings->m.protocol == PROTOCOL_NMEA  ? NMEA_isConnected()  :
                     settings->m.protocol == PROTOCOL_GDL90 ? GDL90_isConnected() :
                     false;

      if (hasData) {

        bool hasFix = settings->m.protocol == PROTOCOL_NMEA  ? isValidGNSSFix()   :
                      settings->m.protocol == PROTOCOL_GDL90 ? GDL90_hasOwnShip() :
                      false;

        if (hasFix) {
          if (Traffic_Count() > 0) {
            TFT_Draw_Text();
          } else {
            TFT_text_Draw_Message("NO", "TRAFFIC");
          }
        } else {
          TFT_text_Draw_Message(NO_FIX_TEXT, NULL);
        }
      } else {
        TFT_text_Draw_Message(NO_DATA_TEXT, NULL);
      }

      TFTTimeMarker = millis();
    }
  }
}

void TFT_text_next()
{
  if (TFT_current < MAX_TRACKING_OBJECTS) {
    TFT_current++;
  }
}

void TFT_text_prev()
{
  if (TFT_current > 1) {
    TFT_current--;
  }
}
