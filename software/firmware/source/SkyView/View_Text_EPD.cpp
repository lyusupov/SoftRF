/*
 * View_Text_EPD.cpp
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

#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>

#include <TimeLib.h>

#include "TrafficHelper.h"
#include "EEPROMHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"

#include "SkyView.h"

typedef struct traffic_table_struct {
  traffic_t *fop;
  float     distance;
} traffic_table_t;

static int EPD_current = 1;

static int EPD_traffic_cmp_by_distance(const void *a, const void *b)
{
  traffic_table_t *ta = (traffic_table_t *)a;
  traffic_table_t *tb = (traffic_table_t *)b;

  if (ta->distance >  tb->distance) return  1;
  if (ta->distance == tb->distance) return  0;
  if (ta->distance <  tb->distance) return -1;
}

static void EPD_Draw_Text()
{
  traffic_table_t traffic[MAX_TRACKING_OBJECTS];
  int j=0;
  int bearing;
  char info_line[32];
  char id_text[32];

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID && (now() - Container[i].timestamp) <= EPD_EXPIRATION_TIME) {

      traffic[j].fop = &Container[i];
      traffic[j].distance = sqrtf(Container[i].RelativeNorth * Container[i].RelativeNorth +
                                  Container[i].RelativeEast  * Container[i].RelativeEast);
      j++;
    }
  }

  if (j > 0) {

    uint8_t db;

    qsort(traffic, j, sizeof(traffic_table_t), EPD_traffic_cmp_by_distance);

    if (EPD_current > j) {
      EPD_current = j;
    }

    bearing = (int) atan2f(traffic[EPD_current - 1].fop->RelativeNorth,
                           traffic[EPD_current - 1].fop->RelativeEast) * 180.0 / PI;  /* -180 ... 180 */

    /* convert from math angle into course relative to north */
    bearing = (bearing <= 90 ? 90 - bearing :
                              450 - bearing);

    /* This bearing is always relative to current ground track */
//  if (settings->orientation == DIRECTION_TRACK_UP) {
      bearing -= ThisAircraft.Track;
//  }

    if (bearing < 0.0) {
      bearing += 360.0;
    }

    int oclock = ((bearing + 15) % 360) / 30;

    switch (traffic[EPD_current - 1].fop->IDType)
    {
    case ADDR_TYPE_RANDOM:
      db = DB_OGN;
      break;
    case ADDR_TYPE_ICAO:
      db = DB_PAW;
      break;
    case ADDR_TYPE_FLARM:
      db = DB_FLN;
      break;
    case ADDR_TYPE_ANONYMOUS:
      db = DB_OGN;
      break;
    case ADDR_TYPE_P3I:
      db = DB_PAW;
      break;
    case ADDR_TYPE_FANET:
      db = DB_OGN;
      break;
    default:
      if (settings->protocol == PROTOCOL_GDL90) {
        db = DB_PAW;
      } else {
        db = DB_FLN;
      }
      break;
    }

    uint32_t id = traffic[EPD_current - 1].fop->ID;

    long start = micros();
    if (SoC->DB_query(db, id, id_text, sizeof(id_text) - 1)) {
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

    display->setPartialWindow(0, 0, display->width(), display->height());

    display->setFont(&FreeMonoBold12pt7b);

    display->firstPage();
    do
    {
      uint16_t x = 5;
      uint16_t y = 5;

      int16_t  tbx, tby;
      uint16_t tbw, tbh;

      display->fillScreen(GxEPD_WHITE);

//      Serial.println();

      snprintf(info_line, sizeof(info_line), "Traffic %d/%d", EPD_current, j);
      display->getTextBounds(info_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(info_line);
//      Serial.println(info_line);

      y += tbh;

      if (oclock == 0) {
        strcpy(info_line, "  ahead"); 
      } else {
        snprintf(info_line, sizeof(info_line), "%d o'clock", oclock);
      }
      display->getTextBounds(info_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(info_line);
//      Serial.println(info_line);

      y += tbh;

      snprintf(info_line, sizeof(info_line), "%4.1f km out",
               traffic[EPD_current - 1].distance / 1000.0);
      display->getTextBounds(info_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(info_line);
//      Serial.println(info_line);

      y += tbh;

      snprintf(info_line, sizeof(info_line), "%d m ",
               abs((int) traffic[EPD_current - 1].fop->RelativeVertical));

      if (traffic[EPD_current - 1].fop->RelativeVertical > 50) {
        strcat(info_line, "above");
      } else if (traffic[EPD_current - 1].fop->RelativeVertical < -50) {
        strcat(info_line, "below");
      } else {
        strcpy(info_line, "  same alt."); 
      }

      display->getTextBounds(info_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(info_line);
//      Serial.println(info_line);

      y += tbh;

      snprintf(info_line, sizeof(info_line), "CoG %d deg",
               traffic[EPD_current - 1].fop->Track);
      display->getTextBounds(info_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(info_line);
//      Serial.println(info_line);

      y += tbh;

      snprintf(info_line, sizeof(info_line), "GS %d kts",
               traffic[EPD_current - 1].fop->GroundSpeed);
      display->getTextBounds(info_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(info_line);
//      Serial.println(info_line);

      y += tbh;

      display->getTextBounds(id_text, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(id_text);
//      Serial.println(id_text);

//      Serial.println();
    }
    while (display->nextPage());

    display->hibernate();
  }
}

static void EPD_text_Draw_Message(const char *msg)
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  display->setPartialWindow(0, 0, display->width(), display->height());

  display->setFont(&FreeMonoBold18pt7b);

  display->getTextBounds(msg, 0, 0, &tbx, &tby, &tbw, &tbh);

  display->firstPage();
  do
  {
    display->fillScreen(GxEPD_WHITE);
    uint16_t x = (display->width() - tbw) / 2;
    uint16_t y = (display->height() + tbh) / 2;
    display->setCursor(x, y);
    display->print(msg);
  }
  while (display->nextPage());

  display->hibernate();
}

static void EPD_Draw_NoTraffic()
{
  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  display->setPartialWindow(0, 0, display->width(), display->height());

  display->setFont(&FreeMonoBold18pt7b);

  display->firstPage();
  do
  {
    display->fillScreen(GxEPD_WHITE);

    display->getTextBounds("NO", 0, 0, &tbx, &tby, &tbw, &tbh);
    uint16_t x = (display->width() - tbw) / 2;
    uint16_t y = display->height() / 2 - tbh;
    display->setCursor(x, y);
    display->print("NO");

    display->getTextBounds("TRAFFIC", 0, 0, &tbx, &tby, &tbw, &tbh);
    x = (display->width() - tbw) / 2;
    y = display->height() / 2 + tbh;
    display->setCursor(x, y);
    display->print("TRAFFIC");
  }
  while (display->nextPage());

  display->hibernate();
}

void EPD_text_setup()
{

}

void EPD_text_loop()
{
  if (!EPD_display_frontpage) {

    EPD_Clear_Screen();

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
          if (Traffic_Count() > 0) {
            EPD_Draw_Text();
          } else {
            EPD_Draw_NoTraffic();
          }
        } else {
          EPD_text_Draw_Message(NO_FIX_TEXT);
        }
      } else {
        EPD_text_Draw_Message(NO_DATA_TEXT);
      }

      EPDTimeMarker = millis();
    }
  }
}

void EPD_text_next()
{
  if (EPD_current < MAX_TRACKING_OBJECTS) {
    EPD_current++;
  }
}

void EPD_text_prev()
{
  if (EPD_current > 1) {
    EPD_current--;
  }
}
