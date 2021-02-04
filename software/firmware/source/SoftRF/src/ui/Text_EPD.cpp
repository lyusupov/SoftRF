/*
 * View_Text_EPD.cpp
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

#include "../driver/EPD.h"

#include <TimeLib.h>

#include "../TrafficHelper.h"
#include "../driver/EEPROM.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../driver/GNSS.h"
#include "../driver/LED.h"

#include <Fonts/FreeMonoBold12pt7b.h>

static int EPD_current = 1;

enum {
   STATE_TVIEW_NONE,
   STATE_TVIEW_TEXT,
   STATE_TVIEW_NOFIX,
   STATE_TVIEW_NODATA,
   STATE_TVIEW_NOTRAFFIC
};

static int view_state_curr = STATE_TVIEW_NONE;
static int view_state_prev = STATE_TVIEW_NONE;

static void EPD_Draw_Text()
{
  int j=0;
  int bearing;
  char info_line [TEXT_VIEW_LINE_LENGTH];
  char id_text   [TEXT_VIEW_LINE_LENGTH];

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr && (now() - Container[i].timestamp) <= EPD_EXPIRATION_TIME) {

      traffic_by_dist[j].fop = &Container[i];
      traffic_by_dist[j].distance = Container[i].distance;
      j++;
    }
  }

  if (j > 0 && !EPD_ready_to_display) {

    uint8_t db;
    const char *u_dist, *u_alt, *u_spd;
    float disp_dist;
    int   disp_alt, disp_spd;

    qsort(traffic_by_dist, j, sizeof(traffic_by_dist_t), traffic_cmp_by_distance);

    if (EPD_current > j) {
      EPD_current = j;
    }

    bearing = (int) traffic_by_dist[EPD_current - 1].fop->bearing;

    /* This bearing is always relative to current ground track */
//  if (ui->orientation == DIRECTION_TRACK_UP) {
      bearing -= ThisAircraft.course;
//  }

    if (bearing < 0) {
      bearing += 360;
    }

    int oclock = ((bearing + 15) % 360) / 30;
    float RelativeVertical = traffic_by_dist[EPD_current - 1].fop->altitude -
                                ThisAircraft.altitude;

    switch (ui->units)
    {
    case UNITS_IMPERIAL:
      u_dist = "nm";
      u_alt  = "f";
      u_spd  = "kts";
      disp_dist = (traffic_by_dist[EPD_current - 1].distance * _GPS_MILES_PER_METER) /
                  _GPS_MPH_PER_KNOT;
      disp_alt  = abs((int) (RelativeVertical * _GPS_FEET_PER_METER));
      disp_spd  = traffic_by_dist[EPD_current - 1].fop->speed;
      break;
    case UNITS_MIXED:
      u_dist = "km";
      u_alt  = "f";
      u_spd  = "kph";
      disp_dist = traffic_by_dist[EPD_current - 1].distance / 1000.0;
      disp_alt  = abs((int) (RelativeVertical * _GPS_FEET_PER_METER));
      disp_spd  = traffic_by_dist[EPD_current - 1].fop->speed * _GPS_KMPH_PER_KNOT;
      break;
    case UNITS_METRIC:
    default:
      u_dist = "km";
      u_alt  = "m";
      u_spd  = "kph";
      disp_dist = traffic_by_dist[EPD_current - 1].distance / 1000.0;
      disp_alt  = abs((int) RelativeVertical);
      disp_spd  = traffic_by_dist[EPD_current - 1].fop->speed * _GPS_KMPH_PER_KNOT;
      break;
    }

    uint32_t id = traffic_by_dist[EPD_current - 1].fop->addr;

    snprintf(id_text, sizeof(id_text), "ID: %06X", id);

    display->setFont(&FreeMonoBold12pt7b);

    {
      uint16_t x = 20;
      uint16_t y = 0;

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

      y += TEXT_VIEW_LINE_SPACING;

      if (oclock == 0) {
        strcpy(info_line, "   ahead");
      } else {
        snprintf(info_line, sizeof(info_line), " %2d o'clock", oclock);
      }
      display->getTextBounds(info_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(info_line);
//      Serial.println(info_line);

      y += TEXT_VIEW_LINE_SPACING;

      snprintf(info_line, sizeof(info_line), "%4.1f %s out", disp_dist, u_dist);
      display->getTextBounds(info_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(info_line);
//      Serial.println(info_line);

      y += TEXT_VIEW_LINE_SPACING;

      snprintf(info_line, sizeof(info_line), "%4d %s ", disp_alt, u_alt);

      if ((int) RelativeVertical > 50) {
        strcat(info_line, "above");
      } else if ((int) RelativeVertical < -50) {
        strcat(info_line, "below");
      } else {
        strcpy(info_line, "  same alt."); 
      }

      display->getTextBounds(info_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(info_line);
//      Serial.println(info_line);

      y += TEXT_VIEW_LINE_SPACING;

      snprintf(info_line, sizeof(info_line), "CoG %3d deg",
               (int) traffic_by_dist[EPD_current - 1].fop->course);
      display->getTextBounds(info_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(info_line);
//      Serial.println(info_line);

      y += TEXT_VIEW_LINE_SPACING;

      snprintf(info_line, sizeof(info_line), "GS  %3d %s", disp_spd, u_spd);
      display->getTextBounds(info_line, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(info_line);
//      Serial.println(info_line);

      y += TEXT_VIEW_LINE_SPACING;

      display->getTextBounds(id_text, 0, 0, &tbx, &tby, &tbw, &tbh);
      y += tbh;
      display->setCursor(x, y);
      display->print(id_text);
//      Serial.println(id_text);

//      Serial.println();
    }

    /* a signal to background EPD update task */
    EPD_ready_to_display = true;
  }
}

void EPD_text_setup()
{

}

void EPD_text_loop()
{
  bool hasFix = isValidGNSSFix();

  if (hasFix) {
      if (Traffic_Count() > 0) {
        view_state_curr = STATE_TVIEW_TEXT;
      } else {
        view_state_curr = STATE_TVIEW_NOTRAFFIC;
      }
  } else {
    view_state_curr = STATE_TVIEW_NOFIX;
  }

  if (EPD_vmode_updated) {
    EPD_Clear_Screen();
    view_state_prev = STATE_TVIEW_NONE;
    EPD_vmode_updated = false;
  }

  if (view_state_curr != view_state_prev &&
      view_state_curr == STATE_TVIEW_NOFIX) {
    EPD_Message(NO_FIX_TEXT, NULL);
    view_state_prev = view_state_curr;
  }

  if (view_state_curr != view_state_prev &&
      view_state_curr == STATE_TVIEW_NOTRAFFIC) {
    EPD_Message("NO", "TRAFFIC");
    view_state_prev = view_state_curr;
  }

  if (view_state_curr == STATE_TVIEW_TEXT) {
    if (view_state_curr != view_state_prev) {
//       EPD_Clear_Screen();
       view_state_prev = view_state_curr;
    }
    EPD_Draw_Text();
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

#endif /* USE_EPAPER */
