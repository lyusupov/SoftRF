/*
 * Baro_EPD.cpp
 * Copyright (C) 2021-2022 Linar Yusupov
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
#include "../driver/Baro.h"

#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMono12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <TinyGPS++.h>

const char Altitude_text[]    = "ALTITUDE, ";
const char Pressure_text[]    = "PRESSURE, MB";
const char Temperature_text[] = "TEMPERATURE, C";

static navbox_t navbox1;
static navbox_t navbox2;
static navbox_t navbox3;

static float alt_scale = 1.0;

void EPD_baro_setup()
{
  if (hw_info.baro != BARO_MODULE_NONE) {
    EPD_pages_mask |= (1 << VIEW_MODE_BARO);
  }

  alt_scale = ui->units == UNITS_METRIC ? 1.0 : _GPS_FEET_PER_METER;

  memcpy(navbox1.title, Altitude_text, strlen(Altitude_text));
  strcpy(navbox1.title + strlen(Altitude_text),
        ui->units == UNITS_METRIC ? "M" : "FT");
  navbox1.x          = 0;
  navbox1.y          = 0;
  navbox1.width      = display->width();
  navbox1.height     = display->height() / 3;
  navbox1.value      = Baro_altitude() * alt_scale;
//  navbox1.prev_value = navbox1.value;
  navbox1.timestamp  = millis();

  memcpy(navbox2.title, Pressure_text, strlen(Pressure_text));
  navbox2.x          = navbox1.x;
  navbox2.y          = navbox1.y + navbox1.height;
  navbox2.width      = navbox1.width;
  navbox2.height     = navbox1.height;
  navbox2.value      = Baro_pressure() / 100;
//  navbox2.prev_value = navbox2.value;
  navbox2.timestamp  = millis();

  memcpy(navbox3.title, Temperature_text, strlen(Temperature_text));
  navbox3.x          = navbox1.x;
  navbox3.y          = navbox2.y + navbox2.height;
  navbox3.width      = navbox1.width;
  navbox3.height     = navbox1.height;
  navbox3.value      = Baro_temperature();
//  navbox3.prev_value = navbox3.value;
  navbox3.timestamp  = millis();
}

static void EPD_Draw_NavBoxes()
{
  char buf[16];
  uint32_t disp_value;

  int16_t  tbx, tby;
  uint16_t tbw, tbh;

#if defined(USE_EPD_TASK)
  if (EPD_update_in_progress == EPD_UPDATE_NONE) {
//  if (SoC->Display_lock()) {
#else
  {
#endif
    display->fillScreen(GxEPD_WHITE);

    display->drawRoundRect( navbox1.x + 1, navbox1.y + 1,
                            navbox1.width - 2, navbox1.height - 2,
                            4, GxEPD_BLACK);

    display->drawRoundRect( navbox2.x + 1, navbox2.y + 1,
                            navbox2.width - 2, navbox2.height - 2,
                            4, GxEPD_BLACK);

    display->drawRoundRect( navbox3.x + 1, navbox3.y + 1,
                            navbox3.width - 2, navbox3.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&FreeMono9pt7b);

    display->getTextBounds(navbox1.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox1.x + 5, navbox1.y + 5 + tbh);
    display->print(navbox1.title);

    display->getTextBounds(navbox2.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox2.x + 5, navbox2.y + 5 + tbh);
    display->print(navbox2.title);

    display->getTextBounds(navbox3.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox3.x + 5, navbox3.y + 5 + tbh);
    display->print(navbox3.title);

    display->setFont(&FreeMonoBold18pt7b);

    display->setCursor(navbox1.x + navbox1.width / 3 + 15, navbox1.y + 52);
    display->print(navbox1.value);

    display->setCursor(navbox2.x + navbox2.width / 3 + 15, navbox2.y + 52);
    display->print(navbox2.value);

    display->setCursor(navbox3.x + navbox3.width / 3 + 15, navbox3.y + 52);
    display->print(navbox3.value);

#if defined(USE_EPD_TASK)
    /* a signal to background EPD update task */
    EPD_update_in_progress = EPD_UPDATE_FAST;
//    SoC->Display_unlock();
//    yield();
#else
    display->display(true);
#endif
  }
}

void EPD_baro_loop()
{
  if (isTimeToEPD()) {

    navbox1.value = Baro_altitude() * alt_scale;
    navbox2.value = Baro_pressure() / 100;
    navbox3.value = Baro_temperature();

    EPD_Draw_NavBoxes();

    EPDTimeMarker = millis();
  }
}

void EPD_baro_next()
{

}

void EPD_baro_prev()
{

}

#endif /* USE_EPAPER */
