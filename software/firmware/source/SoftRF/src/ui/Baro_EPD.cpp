/*
 * Baro_EPD.cpp
 * Copyright (C) 2021 Linar Yusupov
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

const char Altitude_text[]    = "ALTITUDE";
const char Pressure_text[]    = "PRESSURE";
const char Temperature_text[] = "TEMPERATURE";

void EPD_baro_setup()
{

}

void EPD_baro_loop()
{
  char buf[16];

  if (EPD_vmode_updated) {
    EPD_Clear_Screen();

    yield();

    EPD_vmode_updated = false;
  }

  if (!EPD_ready_to_display) {

    display->fillScreen(GxEPD_WHITE);

    display->setFont(&FreeMono9pt7b);

    int16_t x = display->width() / 7;

    display->setCursor(x, (1 * display->height() / 10));
    display->print(Altitude_text);

    display->setCursor(x, (4 * display->height() / 10));
    display->print(Pressure_text);

    display->setCursor(x, (7 * display->height() / 10));
    display->print(Temperature_text);

    display->setFont(&FreeMono12pt7b);

    x = 7 * display->width() / 10;

    display->setCursor(x, (3 * display->height() / 10));
    display->print("M");

    display->setCursor(x, (6 * display->height() / 10));
    display->print("mb");

    display->setCursor(x, (9 * display->height() / 10));
    display->print("C");

    display->setFont(&FreeMonoBold18pt7b);

    x = display->width() / 10;

    display->setCursor(x, (3 * display->height() / 10));
    display->print((int) Baro_altitude());

    display->setCursor(x, (6 * display->height() / 10));
    display->print(Baro_pressure() / 100, 1);

    display->setCursor(x, (9 * display->height() / 10));
    display->print(Baro_temperature(), 1);

    /* a signal to background EPD update task */
    EPD_ready_to_display = true;
  }
}

void EPD_baro_next()
{

}

void EPD_baro_prev()
{

}

#endif /* USE_EPAPER */
