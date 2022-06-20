/*
 * View_IMU_EPD.cpp
 * Copyright (C) 2022 Linar Yusupov
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

#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>

float IMU_g = 0;

static const char G_load_text[] = "G-load";

void EPD_imu_setup()
{
  if (hw_info.imu != IMU_NONE) {
    EPD_pages_mask |= (1 << VIEW_MODE_IMU);
  }
}

void EPD_imu_loop()
{
  char buf_g[8];

  int16_t  tbx, tby;
  uint16_t tbw, tbh;

  if (isTimeToEPD()) {

#if defined(USE_EPD_TASK)
  if (EPD_update_in_progress == EPD_UPDATE_NONE) {
//  if (SoC->Display_lock()) {
#else
  {
#endif
    snprintf(buf_g, sizeof(buf_g), "%.1f", IMU_g);

    display->fillScreen(GxEPD_WHITE);

    display->setFont(&FreeMonoBold12pt7b);
    display->getTextBounds(G_load_text, 0, 0, &tbx, &tby, &tbw, &tbh);

    display->setCursor((display->width() - tbw) / 2, tbh + tbh / 2);
    display->print(G_load_text);

    display->setFont(&FreeMonoBold24pt7b);
    display->getTextBounds(buf_g, 0, 0, &tbx, &tby, &tbw, &tbh);

    display->setCursor((display->width() - tbw) / 2, display->height() / 2);
    display->print(buf_g);

#if defined(USE_EPD_TASK)
    /* a signal to background EPD update task */
    EPD_update_in_progress = EPD_UPDATE_FAST;
//    SoC->Display_unlock();
//    yield();
#else
    display->display(true);
#endif
  }
    EPDTimeMarker = millis();
  }
}

void EPD_imu_next()
{

}

void EPD_imu_prev()
{

}

#endif /* USE_EPAPER */
