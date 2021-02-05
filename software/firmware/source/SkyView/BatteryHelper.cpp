/*
 * BatteryHelper.cpp
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

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#include "SoCHelper.h"
#include "BatteryHelper.h"
#include "SkyView.h"

static unsigned long Battery_TimeMarker = 0;

static float Battery_voltage_cache      = 0;
static int Battery_cutoff_count         = 0;

void Battery_setup()
{
  SoC->Battery_setup();

  Battery_voltage_cache = SoC->Battery_voltage();
  Battery_TimeMarker = millis();
}

float Battery_voltage()
{
  return Battery_voltage_cache;
}

/* low battery voltage threshold */
float Battery_threshold()
{
  return BATTERY_THRESHOLD_LIPO;
}

/* Battery is empty */
float Battery_cutoff()
{
  return BATTERY_CUTOFF_LIPO;
}

void Battery_loop()
{
  if (isTimeToBattery()) {
    float voltage = SoC->Battery_voltage();

    if ( hw_info.model    == SOFTRF_MODEL_SKYVIEW &&
        (hw_info.revision == HW_REV_T5S_1_9 || hw_info.revision == HW_REV_T5S_2_8)) {

      if (voltage > 2.0 && voltage < Battery_cutoff()) {
        if (Battery_cutoff_count > 3) {
          shutdown("LOW BATTERY");
        } else {
          Battery_cutoff_count++;
        }
      } else {
        Battery_cutoff_count = 0;
      }
    }

    Battery_voltage_cache = voltage;
    Battery_TimeMarker = millis();
  }
}
