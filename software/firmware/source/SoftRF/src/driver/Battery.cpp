/*
 * BatteryHelper.cpp
 * Copyright (C) 2016-2020 Linar Yusupov
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
#endif

#include "../system/SoC.h"
#include "Battery.h"

unsigned long Battery_TimeMarker        = 0;

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
  return hw_info.model == SOFTRF_MODEL_PRIME_MK2                             ||
        (hw_info.model == SOFTRF_MODEL_STANDALONE && hw_info.revision == 16) || /* TTGO T3 V2.1.6 */
         hw_info.model == SOFTRF_MODEL_DONGLE                                ||
         hw_info.model == SOFTRF_MODEL_BADGE                                 ||
         hw_info.model == SOFTRF_MODEL_MINI      ? BATTERY_THRESHOLD_LIPO   :
         hw_info.model == SOFTRF_MODEL_UNI       ? BATTERY_THRESHOLD_NIZNX2 :
                                                   BATTERY_THRESHOLD_NIMHX2;
}

/* Battery is empty */
float Battery_cutoff()
{
  return hw_info.model == SOFTRF_MODEL_PRIME_MK2                             ||
        (hw_info.model == SOFTRF_MODEL_STANDALONE && hw_info.revision == 16) || /* TTGO T3 V2.1.6 */
         hw_info.model == SOFTRF_MODEL_DONGLE                                ||
         hw_info.model == SOFTRF_MODEL_BADGE                                 ||
         hw_info.model == SOFTRF_MODEL_MINI      ? BATTERY_CUTOFF_LIPO   :
         hw_info.model == SOFTRF_MODEL_UNI       ? BATTERY_CUTOFF_NIZNX2 :
                                                   BATTERY_CUTOFF_NIMHX2;
}

void Battery_loop()
{
  if (isTimeToBattery()) {
    float voltage = SoC->Battery_voltage();

    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2                             ||
       (hw_info.model == SOFTRF_MODEL_STANDALONE && hw_info.revision == 16) || /* TTGO T3 V2.1.6 */
        hw_info.model == SOFTRF_MODEL_DONGLE                                ||
        hw_info.model == SOFTRF_MODEL_MINI                                  ||
        hw_info.model == SOFTRF_MODEL_BADGE                                 ||
        hw_info.model == SOFTRF_MODEL_UNI       ) {

      if (voltage > 1.8 && voltage < Battery_cutoff()) {
        if (Battery_cutoff_count > 2) {
          shutdown(SOFTRF_SHUTDOWN_LOWBAT);
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
