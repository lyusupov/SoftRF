/*
 * BatteryHelper.cpp
 * Copyright (C) 2016-2019 Linar Yusupov
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

#include <Arduino.h>

#include "SoCHelper.h"
#include "BatteryHelper.h"

void Battery_setup()
{
  SoC->Battery_setup();
}

float Battery_voltage()
{
  return SoC->Battery_voltage();
}

/* low battery voltage threshold */
float Battery_threshold()
{
  return hw_info.model == SOFTRF_MODEL_PRIME_MK2 ?
                          BATTERY_THRESHOLD_LIPO : BATTERY_THRESHOLD_NIMHX2;
}