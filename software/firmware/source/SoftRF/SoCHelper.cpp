/*
 * SoCHelper.cpp
 * Copyright (C) 2018 Linar Yusupov
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

SoC_ops_t *SoC;

byte SoC_setup()
{
#if defined(ESP8266)
  SoC = &ESP8266_ops;
#elif defined(ESP32)
  SoC = &ESP32_ops;
#else
#error "This hardware platform is not supported!"
#endif

  if (SoC && SoC->setup) {
    SoC->setup();
  }

  if (SoC) {
    return SoC->id;
  } else {
    return SOC_NONE;
  }
}
