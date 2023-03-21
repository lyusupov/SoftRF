/*
 * SoCHelper.cpp
 * Copyright (C) 2019-2023 Linar Yusupov
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

const SoC_ops_t *SoC;

byte SoC_setup()
{
#if defined(ESP8266)
  SoC = &ESP8266_ops;
#elif defined(ESP32)
  SoC = &ESP32_ops;
#elif defined(RASPBERRY_PI)
  SoC = &RPi_ops;
#elif defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
  SoC = &CC13XX_ops;
#elif defined(ARDUINO_ARCH_STM32)
  SoC = &STM32_ops;
#elif defined(__ASR6501__) || defined(ARDUINO_ARCH_ASR650X)
  SoC = &PSoC4_ops;
#elif defined(ARDUINO_ARCH_NRF52)
  SoC = &nRF52_ops;
#elif defined(HACKRF_ONE)
  SoC = &LPC43_ops;
#elif defined(ARDUINO_ARCH_SAMD)
  SoC = &SAMD_ops;
#elif defined(ARDUINO_ARCH_AVR)
  SoC = &AVR_ops;
#elif defined(ARDUINO_ARCH_ASR6601)
  SoC = &ASR66_ops;
#elif defined(ARDUINO_ARCH_RP2040)
  SoC = &RP2040_ops;
#else
#error "This hardware platform is not supported!"
#endif

  byte id = SOC_NONE;

  if (SoC) {
    if (SoC->setup) {
      SoC->setup();
    }
    id = SoC->id;
  }

  return id;
}

void SoC_fini()
{
  if (SoC && SoC->fini) {
    SoC->fini();
  }
}
