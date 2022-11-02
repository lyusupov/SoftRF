/*
 * SoCHelper.cpp
 * Copyright (C) 2018-2022 Linar Yusupov
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

#include "SoC.h"

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

void SoC_fini(int reason)
{
  if (SoC && SoC->fini) {
    SoC->fini(reason);
  }
}

uint32_t DevID_Mapper(uint32_t id)
{
  uint8_t id_mask = (id & 0x00FF0000UL) >> 16;

  switch (id_mask)
  {
  /* remap address to avoid overlapping with congested FLARM range */
  case 0xD0:
  case 0xDD:
  case 0xDE:
  case 0xDF:
    id += 0x100000;
    break;
  /* remap 11xxxx addresses to avoid overlapping with congested Skytraxx range */
  case 0x11:
  /*
   * OGN 0.2.8+ does not decode 'Air V6' traffic when leading byte of 24-bit Id is 0x5B
   */
  case 0x5B:
    id += 0x010000;
    break;

  default:
    break;
  }

  return id;
}
