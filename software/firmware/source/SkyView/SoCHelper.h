/*
 * SoCHelper.h
 * Copyright (C) 2019 Linar Yusupov
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

#ifndef SOCHELPER_H
#define SOCHELPER_H

#include "Platform_RPi.h"
#include "Platform_ESP8266.h"
#include "Platform_ESP32.h"

typedef struct SoC_ops_struct {
  uint8_t id;
  const char name[16];
  void (*setup)();
} SoC_ops_t;

enum
{
	SOC_NONE,
	SOC_ESP8266,
	SOC_ESP32,
	SOC_RPi,
	SOC_CC13XX
};

extern const SoC_ops_t *SoC;
#if defined(ESP8266)
extern const SoC_ops_t ESP8266_ops;
#endif
#if defined(ESP32)
extern const SoC_ops_t ESP32_ops;
#endif
#if defined(RASPBERRY_PI)
extern const SoC_ops_t RPi_ops;
#endif
#if defined(ENERGIA_ARCH_CC13XX)
extern const SoC_ops_t CC13XX_ops;
#endif

byte SoC_setup(void);

#endif /* SOCHELPER_H */
