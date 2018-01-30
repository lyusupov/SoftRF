/*
 * SoCHelper.h
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

#ifndef SOCHELPER_H
#define SOCHELPER_H

#include "SoftRF.h"
#include "Platform_ESP8266.h"
#include "Platform_ESP32.h"

typedef struct SoC_ops_struct {
  const char name[16];
  void (*setup)();
  uint32_t (*getChipId)();
  uint32_t (*getFlashChipId)();
  uint32_t (*getFlashChipRealSize)();
  void* (*getResetInfoPtr)();
  String (*getResetInfo)();
  String (*getResetReason)();
  long (*random)(long, long);
  void (*Sound_test)(int);
  uint32_t (*maxSketchSpace)();
  void (*WiFi_setOutputPower)(int);
  IPAddress (*WiFi_get_broadcast)();
  void (*WiFi_transmit_UDP)(int, byte *, size_t);
  void (*WiFiUDP_stopAll)();
  bool (*WiFi_hostname)(String);
  bool (*EEPROM_begin)(size_t);
  void (*SPI_begin)();
  void (*swSer_begin)(unsigned long);
} SoC_ops_t;

extern SoC_ops_t *SoC;
#if defined(ESP8266)
extern SoC_ops_t ESP8266_ops;
#endif
#if defined(ESP32)
extern SoC_ops_t ESP32_ops;
#endif

void SoC_setup(void);

#endif /* SOCHELPER_H */
