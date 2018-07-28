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

#define SOC_UNUSED_PIN 255

#include "SoftRF.h"
#include "Platform_ESP8266.h"
#include "Platform_ESP32.h"
#include "BluetoothHelper.h"

typedef struct SoC_ops_struct {
  uint8_t id;
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
  void (*swSer_enableRx)(boolean);
  Bluetooth_ops_t *Bluetooth;
  void (*OLED_loop)();
  void (*Battery_setup)();
  float (*Battery_voltage)();
  void (*GNSS_PPS_handler)();
  unsigned long (*get_PPS_TimeMarker)();
} SoC_ops_t;

enum
{
	SOC_ESP8266,
	SOC_ESP32
};

extern SoC_ops_t *SoC;
#if defined(ESP8266)
extern SoC_ops_t ESP8266_ops;
#endif
#if defined(ESP32)
extern SoC_ops_t ESP32_ops;
#endif

void SoC_setup(void);

#endif /* SOCHELPER_H */
