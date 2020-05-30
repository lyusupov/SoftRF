/*
 * SoCHelper.h
 * Copyright (C) 2018-2020 Linar Yusupov
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
#include "Platform_RPi.h"
#include "Platform_CC13XX.h"
#include "Platform_STM32.h"
#include "Platform_PSoC4.h"
#include "BluetoothHelper.h"

typedef struct SoC_ops_struct {
  uint8_t id;
  const char name[16];
  void (*setup)();
  void (*loop)();
  void (*fini)();
  void (*reset)();
  uint32_t (*getChipId)();
  void* (*getResetInfoPtr)();
  String (*getResetInfo)();
  String (*getResetReason)();
  uint32_t (*getFreeHeap)();
  long (*random)(long, long);
  void (*Sound_test)(int);
  uint32_t (*maxSketchSpace)();
  void (*WiFi_setOutputPower)(int);
  void (*WiFi_transmit_UDP)(int, byte *, size_t);
  void (*WiFiUDP_stopAll)();
  bool (*WiFi_hostname)(String);
  int  (*WiFi_clients_count)();
  bool (*EEPROM_begin)(size_t);
  void (*SPI_begin)();
  void (*swSer_begin)(unsigned long);
  void (*swSer_enableRx)(boolean);
  Bluetooth_ops_t *Bluetooth;
  byte (*Display_setup)();
  void (*Display_loop)();
  void (*Display_fini)(const char *);
  void (*Battery_setup)();
  float (*Battery_voltage)();
  void (*GNSS_PPS_handler)();
  unsigned long (*get_PPS_TimeMarker)();
  bool (*Baro_setup)();
  void (*UATSerial_begin)(unsigned long);
  void (*UATModule_restart)();
  void (*WDT_setup)();
  void (*WDT_fini)();
  void (*Button_setup)();
  void (*Button_loop)();
  void (*Button_fini)();
} SoC_ops_t;

enum
{
	SOC_NONE,
	SOC_ESP8266,
	SOC_ESP32,
	SOC_RPi,
	SOC_CC13XX,
	SOC_STM32,
	SOC_PSOC4
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
#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
extern const SoC_ops_t CC13XX_ops;
#endif
#if defined(ARDUINO_ARCH_STM32)
extern const SoC_ops_t STM32_ops;
#endif
#if defined(__ASR6501__)
extern const SoC_ops_t PSoC4_ops;
#endif

byte SoC_setup(void);
void SoC_fini(void);

#endif /* SOCHELPER_H */
