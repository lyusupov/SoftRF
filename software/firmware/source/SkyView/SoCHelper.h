/*
 * SoCHelper.h
 * Copyright (C) 2019-2025 Linar Yusupov
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

#include "Platform_RPi.h"
#include "Platform_ESP8266.h"
#include "Platform_ESP32.h"
#include "Platform_RP2XXX.h"
#include "Platform_RK35.h"
#include "SkyView.h"
#include "BluetoothHelper.h"

typedef struct SoC_ops_struct {
  uint8_t id;
  const char name[16];
  void (*setup)();
  void (*post_init)();
  void (*loop)();
  void (*fini)();
  void (*reset)();
  uint32_t (*getChipId)();
  uint32_t (*getFreeHeap)();
  bool (*EEPROM_begin)(size_t);
  void (*EEPROM_extension)(int);
  void (*WiFi_setOutputPower)(int);
  bool (*WiFi_hostname)(String);
  void (*swSer_begin)(unsigned long);
  void (*swSer_enableRx)(boolean);
  uint32_t (*maxSketchSpace)();
  void (*WiFiUDP_stopAll)();
  void (*Battery_setup)();
  float (*Battery_voltage)();
  byte (*Display_setup)(bool);
  void (*Display_loop)();
  void (*Display_fini)(const char *, bool);
  bool (*Display_is_ready)();
  void (*Display_update)(int);
  size_t (*WiFi_Receive_UDP)(uint8_t *, size_t);
  int  (*WiFi_clients_count)();
  bool (*DB_init)();
  bool (*DB_query)(uint8_t, uint32_t, char *, size_t);
  void (*DB_fini)();
  void (*TTS)(char *);
  void (*Button_setup)();
  void (*Button_loop)();
  void (*Button_fini)();
  void (*WDT_setup)();
  void (*WDT_fini)();
  IODev_ops_t *Bluetooth_ops;
  IODev_ops_t *USB_ops;
} SoC_ops_t;

enum
{
	SOC_NONE,
	SOC_ESP8266,
	SOC_ESP32,
	SOC_ESP32S2,
	SOC_ESP32S3,
	SOC_ESP32C2,
	SOC_ESP32C3,
	SOC_ESP32C5,
	SOC_ESP32C6,
	SOC_ESP32C61,
	SOC_ESP32H2,
	SOC_ESP32P4,
	SOC_RPi,
	SOC_CC13X0,
	SOC_CC13X2,
	SOC_STM32,
	SOC_PSOC4,
	SOC_NRF52,
	SOC_LPC43,
	SOC_SAMD,
	SOC_AVR,
	SOC_ASR66,
	SOC_RP2040,
	SOC_RP2350_ARM,
	SOC_RP2350_RISC,
	SOC_RA4M1,
	SOC_EFR32,
	SOC_CH32,
	SOC_RK3506,
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
#if defined(__ASR6501__) || defined(ARDUINO_ARCH_ASR650X)
extern const SoC_ops_t PSoC4_ops;
#endif
#if defined(ARDUINO_ARCH_NRF52)
extern const SoC_ops_t nRF52_ops;
#endif
#if defined(HACKRF_ONE)
extern const SoC_ops_t LPC43_ops;
#endif
#if defined(ARDUINO_ARCH_SAMD)
extern const SoC_ops_t SAMD_ops;
#endif
#if defined(ARDUINO_ARCH_AVR)
extern const SoC_ops_t AVR_ops;
#endif
#if defined(ARDUINO_ARCH_ASR6601)
extern const SoC_ops_t ASR66_ops;
#endif
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
extern const SoC_ops_t RP2xxx_ops;
#endif
#if defined(LUCKFOX_LYRA)
extern const SoC_ops_t RK35_ops;
#endif

byte SoC_setup(void);
void SoC_fini(void);

#endif /* SOCHELPER_H */
