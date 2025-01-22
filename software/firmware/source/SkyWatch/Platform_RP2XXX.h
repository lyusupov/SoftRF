/*
 * Platform_RP2XXX.h
 * Copyright (C) 2023-2025 Linar Yusupov
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
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)

#ifndef PLATFORM_RP2XXX_H
#define PLATFORM_RP2XXX_H

#if defined(ARDUINO_RASPBERRY_PI_PICO_W) || defined(ARDUINO_RASPBERRY_PI_PICO_2W)
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#define Serial_setDebugOutput(x) ({})
#define WIFI_STA_TIMEOUT         20000
extern WebServer server;
#if PICO_SDK_VERSION_MAJOR == 2
#define EXCLUDE_BLUETOOTH
#endif /* PICO_SDK_VERSION_MAJOR */
#else
#include <Arduino.h>
#define EXCLUDE_WIFI
#define EXCLUDE_BLUETOOTH
#endif /* ARDUINO_RASPBERRY_PI_PICO_W or 2W */

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS  9

#define SerialInput           Serial2

#define Serial_GNSS_In        Serial1
#define Serial_GNSS_Out       Serial1

#if !defined(ARDUINO_ARCH_MBED)
#define USBSerial             Serial
#else
#define USBSerial             SerialUSB
#endif /* ARDUINO_ARCH_MBED */

enum RP2xxx_board_id {
  RP2040_RAK11300,
  RP2040_RESERVED1,
  RP2040_RPIPICO,
  RP2040_RPIPICO_W,
  RP2040_WEACT,
  RP2350_RPIPICO_2,
  RP2350_RPIPICO_2W,
};

/* Console I/O */
#define SOC_GPIO_PIN_CONS_RX  (5u)
#define SOC_GPIO_PIN_CONS_TX  (4u)

/* Peripherals */
#define SOC_GPIO_PIN_GNSS_RX  (1u)
#define SOC_GPIO_PIN_GNSS_TX  (0u)
#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN

#define SOC_GPIO_PIN_BUTTON   (23u)

#define SOC_GPIO_PIN_USBH_DP  (20u)  // Pin used as D+ for host, D- = D+ + 1
#define SOC_GPIO_PIN_USBH_DN  (21u)

#define SOC_GPIO_PIN_VSYS     (29u) // Pico
#define SOC_GPIO_PIN_CYW43_EN (25u) // Pico W

#define SOC_GPIO_PIN_STATUS   PIN_LED // Pico/WeAct - 25, W - 32 (CYW43 GPIO 0)
#define SOC_GPIO_PIN_BATTERY  SOC_GPIO_PIN_VSYS
#define SOC_ADC_VOLTAGE_DIV   (3.0) // 20K + 10K voltage divider of VSYS

#if !defined(ARDUINO_ARCH_MBED)
#define USE_BOOTSEL_BUTTON
#else
#define EXCLUDE_EEPROM
#endif /* ARDUINO_ARCH_MBED */

/* Experimental */
#if defined(USE_TINYUSB)
#define USE_USB_HOST
//#define ENABLE_USB_HOST_DEBUG
#endif /* USE_TINYUSB */

#define POWER_SAVING_WIFI_TIMEOUT 300000UL /* 5 minutes */

#define EXCLUDE_RTC
#define EXCLUDE_TFT

#endif /* PLATFORM_RP2XXX_H */
#endif /* ARDUINO_ARCH_RP2XXX */
