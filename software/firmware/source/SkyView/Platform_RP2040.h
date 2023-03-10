/*
 * Platform_RP2040.h
 * Copyright (C) 2023 Linar Yusupov
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
#if defined(ARDUINO_ARCH_RP2040)

#ifndef PLATFORM_RP2040_H
#define PLATFORM_RP2040_H

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#define Serial_setDebugOutput(x) ({})
extern WebServer server;
#else
#include <Arduino.h>
#define EXCLUDE_WIFI
#endif /* ARDUINO_RASPBERRY_PI_PICO_W */

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS  8

#define SOC_A0_VOLTAGE_DIVIDER  (950.0 / 3.2)

#define SerialInput           Serial1
#define SerialOutput          Serial2

#if !defined(ARDUINO_ARCH_MBED)
#define USBSerial             Serial
#else
#define USBSerial             SerialUSB
#endif /* ARDUINO_ARCH_MBED */

enum RP2040_board_id {
  RP2040_RAK11300,
  RP2040_RESERVED1,
  RP2040_RPIPICO,
  RP2040_RPIPICO_W,
  RP2040_WEACT,
};

/* Peripherals */
#define SOC_GPIO_PIN_SWSER_RX D3
#define SOC_GPIO_PIN_SWSER_TX D1 // 9  /* not in use */
#define SOC_GPIO_PIN_BATTERY  A0

#endif /* PLATFORM_RP2040_H */

#endif /* ARDUINO_ARCH_RP2040 */
