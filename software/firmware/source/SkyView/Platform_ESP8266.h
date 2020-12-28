/*
 * Platform_ESP8266.h
 * Copyright (C) 2019-2021 Linar Yusupov
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
#if defined(ESP8266)

#ifndef PLATFORM_ESP8266_H
#define PLATFORM_ESP8266_H

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#include <Exp_SoftwareSerial.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define SOC_A0_VOLTAGE_DIVIDER  (950.0 / 3.2)

/* Peripherals */
#define SOC_GPIO_PIN_SWSER_RX D3
#define SOC_GPIO_PIN_SWSER_TX D1 // 9  /* not in use */
#define SOC_GPIO_PIN_BATTERY  A0

extern "C" {
#include <user_interface.h>
}

extern ESP8266WebServer server;
extern Exp_SoftwareSerial SerialInput;

#endif /* PLATFORM_ESP8266_H */

#endif /* ESP8266 */
