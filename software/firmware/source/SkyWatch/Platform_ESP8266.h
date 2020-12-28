/*
 * Platform_ESP8266.h
 * Copyright (C) 2020-2021 Linar Yusupov
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

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS            9

#define SerialInput                     Serial

#define SOC_A0_VOLTAGE_DIVIDER          (950.0 / 3.2)
#define SOC_GPIO_PIN_BATTERY            A0

extern ESP8266WebServer server;

//#define NMEA_TCP_SERVICE
//#define USE_DNS_SERVER

#define POWER_SAVING_WIFI_TIMEOUT 300000UL /* 5 minutes */

#define EXCLUDE_RTC
#define EXCLUDE_TFT

#endif /* PLATFORM_ESP8266_H */

#endif /* ESP8266 */
