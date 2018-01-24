/*
 * Platform_ESP8266.h
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
#if defined(ESP8266)

#ifndef ESP8266HELPER_H
#define ESP8266HELPER_H

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#include <SoftwareSerial.h>
#include <ESP8266FtpServer.h>

#define SOC_GPIO_PIN_SWSER_RX D3
#define SOC_GPIO_PIN_SWSER_TX 9
#define SOC_GPIO_PIN_LED      D1
#define SOC_GPIO_PIN_BUZZER   10

extern "C" {
#include <user_interface.h>
}

extern ESP8266WebServer server;
extern SoftwareSerial swSer;

#endif /* ESP8266HELPER_H */

#endif /* ESP8266 */
