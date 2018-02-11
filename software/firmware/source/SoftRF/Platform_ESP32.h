/*
 * Platform_ESP32.h
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
#if defined(ESP32)

#ifndef ESP32HELPER_H
#define ESP32HELPER_H

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WiFiClient.h>
#include <SPIFFS.h>

#define SoftwareSerial HardwareSerial
#define swSer Serial1

#define SOC_A0_VOLTAGE_DIVIDER  (1023.0 / 3.9)

/* Peripherals */
#define SOC_GPIO_PIN_GNSS_RX  23
#define SOC_GPIO_PIN_GNSS_TX  2 /* OLED: Heltec - 4 & 15 ; TTGO V2 - 21 & 22 */
#define SOC_GPIO_PIN_LED      25
#define SOC_GPIO_PIN_BUZZER   13
#define SOC_GPIO_PIN_BATTERY  36

/* SPI (does match Heltec & TTGO LoRa32 pins mapping) */
#define SOC_GPIO_PIN_MOSI     27
#define SOC_GPIO_PIN_MISO     19
#define SOC_GPIO_PIN_SCK      5
#define SOC_GPIO_PIN_SS       18

/* NRF905 */
#define SOC_GPIO_PIN_TXE      26
#define SOC_GPIO_PIN_CE       12
#define SOC_GPIO_PIN_PWR      14

#define SOFTRF_LORA_PCB_1_1
//#define SOFTRF_LORA_PCB_1_2_PROTO

/* SX1276 [RFM95W] (does match Heltec & TTGO LoRa32 pins mapping) */
#if defined(SOFTRF_LORA_PCB_1_1)

#define SOC_GPIO_PIN_RST      14
#define SOC_GPIO_PIN_DIO0     26

#elif defined(SOFTRF_LORA_PCB_1_2_PROTO)

#define SOC_GPIO_PIN_RST      14
#define SOC_GPIO_PIN_DIO0     26
#define SOC_GPIO_PIN_SDA      14
#define SOC_GPIO_PIN_SCL      12

#endif /* SOFTRF_LORA_PCB */

extern WebServer server;
extern HardwareSerial Serial1;

enum rst_reason {
  REASON_DEFAULT_RST      = 0,  /* normal startup by power on */
  REASON_WDT_RST          = 1,  /* hardware watch dog reset */
  REASON_EXCEPTION_RST    = 2,  /* exception reset, GPIO status won't change */
  REASON_SOFT_WDT_RST     = 3,  /* software watch dog reset, GPIO status won't change */
  REASON_SOFT_RESTART     = 4,  /* software restart ,system_restart , GPIO status won't change */
  REASON_DEEP_SLEEP_AWAKE = 5,  /* wake up from deep-sleep */
  REASON_EXT_SYS_RST      = 6   /* external system reset */
};

struct rst_info {
  uint32_t reason;
  uint32_t exccause;
  uint32_t epc1;
  uint32_t epc2;
  uint32_t epc3;
  uint32_t excvaddr;
  uint32_t depc;
};

#endif /* ESP32HELPER_H */

#endif /* ESP32 */
