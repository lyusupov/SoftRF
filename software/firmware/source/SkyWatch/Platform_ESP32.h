/*
 * Platform_ESP32.h
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
#if defined(ESP32)

#ifndef PLATFORM_ESP32_H
#define PLATFORM_ESP32_H

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <SPIFFS.h>
#include <pcf8563.h>
#include <bma.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS            9

#define SerialInput                     Serial1

/* TTGO T-Watch section */
// GNSS module
#define SOC_GPIO_PIN_GNSS_RX            34
#define SOC_GPIO_PIN_GNSS_TX            33

// button
#define SOC_GPIO_PIN_TWATCH_BUTTON      36

// PMU
#define SOC_GPIO_PIN_TWATCH_PMU_IRQ     35

// TFT
#define SOC_GPIO_PIN_TWATCH_TFT_MOSI    19
#define SOC_GPIO_PIN_TWATCH_TFT_MISO    SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TWATCH_TFT_SCK     18
#define SOC_GPIO_PIN_TWATCH_TFT_SS      5
#define SOC_GPIO_PIN_TWATCH_TFT_DC      27
#define SOC_GPIO_PIN_TWATCH_TFT_RST     SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TWATCH_TFT_BL      12

#define LV_HOR_RES                      (240) //Horizontal
#define LV_VER_RES                      (240) //vertical
#define BACKLIGHT_CHANNEL               ((uint8_t)1)

/* 1st I2C bus on the T-Watch */
#define SOC_GPIO_PIN_TWATCH_EXT_SDA     25
#define SOC_GPIO_PIN_TWATCH_EXT_SCL     26

/* 2nd I2C bus on the T-Watch */
#define SOC_GPIO_PIN_TWATCH_SEN_SDA     21
#define SOC_GPIO_PIN_TWATCH_SEN_SCL     22

/* touchscreen */
#define SOC_GPIO_PIN_TWATCH_TP_SDA      23
#define SOC_GPIO_PIN_TWATCH_TP_SCL      32
#define SOC_GPIO_PIN_TWATCH_TP_IRQ      38

/* microSD */
#define SOC_GPIO_PIN_TWATCH_SD_MOSI     15
#define SOC_GPIO_PIN_TWATCH_SD_MISO     2
#define SOC_GPIO_PIN_TWATCH_SD_SCK      14
#define SOC_GPIO_PIN_TWATCH_SD_SS       13

// RTC
#define SOC_GPIO_PIN_TWATCH_RTC_IRQ     37

// BMA423
#define SOC_GPIO_PIN_TWATCH_BMA_IRQ     39

/* TBD */
#define SOC_GPIO_PIN_BCLK               26
#define SOC_GPIO_PIN_LRCLK              25
#define SOC_GPIO_PIN_DOUT               19

#define MakeFlashId(v,d)      ((v  << 16) | d)
#define CCCC(c1, c2, c3, c4)  ((c4 << 24) | (c3 << 16) | (c2 << 8) | c1)

#define MAX_FILENAME_LEN      64
#define WAV_FILE_PREFIX       "/Audio/"

/* these are data structures to process wav file */
typedef enum headerState_e {
    HEADER_RIFF, HEADER_FMT, HEADER_DATA, DATA
} headerState_t;

typedef struct wavRiff_s {
    uint32_t chunkID;
    uint32_t chunkSize;
    uint32_t format;
} wavRiff_t;

typedef struct wavProperties_s {
    uint32_t chunkID;
    uint32_t chunkSize;
    uint16_t audioFormat;
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
} wavProperties_t;

extern bool loopTaskWDTEnabled;

extern WebServer server;
extern BMA *bma;
extern portMUX_TYPE BMA_mutex;
extern volatile bool BMA_Irq;
extern PCF8563_Class *rtc;

#define NMEA_TCP_SERVICE
//#define USE_DNS_SERVER

#define POWER_SAVING_WIFI_TIMEOUT 300000UL /* 5 minutes */

#define DEBUG_POWER 0

#define EB_S76G_1_3

#endif /* PLATFORM_ESP32_H */

#endif /* ESP32 */
