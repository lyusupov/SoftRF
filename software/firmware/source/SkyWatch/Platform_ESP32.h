/*
 * Platform_ESP32.h
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
#if defined(ESP32)

#ifndef PLATFORM_ESP32_H
#define PLATFORM_ESP32_H

#include "sdkconfig.h"

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <SPIFFS.h>
#include <pcf8563.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS            9

#define SerialInput                     Serial1
#define Serial_GNSS_In                  Serial
#define Serial_GNSS_Out                 Serial
#define SERIAL_GNSS_BR                  9600

/* TTGO T-Watch section */
// GNSS module
#define SOC_GPIO_PIN_GNSS_RX            34
#define SOC_GPIO_PIN_GNSS_TX            33
#define SOC_GPIO_PIN_GNSS_PPS           SOC_UNUSED_PIN

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

/* TTGO T-Dongle S2 section */
#define SOC_GPIO_PIN_TDONGLE_CONS_RX    43 // 44
#define SOC_GPIO_PIN_TDONGLE_CONS_TX    44 // 43

// USB
#define SOC_GPIO_PIN_TDONGLE_USB_DP     20
#define SOC_GPIO_PIN_TDONGLE_USB_DN     19

// SD SPI
#define SOC_GPIO_PIN_TDONGLE_MOSI       11
#define SOC_GPIO_PIN_TDONGLE_MISO       13
#define SOC_GPIO_PIN_TDONGLE_SCK        12
#define SOC_GPIO_PIN_TDONGLE_SS         10

// TFT
#define SOC_GPIO_PIN_TDONGLE_TFT_MOSI   35
#define SOC_GPIO_PIN_TDONGLE_TFT_MISO   41 /* MTDI, NC */
#define SOC_GPIO_PIN_TDONGLE_TFT_SCK    36
#define SOC_GPIO_PIN_TDONGLE_TFT_SS     34
#define SOC_GPIO_PIN_TDONGLE_TFT_DC     37
#define SOC_GPIO_PIN_TDONGLE_TFT_RST    38
#define SOC_GPIO_PIN_TDONGLE_TFT_BL     33

// button
#define SOC_GPIO_PIN_TDONGLE_BUTTON     0

// I2C
#define SOC_GPIO_PIN_TDONGLE_SDA        3
#define SOC_GPIO_PIN_TDONGLE_SCL        4

// battery voltage
#define SOC_GPIO_PIN_TDONGLE_BATTERY    SOC_UNUSED_PIN

// V3V Power enable
#define SOC_GPIO_PIN_TDONGLE_PWR_EN     SOC_UNUSED_PIN

// 32768 Hz crystal
#define SOC_GPIO_PIN_TDONGLE_XP         SOC_UNUSED_PIN //  15
#define SOC_GPIO_PIN_TDONGLE_XN         SOC_UNUSED_PIN //  16

#define SOC_GPIO_PIN_TDONGLE_LED        39

/* LilyGO T-Ultima C6 section */
#define SOC_GPIO_PIN_TULTIMA_CONS_RX    17 /* U0RXD */
#define SOC_GPIO_PIN_TULTIMA_CONS_TX    16 /* U0TXD */

/* ESP32-Host interface */
#define SOC_GPIO_PIN_TULTIMA_ESP_MOSI   7
#define SOC_GPIO_PIN_TULTIMA_ESP_MISO   2
#define SOC_GPIO_PIN_TULTIMA_ESP_SCK    6
#define SOC_GPIO_PIN_TULTIMA_ESP_SS     5
#define SOC_GPIO_PIN_TULTIMA_ESP_DR     4
#define SOC_GPIO_PIN_TULTIMA_ESP_HS     3

// USB CDC/JTAG
#define SOC_GPIO_PIN_TULTIMA_USB_DP     13
#define SOC_GPIO_PIN_TULTIMA_USB_DN     12

#if !defined(CONFIG_IDF_TARGET_ESP32S2)
#define LV_HOR_RES                      (240) //Horizontal
#else
#define LV_HOR_RES                      (135) //Horizontal
#endif
#define LV_VER_RES                      (240) //vertical
#define BACKLIGHT_CHANNEL               ((uint8_t)1)

/* Boya Microelectronics Inc. */
#define BOYA_ID                         0x68
#define BOYA_BY25Q32AL                  0x4016

/* ST / SGS/Thomson / Numonyx / XMC(later acquired by Micron) */
#define ST_ID                           0x20
#define XMC_XM25QH32B                   0x4016

#define MakeFlashId(v,d)      ((v  << 16) | d)
#define CCCC(c1, c2, c3, c4)  ((c4 << 24) | (c3 << 16) | (c2 << 8) | c1)

#define MAX_FILENAME_LEN                64
#define WAV_FILE_PREFIX                 "/Audio/"

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

enum softrf_usb_pid {
  SOFTRF_USB_PID_WEBTOP     = 0x8131,
  SOFTRF_USB_PID_STANDALONE = 0x8132,
  SOFTRF_USB_PID_PRIME_MK3  = 0x8133,
  SOFTRF_USB_PID_UF2_BOOT   = 0x8134,
};

extern bool loopTaskWDTEnabled;

extern WebServer server;
extern portMUX_TYPE BMA_mutex;
extern volatile bool BMA_Irq;
extern PCF8563_Class *rtc;

#define NMEA_TCP_SERVICE
//#define USE_DNS_SERVER

#define POWER_SAVING_WIFI_TIMEOUT 300000UL /* 5 minutes */

#define DEBUG_POWER 0

#define EB_S76G_1_3

#if (defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)) && \
    (ARDUINO_USB_ON_BOOT == 0)

/* Experimental */
#define USE_USB_HOST
//#define ENABLE_USB_HOST_DEBUG

#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65
#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_UC65
#define EXCLUDE_GNSS_AG33
#define EXCLUDE_LOG_GNSS_VERSION
#endif /* CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 */

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C5) || \
    defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32P4)
#define EXCLUDE_RTC
#define EXCLUDE_TFT
#endif /* CONFIG_IDF_TARGET_ESP32C3-C5-C6-P4 */

#if defined(CONFIG_IDF_TARGET_ESP32C3)
#undef  SOC_GPIO_PIN_GNSS_RX
#undef  SOC_GPIO_PIN_GNSS_TX
#define SOC_GPIO_PIN_GNSS_RX            18
#define SOC_GPIO_PIN_GNSS_TX            19
#endif /* CONFIG_IDF_TARGET_ESP32C3 */

#if defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C6)
#undef  Serial_GNSS_In
#undef  Serial_GNSS_Out
#define Serial_GNSS_In                  Serial0
#define Serial_GNSS_Out                 Serial0

#undef  SOC_GPIO_PIN_GNSS_RX
#undef  SOC_GPIO_PIN_GNSS_TX
#define SOC_GPIO_PIN_GNSS_RX            SOC_GPIO_PIN_TULTIMA_ESP_HS
#define SOC_GPIO_PIN_GNSS_TX            SOC_GPIO_PIN_TULTIMA_ESP_DR

#define USE_NIMBLE
//#define USE_ARDUINOBLE
#if defined(USE_ARDUINOBLE)
extern IODev_ops_t ArdBLE_Bluetooth_ops;
#endif
#endif /* CONFIG_IDF_TARGET_ESP32C6 */

#endif /* PLATFORM_ESP32_H */

#endif /* ESP32 */
