/*
 * Platform_ESP8266.h
 * Copyright (C) 2018-2025 Linar Yusupov
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

#define USE_WIFI_NINA           false
#define USE_WIFI_CUSTOM         true
#include <ESP8266WiFi.h>

#include <ESP8266mDNS.h>
#include <WiFiClient.h>

#define USE_EXP_SW_SERIAL

#if defined(USE_EXP_SW_SERIAL)
#include <Exp_SoftwareSerial.h>
#else
#include <SoftwareSerial.h>
#endif
#include <ESP8266FtpServer.h>
#include <Adafruit_NeoPixel.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_STANDALONE

#define EEPROM_commit()         EEPROM.commit()

#define isValidFix()            isValidGNSSFix()

#define LED_STATE_ON            HIGH  // State when LED is litted

extern Adafruit_NeoPixel *strip;

static inline void uni_begin(void) { if (strip) strip->begin(); }
static inline void uni_show(void)  { if (strip) strip->show();  }
static inline void uni_setPixelColor(uint16_t i, uint32_t c) {
  if (strip) strip->setPixelColor(i, c);
}
static inline uint16_t uni_numPixels() {
  if (strip) return strip->numPixels(); else return 0;
}
static inline uint32_t uni_Color(uint8_t r, uint8_t g, uint8_t b) {
  return Adafruit_NeoPixel::Color(r,g,b);
}

#define color_t                 uint32_t

#define SOC_A0_VOLTAGE_DIVIDER  (950.0 / 3.2)

/* Peripherals */
#define SOC_GPIO_PIN_GNSS_RX  D3
#define SOC_GPIO_PIN_GNSS_TX  D1 // 9  /* not in use */
#define SOC_GPIO_PIN_LED      D1
#define SOC_GPIO_PIN_BUZZER   10
#define SOC_GPIO_PIN_BATTERY  A0

#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN

#define SOC_GPIO_PIN_STATUS   SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_MOSI     D7
#define SOC_GPIO_PIN_MISO     D6
#define SOC_GPIO_PIN_SCK      D5
#define SOC_GPIO_PIN_SS       D8

/* NRF905 */
#define SOC_GPIO_PIN_TXE      D0
#define SOC_GPIO_PIN_CE       D4
#define SOC_GPIO_PIN_PWR      D2

/* SX1276 (RFM95W) */
#define SOC_GPIO_PIN_RST      D2
#define SOC_GPIO_PIN_DIO0     D0
#define SOC_GPIO_PIN_SDA      D2
#define SOC_GPIO_PIN_SCL      D4

#define SerialOutput          Serial
#define Serial_GNSS_In        swSer
#define Serial_GNSS_Out       Serial_GNSS_In
#define UATSerial             Serial /* TBD */

extern "C" {
#include <user_interface.h>
}

#if defined(USE_EXP_SW_SERIAL)
extern Exp_SoftwareSerial swSer;
#else
extern SoftwareSerial swSer;
#endif

#define USE_NMEALIB
//#define USE_BASICMAC
//#define USE_RADIOLIB
//#define EXCLUDE_LR11XX
#define EXCLUDE_CC1101
#define EXCLUDE_SI443X
#define EXCLUDE_SI446X
#define EXCLUDE_SX1231
#define EXCLUDE_SX1280

#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65
#define EXCLUDE_GNSS_UC65
#define EXCLUDE_GNSS_AG33
#define EXCLUDE_LOG_GNSS_VERSION

#define EXCLUDE_CC13XX
#define EXCLUDE_SOFTRF_HEARTBEAT
#define EXCLUDE_LK8EX1
#define EXCLUDE_IMU
#define EXCLUDE_BME680
#define EXCLUDE_BME280AUX

#define EXCLUDE_ETHERNET

/* Experimental */
#define ENABLE_ADSL
#define ENABLE_PROL

#if defined(pgm_read_float_aligned)
#define pgm_read_float(addr)  pgm_read_float_aligned(addr)
#endif

#endif /* PLATFORM_ESP8266_H */

#endif /* ESP8266 */
