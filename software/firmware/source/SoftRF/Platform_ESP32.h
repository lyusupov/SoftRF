/*
 * Platform_ESP32.h
 * Copyright (C) 2018-2019 Linar Yusupov
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

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WiFiClient.h>
#include <SPIFFS.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define SoftwareSerial          HardwareSerial
#define swSer                   Serial1
#define UATSerial               Serial2

#define isValidFix()            isValidGNSSFix()

/* Adafruit_NeoPixel still has "flickering" issue of ESP32 caused by 1 ms scheduler */
//#define USE_ADAFRUIT_NEO_LIBRARY

/*
 * NeoPixelBus is already "flickering-free" on ESP32 (with I2S)
 * but the "Core" needs update onto the most recent one
 */
#define USE_NEOPIXELBUS_LIBRARY

#if defined(USE_NEOPIXELBUS_LIBRARY)
#include <NeoPixelBus.h>

#define uni_begin()             strip.Begin()
#define uni_show()              strip.Show()
#define uni_setPixelColor(i, c) strip.SetPixelColor(i, c)
#define uni_numPixels()         strip.PixelCount()
#define uni_Color(r,g,b)        RgbColor(r,g,b)
#define color_t                 RgbColor

extern NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip;
#else /* USE_ADAFRUIT_NEO_LIBRARY */
#include <Adafruit_NeoPixel.h>

#define uni_begin()             strip.begin()
#define uni_show()              strip.show()
#define uni_setPixelColor(i, c) strip.setPixelColor(i, c)
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

extern Adafruit_NeoPixel strip;
#endif /* USE_NEOPIXELBUS_LIBRARY */

#define LEDC_CHANNEL_BUZZER     0
#define LEDC_RESOLUTION_BUZZER  8

/* Peripherals */
#define SOC_GPIO_PIN_GNSS_RX    23
#define SOC_GPIO_PIN_GNSS_TX    12
#define SOC_GPIO_PIN_LED        25
#define SOC_GPIO_PIN_BUZZER     13
#define SOC_GPIO_PIN_BATTERY    36

#define SOC_GPIO_PIN_GNSS_PPS   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_MODE_PULLDOWN INPUT_PULLDOWN

#define SOC_GPIO_PIN_STATUS   (hw_info.model != SOFTRF_MODEL_PRIME_MK2 ?\
                                SOC_UNUSED_PIN :                        \
                                (hw_info.revision == 2 ?                \
                                  SOC_GPIO_PIN_TBEAM_LED_V02 :          \
                                  SOC_GPIO_PIN_TBEAM_LED_V05))

/* SPI (does match Heltec & TTGO LoRa32 pins mapping) */
#define SOC_GPIO_PIN_MOSI       27
#define SOC_GPIO_PIN_MISO       19
#define SOC_GPIO_PIN_SCK        5
#define SOC_GPIO_PIN_SS         18

/* NRF905 */
#define SOC_GPIO_PIN_TXE        26
#define SOC_GPIO_PIN_CE         2
#define SOC_GPIO_PIN_PWR        14

/* SX1276 [RFM95W] (does match Heltec & TTGO LoRa32 pins mapping) */
#define SOC_GPIO_PIN_RST        14
#define SOC_GPIO_PIN_DIO0       26
#define SOC_GPIO_PIN_SDA        14
#define SOC_GPIO_PIN_SCL        2

/* TTGO T-BEAM section */
// GPS module
#define SOC_GPIO_PIN_TBEAM_RX   12
#define SOC_GPIO_PIN_TBEAM_TX   15
// battery voltage
#define SOC_GPIO_PIN_TBEAM_BATTERY      35
// status LED
#define SOC_GPIO_PIN_TBEAM_LED_V02      21
#define SOC_GPIO_PIN_TBEAM_LED_V05      14
// SX1276 RESET
#define SOC_GPIO_PIN_TBEAM_RF_RST_V02   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TBEAM_RF_RST_V05   23
// 1st I2C bus on the T-Beam
#define SOC_GPIO_PIN_TBEAM_SDA  13
#define SOC_GPIO_PIN_TBEAM_SCL  2

#define SSD1306_OLED_I2C_ADDR   0x3C

// Hardware pin definitions for TTGO V2 Board with OLED SSD1306 0,96" I2C Display
#define TTGO_V2_OLED_PIN_RST    U8X8_PIN_NONE // connected to CPU RST/EN
#define TTGO_V2_OLED_PIN_SDA    21
#define TTGO_V2_OLED_PIN_SCL    22
// Hardware pin definitions for Heltec and TTGO-V1 LoRa-32 Boards with OLED SSD1306 I2C Display
#define HELTEC_OLED_PIN_RST     U8X8_PIN_NONE // 16
#define HELTEC_OLED_PIN_SDA     4
#define HELTEC_OLED_PIN_SCL     15

extern WebServer server;

enum rst_reason {
  REASON_DEFAULT_RST      = 0,  /* normal startup by power on */
  REASON_WDT_RST          = 1,  /* hardware watch dog reset */
  REASON_EXCEPTION_RST    = 2,  /* exception reset, GPIO status won't change */
  REASON_SOFT_WDT_RST     = 3,  /* software watch dog reset, GPIO status won't change */
  REASON_SOFT_RESTART     = 4,  /* software restart ,system_restart , GPIO status won't change */
  REASON_DEEP_SLEEP_AWAKE = 5,  /* wake up from deep-sleep */
  REASON_EXT_SYS_RST      = 6   /* external system reset */
};

enum esp32_board_id {
  ESP32_DEVKIT,
  ESP32_TTGO_V2_OLED,
  ESP32_HELTEC_OLED,
  ESP32_TTGO_T_BEAM
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

#define MakeFlashId(v,d)        ((v << 16) | d)

/* Disable brownout detection (avoid unexpected reset on some boards) */
#define ESP32_DISABLE_BROWNOUT_DETECTOR 0

#define  NMEA_TCP_SERVICE
#define  USE_NMEALIB

#define POWER_SAVING_WIFI_TIMEOUT 600000UL /* 10 minutes */

#endif /* PLATFORM_ESP32_H */

#endif /* ESP32 */
