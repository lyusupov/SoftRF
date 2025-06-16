/*
 * Platform_RA4M1.h
 * Copyright (C) 2024-2025 Linar Yusupov
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
#if defined(ARDUINO_ARCH_RENESAS)

#ifndef PLATFORM_RA4M1_H
#define PLATFORM_RA4M1_H

#include <avr/dtostrf.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_ACADEMY

// #include <core_version.h>    // TODO
#define ARDUINO_CORE_VERSION    "1.2.0"

#define isValidFix()            isValidGNSSFix()

#define uni_begin()             strip.begin()
#define uni_show()              strip.show()
#define uni_setPixelColor(i, c) strip.setPixelColor(i, c)
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

#define EEPROM_commit()         {}

#define LED_STATE_ON            LOW  // State when LED is litted

#if defined(ARDUINO_UNOR4_MINIMA)
#define SerialOutput            SerialUSB
#define Serial_GNSS_In          Serial1
#elif defined(ARDUINO_UNOR4_WIFI)
#if defined(NO_USB)
#define SerialOutput            Serial
#define Serial_GNSS_In          Serial1
#else
#define SerialOutput            Serial1
#define Serial_GNSS_In          Serial2
#endif /* NO_USB */
#else
#error "This Renesas R7FA4M1 build variant is not supported!"
#endif

#define USBSerial               SerialUSB
#define Serial_GNSS_Out         Serial_GNSS_In
#define UATSerial               SerialUSB

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

/* Peripherals */
#if defined(ARDUINO_UNOR4_WIFI)
#define SOC_GPIO_PIN_CONS_RX  UART1_RX_PIN
#define SOC_GPIO_PIN_CONS_TX  UART1_TX_PIN

#define SOC_GPIO_PIN_GNSS_RX  UART2_RX_PIN
#define SOC_GPIO_PIN_GNSS_TX  UART2_TX_PIN

#define SOC_GPIO_PIN_USB_SW   (21)
#else
#define SOC_GPIO_PIN_CONS_RX  24 /* SWD RX */
#define SOC_GPIO_PIN_CONS_TX  23 /* SWD TX */

#define SOC_GPIO_PIN_GNSS_RX  UART1_RX_PIN
#define SOC_GPIO_PIN_GNSS_TX  UART1_TX_PIN
#endif /* ARDUINO_UNOR4_WIFI */

#define SOC_GPIO_PIN_STATUS   LED_BUILTIN
#define SOC_GPIO_PIN_BUZZER   PIN_A0 /* DAC */

#define SOC_GPIO_PIN_RX3      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TX3      SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_MOSI     PIN_SPI_MOSI
#define SOC_GPIO_PIN_MISO     PIN_SPI_MISO
#define SOC_GPIO_PIN_SCK      PIN_SPI_SCK
#define SOC_GPIO_PIN_SS       PIN_SPI_CS

#include <SoftSPI.h>
extern  SoftSPI RadioSPI;
#undef  SPI
#define SPI                   RadioSPI
#define USE_SOFTSPI

/* NRF905 */
#define SOC_GPIO_PIN_TXE      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_CE       SOC_UNUSED_PIN
#define SOC_GPIO_PIN_PWR      SOC_UNUSED_PIN

/* SX1276 */
#define SOC_GPIO_PIN_RST      PIN_D9
#define SOC_GPIO_PIN_BUSY     LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_DIO1     PIN_D6

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX SOC_UNUSED_PIN

/* I2C */
#if defined(ARDUINO_UNOR4_WIFI)
#define SOC_GPIO_PIN_SDA      WIRE1_SDA_PIN
#define SOC_GPIO_PIN_SCL      WIRE1_SCL_PIN
#define Wire                  Wire1
#else
#define SOC_GPIO_PIN_SDA      WIRE_SDA_PIN
#define SOC_GPIO_PIN_SCL      WIRE_SCL_PIN
#endif /* ARDUINO_UNOR4_WIFI */

#define SOC_GPIO_PIN_LED      PIN_A2
#define SOC_GPIO_PIN_GNSS_PPS PIN_A3
//#define NOT_AN_INTERRUPT      SOC_GPIO_PIN_GNSS_PPS
#define SOC_GPIO_PIN_BATTERY  PIN_A1
#define SOC_GPIO_PIN_BUTTON   SOC_UNUSED_PIN

#define SOC_GPIO_RADIO_LED_RX SOC_UNUSED_PIN
#define SOC_GPIO_RADIO_LED_TX SOC_UNUSED_PIN

#define EXCLUDE_ETHERNET
#if defined(ARDUINO_UNOR4_MINIMA)
#define EXCLUDE_WIFI
#define EXCLUDE_BLUETOOTH
#elif defined(ARDUINO_UNOR4_WIFI)
#if defined(NO_USB)
//#define USE_RA4M1_USB
//#define EXCLUDE_WEBUI /* +2.5K HEAP */
//#define EXCLUDE_WIFI  /* +1.7K HEAP */
#define USE_ARDUINO_WIFI
#define EXCLUDE_OTA
#define USE_WIFI_NINA         false
#define USE_WIFI_CUSTOM       true
#include <WiFiS3.h>
#define Serial_setDebugOutput(x) ({})
#if !defined(EXCLUDE_WIFI)
//#define NMEA_TCP_SERVICE   /* -2.4K HEAP */
#define MAX_NMEATCP_CLIENTS   1
//#define EXCLUDE_BLUETOOTH  /* +5.7K HEAP */
#endif /* EXCLUDE_WIFI */
#define EXCLUDE_SOFTRF_HEARTBEAT
#else
#define EXCLUDE_WIFI
#define EXCLUDE_BLUETOOTH
#endif /* NO_USB */
#endif /* ARDUINO_UNOR4 */

#define EXCLUDE_CC13XX
#define EXCLUDE_TEST_MODE
#define EXCLUDE_WATCHOUT_MODE
//#define EXCLUDE_TRAFFIC_FILTER_EXTENSION
//#define EXCLUDE_LK8EX1

#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
//#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65
#define EXCLUDE_GNSS_UC65
#define EXCLUDE_GNSS_AG33
//#define EXCLUDE_LOG_GNSS_VERSION

/* Component                         Cost */
/* -------------------------------------- */
#define EXCLUDE_BMP180           //  -    kb
//#define EXCLUDE_BMP280         //  -    kb
#define EXCLUDE_BME680           //  -    kb
#define EXCLUDE_BME280AUX        //  -    kb
#define EXCLUDE_MPL3115A2        //  -    kb
#define EXCLUDE_NRF905           //  -    kb
#define EXCLUDE_UATM             //  -    kb
#define EXCLUDE_MAVLINK          //  -    kb
#define EXCLUDE_EGM96            //  -    kb
//#define EXCLUDE_LED_RING       //  -    kb
//#define EXCLUDE_SOUND

//#define USE_BASICMAC
//#define EXCLUDE_SX1276         //  -  3 kb
#if defined(ARDUINO_UNOR4_MINIMA)
#define USE_RADIOLIB
//#define EXCLUDE_LR11XX
#define EXCLUDE_CC1101
#define EXCLUDE_SI443X
#define EXCLUDE_SI446X
#define EXCLUDE_SX1231
#define EXCLUDE_SX1280
#endif /* ARDUINO_UNOR4_MINIMA */

#if defined(EXCLUDE_WIFI)
#define USE_OLED                 /* -1.5K HEAP */
#define USE_NMEA_CFG             /* -1.9K HEAP */
#endif /* EXCLUDE_WIFI */
#define EXCLUDE_OLED_049
//#define EXCLUDE_OLED_BARO_PAGE

#define USE_TIME_SLOTS
#define USE_OGN_ENCRYPTION

/* Experimental */
//#define ENABLE_ADSL
//#define ENABLE_PROL

#if !defined(EXCLUDE_LED_RING)
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;
#endif /* EXCLUDE_LED_RING */

#if defined(USE_OLED)
#if defined(ARDUINO_UNOR4_WIFI)
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C
#else
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_HW_I2C
#endif /* ARDUINO_UNOR4_WIFI */
#endif /* USE_OLED */

#endif /* PLATFORM_RA4M1_H */
#endif /* ARDUINO_ARCH_RENESAS */
