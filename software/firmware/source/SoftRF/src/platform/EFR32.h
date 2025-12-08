/*
 * Platform_EFR32.h
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
#if defined(ARDUINO_ARCH_SILABS)

#ifndef PLATFORM_EFR32_H
#define PLATFORM_EFR32_H

#include <avr/dtostrf.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_ACADEMY

#define ARDUINO_CORE_VERSION    ARDUINO_SILABS

#define isValidFix()            isValidGNSSFix()

#define uni_begin()             strip.begin()
#define uni_show()              strip.show()
#define uni_setPixelColor(i, c) strip.setPixelColor(i, c)
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

#define EEPROM_commit()         {}

#define LED_STATE_ON            LOW  // State when LED is litted

#define SerialOutput            Serial

#define Serial_GNSS_In          Serial1
#define Serial_GNSS_Out         Serial_GNSS_In

#define SOC_ADC_VOLTAGE_DIV     2 /* TBD */

#ifndef strnlen
#define strnlen(x,y)            strlen(x) /* TBD */
#endif

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

#if defined(ARDUINO_NANO_MATTER) || defined(ARDUINO_SILABS_BGM220EXPLORERKIT)

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  PIN_SERIAL_RX
#define SOC_GPIO_PIN_CONS_TX  PIN_SERIAL_TX

#define SOC_GPIO_PIN_GNSS_RX  PIN_SERIAL1_RX
#define SOC_GPIO_PIN_GNSS_TX  PIN_SERIAL1_TX

#define SOC_GPIO_PIN_STATUS   LED_BUILTIN
#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_MOSI     PIN_SPI_MOSI
#define SOC_GPIO_PIN_MISO     PIN_SPI_MISO
#define SOC_GPIO_PIN_SCK      PIN_SPI_SCK
#define SOC_GPIO_PIN_SS       PIN_SPI_SS

//#include <SoftSPI.h>
//extern  SoftSPI RadioSPI;
//#undef  SPI
//#define SPI                   RadioSPI
//#define USE_SOFTSPI

/* NRF905 */
#define SOC_GPIO_PIN_TXE      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_CE       SOC_UNUSED_PIN
#define SOC_GPIO_PIN_PWR      SOC_UNUSED_PIN

/* SX1276 */
#define SOC_GPIO_PIN_RST      D0             /* PA0 */
#define SOC_GPIO_PIN_BUSY     LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_DIO1     D5             /* PC6 */

/* I2C */
#define SOC_GPIO_PIN_SDA      PIN_WIRE_SDA
#define SOC_GPIO_PIN_SCL      PIN_WIRE_SCL

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_GNSS_PPS D6             /* PB0 */
#define SOC_GPIO_PIN_BATTERY  SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BUTTON   BTN_BUILTIN

#define SOC_GPIO_RADIO_LED_RX SOC_UNUSED_PIN
#define SOC_GPIO_RADIO_LED_TX SOC_UNUSED_PIN

#else
#error "This EFR32 build variant is not supported!"
#endif

#if defined(ARDUINO_SILABS_BGM220EXPLORERKIT)
#define EXCLUDE_BLUETOOTH
#endif /* ARDUINO_SILABS_BGM220EXPLORERKIT */
#define EXCLUDE_WIFI
#define EXCLUDE_ETHERNET
#define EXCLUDE_CC13XX
#define EXCLUDE_TEST_MODE
#define EXCLUDE_WATCHOUT_MODE
//#define EXCLUDE_TRAFFIC_FILTER_EXTENSION
//#define EXCLUDE_LK8EX1

//#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
//#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65
#define EXCLUDE_GNSS_UC65
#define EXCLUDE_GNSS_AG33
//#define EXCLUDE_LOG_GNSS_VERSION

/* Component                         Cost */
/* -------------------------------------- */
#define USE_NMEA_CFG             //  +    kb
#define EXCLUDE_BMP180           //  -    kb
//#define EXCLUDE_BMP280         //  -    kb
#define EXCLUDE_BME680           //  -    kb
#define EXCLUDE_BME280AUX        //  -    kb
#define EXCLUDE_MPL3115A2        //  -    kb
#define EXCLUDE_NRF905           //  -    kb
#define EXCLUDE_UATM             //  -    kb
#define EXCLUDE_MAVLINK          //  -    kb
#define EXCLUDE_EGM96            //  -    kb
#define EXCLUDE_LED_RING         //  -    kb
#define EXCLUDE_SOUND

#define USE_OLED                 //       kb
#define EXCLUDE_OLED_049
//#define EXCLUDE_OLED_BARO_PAGE

#define USE_BASICMAC
//#define EXCLUDE_SX1276         //  -  3 kb
#define USE_RADIOLIB
//#define EXCLUDE_LR11XX
#define EXCLUDE_LR20XX
#define EXCLUDE_CC1101
#define EXCLUDE_SI443X
#define EXCLUDE_SI446X
#define EXCLUDE_SX1231
#define EXCLUDE_SX1280

#define USE_TIME_SLOTS
#define USE_OGN_ENCRYPTION

/* Experimental */
#define ENABLE_ADSL
#define ENABLE_PROL

#if !defined(EXCLUDE_LED_RING)
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;
#endif /* EXCLUDE_LED_RING */

#if defined(USE_OLED)
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_HW_I2C
#endif /* USE_OLED */

#endif /* PLATFORM_EFR32_H */

#endif /* ARDUINO_ARCH_SILABS */
