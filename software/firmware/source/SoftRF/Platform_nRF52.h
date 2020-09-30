/*
 * Platform_nRF52.h
 * Copyright (C) 2020 Linar Yusupov
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
#if defined(ARDUINO_ARCH_NRF52)

#ifndef PLATFORM_NRF52_H
#define PLATFORM_NRF52_H

#include <avr/dtostrf.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_PRIME_MK3

#define isValidFix()            isValidGNSSFix()

#define uni_begin()             strip.begin()
#define uni_show()              strip.show()
#define uni_setPixelColor(i, c) strip.setPixelColor(i, c)
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

#define yield()                 ({ })
#define snprintf_P              snprintf
#define EEPROM_commit()         {}

#define digitalPinToInterrupt(p) p
#define isPrintable(c)          (isprint(c) == 0 ? false : true)

#define SSD1306_OLED_I2C_ADDR   0x3C

#define SOC_GPIO_PIN_MODE_PULLDOWN INPUT

#define SerialOutput            Serial

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

#define swSer                 Serial2
#define UATSerial             Serial1

#define SOC_ADC_VOLTAGE_DIV   1
#define VREFINT               1200  // mV

#define _PINNUM(port, pin)     ((port)*32 + (pin))

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  _PINNUM(0, 8) // P0.08
#define SOC_GPIO_PIN_CONS_TX  _PINNUM(0, 6) // P0.06

#define SOC_GPIO_PIN_SWSER_RX _PINNUM(1, 9) // P1.09
#define SOC_GPIO_PIN_SWSER_TX _PINNUM(1, 8) // P1.08

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN

#define SOC_GPIO_PIN_GNSS_PPS _PINNUM(1, 4) // P1.04
#define SOC_GPIO_PIN_GNSS_WKE _PINNUM(1, 2) // P1.02
#define SOC_GPIO_PIN_STATUS   _PINNUM(0, 13) // P0.13 (Green)

#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BATTERY  _PINNUM(0, 4) // P0.04 (AIN2)

#define SOC_GPIO_PIN_RX3      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TX3      SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_MOSI     _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_MISO     _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_SCK      _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_SS       _PINNUM(0, 24) // P0.24

/* NRF905 */
#define SOC_GPIO_PIN_TXE      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_CE       SOC_UNUSED_PIN
#define SOC_GPIO_PIN_PWR      SOC_UNUSED_PIN

/* SX1262 */
#define SOC_GPIO_PIN_RST      _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_BUSY     _PINNUM(0, 17) // P0.17
#define SOC_GPIO_PIN_DIO1     _PINNUM(0, 20) // P0.20

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX SOC_UNUSED_PIN

/* I2C */
#define SOC_GPIO_PIN_SDA      _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_SCL      _PINNUM(0, 27) // P0.27

/* button */
#define SOC_GPIO_PIN_BUTTON   _PINNUM(1, 18) // P1.10

/* E-paper */
#define SOC_GPIO_PIN_EINK_EN  _PINNUM(1, 11) // P1.11
#define SOC_GPIO_PIN_EINK_DC  _PINNUM(0, 28) // P0.28
#define SOC_GPIO_PIN_EINK_RST _PINNUM(0,  2) // P0.02
#define SOC_GPIO_PIN_EINK_BSY _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_EINK_PWR _PINNUM(0, 12) // P0.12

#define EXCLUDE_WIFI
#define EXCLUDE_CC13XX
//#define EXCLUDE_TEST_MODE

/* Component                         Cost */
/* -------------------------------------- */
#define USE_NMEALIB
#define USE_NMEA_CFG               //  +    kb
//#define EXCLUDE_BMP180           //  -    kb
//#define EXCLUDE_BMP280           //  -    kb
//#define EXCLUDE_MPL3115A2        //  -    kb
//#define EXCLUDE_NRF905           //  -    kb
//#define EXCLUDE_MAVLINK          //  -    kb
//#define EXCLUDE_UATM             //  -    kb
//#define EXCLUDE_EGM96            //  -    kb
//#define EXCLUDE_LED_RING         //  -    kb

#define USE_BASICMAC
//#define EXCLUDE_SX1276           //  -  3 kb

#define USE_OLED                   //  +    kb

/* SoftRF/PSoC PFLAU NMEA sentence extension(s) */
#define PFLAU_EXT1_FMT  ",%06X,%d,%d,%d,%d"
#define PFLAU_EXT1_ARGS ,ThisAircraft.addr,settings->rf_protocol,rx_packets_counter,tx_packets_counter,(int)(SoC->Battery_voltage()*100)

#if !defined(EXCLUDE_LED_RING)
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;
#endif /* EXCLUDE_LED_RING */

#endif /* PLATFORM_NRF52_H */

#endif /* ARDUINO_ARCH_NRF52 */
