/*
 * Platform_STM32.h
 * Copyright (C) 2019 Linar Yusupov
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
#if defined(ARDUINO_ARCH_STM32)

#ifndef PLATFORM_STM32_H
#define PLATFORM_STM32_H

#include "IPAddress.h"
#include <Adafruit_NeoPixel.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define isValidFix()            isValidGNSSFix()

#define uni_begin()             strip.begin()
#define uni_show()              strip.show()
#define uni_setPixelColor(i, c) strip.setPixelColor(i, c)
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

#define swSer                   Serial2
#define UATSerial               Serial3
#define yield()                 ({ })
#define snprintf_P              snprintf
#define EEPROM_commit()         {}

/* Primary target hardware (S76G) */
#if defined(ARDUINO_NUCLEO_L073RZ)

/* Peripherals */
#define SOC_GPIO_PIN_SWSER_RX PA_3
#define SOC_GPIO_PIN_SWSER_TX PA_2

/* SPI */
#define SOC_GPIO_PIN_MOSI     PB_15
#define SOC_GPIO_PIN_MISO     PB_14
#define SOC_GPIO_PIN_SCK      PB_13
#define SOC_GPIO_PIN_SS       PB_12

/* SX1276 */
#define SOC_GPIO_PIN_RST      PB_10
#define SOC_GPIO_PIN_DIO0     PB_11

/* I2C */
#define SOC_GPIO_PIN_SDA      PB_7
#define SOC_GPIO_PIN_SCL      PB_6

/* Secondary target ("Blue pill") */
#elif defined(ARDUINO_BLUEPILL_F103C8)

#define SOC_ADC9_VOLTAGE_DIVIDER  (4096.0 / 3.3)

/* Peripherals */
#define SOC_GPIO_PIN_SWSER_RX PA3
#define SOC_GPIO_PIN_SWSER_TX PA2
#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN // PA8
#define SOC_GPIO_PIN_BUZZER   PB8
#define SOC_GPIO_PIN_BATTERY  PB1

#define SOC_GPIO_PIN_RX3      PB11
#define SOC_GPIO_PIN_TX3      PB10

#define SOC_GPIO_PIN_MODE_PULLDOWN INPUT_PULLDOWN
#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN   // PA1
#define SOC_GPIO_PIN_STATUS   LED_GREEN

/* SPI */
#define SOC_GPIO_PIN_MOSI     PA7
#define SOC_GPIO_PIN_MISO     PA6
#define SOC_GPIO_PIN_SCK      PA5
#define SOC_GPIO_PIN_SS       PA4

/* NRF905 */
#define SOC_GPIO_PIN_TXE      PB4
#define SOC_GPIO_PIN_CE       PB3
#define SOC_GPIO_PIN_PWR      PB5

/* SX1276 (RFM95W) */
#define SOC_GPIO_PIN_RST      PB5
#define SOC_GPIO_PIN_DIO0     PB4

/* I2C */
#define SOC_GPIO_PIN_SDA      PB7
#define SOC_GPIO_PIN_SCL      PB6

// button
#define SOC_GPIO_PIN_BUTTON   USER_BTN

#define SSD1306_OLED_I2C_ADDR 0x3C

/* Component                         Cost */
/* -------------------------------------- */
/* USB Serial */                 //  + 10 kb
//#define USE_OLED               //  +3.5 kb
#define USE_NMEA_CFG             //  +2.5 kb
//#define EXCLUDE_BMP180         //  -  1 kb
//#define EXCLUDE_BMP280         //  -  2 kb
#define EXCLUDE_MPL3115A2        //  -  1 kb
//#define USE_OGN_RF_DRIVER
//#define WITH_RFM95
//#define WITH_RFM69
#define RFM69_POWER_RATING  1 /* 0 - RFM69xx , 1 - RFM69Hxx */
//#define WITH_SX1272
//#define WITH_SI4X32

#else
#error "This hardware platform is not supported!"
#endif

extern Adafruit_NeoPixel strip;

#endif /* PLATFORM_STM32_H */

#endif /* ARDUINO_ARCH_STM32 */
