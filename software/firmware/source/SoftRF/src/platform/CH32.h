/*
 * Platform_CH32.h
 * Copyright (C) 2024 Linar Yusupov
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
#if defined(ARDUINO_ARCH_CH32)

#ifndef PLATFORM_CH32_H
#define PLATFORM_CH32_H

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_ACADEMY

#define isValidFix()            isValidGNSSFix()

#define uni_begin()             strip.begin()
#define uni_show()              strip.show()
#define uni_setPixelColor(i, c) strip.setPixelColor(i, c)
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

#define EEPROM_commit()         {}

#define LED_STATE_ON            LOW  // State when LED is litted

#define SerialOutput            Serial /* TBD */

#define USBSerial               Serial /* TBD */
#define Serial_GNSS_In          Serial /* TBD */
#define Serial_GNSS_Out         Serial_GNSS_In
#define UATSerial               Serial /* TBD */

#define SOC_ADC_VOLTAGE_DIV     2 /* TBD */

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

#if defined(CH32V30x)

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  PIN_SERIAL_RX  // TBD
#define SOC_GPIO_PIN_CONS_TX  PIN_SERIAL_TX  // TBD

#define SOC_GPIO_PIN_GNSS_RX  PA10
#define SOC_GPIO_PIN_GNSS_TX  PA9

#define SOC_GPIO_PIN_STATUS   SOC_UNUSED_PIN /* PA15 PB4 */
#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN

#define SOC_GPIO_PIN_RX3      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TX3      SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_MOSI     PB15
#define SOC_GPIO_PIN_MISO     PB14
#define SOC_GPIO_PIN_SCK      PB13
#define SOC_GPIO_PIN_SS       PB12

/* NRF905 */
#define SOC_GPIO_PIN_TXE      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_CE       SOC_UNUSED_PIN
#define SOC_GPIO_PIN_PWR      SOC_UNUSED_PIN

/* SX1276 */
#define SOC_GPIO_PIN_RST      PB0
#define SOC_GPIO_PIN_BUSY     SOC_UNUSED_PIN
#define SOC_GPIO_PIN_DIO1     PB8

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX SOC_UNUSED_PIN

/* I2C , EEPROM */
#define SOC_GPIO_PIN_SDA      PIN_WIRE_SDA   // PB11
#define SOC_GPIO_PIN_SCL      PIN_WIRE_SCL   // PB10

/* USB */
#define SOC_GPIO_PIN_USB1_DN  PA11
#define SOC_GPIO_PIN_USB1_DP  PA12

#define SOC_GPIO_PIN_USB2_DN  PB6
#define SOC_GPIO_PIN_USB2_DP  PB7

/* SWD */
#define SOC_GPIO_PIN_SWD_DIO  PA13
#define SOC_GPIO_PIN_SWD_CLK  PA14

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_GNSS_PPS PA3
#define SOC_GPIO_PIN_BATTERY  PA0
#define SOC_GPIO_PIN_BUTTON   SOC_UNUSED_PIN /* PB3 */

#define SOC_GPIO_RADIO_LED_RX SOC_UNUSED_PIN
#define SOC_GPIO_RADIO_LED_TX SOC_UNUSED_PIN

/* uSD */
#define SOC_GPIO_PIN_SD_D0    PC8
#define SOC_GPIO_PIN_SD_D1    PC9
#define SOC_GPIO_PIN_SD_D2    PC10
#define SOC_GPIO_PIN_SD_D3    PC11
#define SOC_GPIO_PIN_SD_CLK   PC12
#define SOC_GPIO_PIN_SD_CMD   PD2
#define SOC_GPIO_PIN_SD_SW    PD7

/* W25Qxx */
#define SOC_GPIO_PIN_FL_MOSI  PA7
#define SOC_GPIO_PIN_FL_MISO  PA6
#define SOC_GPIO_PIN_FL_CLK   PA5
#define SOC_GPIO_PIN_FL_SS    PA2

#else
#error "This CH32 build variant is not supported!"
#endif

//#define EXCLUDE_EEPROM
#define EXCLUDE_WIFI
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
#define USE_NMEA_CFG             //  +    kb
#define EXCLUDE_BMP180           //  -    kb
//#define EXCLUDE_BMP280         //  -    kb
#define EXCLUDE_BME680           //  -    kb
#define EXCLUDE_BME280AUX        //  -    kb
#define EXCLUDE_MPL3115A2        //  -    kb
#define EXCLUDE_NRF905           //  -    kb
#define EXCLUDE_UATM             //  -    kb
#define EXCLUDE_MAVLINK          //  -    kb
//#define EXCLUDE_EGM96          //  -    kb
#define EXCLUDE_LED_RING         //  -    kb
#define EXCLUDE_SOUND

#define USE_OLED                 //       kb
#define EXCLUDE_OLED_049
//#define EXCLUDE_OLED_BARO_PAGE

#define USE_TIME_SLOTS
#define USE_OGN_ENCRYPTION

//#define ENABLE_RECORDER

#if !defined(EXCLUDE_LED_RING)
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;
#endif /* EXCLUDE_LED_RING */

#if defined(USE_OLED)
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_HW_I2C
#endif /* USE_OLED */

extern void CH32_attachInterrupt_func(uint32_t pin, void (*)(void), int mode);

#define plat_attachInterrupt_func CH32_attachInterrupt_func

#define DEBUG 0 /* https://github.com/openwch/arduino_core_ch32/issues/125 */

#endif /* PLATFORM_CH32_H */

#endif /* ARDUINO_ARCH_CH32 */
