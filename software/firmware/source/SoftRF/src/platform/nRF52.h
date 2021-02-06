/*
 * Platform_nRF52.h
 * Copyright (C) 2020-2021 Linar Yusupov
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
#include <pcf8563.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_BADGE

#define isValidFix()            isValidGNSSFix()

#define uni_begin()             strip.begin()
#define uni_show()              strip.show()
#define uni_setPixelColor(i, c) strip.setPixelColor(i, c)
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

#define snprintf_P              snprintf
#define EEPROM_commit()         {}

#if !defined(LED_STATE_ON)
#define LED_STATE_ON            LOW  // State when LED is litted
#endif /* LED_STATE_ON */

#define SerialOutput            Serial1
#define USBSerial               Serial
#define swSer                   Serial2
#define UATSerial               Serial1

enum rst_reason {
  REASON_DEFAULT_RST      = 0,  /* normal startup by power on */
  REASON_WDT_RST          = 1,  /* hardware watch dog reset */
  REASON_EXCEPTION_RST    = 2,  /* exception reset, GPIO status won't change */
  REASON_SOFT_WDT_RST     = 3,  /* software watch dog reset, GPIO status won't change */
  REASON_SOFT_RESTART     = 4,  /* software restart ,system_restart , GPIO status won't change */
  REASON_DEEP_SLEEP_AWAKE = 5,  /* wake up from deep-sleep */
  REASON_EXT_SYS_RST      = 6   /* external system reset */
};

enum nRF52_board_id {
  NRF52_NORDIC_PCA10059,        /* reference low power board */
  NRF52_LILYGO_TECHO_REV_0,     /* 20-8-6 */
  NRF52_LILYGO_TECHO_REV_1,     /* 2020-12-12 */
  NRF52_LILYGO_TECHO_REV_2      /* 2021-1-15 */
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

#define VBAT_MV_PER_LSB       (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define SOC_ADC_VOLTAGE_DIV   (2.0F)          // 100K + 100K voltage divider on VBAT
#define REAL_VBAT_MV_PER_LSB  (SOC_ADC_VOLTAGE_DIV * VBAT_MV_PER_LSB)

#if !defined(_PINNUM)
#define _PINNUM(port, pin)    ((port)*32 + (pin))
#endif

#define DFU_MAGIC_SKIP        (0x6d)
#define BME280_ADDRESS        (0x77)

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  _PINNUM(0, 8) // P0.08
#define SOC_GPIO_PIN_CONS_TX  _PINNUM(0, 6) // P0.06

#define SOC_GPIO_PIN_SWSER_RX _PINNUM(1, 9) // P1.09
#define SOC_GPIO_PIN_SWSER_TX _PINNUM(1, 8) // P1.08

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN

#define SOC_GPIO_PIN_GNSS_PPS _PINNUM(1, 4) // P1.04
#define SOC_GPIO_PIN_GNSS_WKE _PINNUM(1, 2) // P1.02
#define SOC_GPIO_PIN_GNSS_RST _PINNUM(1, 5) // P1.05 (REV_2 only)

#define SOC_GPIO_LED_TECHO_REV_0_GREEN  _PINNUM(0, 13) // P0.13 (Green)
#define SOC_GPIO_LED_TECHO_REV_0_RED    _PINNUM(0, 14) // P0.14 (Red)
#define SOC_GPIO_LED_TECHO_REV_0_BLUE   _PINNUM(0, 15) // P0.15 (Blue)
#define SOC_GPIO_LED_TECHO_REV_1_GREEN  _PINNUM(0, 15)
#define SOC_GPIO_LED_TECHO_REV_1_RED    _PINNUM(0, 13)
#define SOC_GPIO_LED_TECHO_REV_1_BLUE   _PINNUM(0, 14)
#define SOC_GPIO_LED_TECHO_REV_2_GREEN  _PINNUM(1,  3) // P1.03 (Green)
#define SOC_GPIO_LED_TECHO_REV_2_RED    SOC_GPIO_LED_TECHO_REV_0_RED
#define SOC_GPIO_LED_TECHO_REV_2_BLUE   _PINNUM(1,  1) // P1.01 (Blue)

#define SOC_GPIO_LED_PCA10059_STATUS    _PINNUM(0,  6) // P0.06
#define SOC_GPIO_LED_PCA10059_GREEN     _PINNUM(1,  9) // P1.09 (Green)
#define SOC_GPIO_LED_PCA10059_RED       _PINNUM(0,  8) // P0.08 (Red)
#define SOC_GPIO_LED_PCA10059_BLUE      _PINNUM(0, 12) // P0.12 (Blue)

#define SOC_GPIO_PIN_STATUS   (hw_info.revision == 0 ? SOC_GPIO_LED_TECHO_REV_0_GREEN : \
                               hw_info.revision == 1 ? SOC_GPIO_LED_TECHO_REV_1_GREEN : \
                               hw_info.revision == 2 ? SOC_GPIO_LED_TECHO_REV_2_GREEN : \
                               SOC_GPIO_LED_PCA10059_STATUS)

#define SOC_GPIO_LED_USBMSC   (hw_info.revision == 0 ? SOC_GPIO_LED_TECHO_REV_0_RED : \
                               hw_info.revision == 1 ? SOC_GPIO_LED_TECHO_REV_1_RED : \
                               hw_info.revision == 2 ? SOC_GPIO_LED_TECHO_REV_2_RED : \
                               SOC_GPIO_LED_PCA10059_RED)

#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BATTERY  _PINNUM(0, 4) // P0.04 (AIN2)

#define SOC_GPIO_PIN_RX3      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TX3      SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_TECHO_REV_0_MOSI   _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_TECHO_REV_1_MOSI   SOC_GPIO_PIN_TECHO_REV_0_MOSI
#define SOC_GPIO_PIN_TECHO_REV_2_MOSI   SOC_GPIO_PIN_TECHO_REV_0_MOSI
#define SOC_GPIO_PIN_TECHO_REV_0_MISO   _PINNUM(0, 23) // P0.23
#define SOC_GPIO_PIN_TECHO_REV_1_MISO   SOC_GPIO_PIN_TECHO_REV_0_MISO
#define SOC_GPIO_PIN_TECHO_REV_2_MISO   SOC_GPIO_PIN_TECHO_REV_0_MISO
#define SOC_GPIO_PIN_TECHO_REV_0_SCK    _PINNUM(0, 19) // P0.19
#define SOC_GPIO_PIN_TECHO_REV_1_SCK    SOC_GPIO_PIN_TECHO_REV_0_SCK
#define SOC_GPIO_PIN_TECHO_REV_2_SCK    SOC_GPIO_PIN_TECHO_REV_0_SCK
#define SOC_GPIO_PIN_SS                 _PINNUM(0, 24) // P0.24

#define SOC_GPIO_PIN_PCA10059_MOSI      _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_PCA10059_MISO      _PINNUM(0, 13) // P0.13
#define SOC_GPIO_PIN_PCA10059_SCK       _PINNUM(0, 14) // P0.14

/* NRF905 */
#define SOC_GPIO_PIN_TXE      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_CE       SOC_UNUSED_PIN
#define SOC_GPIO_PIN_PWR      SOC_UNUSED_PIN

/* SX1262 or SX1276 */
#define SOC_GPIO_PIN_TECHO_REV_0_RST    _PINNUM(0, 25) // P0.25
#define SOC_GPIO_PIN_TECHO_REV_1_RST    SOC_GPIO_PIN_TECHO_REV_0_RST
#define SOC_GPIO_PIN_TECHO_REV_2_RST    SOC_GPIO_PIN_TECHO_REV_0_RST
#define SOC_GPIO_PIN_PCA10059_RST       _PINNUM(0, 15) // P0.15
#define SOC_GPIO_PIN_TECHO_REV_0_DIO0   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TECHO_REV_1_DIO0   _PINNUM(1,  1) // P1.01
#define SOC_GPIO_PIN_TECHO_REV_2_DIO0   _PINNUM(0, 15) // P0.15
#define SOC_GPIO_PIN_DIO1     _PINNUM(0, 20) // P0.20
#define SOC_GPIO_PIN_BUSY     _PINNUM(0, 17) // P0.17

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX SOC_UNUSED_PIN

/* I2C */
#define SOC_GPIO_PIN_SDA      _PINNUM(0, 26) // P0.26
#define SOC_GPIO_PIN_SCL      _PINNUM(0, 27) // P0.27

/* buttons */
#define SOC_GPIO_PIN_TECHO_REV_0_BUTTON _PINNUM(1, 10) // P1.10
#define SOC_GPIO_PIN_TECHO_REV_1_BUTTON SOC_GPIO_PIN_TECHO_REV_0_BUTTON
#define SOC_GPIO_PIN_TECHO_REV_2_BUTTON SOC_GPIO_PIN_TECHO_REV_0_BUTTON
#define SOC_GPIO_PIN_PCA10059_BUTTON    _PINNUM(1,  6) // P1.06
#define SOC_GPIO_PIN_PAD                _PINNUM(0, 11) // P0.11

#define SOC_GPIO_PIN_BUTTON   SOC_GPIO_PIN_TECHO_REV_0_BUTTON
//#define SOC_GPIO_PIN_BUTTON   SOC_GPIO_PIN_PCA10059_BUTTON

/* E-paper */
#define SOC_GPIO_PIN_EPD_MISO _PINNUM(1,  7) // P1.07
#define SOC_GPIO_PIN_EPD_MOSI _PINNUM(0, 29) // P0.29
#define SOC_GPIO_PIN_EPD_SCK  _PINNUM(0, 31) // P0.31
#define SOC_GPIO_PIN_EPD_SS   _PINNUM(0, 30) // P0.30
#define SOC_GPIO_PIN_EPD_DC   _PINNUM(0, 28) // P0.28
#define SOC_GPIO_PIN_EPD_RST  _PINNUM(0,  2) // P0.02
#define SOC_GPIO_PIN_EPD_BUSY _PINNUM(0,  3) // P0.03
#define SOC_GPIO_PIN_EPD_BLGT _PINNUM(1, 11) // P1.11

/* Power: EINK, RGB, CN1 (, RF) REV_2: FLASH, GNSS, SENSOR */
#define SOC_GPIO_PIN_IO_PWR   _PINNUM(0, 12) // P0.12
/* REV_2 power: RF */
#define SOC_GPIO_PIN_3V3_PWR  _PINNUM(0, 13) // P0.13
/* Modded REV_1 3V3 power */
#define SOC_GPIO_PIN_TECHO_REV_1_3V3_PWR  SOC_GPIO_PIN_TECHO_REV_1_DIO0

/* MX25R1635F SPI flash */
#define SOC_GPIO_PIN_SFL_MOSI _PINNUM(1, 12) // P1.12
#define SOC_GPIO_PIN_SFL_MISO _PINNUM(1, 13) // P1.13
#define SOC_GPIO_PIN_SFL_SCK  _PINNUM(1, 14) // P1.14
#define SOC_GPIO_PIN_SFL_SS   _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_SFL_HOLD _PINNUM(0,  5) // P0.05 (REV_1 and REV_2)
#define SOC_GPIO_PIN_SFL_WP   _PINNUM(0,  7) // P0.07 (REV_1 and REV_2)

/* RTC */
#define SOC_GPIO_PIN_R_INT    _PINNUM(0, 16) // P0.16

#define EXCLUDE_WIFI
#define EXCLUDE_CC13XX
//#define EXCLUDE_TEST_MODE
//#define EXCLUDE_LK8EX1

#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_MTK
//#define EXCLUDE_GNSS_GOKE     /* 'Air530' GK9501 GPS/GLO/BDS (GAL inop.) */
//#define EXCLUDE_GNSS_AT65     /* Quectel L76K */

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

//#define USE_OLED                 //  +    kb
#define USE_EPAPER                 //  +    kb

/* SoftRF/nRF52 PFLAU NMEA sentence extension(s) */
#define PFLAU_EXT1_FMT  ",%06X,%d,%d,%d,%d"
#define PFLAU_EXT1_ARGS ,ThisAircraft.addr,settings->rf_protocol,rx_packets_counter,tx_packets_counter,(int)(Battery_voltage()*100)

#if !defined(EXCLUDE_LED_RING)
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;
#endif /* EXCLUDE_LED_RING */

#if !defined(PIN_SERIAL2_RX) && !defined(PIN_SERIAL2_TX)
extern Uart Serial2;
#endif

typedef struct UI_Settings {
    uint8_t  adapter;

    uint8_t  connection:4;
    uint8_t  units:2;
    uint8_t  zoom:2;

    uint8_t  protocol;
    uint8_t  baudrate;
    char     server  [18];
    char     key     [18];

    uint8_t  resvd1:2;
    uint8_t  orientation:1;
    uint8_t  adb:3;
    uint8_t  idpref:2;

    uint8_t  vmode:2;
    uint8_t  voice:3;
    uint8_t  aghost:3;

    uint8_t  filter:4;
    uint8_t  power_save:4;

    uint32_t team;

    uint8_t  resvd2;
    uint8_t  resvd3;
    uint8_t  resvd4;
    uint8_t  resvd5;
    uint8_t  resvd6;
    uint8_t  resvd7;
    uint8_t  resvd8;
    uint8_t  resvd9;
} __attribute__((packed)) ui_settings_t;

extern PCF8563_Class *rtc;

#if defined(USE_EPAPER)
#include <GxEPD2_BW.h>

typedef void EPD_Task_t;

extern GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> *display;
#endif /* USE_EPAPER */

extern ui_settings_t *ui;

#endif /* PLATFORM_NRF52_H */

#endif /* ARDUINO_ARCH_NRF52 */
