/*
 * Platform_ASR66.h
 * Copyright (C) 2022 Linar Yusupov
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
#if defined(__ASR6601__) || defined(ARDUINO_ARCH_ASR6601)

#ifndef PLATFORM_ASR66_H
#define PLATFORM_ASR66_H

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_OCTAVE

#define isValidFix()            isValidGNSSFix()

#define uni_begin()             strip.begin()
#define uni_show()              strip.show()
#define uni_setPixelColor(i, c) strip.setPixelColor(i, c)
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

#define yield()                 ({ })
#define snprintf_P              snprintf
#define EEPROM_commit()         ({ }) // EEPROM.commit() has an issue

#define LED_STATE_ON            HIGH  // State when LED is litted

#if !defined(NOT_AN_INTERRUPT)
#define NOT_AN_INTERRUPT        -1
#endif
#if !defined(digitalPinToInterrupt)
#define digitalPinToInterrupt(p) (p == 33 ? NOT_AN_INTERRUPT : p)
#endif

#define isPrintable(c)          (isprint(c) == 0 ? false : true)

#define SerialOutput            Serial
#define SERIAL_FLUSH()          ({ while (Serial         .availableForWrite() < 1024); })
#define GNSS_FLUSH()            ({ while (Serial_GNSS_Out.availableForWrite() < 1024); })

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

#define Serial_GNSS_In        Serial1
#define Serial_GNSS_Out       Serial3
#define UATSerial             Serial

#define SOC_ADC_VOLTAGE_DIV   2 // Vbat 100k/100k voltage divider

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  16
#define SOC_GPIO_PIN_CONS_TX  17

#define SOC_GPIO_PIN_GNSS_RX  4
#define SOC_GPIO_PIN_UART1_TX 5  /* NC */

#define SOC_GPIO_PIN_UART3_RX 29 /* NC */
#define SOC_GPIO_PIN_GNSS_TX  44

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN

#define SOC_GPIO_PIN_GNSS_PPS 3
#define SOC_GPIO_PIN_STATUS   SOC_UNUSED_PIN

#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BATTERY  SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_MOSI     10
#define SOC_GPIO_PIN_MISO     11
#define SOC_GPIO_PIN_SCK      8
#define SOC_GPIO_PIN_SS       9

/* NRF905 */
#define SOC_GPIO_PIN_TXE      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_CE       SOC_UNUSED_PIN
#define SOC_GPIO_PIN_PWR      SOC_UNUSED_PIN

/* SX1262 */
#define SOC_GPIO_PIN_RST      LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_BUSY     LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_DIO1     LMIC_UNUSED_PIN

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX 59
#define SOC_GPIO_PIN_ANT_VDD  45

/* I2C2 */
#define SOC_GPIO_PIN_SDA2     15
#define SOC_GPIO_PIN_SCL2     14
#define Wire                  Wire2

/* button */
#define SOC_GPIO_PIN_BUTTON   2

#define EXCLUDE_WIFI
#define EXCLUDE_CC13XX
#define EXCLUDE_TEST_MODE
#define EXCLUDE_WATCHOUT_MODE
#define EXCLUDE_TRAFFIC_FILTER_EXTENSION
#define EXCLUDE_LK8EX1

//#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
//#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
//#define EXCLUDE_GNSS_AT65
#define EXCLUDE_LOG_GNSS_VERSION
//#define GNSS_MASTER_ID        0xc5fb3201

/* Component                         Cost */
/* -------------------------------------- */
#define USE_NMEA_CFG             //  +    kb
#define EXCLUDE_BMP180           //  -    kb
//#define EXCLUDE_BMP280         //  -    kb
#define EXCLUDE_MPL3115A2        //  -    kb
#define EXCLUDE_NRF905           //  -    kb
#define EXCLUDE_UATM             //  -    kb
#define EXCLUDE_MAVLINK          //  -    kb
#define EXCLUDE_EGM96            //  - 16 kb
#define EXCLUDE_LED_RING         //  -    kb
#define EXCLUDE_SOUND

#define USE_BASICMAC
#define EXCLUDE_SX1276           //  -  3 kb

#define USE_TIME_SLOTS

//#define USE_OGN_ENCRYPTION

#define USE_OLED                 //  +    kb
#define EXCLUDE_OLED_049
//#define EXCLUDE_OLED_BARO_PAGE
#define EXCLUDE_IMU

/* trade performance for flash memory usage (-4 Kb) */
#define cosf(x)                 cos  ((double) (x))
#define sinf(x)                 sin  ((double) (x))
/* has no effect yet */
//#define sqrtf(x)              sqrt ((double) (x))
//#define atan2f(y,x)           atan2((double) (y), (double) (x))

/*
 * https://github.com/HelTecAutomation/ASR650x-Arduino/commit/01fea70929a44d9339af149650e7256059098b30
 */
//#define BAT_MON_DISABLE

#if !defined(EXCLUDE_LED_RING)
#include <CubeCell_NeoPixel.h>

extern CubeCell_NeoPixel strip;
#endif /* EXCLUDE_LED_RING */

#if defined(USE_OLED)
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C
#endif /* USE_OLED */

#endif /* PLATFORM_ASR66_H */

#endif /* __ASR6601__ || ARDUINO_ARCH_ASR6601 */
