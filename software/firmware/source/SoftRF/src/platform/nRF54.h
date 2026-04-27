/*
 * Platform_nRF54.h
 * Copyright (C) 2026 Linar Yusupov
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

#if defined(ARDUINO_ARCH_NRF54L15CLEAN)

#ifndef PLATFORM_NRF54_H
#define PLATFORM_NRF54_H

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

extern char *dtostrf_workaround(double, signed char, unsigned char, char *);
#define dtostrf                 dtostrf_workaround

#define snprintf_P              snprintf
#define EEPROM_commit()         EEPROM.commit()

//inline  void yield()            { };

#if defined(ARDUINO_HOLYIOT_25007_NRF54L15) || \
    defined(ARDUINO_NRF54L15DK_PCA10156)    || \
    defined(ARDUINO_GENERIC_NRF54L15_MODULE_36PIN)
#undef LED_STATE_ON
#endif /* GENERIC */
// State when LED is litted
#define LED_STATE_ON            LOW

#define SerialOutput            Serial
#define Serial_GNSS_In          Serial1
#define Serial_GNSS_Out         Serial_GNSS_In

enum rst_reason {
  REASON_DEFAULT_RST      = 0,  /* normal startup by power on */
  REASON_WDT_RST          = 1,  /* hardware watch dog reset */
  REASON_EXCEPTION_RST    = 2,  /* exception reset, GPIO status won't change */
  REASON_SOFT_WDT_RST     = 3,  /* software watch dog reset, GPIO status won't change */
  REASON_SOFT_RESTART     = 4,  /* software restart ,system_restart , GPIO status won't change */
  REASON_DEEP_SLEEP_AWAKE = 5,  /* wake up from deep-sleep */
  REASON_EXT_SYS_RST      = 6   /* external system reset */
};

enum nRF54_board_id {
  NRF54_LR2021EVK1XCS1, /* Seeed + Semtech */
  NRF54_MX25LE02,       /* Minewsemi       */
  NRF54_PCA10156,       /* Nordic Semi DK  */
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

#if !defined(_PINNUM)
#define _PINNUM(port, pin)    ((port)*32 + (pin))
#endif

#define ARDUINO_CORE_VERSION  ARDUINO_NRF54L15_CLEAN_VERSION_STRING

#include "iomap/Semtech_LR2021EVK1XCS1.h"
#include "iomap/Minewsemi_MX25LE02.h"
#include "iomap/Nordic_PCA10156.h"

#define SOC_GPIO_PIN_GNSS_PPS (hw_info.model == SOFTRF_MODEL_ACADEMY  ? \
                               SOC_GPIO_PIN_GNSS_EVK_PPS : SOC_UNUSED_PIN)

#define SOC_GPIO_PIN_STATUS   (hw_info.model == SOFTRF_MODEL_ACADEMY ? \
                               SOC_GPIO_PIN_EVK_STATUS : SOC_UNUSED_PIN)

#define SOC_GPIO_LED_BLE      SOC_UNUSED_PIN

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN

//#define EXCLUDE_EEPROM
#define EXCLUDE_WIFI
#define EXCLUDE_ETHERNET

#define EXCLUDE_CC13XX
//#define EXCLUDE_TEST_MODE
//#define EXCLUDE_SOFTRF_HEARTBEAT
#define EXCLUDE_LK8EX1

//#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE     /* 'Air530' GK9501 GPS/GLO/BDS (GAL inop.) */
#define EXCLUDE_GNSS_AT65     /* Quectel L76K */
#define EXCLUDE_GNSS_UC65
#define EXCLUDE_GNSS_AG33

/* Component                         Cost */
/* -------------------------------------- */
//#define USE_NMEALIB            //  +  8 kb
#define USE_NMEA_CFG             //  +    kb
//#define USE_SKYVIEW_CFG        //  +    kb
#define EXCLUDE_BMP180           //  -    kb
//#define EXCLUDE_BMP280         //  -    kb
#define EXCLUDE_BME680           //  -    kb
#define EXCLUDE_BME280AUX        //  -    kb
#define EXCLUDE_MPL3115A2        //  -    kb
#define EXCLUDE_NRF905           //  -    kb
//#define EXCLUDE_MAVLINK        //  -    kb
#define EXCLUDE_UATM             //  -    kb
#define EXCLUDE_EGM96            //  - 16 kb
//#define EXCLUDE_LED_RING       //  -    kb
#define USE_OLED                 //  +  6 kb

#define USE_BASICMAC
//#define EXCLUDE_SX1276         //  -  3 kb

#define USE_TIME_SLOTS

/* Experimental */
//#define USE_WEBUSB_SERIAL
//#define USE_WEBUSB_SETTINGS
//#define USE_USB_MIDI
#define USE_PWM_SOUND
//#define USE_GDL90_MSL
//#define USE_IBEACON
//#define EXCLUDE_NUS
//#define EXCLUDE_IMU
#define USE_OGN_ENCRYPTION
#define ENABLE_ADSL
#define ENABLE_PROL

#define USE_RADIOLIB
//#define EXCLUDE_LR11XX
#if defined(USE_RADIOLIB)
#include <BuildOpt.h>
#if RADIOLIB_VERSION_MAJOR <= 7 && RADIOLIB_VERSION_MINOR < 6
#define EXCLUDE_LR20XX
#endif /* RADIOLIB_VERSION */
#endif /* USE_RADIOLIB */
#define EXCLUDE_CC1101
#define EXCLUDE_SI443X
#define EXCLUDE_SI446X
#define EXCLUDE_SX1231
#define EXCLUDE_SX1280
//#define ENABLE_RECORDER
//#define EXCLUDE_BLUETOOTH
#define EXCLUDE_BHI260

//#define USE_RTT

/* FTD-012 data port protocol version 8 and 9 */
#define PFLAA_EXT1_FMT  ",%d,%d,%d"
#define PFLAA_EXT1_ARGS ,Container[i].no_track,data_source,Container[i].rssi

#if defined(USE_PWM_SOUND)
#define SOC_GPIO_PIN_BUZZER   (nRF54_board == NRF54_LR2021EVK1XCS1 ? \
                               SOC_GPIO_PIN_EVK_BUZZER : SOC_UNUSED_PIN)

#define ALARM_TONE_HZ         2480 // seems to be the best value for 27 mm piezo buzzer
#else
#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN
#endif /* USE_PWM_SOUND */

#if !defined(EXCLUDE_LED_RING)
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;
#endif /* EXCLUDE_LED_RING */

#if defined(USE_OLED)
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_HW_I2C
#endif /* USE_OLED */

#if defined(USE_RTT)
#include <RTTStream.h>
extern RTTStream RTTSerial;

#undef Serial
#define Serial                RTTSerial
#endif /* USE_RTT */

extern const char *nRF5x_Device_Manufacturer, *nRF5x_Device_Model, *Hardware_Rev[];

//#include <SoftSPI.h>
//extern  SoftSPI RadioSPI;
//#undef  SPI
//#define SPI                   RadioSPI
//#define USE_SOFTSPI

#endif /* PLATFORM_NRF54_H */
#endif /* ARDUINO_ARCH_NRF54L15CLEAN */
