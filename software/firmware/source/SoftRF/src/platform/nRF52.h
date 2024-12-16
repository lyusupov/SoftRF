/*
 * Platform_nRF52.h
 * Copyright (C) 2020-2025 Linar Yusupov
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

#if defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_ARCH_NRF52840)

#ifndef PLATFORM_NRF52_H
#define PLATFORM_NRF52_H

#include <avr/dtostrf.h>
#if !defined(ARDUINO_ARCH_MBED)
#include <pcf8563.h>
#endif /* ARDUINO_ARCH_MBED */

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

// State when LED is litted
#if defined(LED_STATE_ON)
#undef  LED_STATE_ON
#define LED_STATE_ON            (hw_info.model == SOFTRF_MODEL_CARD ? HIGH : LOW)
#else
#define LED_STATE_ON            LOW
#endif /* LED_STATE_ON */

#define SerialOutput            Serial1
#define USBSerial               Serial
#define Serial_GNSS_In          Serial2
#define Serial_GNSS_Out         Serial_GNSS_In
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
  NRF52_LILYGO_TECHO_REV_2,     /* 2021-3-26 */
  NRF52_LILYGO_TULTIMA,
  NRF52_SEEED_T1000E,
  NRF52_HELTEC_T114,
};

enum nRF52_display_id {
  EP_UNKNOWN,
  EP_GDEH0154D67,
  EP_GDEP015OC1,
  EP_DEPG0150BN,
  EP_GDEY037T03,
  TFT_LH114TIF03,
};

typedef struct {
  uint64_t         id;
  nRF52_board_id   rev;
  nRF52_display_id panel;
  uint8_t          tag;
} __attribute__((packed)) prototype_entry_t;

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

#if !defined(_PINNUM)
#define _PINNUM(port, pin)    ((port)*32 + (pin))
#endif

#define DFU_MAGIC_SKIP        (0x6d)

#define BMM150_ADDRESS        (0x10)
#define QMA6100P_ADDRESS      (0x12)
#define XL9555_ADDRESS        (0x20) /* A0 = A1 = A2 = LOW */
#define FT6X36_ADDRESS        (0x38)
#define M10Q_ADDRESS          (0x42)
#define BQ27220_ADDRESS       (0x55)
#define DRV2605_ADDRESS       (0x5A)
#define MPU9250_ADDRESS       (0x68)
#define ICM20948_ADDRESS      (0x68)
#define BME280_ADDRESS        (0x77)

#if defined(ARDUINO_ARCH_MBED)
#define PCF8563_SLAVE_ADDRESS (0x51)

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define ARDUINO_CORE_VERSION  STR(CORE_MAJOR) "." STR(CORE_MINOR) "." STR(CORE_PATCH)
#endif /* ARDUINO_ARCH_MBED */

#define MIDI_CHANNEL_TRAFFIC  1
#define MIDI_CHANNEL_VARIO    2

#include "iomap/LilyGO_TEcho.h"
#include "iomap/LilyGO_TUltima.h"
#include "iomap/Seeed_T1000E.h"
#include "iomap/Heltec_T114.h"

#define SOC_GPIO_LED_PCA10059_STATUS    _PINNUM(0,  6) // P0.06
#define SOC_GPIO_LED_PCA10059_GREEN     _PINNUM(1,  9) // P1.09 (Green)
#define SOC_GPIO_LED_PCA10059_RED       _PINNUM(0,  8) // P0.08 (Red)
#define SOC_GPIO_LED_PCA10059_BLUE      _PINNUM(0, 12) // P0.12 (Blue)

#define SOC_GPIO_PIN_STATUS   (hw_info.model == SOFTRF_MODEL_CARD ? SOC_GPIO_LED_T1000_GREEN : \
                               hw_info.model == SOFTRF_MODEL_COZY ? SOC_GPIO_LED_T114_GREEN  : \
                               hw_info.revision == 0 ? SOC_GPIO_LED_TECHO_REV_0_GREEN : \
                               hw_info.revision == 1 ? SOC_GPIO_LED_TECHO_REV_1_GREEN : \
                               hw_info.revision == 2 ? SOC_GPIO_LED_TECHO_REV_2_GREEN : \
                               SOC_GPIO_LED_PCA10059_STATUS)

#define SOC_GPIO_LED_USBMSC   (hw_info.revision == 0 ? SOC_GPIO_LED_TECHO_REV_0_RED : \
                               hw_info.revision == 1 ? SOC_GPIO_LED_TECHO_REV_1_RED : \
                               hw_info.revision == 2 ? SOC_GPIO_LED_TECHO_REV_2_RED : \
                               SOC_GPIO_LED_PCA10059_RED)

#define SOC_GPIO_LED_BLE      (hw_info.revision == 0 ? SOC_GPIO_LED_TECHO_REV_0_BLUE : \
                               hw_info.revision == 1 ? SOC_GPIO_LED_TECHO_REV_1_BLUE : \
                               hw_info.revision == 2 ? SOC_GPIO_LED_TECHO_REV_2_BLUE : \
                               SOC_GPIO_LED_PCA10059_BLUE)

#define SOC_GPIO_PIN_PCA10059_MOSI      _PINNUM(0, 22) // P0.22
#define SOC_GPIO_PIN_PCA10059_MISO      _PINNUM(0, 13) // P0.13
#define SOC_GPIO_PIN_PCA10059_SCK       _PINNUM(0, 14) // P0.14

#define SOC_GPIO_PIN_WB_MOSI            _PINNUM(1, 12) // P1.12
#define SOC_GPIO_PIN_WB_MISO            _PINNUM(1, 13) // P0.13
#define SOC_GPIO_PIN_WB_SCK             _PINNUM(1, 11) // P1.11
#define SOC_GPIO_PIN_WB_SS              _PINNUM(1, 10) // P1.10

/* SX1262 or SX1276 */
#define SOC_GPIO_PIN_PCA10059_RST       _PINNUM(0, 15) // P0.15

#define SOC_GPIO_PIN_WB_RST             _PINNUM(1,  6) // P1.06
#define SOC_GPIO_PIN_WB_DIO1            _PINNUM(1, 15) // P1.15
#define SOC_GPIO_PIN_WB_BUSY            _PINNUM(1, 14) // P1.14

/* RF antenna switch */
#define SOC_GPIO_PIN_WB_TXEN            _PINNUM(1,  7) // P1.07
#define SOC_GPIO_PIN_WB_RXEN            _PINNUM(1,  5) // P1.05

/* buttons */
#define SOC_GPIO_PIN_PCA10059_BUTTON    _PINNUM(1,  6) // P1.06

#define SOC_GPIO_PIN_BUTTON   (nRF52_board == NRF52_NORDIC_PCA10059 ? \
                               SOC_GPIO_PIN_PCA10059_BUTTON :         \
                               SOC_GPIO_PIN_TECHO_REV_0_BUTTON)

#define EXCLUDE_WIFI
//#define EXCLUDE_OTA
//#define USE_ARDUINO_WIFI
//#define USE_WIFI_NINA         false
//#define USE_WIFI_CUSTOM       true

//#include <WiFiEspAT.h>

//#define SPIWIFI               WiFiSPI
//#define SPIWIFI_SS            _PINNUM(0, 6)
//#define ESP32_RESETN          _PINNUM(1, 3)
//#define SPIWIFI_ACK           _PINNUM(0,15)
//#define ESP32_GPIO0           -1
//#include <WiFiNINA.h>
//#include <WiFiNINA_Generic.h>
//#define Serial_setDebugOutput(x) ({})
//#define WIFI_STA_TIMEOUT      20000

#define EXCLUDE_CC13XX
//#define EXCLUDE_TEST_MODE
//#define EXCLUDE_SOFTRF_HEARTBEAT
//#define EXCLUDE_LK8EX1

#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_MTK
//#define EXCLUDE_GNSS_GOKE     /* 'Air530' GK9501 GPS/GLO/BDS (GAL inop.) */
//#define EXCLUDE_GNSS_AT65     /* Quectel L76K */
#define EXCLUDE_GNSS_UC65
//#define EXCLUDE_GNSS_AG33

/* Component                         Cost */
/* -------------------------------------- */
#define USE_NMEALIB
#define USE_NMEA_CFG               //  +    kb
#define USE_SKYVIEW_CFG            //  +    kb
//#define EXCLUDE_BMP180           //  -    kb
//#define EXCLUDE_BMP280           //  -    kb
#define EXCLUDE_BME680             //  -    kb
#define EXCLUDE_BME280AUX          //  -    kb
//#define EXCLUDE_MPL3115A2        //  -    kb
//#define EXCLUDE_NRF905           //  -    kb
//#define EXCLUDE_MAVLINK          //  -    kb
//#define EXCLUDE_UATM             //  -    kb
//#define EXCLUDE_EGM96            //  -    kb
//#define EXCLUDE_LED_RING         //  -    kb

#define USE_BASICMAC
//#define EXCLUDE_SX1276           //  -  3 kb

//#define USE_OLED                 //  +    kb
//#define EXCLUDE_OLED_BARO_PAGE
//#define EXCLUDE_OLED_049
#define USE_EPAPER                 //  +    kb
#define EPD_ASPECT_RATIO_1C1
#define USE_EPD_TASK
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
#if !defined(ARDUINO_ARCH_MBED)
#define USE_BLE_MIDI
#define ENABLE_REMOTE_ID
#define USE_EXT_I2S_DAC
#define USE_TFT
#define USE_RADIOLIB
//#define ENABLE_RECORDER
//#define ENABLE_NFC
#else
#undef USE_EPAPER
//#define EXCLUDE_BLUETOOTH
#define USE_ARDUINOBLE
#define EXCLUDE_IMU
#define EXCLUDE_BME280AUX
#endif /* ARDUINO_ARCH_MBED */
//#define EXCLUDE_PMU

/* FTD-012 data port protocol version 8 and 9 */
#define PFLAA_EXT1_FMT  ",%d,%d,%d"
#define PFLAA_EXT1_ARGS ,Container[i].no_track,data_source,Container[i].rssi

#if defined(USE_PWM_SOUND)
#define SOC_GPIO_PIN_BUZZER   (nRF52_board == NRF52_SEEED_T1000E ? SOC_GPIO_PIN_T1000_BUZZER : \
                               hw_info.rf != RF_IC_SX1262 ? SOC_UNUSED_PIN           : \
                               hw_info.revision == 1 ? SOC_GPIO_PIN_TECHO_REV_1_DIO0 : \
                               hw_info.revision == 2 ? SOC_GPIO_PIN_TECHO_REV_2_DIO0 : \
                               SOC_UNUSED_PIN)

#define ALARM_TONE_HZ         2480 // seems to be the best value for 27 mm piezo buzzer
#else
#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN
#endif /* USE_PWM_SOUND */

#if !defined(EXCLUDE_LED_RING)
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;
#endif /* EXCLUDE_LED_RING */

#if !defined(PIN_SERIAL2_RX) && !defined(PIN_SERIAL2_TX)
extern Uart Serial2;
#endif

#if !defined(ARDUINO_ARCH_MBED)
extern PCF8563_Class *rtc;
#endif /* ARDUINO_ARCH_MBED */
extern const char *nRF52_Device_Manufacturer, *nRF52_Device_Model, *Hardware_Rev[];

#if defined(USE_EPAPER)
typedef void EPD_Task_t;
#endif /* USE_EPAPER */

#if defined(USE_TFT)
#define LV_HOR_RES                      (135) // Horizontal
#define LV_VER_RES                      (240) // Vertical
#endif /* USE_TFT */

#endif /* PLATFORM_NRF52_H */

#endif /* ARDUINO_ARCH_NRF52 */
