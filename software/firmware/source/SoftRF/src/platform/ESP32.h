/*
 * Platform_ESP32.h
 * Copyright (C) 2018-2024 Linar Yusupov
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

#include "sdkconfig.h"

#define USE_WIFI_NINA           false
#define USE_WIFI_CUSTOM         true
#include <WiFi.h>

#include <ESPmDNS.h>
#include <Update.h>
#include <WiFiClient.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_STANDALONE

#define SerialOutput            Serial
#define SoftwareSerial          HardwareSerial
#define Serial_GNSS_In          Serial1
#define Serial_GNSS_Out         Serial_GNSS_In

#if defined(CONFIG_IDF_TARGET_ESP32)
#define UATSerial               Serial2
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#if ARDUINO_USB_CDC_ON_BOOT
#define UATSerial               Serial0
#undef  SerialOutput
#define SerialOutput            Serial0
#else
#define UATSerial               Serial
#endif /* ARDUINO_USB_CDC_ON_BOOT */
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define UATSerial               Serial2
#define SA8X8_Serial            Serial2
#if ARDUINO_USB_CDC_ON_BOOT
#undef  SerialOutput
#define SerialOutput            Serial0
#endif /* ARDUINO_USB_CDC_ON_BOOT */
#elif defined(CONFIG_IDF_TARGET_ESP32C2) || \
      defined(CONFIG_IDF_TARGET_ESP32C3) || \
      defined(CONFIG_IDF_TARGET_ESP32C6) || \
      defined(CONFIG_IDF_TARGET_ESP32H2)
#if ARDUINO_USB_CDC_ON_BOOT
#define UATSerial               Serial0
#undef  SerialOutput
#define SerialOutput            Serial0
#else
#define UATSerial               Serial
#endif /* ARDUINO_USB_CDC_ON_BOOT */
#else
#error "This ESP32 family build variant is not supported!"
#endif /* CONFIG_IDF_TARGET_ESP32 */

#define EEPROM_commit()         EEPROM.commit()

#define isValidFix()            isValidGNSSFix()

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR>=4
#define WIRE_FINI(bus)          { bus.end(); }
#else
#define WIRE_FINI(bus)          { } /* AC 1.0.x has no Wire.end() */
#endif

#define LED_STATE_ON            HIGH  // State when LED is litted

/* Adafruit_NeoPixel still has "flickering" issue of ESP32 caused by 1 ms scheduler */
//#define USE_ADAFRUIT_NEO_LIBRARY

/*
 * NeoPixelBus is already "flickering-free" on ESP32 (with I2S or RMT)
 * but the "Core" needs update onto the most recent one
 */
#if defined(CONFIG_IDF_TARGET_ESP32C2)
#define EXCLUDE_LED_RING
#elif !defined(CONFIG_IDF_TARGET_ESP32C6) && !defined(CONFIG_IDF_TARGET_ESP32H2)
#define USE_NEOPIXELBUS_LIBRARY
#else
#define USE_ADAFRUIT_NEO_LIBRARY
#endif /* CONFIG_IDF_TARGET_ESP32C6 */

#if !defined(EXCLUDE_LED_RING)
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
#endif /* EXCLUDE_LED_RING */

#define LEDC_CHANNEL_BUZZER     0
#define BACKLIGHT_CHANNEL       ((uint8_t)1)
#define LEDC_RESOLUTION_BUZZER  8

/* Peripherals */
#define SOC_GPIO_PIN_GNSS_RX    23
#define SOC_GPIO_PIN_GNSS_TX    12
#define SOC_GPIO_PIN_BATTERY    36

#if defined(CONFIG_IDF_TARGET_ESP32)
#define SOC_GPIO_PIN_LED        25
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#define SOC_GPIO_PIN_LED        7
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define SOC_GPIO_PIN_LED        SOC_UNUSED_PIN /* TBD 14? */
#elif defined(CONFIG_IDF_TARGET_ESP32C2)
#define SOC_GPIO_PIN_LED        SOC_UNUSED_PIN /* TBD */
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#define SOC_GPIO_PIN_LED        19 /* D1 */
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
#define SOC_GPIO_PIN_LED        3 /* D1 */
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
#define SOC_GPIO_PIN_LED        SOC_UNUSED_PIN /* TBD */
#else
#error "This ESP32 family build variant is not supported!"
#endif

#define SOC_GPIO_PIN_STATUS   (hw_info.model == SOFTRF_MODEL_MIDI      ?\
                                SOC_GPIO_PIN_HELTRK_LED :               \
                               hw_info.model != SOFTRF_MODEL_PRIME_MK2 ?\
                                SOC_UNUSED_PIN :                        \
                                (hw_info.revision == 2 ?                \
                                  SOC_GPIO_PIN_TBEAM_LED_V02 :          \
                                  (hw_info.revision == 5 ?              \
                                    SOC_GPIO_PIN_TBEAM_LED_V05 :        \
                                    (hw_info.revision == 11 ?           \
                                      SOC_GPIO_PIN_TBEAM_LED_V11 :      \
                                      SOC_UNUSED_PIN))))

#define SOC_GPIO_PIN_GNSS_PPS (hw_info.model == SOFTRF_MODEL_PRIME_MK3 ?  \
                                SOC_GPIO_PIN_S3_GNSS_PPS :                \
                                (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ?\
                                  (hw_info.revision >= 8 ?                \
                                    SOC_GPIO_PIN_TBEAM_V08_PPS :          \
                                    SOC_UNUSED_PIN) :                     \
                                (hw_info.model == SOFTRF_MODEL_MIDI ?     \
                                  SOC_GPIO_PIN_HELTRK_GNSS_PPS :          \
                                  SOC_UNUSED_PIN)))

#define SOC_GPIO_PIN_BUZZER   (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ? \
                                SOC_UNUSED_PIN :                         \
                                (esp32_board == ESP32_DEVKIT    ? 13 :   \
                                (esp32_board == ESP32_C3_DEVKIT ?        \
                                SOC_GPIO_PIN_C3_BUZZER :                 \
                                (esp32_board == ESP32_C6_DEVKIT ?        \
                                SOC_GPIO_PIN_C6_BUZZER : SOC_UNUSED_PIN))))

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

#include "iomap/LilyGO_T22.h"
#include "iomap/LilyGO_T3.h"

// Hardware pin definitions for Heltec and TTGO-V1 LoRa-32 Boards with OLED SSD1306 I2C Display
#define HELTEC_OLED_PIN_RST             U8X8_PIN_NONE // 16
#define HELTEC_OLED_PIN_SDA             4
#define HELTEC_OLED_PIN_SCL             15

#include "iomap/LilyGO_TWatch.h"
#include "iomap/LilyGO_T8S2.h"

#define SOC_GPIO_PIN_TDISPLAY_S2_LED    39
#define SOC_GPIO_PIN_TDONGLE_S2_LED     39

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define LV_HOR_RES                      (135) //Horizontal
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define LV_HOR_RES                      (80) //Horizontal
#else
#define LV_HOR_RES                      (240) //Horizontal
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define LV_VER_RES                      (160) //vertical
#else
#define LV_VER_RES                      (240) //vertical
#endif

#include "iomap/LilyGO_TBeam_Supreme.h"

/* ESP32-S3 section 3 (spare pins) */
// 3(S), 21, 39, 45(S), 46(S), 48

// battery voltage (ADC)
#define SOC_GPIO_PIN_S3_BATTERY         3 // (S)

/* auxillary */
// Devkit LEDs, active state - HIGH
#define SOC_GPIO_PIN_S3_LED_RED         5
#define SOC_GPIO_PIN_S3_LED_GREEN       6
#define SOC_GPIO_PIN_S3_LED_BLUE        7
#define SOC_GPIO_PIN_S3_LED_WHITE       38
#define SOC_GPIO_PIN_S3_LED_YELLOW      39

#include "iomap/AiThinker_C3_12F.h"
#include "iomap/LilyGO_TTWR.h"
#include "iomap/Heltec_Tracker.h"
#include "iomap/WT0132C6.h"
#include "iomap/LilyGO_T3C6.h"
#include "iomap/LilyGO_T3S3_EPD.h"

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
  ESP32_S3_DEVKIT,
  ESP32_C2_DEVKIT,
  ESP32_C3_DEVKIT,
  ESP32_C6_DEVKIT,
  ESP32_H2_DEVKIT,
  ESP32_TTGO_V2_OLED,
  ESP32_HELTEC_OLED,
  ESP32_TTGO_T_BEAM,
  ESP32_TTGO_T_BEAM_SUPREME,
  ESP32_TTGO_T_WATCH,
  ESP32_S2_T8_V1_1,
  ESP32_LILYGO_T_TWR2,
  ESP32_HELTEC_TRACKER,
  ESP32_LILYGO_T3C6,
  ESP32_LILYGO_T3S3_EPD,
};

enum ep_model_id {
	EP_UNKNOWN,
	EP_GDEW027W3,
	EP_GDEY027T91,
	EP_DEPG0213BN,
};

/* https://github.com/espressif/usb-pids/blob/main/allocated-pids.txt#L313 */
enum softrf_usb_pid {
  SOFTRF_USB_PID_WEBTOP     = 0x8131,
  SOFTRF_USB_PID_STANDALONE = 0x8132,
  SOFTRF_USB_PID_PRIME_MK3  = 0x8133,
  SOFTRF_USB_PID_UF2_BOOT   = 0x8134,
  SOFTRF_USB_PID_HAM        = 0x818F,
  SOFTRF_USB_PID_MIDI       = 0x81A0,
  SOFTRF_USB_PID_INK        = 0x820A,
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

/* Boya Microelectronics Inc. */
#define BOYA_ID                 0x68
#define BOYA_BY25Q32AL          0x4016

/* ST / SGS/Thomson / Numonyx / XMC(later acquired by Micron) */
#define ST_ID                   0x20
#define XMC_XM25QH32B           0x4016

/* Zbit Semiconductor, Inc. */
#define ZBIT_ID                 0x5E
#define ZBIT_ZB25VQ32B          0x4016

/* Shanghai Fudan Microelectronics Group Co., Ltd. */
#define FMICRO_ID               0xA1
#define FMICRO_FM25Q16          0x4015

#define MakeFlashId(v,d)        ((v << 16) | d)

#define MPU6886_REG_PWR_MGMT_1  (0x6B)
#define MPU6886_REG_WHOAMI      (0x75)
#define MPU9250_ADDRESS         (0x68)
#define MPU9250_REG_PWR_MGMT_1  (0x6B)
#define MPU9250_REG_WHOAMI      (0x75)
#define QMI8658_REG_RESET       (0x60)
#define QMI8658_REG_WHOAMI      (0x00)

/* Disable brownout detection (avoid unexpected reset on some boards) */
#define ESP32_DISABLE_BROWNOUT_DETECTOR 0

#define NMEA_TCP_SERVICE
#define USE_NMEALIB
#define USE_OLED
#define EXCLUDE_OLED_049
//#define EXCLUDE_OLED_BARO_PAGE
#define USE_TFT
#define USE_NMEA_CFG
#define USE_BASICMAC
//#define USE_RADIOLIB

#define USE_TIME_SLOTS

/* Experimental */
#define USE_BLE_MIDI
//#define USE_GDL90_MSL
#define USE_OGN_ENCRYPTION
#define ENABLE_PROL
#define ENABLE_ADSL

//#define EXCLUDE_GNSS_UBLOX    /* Neo-6/7/8, M10 */
#define ENABLE_UBLOX_RFS        /* revert factory settings (when necessary)  */
#define EXCLUDE_GNSS_GOKE       /* 'Air530' GK9501 GPS/GLO/BDS (GAL inop.)   */
//#define EXCLUDE_GNSS_AT65     /* L76K, Air530Z */
#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_MTK
//#define EXCLUDE_GNSS_UC65

#define EXCLUDE_CC13XX
#define EXCLUDE_SOFTRF_HEARTBEAT
#define EXCLUDE_LK8EX1
//#define EXCLUDE_IMU
//#define EXCLUDE_MAG
#define EXCLUDE_BME680

#if defined(CONFIG_IDF_TARGET_ESP32)
//#define ENABLE_BT_VOICE
//#define USE_NIMBLE
#else

#define EXCLUDE_UATM

#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
#define EXCLUDE_NRF905
#define EXCLUDE_LED_RING

/* Experimental */
//#define USE_ADAFRUIT_MSC
//#define USE_USB_HOST

#if defined(USE_USB_HOST)
#undef  SOC_GPIO_PIN_T8_S2_CONS_RX
#undef  SOC_GPIO_PIN_T8_S2_CONS_TX
#define SOC_GPIO_PIN_T8_S2_CONS_RX      46 // 43
#define SOC_GPIO_PIN_T8_S2_CONS_TX      45 // 44

/* Experimental */
#define ENABLE_D1090_INPUT

#include <cdc_acm_host.h>

typedef struct {
    bool         connected;
    int          index;
    CdcAcmDevice *device;
} ESP32_USBSerial_device_t;

typedef struct {
    uint16_t   vid;
    uint16_t   pid;
    uint8_t    type;
    uint8_t    model;
    const char *first_name;
    const char *last_name;
} USB_Device_List_t;

extern ESP32_USBSerial_device_t ESP32_USB_Serial;
extern const USB_Device_List_t supported_USB_devices[];

#endif /* USE_USB_HOST */
#elif defined(CONFIG_IDF_TARGET_ESP32C2) || \
      defined(CONFIG_IDF_TARGET_ESP32C3) || \
      defined(CONFIG_IDF_TARGET_ESP32C6) || \
      defined(CONFIG_IDF_TARGET_ESP32H2)
#undef USE_OLED
#undef USE_TFT
#if defined(CONFIG_IDF_TARGET_ESP32C2) || \
    defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32H2)
#define EXCLUDE_EGM96
#define EXCLUDE_TEST_MODE
#define EXCLUDE_WATCHOUT_MODE
#undef USE_NMEALIB
#undef ENABLE_PROL
//#define USE_NIMBLE
#endif /* C2 || C6 || H2 */
#endif /* SX || CX || H2 */
#endif /* CONFIG_IDF_TARGET_ESP32 */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define USE_U10_EXT
#define ENABLE_RECORDER
#define USE_SA8X8
/* Experimental */
#define ENABLE_REMOTE_ID
//#define EXCLUDE_VOICE_MESSAGE
//#define USE_EPAPER
//#define USE_EPD_TASK
#endif /* S3 */

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define EXCLUDE_BLUETOOTH
#endif /* S2 */

#if defined(CONFIG_IDF_TARGET_ESP32H2)
#define EXCLUDE_WIFI
#undef NMEA_TCP_SERVICE
#endif /* H2 */

#define POWER_SAVING_WIFI_TIMEOUT 600000UL /* 10 minutes */

//#define PMK2_SLEEP_MODE 1    // 0.6 mA : esp_deep_sleep_start()
//#define PMK2_SLEEP_MODE 2    // 0.9 mA : axp.setSleep()
#define PMK2_SLEEP_MODE 3      //  60 uA : axp.shutdown()

#if defined(USE_OLED)
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_HW_I2C
#define ENABLE_OLED_TEXT_PAGE
#else
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
#endif /* USE_OLED */

#if defined(USE_EPAPER)
typedef void EPD_Task_t;

extern const char *Hardware_Rev[];
#endif /* USE_EPAPER */

#endif /* PLATFORM_ESP32_H */

#endif /* ESP32 */
