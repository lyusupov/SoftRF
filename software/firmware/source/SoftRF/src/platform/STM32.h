/*
 * Platform_STM32.h
 * Copyright (C) 2019-2022 Linar Yusupov
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

#if defined(ARDUINO_ARCH_STM32)
#include "IPAddress.h"
#include "stm32yyxx_ll_adc.h"
#endif /* ARDUINO_ARCH_STM32 */

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_RETRO

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

#define SerialOutput            Serial1

#define AN3155_BR               115200
#define AN3155_BITS             SERIAL_8E1

#define LED_STATE_ON            HIGH  // State when LED is litted

/* Analog read resolution */
#if ADC_RESOLUTION == 10
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_10B
#define ADC_RANGE 1024
#else
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_12B
#define ADC_RANGE 4096
#endif

#define ICM20948_ADDRESS        (0x69)

enum rst_reason {
  REASON_DEFAULT_RST      = 0,  /* normal startup by power on */
  REASON_WDT_RST          = 1,  /* hardware watch dog reset */
  REASON_EXCEPTION_RST    = 2,  /* exception reset, GPIO status won't change */
  REASON_SOFT_WDT_RST     = 3,  /* software watch dog reset, GPIO status won't change */
  REASON_SOFT_RESTART     = 4,  /* software restart ,system_restart , GPIO status won't change */
  REASON_DEEP_SLEEP_AWAKE = 5,  /* wake up from deep-sleep */
  REASON_EXT_SYS_RST      = 6   /* external system reset */
};

enum stm32_board_id {
  STM32_BLUE_PILL,
  STM32_TTGO_TWATCH_EB_1_3,
  STM32_TTGO_TWATCH_EB_1_6,
  STM32_TTGO_TMOTION_1_1,
  STM32_TTGO_TIMPULSE_1_0,
  STM32_OLIMEX_WLE5CC, /* 15ba:0044 */
  STM32_EBYTE_E77,
  STM32_SEEED_E5,
  STM32_ACSIP_ST50H,
};

enum stm32_boot_action {
  STM32_BOOT_NORMAL,
  STM32_BOOT_SHUTDOWN,
  STM32_BOOT_SERIAL_DEEP_SLEEP
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

typedef struct stm32_backup_struct {
    uint32_t   dr0;         /* not in use ? */
    uint32_t   rtc;         /* in use by AC */
    uint32_t   boot_count;
    uint32_t   boot_action;
    uint32_t   bootloader;  /* in use by AC */
} stm32_backup_t;

#define STM32_BKP_REG_NUM     5 /* L0 has 5, F1 has 10 */
#define BOOT_COUNT_INDEX      2
#define SHUTDOWN_REASON_INDEX 3

/* Primary target hardware (S76G) */
#if defined(ARDUINO_NUCLEO_L073RZ)

#define Serial_GNSS_In        Serial4
#define Serial_GNSS_Out       Serial_GNSS_In
#define UATSerial             Serial2  /* PA3, PA2 */

/* S76G GNSS is operating at 115200 baud (by default) */
#define SERIAL_IN_BR          115200
#define SERIAL_IN_BITS        SERIAL_8N1

/*
 * Make use of AN3155 specs for S76G (valid for SkyWatch only)
 * to simplify SoftRF firmware installation
 * via ESP32 UART bypass code
 */
#define SERIAL_OUT_BR   (hw_info.model == SOFTRF_MODEL_DONGLE ? STD_OUT_BR   : AN3155_BR)
#define SERIAL_OUT_BITS (hw_info.model == SOFTRF_MODEL_DONGLE ? STD_OUT_BITS : AN3155_BITS)

#define SOC_ADC_VOLTAGE_DIV   2.3   // T-Motion has 100k/100k voltage divider
#define VREFINT               1224  // mV, STM32L073 datasheet value

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  PA10
#define SOC_GPIO_PIN_CONS_TX  PA9

#define SOC_GPIO_PIN_GNSS_RX  PC11
#define SOC_GPIO_PIN_GNSS_TX  PC10

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN // PB0

#define SOC_GPIO_PIN_GNSS_RST PB2
#define SOC_GPIO_PIN_GNSS_LS  PC6
#define SOC_GPIO_PIN_GNSS_PPS PB5
#define SOC_GPIO_PIN_STATUS   (hw_info.model == SOFTRF_MODEL_DONGLE ?\
                                PA0 : SOC_UNUSED_PIN)

#define SOC_GPIO_PIN_BUZZER   (hw_info.model == SOFTRF_MODEL_DONGLE ?\
                                PA8 : SOC_UNUSED_PIN)

#define SOC_GPIO_PIN_BATTERY  (hw_info.model == SOFTRF_MODEL_DONGLE   ? PB1 :\
                               hw_info.model == SOFTRF_MODEL_BRACELET ? PC4 :\
                               SOC_UNUSED_PIN)

/* SPI0 */
#define SOC_GPIO_PIN_MOSI     PB15
#define SOC_GPIO_PIN_MISO     PB14
#define SOC_GPIO_PIN_SCK      PB13
#define SOC_GPIO_PIN_SS       PB12

/* NRF905 */
#define SOC_GPIO_PIN_TXE      PB11
#define SOC_GPIO_PIN_CE       PB8  // NC
#define SOC_GPIO_PIN_PWR      PB10

/* SX1276 */
#define SOC_GPIO_PIN_RST      PB10
#define SOC_GPIO_PIN_DIO0     PB11
#define SOC_GPIO_PIN_DIO1     PC13
#define SOC_GPIO_PIN_DIO2     PB9
#define SOC_GPIO_PIN_DIO3     PB4
#define SOC_GPIO_PIN_DIO4     PB3
#define SOC_GPIO_PIN_DIO5     PA15

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX PA1 // 1:Rx, 0:Tx

/* RF clock source */
#define SOC_GPIO_PIN_TCXO_OE  PD_7
#define SOC_GPIO_PIN_OSC_SEL  PC1

/* I2C */
#define SOC_GPIO_PIN_SDA      PB7
#define SOC_GPIO_PIN_SCL      PB6

/* SPI1 */
#define SOC_GPIO_SPI1_MOSI    PA7
#define SOC_GPIO_SPI1_MISO    PA6
#define SOC_GPIO_SPI1_SCK     PA5
#define SOC_GPIO_SPI1_SS      PA4

#define TTGO_TIMPULSE_OLED_PIN_RST PA8
#define TTGO_TIMPULSE_GPIO_PAD_OUT PA0
#define TTGO_TIMPULSE_GPIO_PAD_PWR PA2
#define TTGO_TIMPULSE_SENSOR_INT   PB1 // PB0
#define TTGO_TIMPULSE_GPIO_CHRG    PB8
#define TTGO_TIMPULSE_VDD_1V8_EN   PB0 /* HIGH - enable 1.8V power supply        */
#define TTGO_TIMPULSE_GPS_PWR_EN   PA3 /* HIGH - enable GNSS and LS power supply */

/* button */
//#define SOC_GPIO_PIN_BUTTON   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BUTTON   (hw_info.model == SOFTRF_MODEL_BRACELET ? \
                              TTGO_TIMPULSE_GPIO_PAD_OUT : PC4)
//#define SOC_GPIO_PIN_BUTTON   PA3

#define EXCLUDE_WIFI
#define EXCLUDE_CC13XX
#define EXCLUDE_TEST_MODE

#define TAKE_CARE_OF_MILLIS_ROLLOVER

#define EXCLUDE_GNSS_UBLOX
//#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65

#define USE_OLED                 //  +3.5 kb
//#define EXCLUDE_OLED_049
//#define EXCLUDE_OLED_BARO_PAGE
#define USE_NMEA_CFG             //  +2.5 kb
#define EXCLUDE_MPL3115A2        //  -  1 kb
#define EXCLUDE_NRF905           //  -  2 kb
#define EXCLUDE_EGM96            //  - 16 kb
#define USE_SERIAL_DEEP_SLEEP    //  + 12 kb
//#define USE_BASICMAC           //  +  7 kb
#define EXCLUDE_LED_RING         //  -    kb
#define EXCLUDE_SOUND
//#define USE_GNSS_PSM
//#define USE_GDL90_MSL
//#define USE_OGN_ENCRYPTION
#define EXCLUDE_LK8EX1
#define EXCLUDE_IMU

//#define ENFORCE_S78G
#define USE_TIME_SLOTS

/* Secondary target ("Blue pill") */
#elif defined(ARDUINO_BLUEPILL_F103CB)

#define Serial_GNSS_In        Serial2
#define Serial_GNSS_Out       Serial_GNSS_In
#define UATSerial             Serial3

#define SOC_ADC_VOLTAGE_DIV   1
#define VREFINT               1200  // mV, STM32F103x8 datasheet value

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  PA10
#define SOC_GPIO_PIN_CONS_TX  PA9

#define SOC_GPIO_PIN_GNSS_RX  PA3
#define SOC_GPIO_PIN_GNSS_TX  PA2

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN // PA8

#define SOC_GPIO_PIN_GNSS_PPS PA1
#define SOC_GPIO_PIN_STATUS   LED_GREEN

#define SOC_GPIO_PIN_BUZZER   PB8
#define SOC_GPIO_PIN_BATTERY  PB1

#define SOC_GPIO_PIN_RX3      PB11
#define SOC_GPIO_PIN_TX3      PB10

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

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX LMIC_UNUSED_PIN

/* I2C */
#define SOC_GPIO_PIN_SDA      PB7
#define SOC_GPIO_PIN_SCL      PB6

/* button */
#define SOC_GPIO_PIN_BUTTON   SOC_UNUSED_PIN
//#define SOC_GPIO_PIN_BUTTON   USER_BTN

#define EXCLUDE_WIFI
#define EXCLUDE_CC13XX
#define EXCLUDE_TEST_MODE
#define EXCLUDE_WATCHOUT_MODE

//#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65

//#define EXCLUDE_LOG_GNSS_VERSION

/* Component                         Cost */
/* -------------------------------------- */
/* USB Serial */                 //  + 10 kb
#define USE_OLED                 //  +3.5 kb
#define EXCLUDE_OLED_BARO_PAGE
#define EXCLUDE_OLED_049
#define USE_NMEA_CFG             //  +2.5 kb
//#define EXCLUDE_BMP180         //  -  1 kb
//#define EXCLUDE_BMP280         //  -  2 kb
#define EXCLUDE_MPL3115A2        //  -  1 kb
//#define EXCLUDE_NRF905         //  -  2 kb
#define EXCLUDE_UATM             //  -    kb
#define EXCLUDE_MAVLINK          //  -    kb
#define EXCLUDE_EGM96            //  - 16 kb
#define EXCLUDE_LED_RING         //  -    kb
#define EXCLUDE_SOUND
#define EXCLUDE_LK8EX1
#define EXCLUDE_IMU
//#define USE_OGN_RF_DRIVER
//#define WITH_RFM95
//#define WITH_RFM69
#define RFM69_POWER_RATING  1 /* 0 - RFM69xx , 1 - RFM69Hxx */
//#define WITH_SX1272
//#define WITH_SI4X32

//#define USE_TIME_SLOTS

#elif defined(ARDUINO_GENERIC_WLE5CCUX)

#define Serial_GNSS_In        Serial2
#define Serial_GNSS_Out       Serial_GNSS_In
#define UATSerial             Serial

#define SOC_ADC_VOLTAGE_DIV   1    /* TBD */
#define VREFINT               1200 /* TBD */

#define SOC_GPIO_PIN_CONS_RX  PB7
#define SOC_GPIO_PIN_CONS_TX  PB6

#define SOC_GPIO_PIN_GNSS_RX  PA3
#define SOC_GPIO_PIN_GNSS_TX  PA2
#define SOC_GPIO_PIN_GNSS_PPS PA1 /* PA11 */

#define SOC_GPIO_PIN_STATUS   PA15
#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BATTERY  SOC_UNUSED_PIN /* PB2 */
#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_MOSI     PA7
#define SOC_GPIO_PIN_MISO     PA11
#define SOC_GPIO_PIN_SCK      PA5
#define SOC_GPIO_PIN_SS       PA4

/* NRF905 */
#define SOC_GPIO_PIN_TXE      PB4
#define SOC_GPIO_PIN_CE       PB3
#define SOC_GPIO_PIN_PWR      PB5

/* SX1262 */
#define SOC_GPIO_PIN_RST      LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_BUSY     LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_DIO1     LMIC_UNUSED_PIN

/* RF antenna switch */
#define SOC_GPIO_ANT_RX_OLI   PB8  /* active LOW */
#define SOC_GPIO_ANT_TX_OLI   PC13 /* active LOW */

#define SOC_GPIO_ANT_RX_E77   PA7  /* active HIGH */
#define SOC_GPIO_ANT_TX_E77   PA6  /* active LOW  */

#define SOC_GPIO_ANT_RX_E5    PA5  /* active LOW */
#define SOC_GPIO_ANT_TX_E5    PA4  /* active LOW */

#define SOC_GPIO_ANT_RX_ST50  PA1  /* active LOW */
#define SOC_GPIO_ANT_TX_ST50  PA0  /* active LOW */

/* I2C */
#define SOC_GPIO_PIN_SDA      PA10
#define SOC_GPIO_PIN_SCL      PA9

/* button */
#define SOC_GPIO_PIN_BUTTON   PA0

/* 32768 Hz crystal */
#define SOC_GPIO_PIN_XP       PC14
#define SOC_GPIO_PIN_XN       PC15

#define EXCLUDE_WIFI
#define EXCLUDE_CC13XX
#define EXCLUDE_TEST_MODE
#define EXCLUDE_WATCHOUT_MODE

//#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65

/* Component                         Cost */
/* -------------------------------------- */
#define USE_OLED                 //  +3.5 kb
//#define EXCLUDE_OLED_BARO_PAGE
#define EXCLUDE_OLED_049
#define USE_NMEA_CFG             //  +2.5 kb
#define EXCLUDE_BMP180           //  -  1 kb
//#define EXCLUDE_BMP280         //  -  2 kb
#define EXCLUDE_MPL3115A2        //  -  1 kb
#define EXCLUDE_NRF905           //  -  2 kb
#define EXCLUDE_EGM96            //  - 16 kb
#define EXCLUDE_LED_RING         //  -    kb
#define EXCLUDE_SOUND
#define EXCLUDE_LK8EX1
#define EXCLUDE_IMU

#define USE_BASICMAC             //  +  7 kb
#define EXCLUDE_SX1276           //  -  3 kb

#else
#error "This hardware platform is not supported!"
#endif

#if !defined(EXCLUDE_LED_RING)
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;
#endif /* EXCLUDE_LED_RING */

#if defined(USE_OLED)
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_HW_I2C
#endif /* USE_OLED */

#endif /* PLATFORM_STM32_H */

#endif /* ARDUINO_ARCH_STM32 */
