/*
 * Platform_RP2040.h
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
#if defined(ARDUINO_ARCH_RP2040)

#ifndef PLATFORM_RP2040_H
#define PLATFORM_RP2040_H

#include <avr/dtostrf.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_LEGO

#define isValidFix()            isValidGNSSFix()

#define uni_begin()             strip.begin()
#define uni_show()              strip.show()
#define uni_setPixelColor(i, c) strip.setPixelColor(i, c)
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

#define EEPROM_commit()         EEPROM.commit()

#define LED_STATE_ON            HIGH  // State when LED is litted

#define SerialOutput            Serial1

#define USBSerial               Serial
#define Serial_GNSS_In          Serial2
#define Serial_GNSS_Out         Serial_GNSS_In
#define UATSerial               Serial

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

enum RP2040_board_id {
  RP2040_RAK11300,
  RP2040_RPIPICO,
  RP2040_WEACT,
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

#if defined(ARDUINO_RASPBERRY_PI_PICO)

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  (1u)
#define SOC_GPIO_PIN_CONS_TX  (0u)

#define SOC_GPIO_PIN_GNSS_RX  (5u)
#define SOC_GPIO_PIN_GNSS_TX  (4u)

#define SOC_GPIO_PIN_STATUS   SOC_UNUSED_PIN // LED (25u) Pico/WeAct
#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN

/* SPI0 */
#define SOC_GPIO_PIN_MOSI0    (19u)
#define SOC_GPIO_PIN_MISO0    (16u)
#define SOC_GPIO_PIN_SCK0     (18u)
#define SOC_GPIO_PIN_SS0      (17u)

/* SPI1 */
#define SOC_GPIO_PIN_MOSI     (11u)
#define SOC_GPIO_PIN_MISO     (12u)
#define SOC_GPIO_PIN_SCK      (10u)
#define SOC_GPIO_PIN_SS       (13u)

/* NRF905 */
#define SOC_GPIO_PIN_TXE      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_CE       SOC_UNUSED_PIN
#define SOC_GPIO_PIN_PWR      SOC_UNUSED_PIN

/* SX1262 */
#define SOC_GPIO_PIN_RST      (14u)
#define SOC_GPIO_PIN_BUSY     (15u)
#define SOC_GPIO_PIN_DIO1     (29u)

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX (25u) // RXEN

/* I2C0 */
#define SOC_GPIO_PIN_SDA0     (20u)
#define SOC_GPIO_PIN_SCL0     (21u)

/* I2C1 */
#define SOC_GPIO_PIN_SDA      (2u)
#define SOC_GPIO_PIN_SCL      (3u)
#define Wire                  Wire1

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_GNSS_PPS (6u)  // IO1
#define SOC_GPIO_PIN_GNSS_RST (22u) // IO2
#define SOC_GPIO_PIN_BATTERY  SOC_UNUSED_PIN // ADC0 (26u) or ADC1 (27u)
#define SOC_GPIO_PIN_VBUS     (24u) // Pico
#define SOC_GPIO_PIN_VSYS     (29u) // Pico
#define SOC_GPIO_PIN_PS       (23u) // Pico
#define SOC_GPIO_PIN_BUTTON   (23u) // WeAct

#define SOC_GPIO_RADIO_LED_RX SOC_UNUSED_PIN // LED2 (24u)
#define SOC_GPIO_RADIO_LED_TX SOC_UNUSED_PIN

#define SOC_GPIO_PIN_IO1      (6u)
#define SOC_GPIO_PIN_IO2      (22u)
#define SOC_GPIO_PIN_IO3      (7u)
#define SOC_GPIO_PIN_IO4      (28u)
#define SOC_GPIO_PIN_IO5      (9u)
#define SOC_GPIO_PIN_IO6      (8u)
#define SOC_GPIO_PIN_A0       (26u)
#define SOC_GPIO_PIN_A1       (27u)

#else
#error "This RP2040 build variant is not supported!"
#endif

#define EXCLUDE_WIFI
#define EXCLUDE_CC13XX
#define EXCLUDE_TEST_MODE
#define EXCLUDE_WATCHOUT_MODE
//#define EXCLUDE_TRAFFIC_FILTER_EXTENSION
#define EXCLUDE_LK8EX1

//#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
//#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65
//#define EXCLUDE_LOG_GNSS_VERSION

/* Component                         Cost */
/* -------------------------------------- */
#define USE_NMEA_CFG             //  +    kb
#define EXCLUDE_BMP180           //  -    kb
//#define EXCLUDE_BMP280         //  -    kb
#define EXCLUDE_MPL3115A2        //  -    kb
#define EXCLUDE_NRF905           //  -    kb
#define EXCLUDE_UATM             //  -    kb
#define EXCLUDE_MAVLINK          //  -    kb
//#define EXCLUDE_EGM96          //  -    kb
#define EXCLUDE_LED_RING         //  -    kb
//#define EXCLUDE_SOUND

#define USE_OLED                 //       kb
#define EXCLUDE_OLED_049
//#define EXCLUDE_OLED_BARO_PAGE

#define USE_BOOTSEL_BUTTON

#define USE_BASICMAC

#define USE_TIME_SLOTS

#define USE_OGN_ENCRYPTION

#if !defined(EXCLUDE_LED_RING)
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;
#endif /* EXCLUDE_LED_RING */

#if defined(USE_OLED)
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C
#endif /* USE_OLED */

#endif /* PLATFORM_RP2040_H */
#endif /* ARDUINO_ARCH_RP2040 */
