/*
 * Platform_PSoC4.h
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
#if defined(__ASR6501__)

#ifndef PLATFORM_PSOC4_H
#define PLATFORM_PSOC4_H

#include <board-config.h>
#include <Adafruit_NeoPixel.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_MULTI

#define isValidFix()            isValidGNSSFix()

#define uni_begin()             strip.begin()
#define uni_show()              strip.show()
#define uni_setPixelColor(i, c) strip.setPixelColor(i, c)
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

#define yield()                 ({ })
#define snprintf_P              snprintf
#define EEPROM_commit()         EEPROM.commit()

#define digitalPinToInterrupt(p) p
#define isPrintable(c)          (isprint(c) == 0 ? false : true)

#define SSD1306_OLED_I2C_ADDR   0x3C

#define SOC_GPIO_PIN_MODE_PULLDOWN INPUT

#define SerialOutput            Serial

// button
#define SOC_GPIO_PIN_BUTTON     USER_BTN

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

#define swSer                 Serial
#define UATSerial             Serial

#define SERIAL_OUT_BR         STD_OUT_BR
#define SERIAL_OUT_BITS       -1

#define SOC_ADC_VOLTAGE_DIV   1
#define VREFINT               1200  // mV

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  UART_RX
#define SOC_GPIO_PIN_CONS_TX  UART_TX

#define SOC_GPIO_PIN_SWSER_RX UART_RX2
#define SOC_GPIO_PIN_SWSER_TX UART_TX2

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN

#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN
#define SOC_GPIO_PIN_STATUS   SOC_UNUSED_PIN

#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BATTERY  SOC_UNUSED_PIN

#define SOC_GPIO_PIN_RX3      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TX3      SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_MOSI     RADIO_MOSI  // P4_0
#define SOC_GPIO_PIN_MISO     RADIO_MISO  // P4_1
#define SOC_GPIO_PIN_SCK      RADIO_SCLK  // P4_2
#define SOC_GPIO_PIN_SS       RADIO_NSS   // P4_3

/* NRF905 */
#define SOC_GPIO_PIN_TXE      RADIO_BUSY
#define SOC_GPIO_PIN_CE       SCL
#define SOC_GPIO_PIN_PWR      RADIO_RESET

/* SX1262 */
#define SOC_GPIO_PIN_RST      RADIO_RESET // P5_7
#define SOC_GPIO_PIN_BUSY     RADIO_BUSY  // P4_7
#define SOC_GPIO_PIN_DIO1     RADIO_DIO_1 // P4_6

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX RADIO_ANT_SWITCH_POWER  // P6_1

/* I2C */
#define SOC_GPIO_PIN_SDA      SDA         // P0_1
#define SOC_GPIO_PIN_SCL      SCL         // P0_0

#define EXCLUDE_WIFI
#define EXCLUDE_CC13XX
#define EXCLUDE_TEST_MODE

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

//#define USE_BASICMAC
//#define EXCLUDE_SX1276           //  -  3 kb

extern Adafruit_NeoPixel strip;

#endif /* PLATFORM_PSOC4_H */

#endif /* __ASR6501__ */
