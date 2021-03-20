/*
 * Platform_PSoC4.h
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
#if defined(__ASR6501__)

#ifndef PLATFORM_PSOC4_H
#define PLATFORM_PSOC4_H

#include <board-config.h>

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

#define LED_STATE_ON            HIGH  // State when LED is litted

#if !defined(digitalPinToInterrupt)
#define digitalPinToInterrupt(p) ( p )
#endif

#define isPrintable(c)          (isprint(c) == 0 ? false : true)

#define SerialOutput            Serial

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

#if defined(CubeCell_GPS)
#define swSer                 Serial1
#else
#define swSer                 Serial
#endif /* CubeCell_GPS */
#define UATSerial             Serial

#define SOC_ADC_VOLTAGE_DIV   2 // HTCC-AB02S has Vbat 100k/100k voltage divider

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  UART_RX
#define SOC_GPIO_PIN_CONS_TX  UART_TX

#define SOC_GPIO_PIN_SWSER_RX UART_RX2
#define SOC_GPIO_PIN_SWSER_TX UART_TX2

#define SOC_GPIO_PIN_STATUS   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BUZZER   SOC_UNUSED_PIN

#define SOC_GPIO_PIN_RX3      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_TX3      SOC_UNUSED_PIN

/* SPI */
#define SOC_GPIO_PIN_MOSI     RADIO_MOSI   // P4_0
#define SOC_GPIO_PIN_MISO     RADIO_MISO   // P4_1
#define SOC_GPIO_PIN_SCK      RADIO_SCLK   // P4_2
#define SOC_GPIO_PIN_SS       RADIO_NSS    // P4_3

/* NRF905 */
#define SOC_GPIO_PIN_TXE      RADIO_BUSY
#define SOC_GPIO_PIN_CE       SCL
#define SOC_GPIO_PIN_PWR      RADIO_RESET

/* SX1262 */
#define SOC_GPIO_PIN_RST      RADIO_RESET  // P5_7
#define SOC_GPIO_PIN_BUSY     RADIO_BUSY   // P4_7
#define SOC_GPIO_PIN_DIO1     RADIO_DIO_1  // P4_6

/* RF antenna switch */
#define SOC_GPIO_PIN_ANT_RXTX RADIO_ANT_SWITCH_POWER  // P6_1

/* I2C */
#define SOC_GPIO_PIN_SDA      SDA          // P0_1
#define SOC_GPIO_PIN_SCL      SCL          // P0_0

#if defined(CubeCell_GPS)
#define SOC_GPIO_PIN_LED      RGB          // P0_6
#define SOC_GPIO_PIN_GNSS_PPS GPIO8
#define SOC_GPIO_PIN_BATTERY  ADC1         // P2_0

#define SOC_GPIO_PIN_GNSS_PWR GPIO14       // P0_7
#define SOC_GPIO_PIN_OLED_RST GPIO10       // P7_2
#define SOC_GPIO_PIN_OLED_PWR Vext         // P3_2
#define SOC_GPIO_PIN_BAT_CTL  VBAT_ADC_CTL // P3_3
#define SOC_GPIO_PIN_BUTTON   USER_KEY     // P3_3

#define SOC_GPIO_PIN_BMON_DIS GPIO7        // P3_7
#else /* CubeCell_GPS */

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BATTERY  SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BUTTON   SOC_UNUSED_PIN
#endif /* CubeCell_GPS */

#define EXCLUDE_WIFI
#define EXCLUDE_CC13XX
#define EXCLUDE_TEST_MODE
#define EXCLUDE_WATCHOUT_MODE
#define EXCLUDE_TRAFFIC_FILTER_EXTENSION
#define EXCLUDE_LK8EX1

#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_MTK
//#define EXCLUDE_GNSS_GOKE
//#define EXCLUDE_GNSS_AT65

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

#if defined(CubeCell_GPS)
#define USE_OLED                 //  +    kb
#define EXCLUDE_OLED_BARO_PAGE
#endif

/* SoftRF/PSoC PFLAU NMEA sentence extension. In use by WebTop adapter */
#define PFLAU_EXT1_FMT  ",%06X,%d,%d,%d"
#define PFLAU_EXT1_ARGS ,ThisAircraft.addr,settings->rf_protocol,rx_packets_counter,tx_packets_counter

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
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_HW_I2C
#endif /* USE_OLED */

#endif /* PLATFORM_PSOC4_H */

#endif /* __ASR6501__ */
