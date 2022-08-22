/*
 * Platform_CC13XX.h
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
#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)

#ifndef PLATFORM_CC13XX_H
#define PLATFORM_CC13XX_H

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_UAT

#define isValidFix()            isValidGNSSFix()

#define uni_begin()             strip.begin()
#define uni_show()              strip.sendBuffer(LEDs, strip.numPixels())
#define uni_setPixelColor(i, c) ({ uint32_t temp = c;          \
                                   LEDs[i][2] = (temp & 0xff); \
                                   temp = temp >> 8;           \
                                   LEDs[i][1] = (temp & 0xff); \
                                   temp = temp >> 8;           \
                                   LEDs[i][0] = (temp & 0xff); })
#define uni_numPixels()         strip.numPixels()
#define uni_Color(r,g,b)        strip.Color(r,g,b)
#define color_t                 uint32_t

#define SerialOutput            Serial
#define UATSerial               Serial /* TBD */

#if defined(SERIAL_OUT_BITS)
#undef SERIAL_OUT_BITS
#endif
#define SERIAL_OUT_BITS         false

#define yield()                 ({ })
#define snprintf_P              snprintf
#define EEPROM_commit()         EEPROM.commit()

#define LED_STATE_ON            HIGH  // State when LED is litted
#define WD_TIMEOUT_MS           4000

enum rst_reason {
  REASON_DEFAULT_RST      = 0,  /* normal startup by power on */
  REASON_WDT_RST          = 1,  /* hardware watch dog reset */
  REASON_EXCEPTION_RST    = 2,  /* exception reset, GPIO status won't change */
  REASON_SOFT_WDT_RST     = 3,  /* software watch dog reset, GPIO status won't change */
  REASON_SOFT_RESTART     = 4,  /* software restart ,system_restart , GPIO status won't change */
  REASON_DEEP_SLEEP_AWAKE = 5,  /* wake up from deep-sleep */
  REASON_EXT_SYS_RST      = 6   /* external system reset */
};

enum cc13xx_board_id {
  SOFTRF_UAT_MODULE_19,
  SOFTRF_UAT_MODULE_20,
  TI_CC1352R1_LAUNCHXL,
  TI_LPSTK_CC1352R
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

#define USE_NMEA_CFG
#define EXCLUDE_WIFI
//#define EXCLUDE_EEPROM
#define EXCLUDE_LED_RING
#define EXCLUDE_SOUND
#define EXCLUDE_EGM96
#define EXCLUDE_NRF905
#define EXCLUDE_UATM

#include "../../hal_conf_extra.h"   // Sketch-specific definitions are located there

#if defined(ENERGIA_ARCH_CC13XX)

#include <SCSerial.h>

#define Serial_GNSS_In          scSerial
#define Serial_GNSS_Out         Serial_GNSS_In

#if defined(BOARD_CC1310_LAUNCHXL)
/*
 * TI LaunchPad LAUNCHXL-CC1310 or Ebyte E70-XXXT14S
 * MCU: CC1310F128RGZ (7x7 mm)
 */

/*
 * BootLoader                   GPIO 1
 */

#define SOC_GPIO_PIN_CONS_RX    IOID_2
#define SOC_GPIO_PIN_CONS_TX    IOID_3

#define SOC_GPIO_PIN_GNSS_RX    IOID_24
#define SOC_GPIO_PIN_GNSS_TX    IOID_25

#define SOC_GPIO_PIN_GNSS_PPS   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_STATUS     SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BUZZER     GREEN_LED        // GPIO 7

/* Optional SX1276 SPI radio */
#define SOC_GPIO_PIN_SS         18 // GPIO 11
#define SOC_GPIO_PIN_RST        LMIC_UNUSED_PIN

// button
#define SOC_GPIO_PIN_BUTTON     SOC_UNUSED_PIN

#define SOC_GPIO_PIN_LED        15 // MOSI (DIO_09)

#define EXCLUDE_BMP280

#elif defined(BOARD_E70_XXXT14S2)
/*
 * Ebyte E70-XXXT14S2
 * MCU: CC1310F128RSM (4x4 mm)
 */

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX    IOID_1
#define SOC_GPIO_PIN_CONS_TX    IOID_2

#define SOC_GPIO_PIN_GNSS_RX    IOID_6
#define SOC_GPIO_PIN_GNSS_TX    IOID_7

#define SOC_GPIO_PIN_GNSS_PPS   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_STATUS     SOC_UNUSED_PIN // 29 (DIO_00)
#define SOC_GPIO_PIN_BUZZER     SOC_UNUSED_PIN

/* Optional SX1276 SPI radio */
#define SOC_GPIO_PIN_SS         LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_RST        LMIC_UNUSED_PIN

// button
#define SOC_GPIO_PIN_BUTTON     10 // DIO_05

#define SOC_GPIO_PIN_LED        SOC_UNUSED_PIN

/* I2C */
#define SOC_GPIO_PIN_SDA        IOID_8
#define SOC_GPIO_PIN_SCL        IOID_9

//#define USE_OLED              //  +4.2 kb
#define EXCLUDE_OLED_049
#define EXCLUDE_OLED_BARO_PAGE

/* trade performance for flash memory usage (-5.8 Kb) */
#define cosf(x)                 cos  ((double) (x))
#define sinf(x)                 sin  ((double) (x))
#define sqrtf(x)                sqrt ((double) (x))
#define atan2f(y,x)             atan2((double) (y), (double) (x))

#else
#error "This board is not supported!"
#endif /* BOARD_CC1310_LAUNCHXL || BOARD_E70_XXXT14S2 */

#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65

#define EXCLUDE_BMP180
//#define EXCLUDE_BMP280
#define EXCLUDE_MPL3115A2
#define EXCLUDE_D1090

#define EXCLUDE_MAVLINK
#define EXCLUDE_LK8EX1
#define EXCLUDE_WATCHOUT_MODE
#define EXCLUDE_TRAFFIC_FILTER_EXTENSION
#define EXCLUDE_LOG_GNSS_VERSION

//#define USE_TIME_SLOTS

extern SCSerial                 scSerial;

#if defined(USE_OGN_ENCRYPTION)
#error "OGN encryption is not supported by CC1310 platform"
// TinyGPS++.h needs to increase _GPS_MAX_FIELD_SIZE up to 33
#endif

#elif defined(ENERGIA_ARCH_CC13X2)

#if defined(BOARD_CC1352R1_LAUNCHXL) || defined(BOARD_CC1312R1_LAUNCHXL)
/*
 *  UART pins
 *
 * Board_UART0_TX               GPIO 13
 * Board_UART0_RX               GPIO 12
 * BootLoader                   GPIO 15
 *
 */

#define Serial_GNSS_In          Serial2
#define Serial_GNSS_Out         Serial_GNSS_In

#define EasyLink_setRfPwr       EasyLink_setRfPower

#define SOC_GPIO_PIN_GNSS_RX    23  // GPIO 25
#define SOC_GPIO_PIN_GNSS_TX    24  // GPIO 26

#define SOC_GPIO_PIN_GNSS_PPS   25  // GPIO 27

/* Optional SX12XX SPI radio */
#define SOC_GPIO_PIN_SS         36  // GPIO 18 'RTS'
#define SOC_GPIO_PIN_RST        LMIC_UNUSED_PIN
#define SOC_GPIO_PIN_BUSY       37  // GPIO 19 'CTS'

/* Built-in TI SensorTag components */
#define SOC_GPIO_PIN_ADXL_SS    18  // GPIO 11
#define SOC_GPIO_PIN_MX25_SS    38  // GPIO 20
#define SOC_GPIO_PIN_LED_BLUE   8   // GPIO 21

#define SOC_GPIO_PIN_STATUS     GREEN_LED  // GPIO 7
#define SOC_GPIO_PIN_BUZZER     SOC_UNUSED_PIN

// button
#define SOC_GPIO_PIN_BUTTON     PUSH2 // GPIO 14

#define MACRONIX_MX25R8035F     0xC228

#define USE_OLED                 //  +5.5 kb
#define EXCLUDE_OLED_049
//#define EXCLUDE_OLED_BARO_PAGE
//#define EXCLUDE_LK8EX1
#define USE_GNSS_PSM
//#define USE_GDL90_MSL

//#define USE_BASICMAC
#define USE_TIME_SLOTS

#define USE_OGN_ENCRYPTION

//#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
//#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65

#else
#error "This board is not supported!"
#endif /* BOARD_CC1352R1_LAUNCHXL || BOARD_CC1312R1_LAUNCHXL */

#else
#error "This hardware platform is not supported!"
#endif /* ENERGIA_ARCH_CC13X0 & ENERGIA_ARCH_CC13X2 */

extern char *itoa(int, char *, int);

#if defined(ENERGIA_ARCH_CC13X2)
extern size_t strnlen (const char *string, size_t length);
#endif /* ENERGIA_ARCH_CC13X2 */

#if !defined(EXCLUDE_LED_RING)
#include <WS2812.h>
extern WS2812 strip;
extern uint8_t LEDs[][3];
#endif /* EXCLUDE_LED_RING */

#if defined(USE_OLED)

#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_HW_I2C

/*
 * BUG:
 * return value of Wire.endTransmission() is always '4'
 * with Arduino Core(s) for CC13X0 and CC13X2
 */
#define plat_oled_probe_func()    (true)

#endif /* USE_OLED */

#endif /* PLATFORM_CC13XX_H */

#endif /* ENERGIA_ARCH_CC13X0 || ENERGIA_ARCH_CC13X2 */
