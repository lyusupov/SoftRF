/*
 * Platform_RK35.h
 * Copyright (C) 2025 Linar Yusupov
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

#if defined(LUCKFOX_LYRA)

#ifndef PLATFORM_RK35_H
#define PLATFORM_RK35_H

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS  8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_LYRA

//#include <raspi/HardwareSerial.h>
#include <raspi/TTYSerial.h>

#include "JSON.h"

#define SerialOutput          Serial
#define Serial_GNSS_In        Serial1
#define Serial_GNSS_Out       Serial_GNSS_In
#define UATSerial             Serial2

#define isValidFix()          (isValidGNSSFix() || isValidGPSDFix())

#if defined(USE_NEOPIXEL)
#define uni_begin()             ws281x_init()
#define uni_show()              ws281x_show()
#define uni_setPixelColor(i, c) ws281x_setPixelColor(i, c)
#define uni_numPixels()         ws281x_numPixels()
#define uni_Color(r,g,b)        ws281x_Color(r, g, b)
#define color_t                 uint32_t
#else

#define EXCLUDE_LED_RING

#endif /* USE_NEOPIXEL */

#define EEPROM_commit()       {}

#define LED_STATE_ON          HIGH  // State when LED is litted

enum rst_reason {
  REASON_DEFAULT_RST      = 0,  /* normal startup by power on */
  REASON_WDT_RST          = 1,  /* hardware watch dog reset */
  REASON_EXCEPTION_RST    = 2,  /* exception reset, GPIO status won't change */
  REASON_SOFT_WDT_RST     = 3,  /* software watch dog reset, GPIO status won't change */
  REASON_SOFT_RESTART     = 4,  /* software restart ,system_restart , GPIO status won't change */
  REASON_DEEP_SLEEP_AWAKE = 5,  /* wake up from deep-sleep */
  REASON_EXT_SYS_RST      = 6   /* external system reset */
};

enum RK35_board_id {
  RK35_LUCKFOX_LYRA_B,  /* Pico form factor */
  RK35_LUCKFOX_LYRA_ZW, /* Zero Wireless    */
};

enum RK35_hat_id {
  RK35_WAVESHARE_PICO_LORA,     /* Waveshare Pico-LoRa-SX1262 */
  RK35_WAVESHARE_HAT_LORA_GNSS, /* Waveshare SX1262-L76K HAT  */
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

/* Peripherals */

/* Content of /etc/luckfox.cfg for Lyra board

TP_STATUS=0
SPI0_STATUS=1
SPI0_SPEED=2000000
SPI0_SCLK_RM_IO=7
SPI0_MOSI_RM_IO=6
SPI0_MISO_RM_IO=5
SPI0_CS_RM_IO=13
I2C1_STATUS=1
I2C1_SPEED=400000
I2C1_SDA_RM_IO=27
I2C1_SCL_RM_IO=26
SPI1_STATUS=1
SPI1_SPEED=8000000
SPI1_SCLK_RM_IO=3
SPI1_MOSI_RM_IO=4
SPI1_MISO_RM_IO=9
SPI1_CS_RM_IO=10
UART1_STATUS=1
UART1_RX_RM_IO=1
UART1_TX_RM_IO=0

*/

#define SOC_GPIO_PIN_CONS_RX  23 // RMIO23
#define SOC_GPIO_PIN_CONS_TX  22 // RMIO22

/* Waveshare Pico-LoRa-SX1262-868M, SX1262 */
#define SOC_GPIO_PIN_MOSI     6  // RMIO6
#define SOC_GPIO_PIN_MISO     5  // RMIO5
#define SOC_GPIO_PIN_SCK      7  // RMIO7
#define SOC_GPIO_PIN_SS       13 // RMIO13
#define SOC_GPIO_PIN_RST      2  // RMIO2
#define SOC_GPIO_PIN_BUSY     12 // RMIO12
#define SOC_GPIO_PIN_DIO0     27 // RMIO27, may cause conflict with SDA

/* Waveshare Pico-GPS-L76B (MTK) */
#define SOC_GPIO_PIN_GNSS_RX  1  // RMIO1
#define SOC_GPIO_PIN_GNSS_TX  0  // RMIO0
#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN // RMIO31, R20 (NC by default)

/* Waveshare Pico-Environment-Sensor, BME280 */
#define SOC_GPIO_PIN_SDA      27 // RMIO27
#define SOC_GPIO_PIN_SCL      26 // RMIO26

#define SOC_GPIO_PIN_NEOPIXEL 4  // RMIO4
#define SOC_GPIO_PIN_LED      SOC_GPIO_PIN_NEOPIXEL
#define SOC_GPIO_PIN_STATUS   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BUZZER   (RK35_board == RK35_LUCKFOX_LYRA_B ? \
                               8 /* RMIO8 */ : SOC_UNUSED_PIN)

/* Content of /etc/luckfox.cfg for Lyra Zero W board - TBD */

// Luckfox Lyra Zero W with Waveshare SX1262+L76K HAT
#define SOC_GPIO_PIN_HAT_MOSI 6  // RMIO6
#define SOC_GPIO_PIN_HAT_MISO 7  // RMIO7
#define SOC_GPIO_PIN_HAT_SCK  8  // RMIO8
#define SOC_GPIO_PIN_HAT_SS   18 // RMIO18
#define SOC_GPIO_PIN_HAT_RST  14 // RMIO14
#define SOC_GPIO_PIN_HAT_DIO  29 // RMIO29
#define SOC_GPIO_PIN_HAT_BUSY 17 // RMIO17
#define SOC_GPIO_PIN_HAT_SDA  0  // RMIO0
#define SOC_GPIO_PIN_HAT_SCL  1  // RMIO1
#define SOC_GPIO_PIN_HAT_GNSS_RX 23 // RMIO23
#define SOC_GPIO_PIN_HAT_GNSS_TX 22 // RMIO22
#define SOC_GPIO_PIN_HAT_GNSS_PPS   SOC_UNUSED_PIN

#define HTTP_SRV_PORT         8081 /* port 8080 can cause conflict with dump1090 */
#define JSON_SRV_TCP_PORT     30007

extern TTYSerial Serial1;
extern TTYSerial Serial2;

extern const char *Hardware_Rev[];

#define EXCLUDE_WIFI
#define EXCLUDE_ETHERNET
//#define EXCLUDE_SOUND
//#define EXCLUDE_EEPROM
#define EXCLUDE_CC13XX
#define EXCLUDE_LK8EX1

#define USE_NMEALIB
//#define USE_EPAPER

#define TAKE_CARE_OF_MILLIS_ROLLOVER

//#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
//#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
//#define EXCLUDE_GNSS_AT65
#define EXCLUDE_GNSS_UC65
#define EXCLUDE_GNSS_AG33

#define EXCLUDE_BMP180
//#define EXCLUDE_BMP280
#define EXCLUDE_BME680
#define EXCLUDE_BME280AUX
#define EXCLUDE_MPL3115A2
//#define EXCLUDE_MAVLINK

//#define EXCLUDE_LR11XX
#define EXCLUDE_CC1101
#define EXCLUDE_SI443X
#define EXCLUDE_SI446X
#define EXCLUDE_SX1231
#define EXCLUDE_SX1280

//#define USE_TIME_SLOTS
//#define USE_OGN_ENCRYPTION
//#define ENABLE_D1090_INPUT

/* Experimental */
#define ENABLE_ADSL
//#define ENABLE_PROL
#define USE_OLED

//#define USE_OGN_RF_DRIVER
//#define WITH_RFM95
//#define WITH_RFM69
//#define WITH_SX1272
//#define WITH_SI4X32

#if defined(USE_EPAPER)
typedef void* EPD_Task_t;
#endif /* USE_EPAPER */

#if defined(USE_OLED)
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_HW_I2C
#define plat_oled_probe_func    RK35_OLED_probe_func

extern char* itoa(int, char *, int);
extern bool RK35_OLED_probe_func(void);
#endif /* USE_OLED */

#if defined(USE_NEOPIXEL)
extern int ws281x_init(void);
extern void ws281x_show(void);
extern int ws281x_numPixels(void);
extern void ws281x_setPixelColor(int, color_t);
extern color_t ws281x_Color(uint8_t, uint8_t, uint8_t);
#endif /* USE_NEOPIXEL */

#endif /* PLATFORM_RK35_H */

#endif /* LUCKFOX_LYRA */
