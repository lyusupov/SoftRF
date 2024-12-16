/*
 * Platform_LPC43.h
 * Copyright (C) 2021-2025 Linar Yusupov
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

#if defined(HACKRF_ONE)

#ifndef PLATFORM_LPC43_H
#define PLATFORM_LPC43_H

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS  8

#define DEFAULT_SOFTRF_MODEL  SOFTRF_MODEL_ES
#define PLAT_LPC43_NAME       "LPC43"

#define USBSerial             Serial
#define SerialOutput          Serial1
#define Serial_GNSS_In        Serial4
#define Serial_GNSS_Out       Serial_GNSS_In

#define SERIAL_FLUSH()        LPC43_USB_CDC_Sync()

#define isValidFix()          isValidGNSSFix()

#define EEPROM_commit()       EEPROM.commit()

#define LED_STATE_ON          HIGH // State when LED is litted

#define UDP_PACKET_BUFSIZE    128  // GDL90 buffer

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
};

typedef enum {
        USB_DATA_GDL90,
        USB_DATA_D1090,
        USB_DATA_NMEA,
} usb_data_t;

#ifdef __cplusplus
#include <Uart.h>

#if defined(static_assert)
#undef static_assert
#endif

extern Uart Serial1;
extern Uart Serial4;

extern "C" bool dfu_button_state(void);

extern void LPC43_USB_CDC_Sync(void);

#endif /* __cplusplus */

extern usb_data_t usb_data_type;

/* Peripherals */
#define SOC_GPIO_PIN_CONS_RX  (P2_1)
#define SOC_GPIO_PIN_CONS_TX  (P2_0)

#define SOC_GPIO_PIN_GNSS_RX  (P2_4)
#define SOC_GPIO_PIN_GNSS_TX  (P2_3)
#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN /* P2_13 */

/* I2C */
#define SOC_GPIO_PIN_SDA      (I2C0_SDA)
#define SOC_GPIO_PIN_SCL      (I2C0_SCL)

#define SOC_GPIO_PIN_BATTERY  SOC_UNUSED_PIN
#define SOC_GPIO_PIN_BUTTON   (P2_8) /* active HIGH */

#define SOC_GPIO_RADIO_LED_RX (P4_2)  /* Yellow */
#define SOC_GPIO_RADIO_LED_TX (P6_12) /* Red */
#define SOC_GPIO_LED_USB      (P4_1)  /* Green */

#define EXCLUDE_WIFI
#define EXCLUDE_LED_RING
#define EXCLUDE_SOUND
#define EXCLUDE_LK8EX1
#define EXCLUDE_EGM96
#define EXCLUDE_CC13XX
#define EXCLUDE_NRF905
#define EXCLUDE_UATM
#define EXCLUDE_SX1276
#define EXCLUDE_OGLEP3
#define EXCLUDE_MAVLINK
#define EXCLUDE_TEST_MODE
#define EXCLUDE_WATCHOUT_MODE
#define EXCLUDE_TRAFFIC_FILTER_EXTENSION

//#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
//#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65
#define EXCLUDE_GNSS_UC65
#define EXCLUDE_GNSS_AG33
#define EXCLUDE_LOG_GNSS_VERSION

#define EXCLUDE_BMP180
//#define EXCLUDE_BMP280
#define EXCLUDE_BME680
#define EXCLUDE_BME280AUX
#define EXCLUDE_MPL3115A2

#define USE_NMEA_CFG

#define USE_OLED
#define EXCLUDE_OLED_049
#define EXCLUDE_OLED_BARO_PAGE
#define EXCLUDE_IMU

/* trade performance for flash memory usage (-5 Kb) */
#define cosf(x)               cos  ((double) (x))
#define sinf(x)               sin  ((double) (x))
#define sqrtf(x)              sqrt ((double) (x))
//#define atanf(x)            atan ((double) (x))
#define atan2f(y,x)           atan2((double) (y), (double) (x))
//#define powf(x,y)           pow  ((double) (x), (double) (y))

#if defined(USE_OLED)
#define U8X8_OLED_I2C_BUS_TYPE  U8X8_SSD1306_128X64_NONAME_HW_I2C

extern bool LPC43_OLED_probe_func();

#define plat_oled_probe_func LPC43_OLED_probe_func
#endif /* USE_OLED */

#endif /* PLATFORM_LPC43_H */
#endif /* HACKRF_ONE */
