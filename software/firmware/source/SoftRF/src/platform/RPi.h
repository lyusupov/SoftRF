/*
 * Platform_RPi.h
 * Copyright (C) 2018-2025 Linar Yusupov
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

#if defined(RASPBERRY_PI)

#ifndef PLATFORM_RPI_H
#define PLATFORM_RPI_H

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS  8

#define DEFAULT_SOFTRF_MODEL    SOFTRF_MODEL_RASPBERRY

//#include <raspi/HardwareSerial.h>
#include <raspi/TTYSerial.h>

#include "JSON.h"

#define SerialOutput          Serial
#define Serial_GNSS_In        Serial1
#define Serial_GNSS_Out       Serial_GNSS_In
#define UATSerial             Serial2

#define isValidFix()          (isValidGNSSFix() || isValidGPSDFix())

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

struct rst_info {
  uint32_t reason;
  uint32_t exccause;
  uint32_t epc1;
  uint32_t epc2;
  uint32_t epc3;
  uint32_t excvaddr;
  uint32_t depc;
};

/* Dragino LoRa/GPS HAT */
#if defined(USE_LGPIO)
#if defined(USE_SPI1)
#define SOC_GPIO_PIN_MOSI     20
#define SOC_GPIO_PIN_MISO     19
#define SOC_GPIO_PIN_SCK      21
#define SOC_GPIO_PIN_SS       16
#define SOC_GPIO_PIN_RST      26
#define SOC_GPIO_PIN_DIO0     13  // IRQ on GPIO13 so P1 connector pin #33
#define SOC_GPIO_PIN_BUSY     12
#else
#define SOC_GPIO_PIN_MOSI     10
#define SOC_GPIO_PIN_MISO     9
#define SOC_GPIO_PIN_SCK      11
#define SOC_GPIO_PIN_SS       25 // Slave Select on GPIO25 so P1 connector pin #22
#define SOC_GPIO_PIN_RST      17 // Reset on GPIO17 so P1 connector pin #11
#define SOC_GPIO_PIN_DIO0     4 // IRQ on GPIO4 so P1 connector pin #7
#define SOC_GPIO_PIN_BUSY     LMIC_UNUSED_PIN /* TBD */
#endif

#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN // 1 /* rev. 1.4 only */
#else /* BCM */

#if defined(USE_SPI1)
#define SOC_GPIO_PIN_MOSI     RPI_V2_GPIO_P1_38
#define SOC_GPIO_PIN_MISO     RPI_V2_GPIO_P1_35
#define SOC_GPIO_PIN_SCK      RPI_V2_GPIO_P1_40
#define SOC_GPIO_PIN_SS       RPI_V2_GPIO_P1_36
#define SOC_GPIO_PIN_RST      RPI_V2_GPIO_P1_37
#define SOC_GPIO_PIN_DIO0     RPI_V2_GPIO_P1_33  // IRQ on GPIO13 so P1 connector pin #33
#define SOC_GPIO_PIN_BUSY     LMIC_UNUSED_PIN
#else
#define SOC_GPIO_PIN_MOSI     RPI_V2_GPIO_P1_19
#define SOC_GPIO_PIN_MISO     RPI_V2_GPIO_P1_21
#define SOC_GPIO_PIN_SCK      RPI_V2_GPIO_P1_23
#define SOC_GPIO_PIN_SS       RPI_V2_GPIO_P1_22 // Slave Select on GPIO25 so P1 connector pin #22
#define SOC_GPIO_PIN_RST      RPI_V2_GPIO_P1_11 // Reset on GPIO17 so P1 connector pin #11
#define SOC_GPIO_PIN_DIO0     RPI_V2_GPIO_P1_07 // IRQ on GPIO4 so P1 connector pin #7
#define SOC_GPIO_PIN_BUSY     LMIC_UNUSED_PIN
#endif

#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN // RPI_V2_GPIO_P1_12 /* rev. 1.4 */
#endif /* GPIO */

#define SOC_GPIO_PIN_LED      SOC_UNUSED_PIN
#define SOC_GPIO_PIN_STATUS   SOC_UNUSED_PIN

#if defined(USE_SPI1)
#define JSON_SRV_TCP_PORT     30008
#else
#define JSON_SRV_TCP_PORT     30007
#endif

extern TTYSerial Serial1;
extern TTYSerial Serial2;

extern const char *Hardware_Rev[];

#define EXCLUDE_WIFI
#define EXCLUDE_ETHERNET
#define EXCLUDE_LED_RING
#define EXCLUDE_SOUND
#define EXCLUDE_EEPROM
#define EXCLUDE_CC13XX
#define EXCLUDE_LK8EX1

#define USE_NMEALIB
//#define USE_EPAPER

#define TAKE_CARE_OF_MILLIS_ROLLOVER

//#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
//#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65
#define EXCLUDE_GNSS_UC65
#define EXCLUDE_GNSS_AG33

#define EXCLUDE_BMP180
#define EXCLUDE_BMP280
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

//#define USE_OGN_RF_DRIVER
//#define WITH_RFM95
//#define WITH_RFM69
//#define WITH_SX1272
//#define WITH_SI4X32

#if defined(USE_EPAPER)
typedef void* EPD_Task_t;
#endif /* USE_EPAPER */

#endif /* PLATFORM_RPI_H */

#endif /* RASPBERRY_PI */
