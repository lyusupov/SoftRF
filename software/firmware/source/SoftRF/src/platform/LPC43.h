/*
 * Platform_LPC43.h
 * Copyright (C) 2021-2022 Linar Yusupov
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

#define USBSerial             Serial
#define SerialOutput          Serial
#define swSer                 Serial

#define isValidFix()          isValidGNSSFix()

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

#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN

#define EXCLUDE_WIFI
#define EXCLUDE_LED_RING
#define EXCLUDE_SOUND
#define EXCLUDE_EEPROM
#define EXCLUDE_LK8EX1
#define EXCLUDE_EGM96
#define EXCLUDE_CC13XX
#define EXCLUDE_NRF905
#define EXCLUDE_UATM
#define EXCLUDE_SX1276
#define EXCLUDE_OGLEP3
#define EXCLUDE_MAVLINK
#define EXCLUDE_TEST_MODE
#define EXCLUDE_TRAFFIC_FILTER_EXTENSION

#define EXCLUDE_GNSS_UBLOX
#define EXCLUDE_GNSS_SONY
#define EXCLUDE_GNSS_MTK
#define EXCLUDE_GNSS_GOKE
#define EXCLUDE_GNSS_AT65

#define EXCLUDE_BMP180
#define EXCLUDE_BMP280
#define EXCLUDE_MPL3115A2

#endif /* PLATFORM_LPC43_H */
#endif /* HACKRF_ONE */
