/*
 * Platform_LPC43.h
 * Copyright (C) 2021 Linar Yusupov
 *
 */

#if defined(HACKRF_ONE)

#ifndef PLATFORM_LPC43_H
#define PLATFORM_LPC43_H

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS  8

#define SerialOutput          Serial
#define swSer                 Serial

#define isValidFix()          isValidGNSSFix()

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
