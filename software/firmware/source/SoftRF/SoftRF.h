/*
 * SoftRF.h
 * Copyright (C) 2016-2019 Linar Yusupov
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

#ifndef SOFTRF_H
#define SOFTRF_H

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#if defined(ENERGIA_ARCH_CC13XX)
#include <TimeLib.h>
#endif /* ENERGIA_ARCH_CC13XX */

#if defined(RASPBERRY_PI)
#include <raspi/raspi.h>
#endif /* RASPBERRY_PI */

#define SOFTRF_FIRMWARE_VERSION "1.0-rc6"
#define SOFTRF_UAT_IDENT  "SoftRF-UAT"

#define ENTRY_EXPIRATION_TIME   10 /* seconds */
#define LED_EXPIRATION_TIME     5 /* seconds */
#define EXPORT_EXPIRATION_TIME  5 /* seconds */

/*
 * If you need for SoftRF to operate in wireless
 * client mode - specify your local AP's SSID/PSK:
 *
 * #define MY_ACCESSPOINT_SSID "My_AP_SSID"
 * #define MY_ACCESSPOINT_PSK  "My_AP_PSK"
 *
 * If SoftRF's built-in AP is not stable enough for you, consider
 * to use "reverse" operation when your smartphone is acting
 * as an AP for the SoftRF unit as a client:
 *
 * #define MY_ACCESSPOINT_SSID "AndroidAP"
 * #define MY_ACCESSPOINT_PSK  "12345678"
 */

// Default mode is AP with
// SSID: SoftRF-XXXXXX
// KEY:  12345678
// IP: 192.168.1.1
// NETMASK: 255.255.255.0
#define MY_ACCESSPOINT_SSID ""
#define MY_ACCESSPOINT_PSK  ""

#define RELAY_DST_PORT  12390
#define RELAY_SRC_PORT  (RELAY_DST_PORT - 1)

#define GDL90_DST_PORT    4000
#define NMEA_UDP_PORT     10110
#define NMEA_TCP_PORT     2000

#if defined(PREMIUM_PACKAGE) && !defined(RASPBERRY_PI)
#define ENABLE_AHRS
#endif /* PREMIUM_PACKAGE */

typedef struct UFO {
    uint8_t   raw[34];
    time_t    timestamp;

    uint8_t   protocol;

    uint32_t  addr;
    uint8_t   addr_type;
    float     latitude;
    float     longitude;
    float     altitude;
    float     pressure_altitude;
    float     course;     /* CoG */
    float     speed;      /* ground speed in knots */
    uint8_t   aircraft_type;

    float     vs; /* feet per minute */

    bool      stealth;
    bool      no_track;

    int8_t    ns[4];
    int8_t    ew[4];

    float     geoid_separation; /* metres */
    uint16_t  hdop; /* cm */
    int8_t    rssi; /* SX1276 only */

    float     distance;
    float     bearing;
    int8_t    alarm_level;
} ufo_t;

typedef struct hardware_info {
    byte  model;
    byte  revision;
    byte  soc;
    byte  rf;
    byte  gnss;
    byte  baro;
    byte  display;
#if defined(ENABLE_AHRS)
    byte  ahrs;
#endif /* ENABLE_AHRS */
} hardware_info_t;

enum
{
	SOFTRF_MODE_NORMAL,
	SOFTRF_MODE_WATCHOUT,
	SOFTRF_MODE_BRIDGE,
	SOFTRF_MODE_RELAY,
	SOFTRF_MODE_TXRX_TEST,
	SOFTRF_MODE_LOOPBACK,
	SOFTRF_MODE_UAV
};

enum
{
	SOFTRF_MODEL_STANDALONE,
	SOFTRF_MODEL_PRIME,
	SOFTRF_MODEL_UAV,
	SOFTRF_MODEL_PRIME_MK2,
	SOFTRF_MODEL_RASPBERRY,
	SOFTRF_MODEL_UAT
};

extern ufo_t ThisAircraft;
extern hardware_info_t hw_info;
extern const float txrx_test_positions[90][2] PROGMEM;

#define TXRX_TEST_NUM_POSITIONS (sizeof(txrx_test_positions) / sizeof(float) / 2)
#define TXRX_TEST_ALTITUDE    438.0
#define TXRX_TEST_COURSE      280.0
#define TXRX_TEST_SPEED       50.0
#define TXRX_TEST_VS          -300.0

//#define ENABLE_TTN
//#define ENABLE_BT_VOICE
//#define TEST_PAW_ON_NICERF_SV610_FW466
#define  DO_GDL90_FF_EXT

#define LOGGER_IS_ENABLED 0

#if LOGGER_IS_ENABLED
#define StdOut  LogFile
#else
#define StdOut  Serial
#endif /* LOGGER_IS_ENABLED */

#endif /* SOFTRF_H */
