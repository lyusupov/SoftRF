/*
 * SoftRF.h
 * Copyright (C) 2016-2018 Linar Yusupov
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

#include <Arduino.h>

#define SOFTRF_FIRMWARE_VERSION "0.9"

//#define TEST_PAW_ON_NICERF_SV610_FW466

#define LOGGER_IS_ENABLED 0

#if LOGGER_IS_ENABLED
#define StdOut  LogFile
#else
#define StdOut  Serial
#endif /* LOGGER_IS_ENABLED */

#define PKT_SIZE  24  /* LEGACY_PAYLOAD_SIZE */

/* Max. paket's payload size for all supported RF protocols */
#define MAX_PKT_SIZE  32 

#define RXADDR {0x31, 0xfa , 0xb6} // Address of this device (4 bytes)
#define TXADDR {0x31, 0xfa , 0xb6} // Address of device to send to (4 bytes)

#define TIMEOUT   500

#define RRB_SIZE  10
#define MAX_TRACKING_OBJECTS    8

#define ENTRY_EXPIRATION_TIME   10 /* seconds */
#define LED_EXPIRATION_TIME     5 /* seconds */
#define EXPORT_EXPIRATION_TIME  5 /* seconds */

#define MY_ACCESSPOINT_SSID ""
#define MY_ACCESSPOINT_PSK  ""

#define RELAY_DST_PORT  12390
#define RELAY_SRC_PORT  (RELAY_DST_PORT - 1)

#define GDL90_DST_PORT    4000
#define NMEA_DST_PORT     10110
#define AIR_CONNECT_PORT  2000

#define EXPORT_DISTANCE_CLOSE  500
#define EXPORT_DISTANCE_NEAR   1500
#define EXPORT_DISTANCE_FAR    10000

typedef struct UFO {
    String    raw;
    time_t    timestamp;

    uint8_t   protocol;

    uint32_t  addr;
    uint8_t   addr_type;
    float     latitude;
    float     longitude;
    float     altitude;
    float     course;     /* CoG */
    float     speed;      /* ground speed in knots */
    uint8_t   aircraft_type;

    int32_t   vs;

    bool      stealth;
    bool      no_track;

    int8_t ns[4];
    int8_t ew[4];
} ufo_t;

enum
{
	SOFTRF_MODE_NORMAL,
	SOFTRF_MODE_ALARM,
	SOFTRF_MODE_BRIDGE,
	SOFTRF_MODE_OGN,
	SOFTRF_MODE_TX_TEST,
	SOFTRF_MODE_RX_TEST,
	SOFTRF_MODE_LOOPBACK,
	SOFTRF_MODE_UAV_BEACON  
};

enum
{
	DIRECTION_TRACK_UP,
	DIRECTION_NORTH_UP  
};

extern void Misc_info(void);
extern void ParseData(void);
extern void ClearExpired(void);
extern size_t Raw_Receive_UDP(uint8_t *);
extern void Raw_Transmit_UDP(void);

extern const float tx_test_positions[90][2] PROGMEM;

#define TX_TEST_NUM_POSITIONS (sizeof(tx_test_positions) / sizeof(float) / 2)
#define TEST_ALTITUDE    438.0
#define TEST_COURSE      0.0
#define TEST_SPEED       50.0

#endif /* SOFTRF_H */

