/*
 * SoftRF.h
 * Copyright (C) 2016-2017 Linar Yusupov
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

#include <nRF905.h>

#define SOFTRF_FIRMWARE_VERSION "0.9"

#define LOGGER_IS_ENABLED 0

#if LOGGER_IS_ENABLED
#define StdOut  LogFile
#else
#define StdOut  Serial
#endif /* LOGGER_IS_ENABLED */

#define RF_FREQ   NRF905_FREQ
#define PKT_SIZE  NRF905_PAYLOAD_SIZE

#define NRF905_TX_PWR_OFF  0xFF

#define RXADDR {0x31, 0xfa , 0xb6} // Address of this device (4 bytes)
#define TXADDR {0x31, 0xfa , 0xb6} // Address of device to send to (4 bytes)

#define TIMEOUT   100 // 1000

#define RRB_SIZE  10
#define MAX_TRACKING_OBJECTS 8
#define LED_EXPIRATION_TIME 5 /* seconds */
#define EXPORT_EXPIRATION_TIME 5 /* seconds */

#define ALTITUDE    138

#define MY_ACCESSPOINT_SSID ""
#define MY_ACCESSPOINT_PSK  ""

//#define ARGUS_HOSTNAME "192.168.157.129"
//#define ARGUS_PORT  7777

#define XCSOAR_HOSTNAME "192.168.157.129" // "192.168.157.248"
#define XCSOAR_PORT 10110

//#define CLOUD_HOSTNAME "192.168.157.129" // glidern1.glidernet.org
//#define CLOUD_PORT 4352 /* echo TCP/IP service to test response */ // 14580
//#define CLOUD_MODE  0

#define RELAY_PORT 12390

//#define STATION_ID  String(ESP.getChipId(), HEX).c_str()

typedef struct UFO {
    String    raw;
    time_t    timestamp;

    uint32_t  addr;
    uint8_t   addr_type;
    float     latitude;
    float     longtitude;
    int32_t   altitude;
    float     course;
    unsigned int aircraft_type;

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
extern bool Import(void);
extern void Export(void);
extern void ParseData(void);
extern void ClearExpired(void);
extern void *WiFi_relay_from_android(void);
extern void WiFi_relay_to_android(void);

#endif /* SOFTRF_H */

