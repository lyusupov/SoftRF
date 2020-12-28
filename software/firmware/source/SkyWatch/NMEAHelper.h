/*
 * NMEAHelper.h
 * Copyright (C) 2019-2021 Linar Yusupov
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

#ifndef NMEAHELPER_H
#define NMEAHELPER_H

#include <TinyGPS++.h>

#include "SoCHelper.h"

/* FTD-12 Version: 7.00 */
enum
{
	ADDR_TYPE_RANDOM,
	ADDR_TYPE_ICAO,
	ADDR_TYPE_FLARM,
	ADDR_TYPE_ANONYMOUS, /* FLARM stealth, OGN */
	ADDR_TYPE_P3I,
	ADDR_TYPE_FANET
};

enum
{
	AIRCRAFT_TYPE_UNKNOWN,
	AIRCRAFT_TYPE_GLIDER,
	AIRCRAFT_TYPE_TOWPLANE,
	AIRCRAFT_TYPE_HELICOPTER,
	AIRCRAFT_TYPE_PARACHUTE,
	AIRCRAFT_TYPE_DROPPLANE,
	AIRCRAFT_TYPE_HANGGLIDER,
	AIRCRAFT_TYPE_PARAGLIDER,
	AIRCRAFT_TYPE_POWERED,
	AIRCRAFT_TYPE_JET,
	AIRCRAFT_TYPE_UFO,
	AIRCRAFT_TYPE_BALLOON,
	AIRCRAFT_TYPE_ZEPPELIN,
	AIRCRAFT_TYPE_UAV,
	AIRCRAFT_TYPE_RESERVED,
	AIRCRAFT_TYPE_STATIC
};

enum
{
	ALARM_LEVEL_NONE,
	ALARM_LEVEL_LOW,       /* 13-18 seconds to impact */
	ALARM_LEVEL_IMPORTANT, /*  9-12 seconds to impact */
	ALARM_LEVEL_URGENT     /*   0-8 seconds to impact */
};

enum
{
	ALARM_TYPE_TRAFFIC,
	ALARM_TYPE_SILENT,
	ALARM_TYPE_AIRCRAFT,
	ALARM_TYPE_OBSTACLE
};

enum
{
	GNSS_STATUS_NONE,
	GNSS_STATUS_3D_GROUND,
	GNSS_STATUS_3D_MOVING
};

enum
{
	POWER_STATUS_BAD,
	POWER_STATUS_GOOD
};

enum
{
	TX_STATUS_OFF,
	TX_STATUS_ON
};

typedef struct nmea_status_struct {
    time_t    timestamp;

    int8_t    RX;
    int8_t    TX;
    int8_t    GPS;
    int8_t    Power;
    int8_t    AlarmLevel;
    int16_t   RelativeBearing;
    uint8_t   AlarmType;
    int16_t   RelativeVertical;
    uint32_t  RelativeDistance;
    uint32_t  ID;
} status_t;

#define NMEA_UDP_PORT     10110
#define NMEA_TCP_PORT     2000

/*
 * Both GGA and RMC NMEA sentences are required.
 * No fix when any of them is missing or lost.
 * Valid date is critical for legacy protocol (only).
 */
#define NMEA_EXP_TIME  3500 /* 3.5 seconds */
#define isValidGNSSFix()  ( nmea.location.isValid()               && \
                            nmea.altitude.isValid()               && \
                            nmea.date.isValid()                   && \
                           (nmea.location.age() <= NMEA_EXP_TIME) && \
                           (nmea.altitude.age() <= NMEA_EXP_TIME) && \
                           (nmea.date.age()     <= NMEA_EXP_TIME))

#define NMEA_BUFFER_SIZE    128

#define PSRFC_VERSION       1

void NMEA_setup(void);
void NMEA_loop(void);

bool NMEA_Save_Settings(void);

bool NMEA_isConnected(void);
bool NMEA_hasGNSS(void);
bool NMEA_hasFLARM(void);
bool NMEA_has3DFix(void);
void NMEA_Out(byte *, size_t, bool);
void NMEA_fini();

extern status_t NMEA_Status;
extern TinyGPSPlus nmea;

extern char NMEABuffer[NMEA_BUFFER_SIZE];

#if defined(NMEA_TCP_SERVICE)

typedef struct NmeaTCP_struct {
  WiFiClient client;
  time_t connect_ts;  /* connect time stamp */
  bool ack;           /* acknowledge */
} NmeaTCP_t;

#define MAX_NMEATCP_CLIENTS    2
#define NMEATCP_ACK_TIMEOUT    2 /* seconds */

#endif

#endif /* NMEAHELPER_H */