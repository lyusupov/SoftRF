/*
 * NMEAHelper.h
 * Copyright (C) 2017-2022 Linar Yusupov
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

#ifdef __cplusplus
#include "../../system/SoC.h"
#endif /* __cplusplus */

enum
{
	NMEA_OFF,
	NMEA_UART,
	NMEA_UDP,
	NMEA_TCP,
	NMEA_USB,
	NMEA_BLUETOOTH
};

#define NMEA_BUFFER_SIZE    128
#define NMEA_CALLSIGN_SIZE  (3 /* prefix */ + 1 /* _ */ + 6 /* ICAO */ + 1 /* EOL */)

#define PSRFC_VERSION       1
#define MAX_PSRFC_LEN       64

#define PSRFS_VERSION       1
#define MAX_PSRFS_LEN       64

#define PSKVC_VERSION       1
#define MAX_PSKVC_LEN       64

void NMEA_setup(void);
void NMEA_loop(void);
void NMEA_fini();
void NMEA_Export(void);
void NMEA_Position(void);
void NMEA_Out(uint8_t, byte *, size_t, bool);
void NMEA_GGA(void);
void NMEA_add_checksum(char *, size_t);

extern char NMEABuffer[NMEA_BUFFER_SIZE];

#if defined(USE_NMEA_CFG)
void NMEA_Process_SRF_SKV_Sentences(void);

extern uint8_t C_NMEA_Source;
#endif /* USE_NMEA_CFG */

#if defined(NMEA_TCP_SERVICE)

typedef struct NmeaTCP_struct {
  WiFiClient client;
  time_t connect_ts;  /* connect time stamp */
  bool ack;           /* acknowledge */
} NmeaTCP_t;

#define MAX_NMEATCP_CLIENTS    2
#define NMEATCP_ACK_TIMEOUT    2 /* seconds */

#endif

#if !defined(PFLAA_EXT1_FMT)
#define PFLAA_EXT1_FMT  ""
#endif /* PFLAA_EXT1_FMT */

#if !defined(PFLAA_EXT1_ARGS)
#define PFLAA_EXT1_ARGS
#endif /* PFLAA_EXT1_ARGS */

#if !defined(PFLAU_EXT1_FMT)
#define PFLAU_EXT1_FMT  ""
#endif /* PFLAU_EXT1_FMT */

#if !defined(PFLAU_EXT1_ARGS)
#define PFLAU_EXT1_ARGS
#endif /* PFLAU_EXT1_ARGS */

#endif /* NMEAHELPER_H */
