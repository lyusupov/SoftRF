/*
 * GNSSHelper.h
 * Copyright (C) 2016-2021 Linar Yusupov
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

#ifndef GNSSHELPER_H
#define GNSSHELPER_H

#include <TinyGPS++.h>

typedef enum
{
  GNSS_MODULE_NONE,
  GNSS_MODULE_NMEA, /* generic NMEA */
  GNSS_MODULE_U6,   /* Ublox 6 */
  GNSS_MODULE_U7,   /* Ublox 7 */
  GNSS_MODULE_U8,   /* Ublox 8 */
  GNSS_MODULE_U9,   /* reserved for Ublox 9 */
  GNSS_MODULE_MAV,  /* MAVLink */
  GNSS_MODULE_SONY, /* S7XG */
  GNSS_MODULE_AT65, /* AT6558 */
  GNSS_MODULE_MT33, /* L80 */
  GNSS_MODULE_GOKE  /* Air530 */
} gnss_id_t;

typedef struct gnss_chip_ops_struct {
  gnss_id_t (*probe)();
  bool      (*setup)();
  void      (*loop)();
  void      (*fini)();
} gnss_chip_ops_t;

/*
 * Both GGA and RMC NMEA sentences are required.
 * No fix when any of them is missing or lost.
 * Valid date is critical for legacy protocol (only).
 */
#define NMEA_EXP_TIME  3500 /* 3.5 seconds */
#define isValidGNSSFix()  ( gnss.location.isValid()               && \
                            gnss.altitude.isValid()               && \
                            gnss.date.isValid()                   && \
                           (gnss.location.age() <= NMEA_EXP_TIME) && \
                           (gnss.altitude.age() <= NMEA_EXP_TIME) && \
                           (gnss.date.age()     <= NMEA_EXP_TIME))

byte GNSS_setup      (void);
void GNSS_loop       (void);
void GNSS_fini       (void);
void GNSSTimeSync    (void);
void PickGNSSFix     (void);
int LookupSeparation (float, float);

extern TinyGPSPlus gnss;
extern volatile unsigned long PPS_TimeMarker;
extern const char *GNSS_name[];

#endif /* GNSSHELPER_H */