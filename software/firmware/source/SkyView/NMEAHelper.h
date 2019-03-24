/*
 * NMEAHelper.h
 * Copyright (C) 2019 Linar Yusupov
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

#include "SoCHelper.h"

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
#define isValidGNSSFix()  ( gnss.location.isValid()               && \
                            gnss.altitude.isValid()               && \
                            gnss.date.isValid()                   && \
                           (gnss.location.age() <= NMEA_EXP_TIME) && \
                           (gnss.altitude.age() <= NMEA_EXP_TIME) && \
                           (gnss.date.age()     <= NMEA_EXP_TIME))

void NMEA_setup(void);
void NMEA_loop(void);

bool NMEA_isConnected(void);
bool NMEA_hasGNSS(void);
bool NMEA_hasFLARM(void);

extern status_t NMEA_Status;

#endif /* NMEAHELPER_H */