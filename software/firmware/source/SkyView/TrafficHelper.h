/*
 * TrafficHelper.h
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

#ifndef TRAFFICHELPER_H
#define TRAFFICHELPER_H

#include "SoCHelper.h"

typedef struct traffic_struct {
    time_t    timestamp;

    int8_t    AlarmLevel;
    int16_t   RelativeNorth;
    int16_t   RelativeEast;
    int16_t   RelativeVertical;
    int8_t    IDType;
    uint32_t  ID;
    uint16_t  Track;
    int16_t   TurnRate;
    uint16_t  GroundSpeed;
    float     ClimbRate;
    int8_t    AcftType;
} traffic_t;

#define ENTRY_EXPIRATION_TIME   10 /* seconds */

void ClearExpired(void);

extern traffic_t fo, Container[MAX_TRACKING_OBJECTS], EmptyFO;

#endif /* TRAFFICHELPER_H */
