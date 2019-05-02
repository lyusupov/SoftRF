/*
 * TrafficHelper.cpp
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

#include <TimeLib.h>

#include "SoCHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"

traffic_t ThisAircraft, Container[MAX_TRACKING_OBJECTS], fo, EmptyFO;
traffic_by_dist_t traffic[MAX_TRACKING_OBJECTS];

unsigned long UpdateTrafficTimeMarker = 0;

void Traffic_Update(int ndx)
{
  float distance = nmea.distanceBetween( ThisAircraft.latitude,
                                         ThisAircraft.longitude,
                                         Container[ndx].latitude,
                                         Container[ndx].longitude);

  float bearing  = nmea.courseTo( ThisAircraft.latitude,
                                  ThisAircraft.longitude,
                                  Container[ndx].latitude,
                                  Container[ndx].longitude);

  float RelativeNorth     = constrain(distance * cos(radians(bearing)),
                                       -32768, 32767);
  float RelativeEast      = constrain(distance * sin(radians(bearing)),
                                       -32768, 32767);
  float RelativeVertical  = constrain(Container[ndx].altitude - ThisAircraft.altitude,
                                       -32768, 32767);

  Container[ndx].RelativeNorth    = (int16_t) RelativeNorth;
  Container[ndx].RelativeEast     = (int16_t) RelativeEast;
  Container[ndx].RelativeVertical = (int16_t) RelativeVertical;
}

void Traffic_loop()
{
  if (isTimeToUpdateTraffic()) {
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

      if (Container[i].ID &&
          (ThisAircraft.timestamp - Container[i].timestamp) <= ENTRY_EXPIRATION_TIME) {
        if ((ThisAircraft.timestamp - Container[i].timestamp) >= TRAFFIC_VECTOR_UPDATE_INTERVAL)
          Traffic_Update(i);
      } else {
        Container[i] = EmptyFO;
      }
    }

    UpdateTrafficTimeMarker = millis();
  }
}

void Traffic_ClearExpired()
{
  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID && (now() - Container[i].timestamp) > ENTRY_EXPIRATION_TIME) {
      Container[i] = EmptyFO;
    }
  }
}

int Traffic_Count()
{
  int count = 0;

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID) {
      count++;
    }
  }

  return count;
}

int traffic_cmp_by_distance(const void *a, const void *b)
{
  traffic_by_dist_t *ta = (traffic_by_dist_t *)a;
  traffic_by_dist_t *tb = (traffic_by_dist_t *)b;

  if (ta->distance >  tb->distance) return  1;
  if (ta->distance == tb->distance) return  0;
  if (ta->distance <  tb->distance) return -1;
}
