/*
 * TrafficHelper.cpp
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

#include <TimeLib.h>

#include "SoCHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "EEPROMHelper.h"

traffic_t ThisAircraft, Container[MAX_TRACKING_OBJECTS], fo, EmptyFO;
traffic_by_dist_t traffic[MAX_TRACKING_OBJECTS];

static unsigned long UpdateTrafficTimeMarker = 0;
static unsigned long Traffic_Voice_TimeMarker = 0;
static uint32_t Traffic_Voice_ID_prev = 0;

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

static void Traffic_Voice()
{
  int j=0;
  int bearing;
  char message[80];

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID && (now() - Container[i].timestamp) <= VOICE_EXPIRATION_TIME) {

      traffic[j].fop = &Container[i];
      traffic[j].distance = sqrtf(Container[i].RelativeNorth * Container[i].RelativeNorth +
                                  Container[i].RelativeEast  * Container[i].RelativeEast);
      j++;
    }
  }

  if (j > 0 && traffic[0].fop->ID != Traffic_Voice_ID_prev) {

    const char *u_dist, *u_alt;
    float voc_dist;
    int   voc_alt;
    const char *where;
    char how_far[32];
    char elev[32];

    qsort(traffic, j, sizeof(traffic_by_dist_t), traffic_cmp_by_distance);

    bearing = (int) (atan2f(traffic[0].fop->RelativeNorth,
                            traffic[0].fop->RelativeEast) * 180.0 / PI);  /* -180 ... 180 */

    /* convert from math angle into course relative to north */
    bearing = (bearing <= 90 ? 90 - bearing :
                              450 - bearing);

    /* This bearing is always relative to current ground track */
//  if (settings->m.orientation == DIRECTION_TRACK_UP) {
      bearing -= ThisAircraft.Track;
//  }

    if (bearing < 0) {
      bearing += 360;
    }

    int oclock = ((bearing + 15) % 360) / 30;

    switch (oclock)
    {
    case 0:
      where = "ahead";
      break;
    case 1:
      where = "1oclock";
      break;
    case 2:
      where = "2oclock";
      break;
    case 3:
      where = "3oclock";
      break;
    case 4:
      where = "4oclock";
      break;
    case 5:
      where = "5oclock";
      break;
    case 6:
      where = "6oclock";
      break;
    case 7:
      where = "7oclock";
      break;
    case 8:
      where = "8oclock";
      break;
    case 9:
      where = "9oclock";
      break;
    case 10:
      where = "10oclock";
      break;
    case 11:
      where = "11oclock";
      break;
    }

    switch (settings->m.units)
    {
    case UNITS_IMPERIAL:
      u_dist = "nautical miles";
      u_alt  = "feet";
      voc_dist = (traffic[0].distance * _GPS_MILES_PER_METER) /
                  _GPS_MPH_PER_KNOT;
      voc_alt  = abs((int) (traffic[0].fop->RelativeVertical *
                  _GPS_FEET_PER_METER));
      break;
    case UNITS_MIXED:
      u_dist = "kms";
      u_alt  = "feet";
      voc_dist = traffic[0].distance / 1000.0;
      voc_alt  = abs((int) (traffic[0].fop->RelativeVertical *
                  _GPS_FEET_PER_METER));
      break;
    case UNITS_METRIC:
    default:
      u_dist = "kms";
      u_alt  = "metres";
      voc_dist = traffic[0].distance / 1000.0;
      voc_alt  = abs((int) traffic[0].fop->RelativeVertical);
      break;
    }

    if (voc_dist < 1.0) {
      strcpy(how_far, "near");
    } else {
      if (voc_dist > 9.0) {
        voc_dist = 9.0;
      }
      snprintf(how_far, sizeof(how_far), "%u %s", (int) voc_dist, u_dist);
    }

    if (voc_alt < 100) {
      strcpy(elev, "near");
    } else {
      if (voc_alt > 500) {
        voc_alt = 500;
      }

      snprintf(elev, sizeof(elev), "%u hundred %s %s",
        (voc_alt / 100), u_alt,
        traffic[0].fop->RelativeVertical > 0 ? "above" : "below");
    }

    snprintf(message, sizeof(message),
                "traffic %s distance %s altitude %s",
                where, how_far, elev);

    SoC->TTS(message);

    Traffic_Voice_ID_prev = traffic[0].fop->ID;
  }
}

void Traffic_setup()
{
  UpdateTrafficTimeMarker = millis();
  Traffic_Voice_TimeMarker = millis();
}

void Traffic_loop()
{
  if (settings->m.protocol == PROTOCOL_GDL90) {
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

  if (isTimeToVoice()) {
    if (settings->m.voice != VOICE_OFF) {
      Traffic_Voice();
    }

    Traffic_Voice_TimeMarker = millis();
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
