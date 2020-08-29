/*
 * TrafficHelper.cpp
 * Copyright (C) 2018-2020 Linar Yusupov
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

#include "TrafficHelper.h"
#include "EEPROMHelper.h"
#include "RFHelper.h"
#include "GNSSHelper.h"
#include "WebHelper.h"
#include "Protocol_Legacy.h"

unsigned long UpdateTrafficTimeMarker = 0;

ufo_t fo, Container[MAX_TRACKING_OBJECTS], EmptyFO;

static int8_t (*Alarm_Level)(ufo_t *, ufo_t *);

/*
 * No any alarms issued by the firmware.
 * Rely upon high-level flight management software.
 */
static int8_t Alarm_None(ufo_t *this_aircraft, ufo_t *fop)
{
  return ALARM_LEVEL_NONE;
}

/*
 * Simple, distance based alarm level assignment.
 */
static int8_t Alarm_Distance(ufo_t *this_aircraft, ufo_t *fop)
{
  int distance = (int) fop->distance;
  int8_t rval = ALARM_LEVEL_NONE;
  int alt_diff = (int) (fop->altitude - this_aircraft->altitude);

  if (abs(alt_diff) < VERTICAL_SEPARATION) { /* no warnings if too high or too low */
    if (distance < ALARM_ZONE_URGENT) {
      rval = ALARM_LEVEL_URGENT;
    } else if (distance < ALARM_ZONE_IMPORTANT) {
      rval = ALARM_LEVEL_IMPORTANT;
    } else if (distance < ALARM_ZONE_LOW) {
      rval = ALARM_LEVEL_LOW;
    }
  }

  return rval;
}

/*
 * EXPERIMENTAL
 *
 * Linear, CoG and GS based collision prediction.
 */
static int8_t Alarm_Vector(ufo_t *this_aircraft, ufo_t *fop)
{
  int8_t rval = ALARM_LEVEL_NONE;
  int alt_diff = (int) (fop->altitude - this_aircraft->altitude);

  if (abs(alt_diff) < VERTICAL_SEPARATION) { /* no warnings if too high or too low */

    /* Subtract 2D velocity vector of traffic from 2D velocity vector of this aircraft */ 
    float V_rel_x = this_aircraft->speed * cosf(radians(90.0 - this_aircraft->course)) -
                    fop->speed * cosf(radians(90.0 - fop->course)) ;
    float V_rel_y = this_aircraft->speed * sinf(radians(90.0 - this_aircraft->course)) -
                    fop->speed * sinf(radians(90.0 - fop->course)) ;

    float V_rel_magnitude = sqrtf(V_rel_x * V_rel_x + V_rel_y * V_rel_y) * _GPS_MPS_PER_KNOT;
    float V_rel_direction = atan2f(V_rel_y, V_rel_x) * 180.0 / PI;  /* -180 ... 180 */

    /* convert from math angle into course relative to north */
    V_rel_direction = (V_rel_direction <= 90.0 ? 90.0 - V_rel_direction :
                                                450.0 - V_rel_direction);

    /* +- 10 degrees tolerance for collision course */
    if (V_rel_magnitude > 0.1 && fabs(V_rel_direction - fop->bearing) < 10.0) {

      /* time is seconds prior to impact */
      float t = fop->distance / V_rel_magnitude;

      /* time limit values are compliant with FLARM data port specs */
      if (t < 9.0) {
        rval = ALARM_LEVEL_URGENT;
      } else if (t < 13.0) {
        rval = ALARM_LEVEL_IMPORTANT;
      } else if (t < 19.0) {
        rval = ALARM_LEVEL_LOW;
      }    
    }
  }
  return rval;
}

/*
 * "Legacy" method is based on short history of 2D velocity vectors (NS/EW)
 */
static int8_t Alarm_Legacy(ufo_t *this_aircraft, ufo_t *fop)
{

  int8_t rval = ALARM_LEVEL_NONE;

  /* TBD */

  return rval;
}

void Traffic_Update(int ndx)
{
  Container[ndx].distance = gnss.distanceBetween( ThisAircraft.latitude,
                                                  ThisAircraft.longitude,
                                                  Container[ndx].latitude,
                                                  Container[ndx].longitude);

  Container[ndx].bearing = gnss.courseTo( ThisAircraft.latitude,
                                          ThisAircraft.longitude,
                                          Container[ndx].latitude,
                                          Container[ndx].longitude);

  if (Alarm_Level) {
    Container[ndx].alarm_level = (*Alarm_Level)(&ThisAircraft, &Container[ndx]);
  }
}

void ParseData()
{
    size_t rx_size = RF_Payload_Size(settings->rf_protocol);
    rx_size = rx_size > sizeof(fo.raw) ? sizeof(fo.raw) : rx_size;

#if DEBUG
    Hex2Bin(TxDataTemplate, RxBuffer);
#endif

    memset(fo.raw, 0, sizeof(fo.raw));
    memcpy(fo.raw, RxBuffer, rx_size);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,"));
      StdOut.print((unsigned long) now()); StdOut.print(F(","));
      StdOut.print(Bin2Hex(fo.raw, rx_size)); StdOut.print(F(","));
      StdOut.println(RF_last_rssi);
    }

    if (protocol_decode && (*protocol_decode)((void *) RxBuffer, &ThisAircraft, &fo)) {

      /* ignore if the received packet is from myself. */
      if (fo.addr == ThisAircraft.addr)
        return;

      fo.rssi = RF_last_rssi;

      for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

        if (Container[i].addr == fo.addr) {
          Container[i] = fo;
          Traffic_Update(i);
          break;
        } else {
          if (now() - Container[i].timestamp > ENTRY_EXPIRATION_TIME) {
            Container[i] = fo;
            Traffic_Update(i);
            break;
          }
        }
      }
    }
}

void Traffic_setup()
{
  switch (settings->alarm)
  {
  case TRAFFIC_ALARM_NONE:
    Alarm_Level = &Alarm_None;
    break;
  case TRAFFIC_ALARM_VECTOR:
    Alarm_Level = &Alarm_Vector;
    break;
  case TRAFFIC_ALARM_LEGACY:
    Alarm_Level = &Alarm_Legacy;
    break;
  case TRAFFIC_ALARM_DISTANCE:
  default:
    Alarm_Level = &Alarm_Distance;
    break;
  }
}

void Traffic_loop()
{
  if (isTimeToUpdateTraffic()) {
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

      if (Container[i].addr &&
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

void ClearExpired()
{
  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr && (ThisAircraft.timestamp - Container[i].timestamp) > ENTRY_EXPIRATION_TIME) {
      Container[i] = EmptyFO;
    }
  }
}
