/*
 * TrafficHelper.cpp
 * Copyright (C) 2018-2026 Linar Yusupov
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
#include "driver/EEPROM.h"
#include "driver/RF.h"
#include "driver/GNSS.h"
#include "driver/Sound.h"
#include "driver/EPD.h"
#include "ui/Web.h"
#include "protocol/radio/Legacy.h"

unsigned long UpdateTrafficTimeMarker = 0;

static unsigned long Traffic_Voice_TimeMarker = 0;

ufo_t fo, Container[MAX_TRACKING_OBJECTS], EmptyFO;
traffic_by_dist_t traffic_by_dist[MAX_TRACKING_OBJECTS];

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

void Traffic_Update(ufo_t *fop)
{
  fop->distance = gnss.distanceBetween( ThisAircraft.latitude,
                                        ThisAircraft.longitude,
                                        fop->latitude,
                                        fop->longitude);

  fop->bearing  = gnss.courseTo( ThisAircraft.latitude,
                                 ThisAircraft.longitude,
                                 fop->latitude,
                                 fop->longitude);

  if (Alarm_Level) {
    fop->alarm_level = (*Alarm_Level)(&ThisAircraft, fop);
  }
}

bool Traffic_Add(ufo_t *fop)
{
  int i;

  for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr == fo.addr) {
      uint8_t alert_bak = Container[i].alert;
      Container[i] = fo;
      Container[i].alert = alert_bak;
      return true;
    }
  }

  int max_dist_ndx = 0;
  int min_level_ndx = 0;

  for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (now() - Container[i].timestamp > ENTRY_EXPIRATION_TIME) {
      Container[i] = fo;
      return true;
    }
#if !defined(EXCLUDE_TRAFFIC_FILTER_EXTENSION)
    if  (Container[i].distance > Container[max_dist_ndx].distance)  {
      max_dist_ndx = i;
    }
    if  (Container[i].alarm_level < Container[min_level_ndx].alarm_level)  {
      min_level_ndx = i;
    }
#endif /* EXCLUDE_TRAFFIC_FILTER_EXTENSION */
  }

#if !defined(EXCLUDE_TRAFFIC_FILTER_EXTENSION)
  if (fo.alarm_level > Container[min_level_ndx].alarm_level) {
    Container[min_level_ndx] = fo;
    return true;
  }

  if (fo.distance    <  Container[max_dist_ndx].distance &&
      fo.alarm_level >= Container[max_dist_ndx].alarm_level) {
    Container[max_dist_ndx] = fo;
    return true;
  }
#endif /* EXCLUDE_TRAFFIC_FILTER_EXTENSION */

  return false;
}

static traffic_voice_alert_t voice_alerts_cache[MAX_TRACKING_OBJECTS];

static void voice_alert_add(uint32_t id)
{
  int i;

  time_t min_ts = now();

  for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (voice_alerts_cache[i].addr == 0) {
      voice_alerts_cache[i].addr  = id;
      voice_alerts_cache[i].ts    = now();

      break;
    }

    if (voice_alerts_cache[i].ts < min_ts) {
      min_ts = voice_alerts_cache[i].ts;
    }
  }

  if (i < MAX_TRACKING_OBJECTS) {
    return;
  }

  for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (voice_alerts_cache[i].ts == min_ts) {
      voice_alerts_cache[i].addr  = id;
      voice_alerts_cache[i].ts    = now();

      break;
    }
  }
}

static bool voice_alert_find(uint32_t id)
{
  bool rval = false;

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (voice_alerts_cache[i].addr == id) {
      rval = true;
      break;
    }
  }

  return rval;
}

static void Traffic_Voice()
{
  int j=0;
  int bearing;
  char message[80];

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr && (now() - Container[i].timestamp) <= VOICE_EXPIRATION_TIME) {

      traffic_by_dist[j].fop = &Container[i];
      traffic_by_dist[j].distance = Container[i].distance;
      j++;
    }
  }

  if (j > 0 && voice_alert_find(traffic_by_dist[0].fop->addr) == false) {

    const char *u_dist, *u_alt;
    float voc_dist;
    int   voc_alt;
    const char *where;
    char how_far[32];
    char elev[32];

    qsort(traffic_by_dist, j, sizeof(traffic_by_dist_t), traffic_cmp_by_distance);

    bearing = (int) traffic_by_dist[0].fop->bearing;

    /* This bearing is always relative to current ground track */
//  if (settings->m.orientation == DIRECTION_TRACK_UP) {
      bearing -= ThisAircraft.course;
//  }

    if (bearing < 0) {
      bearing += 360;
    }

    int oclock = ((bearing + 15) % 360) / 30;
    float Relative_Vertical = traffic_by_dist[0].fop->altitude -
                              ThisAircraft.altitude;

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

    switch (ui->units)
    {
    case UNITS_IMPERIAL:
      u_dist = "nautical miles";
      u_alt  = "feet";
      voc_dist = (traffic_by_dist[0].distance * _GPS_MILES_PER_METER) /
                  _GPS_MPH_PER_KNOT;
      voc_alt  = abs((int) (Relative_Vertical * _GPS_FEET_PER_METER));
      break;
    case UNITS_MIXED:
      u_dist = "kms";
      u_alt  = "feet";
      voc_dist = traffic_by_dist[0].distance / 1000.0;
      voc_alt  = abs((int) (Relative_Vertical * _GPS_FEET_PER_METER));
      break;
    case UNITS_METRIC:
    default:
      u_dist = "kms";
      u_alt  = "metres";
      voc_dist = traffic_by_dist[0].distance / 1000.0;
      voc_alt  = abs((int) Relative_Vertical);
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
        Relative_Vertical > 0 ? "above" : "below");
    }

    snprintf(message, sizeof(message),
                "traffic %s distance %s altitude %s",
                where, how_far, elev);

    SoC->TTS(message);

    voice_alert_add(traffic_by_dist[0].fop->addr);
  }

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (now() - voice_alerts_cache[i].ts > (5 * 60) /* seconds */) {
      voice_alerts_cache[i].addr  = 0;
      voice_alerts_cache[i].ts    = 0;
    }
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

    if (memcmp(RxBuffer, TxBuffer, rx_size) == 0) {
      if (settings->nmea_p) {
        StdOut.println(F("$PSRFE,RF loopback is detected on Rx"));
      }

      rx_packets_counter--;
      return;
    }

    if (protocol_decode && (*protocol_decode)((void *) RxBuffer, &ThisAircraft, &fo)) {
      fo.rssi = RF_last_rssi;
      Traffic_Update(&fo);
      Traffic_Add(&fo);
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

  UpdateTrafficTimeMarker  = millis();
  Traffic_Voice_TimeMarker = millis();
}

void Traffic_loop()
{
  if (isTimeToUpdateTraffic()) {
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

      if (Container[i].addr &&
          (ThisAircraft.timestamp - Container[i].timestamp) <= ENTRY_EXPIRATION_TIME) {
        if ((ThisAircraft.timestamp - Container[i].timestamp) >= TRAFFIC_VECTOR_UPDATE_INTERVAL) {
          Traffic_Update(&Container[i]);
        }
        if ((Container[i].alert & TRAFFIC_ALERT_SOUND) == 0) {
          Sound_Notify();
          Container[i].alert |= TRAFFIC_ALERT_SOUND;
        }
      } else {
        Container[i] = EmptyFO;
      }
    }

    UpdateTrafficTimeMarker = millis();
  }

  if (isTimeToVoice()) {
#if defined(USE_EPAPER) || defined(USE_DSI)
    if (ui->voice != VOICE_OFF) {
      Traffic_Voice();
    }
#endif

    Traffic_Voice_TimeMarker = millis();
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

int Traffic_Count()
{
  int count = 0;

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr) {
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
#if 0
  if (ta->distance <  tb->distance) return -1;
#else
  else return -1;
#endif
}
