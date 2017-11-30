/*
 * MAVLinkHelper.cpp
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

#include <TimeLib.h>
#include <SoftwareSerial.h>

#include "MAVLinkHelper.h"

unsigned long MAVLinkTimeSyncMarker = 0;
extern SoftwareSerial swSer;

void MAVLink_setup()
{
  swSer.begin(57600);    
}

void PickMAVLinkFix()
{
  read_mavlink();
}

void MAVLinkTimeSync()
{
  if (MAVLinkTimeSyncMarker == 0 && the_aircraft.gps.fix_type > 1) {
      setTime((time_t) (the_aircraft.location.gps_time_stamp / 1000000));
      MAVLinkTimeSyncMarker = millis();
  } else {

    if ((millis() - MAVLinkTimeSyncMarker > 60000) /* 1m */ && the_aircraft.gps.fix_type > 1 ) {
      setTime((time_t) (the_aircraft.location.gps_time_stamp / 1000000));
      MAVLinkTimeSyncMarker = millis();
    }  
  }
}

void MAVLinkShareTraffic()
{
    time_t this_moment = now();

    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].addr && (this_moment - Container[i].timestamp) <= EXPORT_EXPIRATION_TIME) {

        write_mavlink(Container[i].addr, Container[i].latitude, Container[i].longitude, (int32_t) Container[i].altitude);

      }
    }
}