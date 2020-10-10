/*
 * MAVLinkHelper.cpp
 * Copyright (C) 2016-2020 Linar Yusupov
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
#include <TinyGPS++.h>

#include "SoCHelper.h"
#include "MAVLinkHelper.h"
#include "GDL90Helper.h"
#include "WiFiHelper.h"
#include "TrafficHelper.h"
#include "EEPROMHelper.h"
#include "RFHelper.h"

static unsigned long MAVLinkTimeSyncMarker = 0;
static bool MAVLinkAPisArmed = false;

void MAVLink_setup()
{
  SoC->swSer_begin(57600);
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

        char hexbuf[8];
        char callsign[8+1];

        snprintf(hexbuf, sizeof(hexbuf), "%06X", Container[i].addr);
        memcpy(callsign, GDL90_CallSign_Prefix[Container[i].protocol],
          strlen(GDL90_CallSign_Prefix[Container[i].protocol]));
        memcpy(callsign + strlen(GDL90_CallSign_Prefix[Container[i].protocol]),
          hexbuf, strlen(hexbuf) + 1);

        write_mavlink(  Container[i].addr,
                        Container[i].latitude,
                        Container[i].longitude,
                        Container[i].altitude,
                        Container[i].course,
                        Container[i].speed * _GPS_MPS_PER_KNOT, /* m/s */
                        Container[i].vs / (_GPS_FEET_PER_METER * 60.0), /* m/s */
                        (settings->band == RF_BAND_US ? 1200 : 7000),
                        callsign,
                        AT_TO_GDL90(Container[i].aircraft_type));

      }
    }
}

void MAVLinkSetWiFiPower()
{
  if (get_num_heartbeats() == 0) {
    return;
  }

  if (!MAVLinkAPisArmed && (the_aircraft.nav_mode & MAV_MODE_FLAG_SAFETY_ARMED)) {
    SoC->WiFi_setOutputPower(WIFI_TX_POWER_MIN);
    MAVLinkAPisArmed = true;
    return;
  }

  if (MAVLinkAPisArmed && !(the_aircraft.nav_mode & MAV_MODE_FLAG_SAFETY_ARMED)) {
    SoC->WiFi_setOutputPower(WIFI_TX_POWER_MED);
    MAVLinkAPisArmed = false;
  }
}