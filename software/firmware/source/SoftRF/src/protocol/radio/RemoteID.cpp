/*
 * Protocol_RID.cpp
 *
 * Encoder for Remote ID radio protocol
 * Copyright (C) 2023 Linar Yusupov
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

#include <stdint.h>

#include "../../../SoftRF.h"
#include "../../driver/RF.h"

#if defined(ENABLE_REMOTE_ID)
#include <id_open.h>

ID_OpenDrone          squitter;
UTM_Utilities         utm_utils;

struct UTM_parameters utm_parameters;
struct UTM_data       utm_data;

double RID_Base_Lat = 0.0;
double RID_Base_Lon = 0.0;
float  RID_Base_Alt = 0.0;

size_t rid_encode(void *pkt, ufo_t *this_aircraft) {

  uint32_t id = this_aircraft->addr & 0x00FFFFFF;

#if !defined(SOFTRF_ADDRESS)
  uint8_t addr_type = ADDR_TYPE_ANONYMOUS;
#else
  uint8_t addr_type = id == SOFTRF_ADDRESS ? ADDR_TYPE_ICAO : ADDR_TYPE_ANONYMOUS;
#endif

  uint8_t acft_type = this_aircraft->aircraft_type > AIRCRAFT_TYPE_STATIC ?
          AIRCRAFT_TYPE_UNKNOWN : this_aircraft->aircraft_type;

  utm_data.latitude_d  = (double) this_aircraft->latitude;
  utm_data.longitude_d = (double) this_aircraft->longitude;
  utm_data.alt_msl_m   = (float)  this_aircraft->altitude;
  utm_data.alt_agl_m   = (float)  0.0 /* TBD */;
  utm_data.speed_kn    = (int)    this_aircraft->speed;
  utm_data.heading     = (int)    this_aircraft->course;

  utm_data.base_valid  = (RID_Base_Lat == 0.0 && RID_Base_Lon == 0.0) ? 0 : 1;
  if (utm_data.base_valid) {
    utm_data.base_latitude  = RID_Base_Lat;
    utm_data.base_longitude = RID_Base_Lon;
    utm_data.base_alt_m     = RID_Base_Alt;
  } else if (this_aircraft->latitude != 0.0 && this_aircraft->longitude != 0.0) {
    RID_Base_Lat = utm_data.latitude_d;
    RID_Base_Lon = utm_data.longitude_d;
    RID_Base_Alt = utm_data.alt_msl_m;
  }

#if 0
  char text[64], lat_s[16], long_s[16];

  dtostrf(utm_data.latitude_d, 10,7,lat_s);
  dtostrf(utm_data.longitude_d,10,7,long_s);

  sprintf(text,"%s,%s\r\n", lat_s,long_s);
  Serial.print(text);
#endif

  return 0;
}

bool rid_decode(void *pkt, ufo_t *this_aircraft, ufo_t *fop) {

  /* N/A */

  return false;
}
#endif /* ENABLE_REMOTE_ID */
