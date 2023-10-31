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
#include "../../driver/EEPROM.h"

#if defined(ENABLE_REMOTE_ID)
#include <id_open.h>

/*
 * Source:
 * https://asd-stan.org/wp-content/uploads/ASD-STAN_DRI_Introduction_to_the_European_digital_RID_UAS_Standard.pdf
 */

char RID_Operator_ID[ID_SIZE] = "";
char RID_Drone_ID   [ID_SIZE] = "";

ID_OpenDrone          squitter;

struct UTM_parameters utm_parameters;
struct UTM_data       utm_data;

double RID_Base_Lat = 0.0;
double RID_Base_Lon = 0.0;
float  RID_Base_Alt = 0.0;

static bool rid_status = false;

const uint8_t aircraft_type_to_odid[] PROGMEM = {
	ODID_UATYPE_OTHER,
	ODID_UATYPE_GLIDER,
	ODID_UATYPE_TETHERED_POWERED_AIRCRAFT,
	ODID_UATYPE_HELICOPTER_OR_MULTIROTOR,
	ODID_UATYPE_FREE_FALL_PARACHUTE,
	ODID_UATYPE_AEROPLANE,
	ODID_UATYPE_GLIDER,
	ODID_UATYPE_GLIDER,
	ODID_UATYPE_AEROPLANE,
	ODID_UATYPE_AEROPLANE,
	ODID_UATYPE_OTHER,
	ODID_UATYPE_FREE_BALLOON,
	ODID_UATYPE_AIRSHIP,
	ODID_UATYPE_OTHER,
	ODID_UATYPE_OTHER,
	ODID_UATYPE_GROUND_OBSTACLE
};

#define AT_TO_ODID(x)  (x > 15 ? \
   ODID_UATYPE_OTHER : pgm_read_byte(&aircraft_type_to_odid[x]))

bool rid_init() {
  memset(&utm_parameters,0,sizeof(utm_parameters));

  if (strlen(RID_Operator_ID) > 0) {
    strcpy(utm_parameters.UAS_operator, RID_Operator_ID);
    rid_status = true;
  }

  if (strlen(RID_Drone_ID) > 0) {
    strcpy(utm_parameters.UAV_id, RID_Drone_ID);
    utm_parameters.ID_type = ODID_IDTYPE_CAA_REGISTRATION_ID;
  } else {
    snprintf(utm_parameters.UAV_id, sizeof(utm_parameters.UAV_id), "%08X",
             SoC->getChipId());
    utm_parameters.ID_type = ODID_IDTYPE_SERIAL_NUMBER;
  }

  utm_parameters.UA_type     = AT_TO_ODID(settings->aircraft_type);
  utm_parameters.ID_type2    = ODID_IDTYPE_NONE;
  utm_parameters.region      = ODID_CLASSIFICATION_TYPE_EU;
  utm_parameters.EU_category = ODID_CATEGORY_EU_OPEN;
  utm_parameters.EU_class    = ODID_CLASS_EU_CLASS_4;

  squitter.init(&utm_parameters);

  memset(&utm_data,0,sizeof(utm_data));

  return rid_status;
}

bool rid_enabled() {
  return rid_status;
}

size_t rid_encode(void *pkt, ufo_t *this_aircraft) {

  utm_data.latitude_d  = (double) this_aircraft->latitude;
  utm_data.longitude_d = (double) this_aircraft->longitude;
  utm_data.alt_msl_m   = (float)  this_aircraft->altitude;
  utm_data.alt_agl_m   = (float)  INV_ALT /* TBD */;
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

  utm_data.csecs      = (int) gnss.time.centisecond();
  utm_data.seconds    = (int) gnss.time.second();
  utm_data.minutes    = (int) gnss.time.minute();
  utm_data.hours      = (int) gnss.time.hour();

  utm_data.days       = (int) gnss.date.day();
  utm_data.months     = (int) gnss.date.month();
  utm_data.years      = (int) gnss.date.year();

/*
 * SATS_LEVEL_1   4
 * SATS_LEVEL_2   7
 * SATS_LEVEL_3  10
 */
  utm_data.satellites = (int) gnss.satellites.value();

  return 0;
}

bool rid_decode(void *pkt, ufo_t *this_aircraft, ufo_t *fop) {

  /* N/A */

  return false;
}
#endif /* ENABLE_REMOTE_ID */
