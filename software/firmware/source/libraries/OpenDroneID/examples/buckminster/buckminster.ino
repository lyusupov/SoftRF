/* -*- tab-width: 2; mode: c; -*-
 * 
 * Points around the BMFA's Buckminster flying centre.
 * 
 */

#pragma GCC diagnostic warning "-Wunused-variable"

#include <Arduino.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "id_open.h"

#define WAYPOINTS 6

static ID_OpenDrone          squitter;
static UTM_Utilities         utm_utils;

static struct UTM_parameters utm_parameters;
static struct UTM_data       utm_data;

static int    speed_kn = 40, waypoint = 0;
static float  z = 100.0;
static double deg2rad = 0.0, m_deg_lat = 0.0, m_deg_long = 0.0;
static double latitude[WAYPOINTS], longitude[WAYPOINTS];

void setup() {

  char            text[64];
  double          lat_d, long_d;
  time_t          time_2;
  struct tm       clock_tm;
  struct timeval  tv = {0,0};
  struct timezone utc = {0,0};
  
  //

  Serial.begin(115200);
  Serial.print("\nSerial\n\n\r");

// Do not use Serial1 if using an ESP32.
#if defined(ARDUINO_ARCH_RP2040)
  Serial1.begin(115200);
  Serial1.print("\nSerial1\n\n\r");
#endif

#if 0
  Serial2.begin(115200);
  Serial2.print("\nSerial2\n\n\r");
#endif

  deg2rad = (4.0 * atan(1.0)) / 180.0;

  //

  memset(&clock_tm,0,sizeof(struct tm));

  clock_tm.tm_hour  =  10;
  clock_tm.tm_mday  =  27;
  clock_tm.tm_mon   =  11;
  clock_tm.tm_year  = 122;

  tv.tv_sec =
  time_2    = mktime(&clock_tm);
  
  settimeofday(&tv,&utc);

  delay(500);

  Serial.print(ctime(&time_2));
  
  //

  memset(&utm_parameters,0,sizeof(utm_parameters));

#if 0
  strcpy(utm_parameters.UAS_operator,"GBR-OP-1234ABCDEFGH");
#elif defined(ARDUINO_ARCH_ESP32)
  strcpy(utm_parameters.UAS_operator,"GBR-OP-ESP32");
#elif defined(ARDUINO_ARCH_ESP8266)
  strcpy(utm_parameters.UAS_operator,"GBR-OP-ESP8266");
#elif defined(ARDUINO_ARCH_RP2040)
  strcpy(utm_parameters.UAS_operator,"GBR-OP-PICOW");
#else
  strcpy(utm_parameters.UAS_operator,"GBR-OP-UNKNOWN");
#endif

  utm_parameters.region      = 1;
  utm_parameters.EU_category = 1;
  utm_parameters.EU_class    = 5;

  squitter.init(&utm_parameters);
  
  memset(&utm_data,0,sizeof(utm_data));

  //

  latitude[0]  = 52.0 + (46.0 / 60.0) + (49.27 / 3600.0);
  longitude[0] =  0.0 - (42.0 / 60.0) - (27.73 / 3600.0);
  
  latitude[1]  = 52.0 + (46.0 / 60.0) + (51.91 / 3600.0);
  longitude[1] =  0.0 - (42.0 / 60.0) - (20.74 / 3600.0);

  latitude[2]  = 52.0 + (46.0 / 60.0) + (48.80 / 3600.0);
  longitude[2] =  0.0 - (42.0 / 60.0) - (33.52 / 3600.0);

  latitude[3]  = 52.0 + (46.0 / 60.0) + (50.89 / 3600.0);
  longitude[3] =  0.0 - (42.0 / 60.0) - (36.58 / 3600.0);

  latitude[4]  = 52.0 + (46.0 / 60.0) + (54.11 / 3600.0);
  longitude[4] =  0.0 - (42.0 / 60.0) - (29.52 / 3600.0);

  latitude[5]  = 52.0 + (46.0 / 60.0) + (55.54 / 3600.0);
  longitude[5] =  0.0 - (42.0 / 60.0) - (20.00 / 3600.0);

  //
  
  utm_data.latitude_d     = latitude[1];
  utm_data.longitude_d    = longitude[1];

  lat_d                   = 
  utm_data.base_latitude  = latitude[0];
  long_d                  =
  utm_data.base_longitude = longitude[0];

  utm_data.base_alt_m     = 137.0;

  utm_data.alt_msl_m      = utm_data.base_alt_m + z;
  utm_data.alt_agl_m      = z;

  utm_data.speed_kn   = speed_kn;
  utm_data.satellites = 12;
  utm_data.base_valid =  1;

  //

  utm_utils.calc_m_per_deg(lat_d,&m_deg_lat,&m_deg_long);

  //

  Serial.print("\r\n");

  sprintf(text,"%d degrees/radian\r\n",(int) (1.0 / deg2rad));
  Serial.print(text);

  sprintf(text,"%d m per degree latitude\r\n",(int) m_deg_lat);
  Serial.print(text);

  sprintf(text,"%d m per degree longitude\r\n",(int) m_deg_long);
  Serial.print(text);

  Serial.print("\r\n");

  //

  srand(micros());

  return;
}

//

void loop() {

  char            text[64], lat_s[16], long_s[16];
  uint32_t        msecs;
  time_t          time_2;
  struct tm      *gmt;
  static uint32_t last_update = 0, last_waypoint = 0;

  msecs = millis();

  if ((msecs - last_waypoint) > 9999) {

    last_waypoint = msecs;

    utm_data.latitude_d  = latitude[waypoint];
    utm_data.longitude_d = longitude[waypoint];

    dtostrf(utm_data.latitude_d,10,7,lat_s);
    dtostrf(utm_data.longitude_d,10,7,long_s);

#if 1
    sprintf(text,"%d,%s,%s,%d,%d\r\n",
            waypoint,lat_s,long_s,utm_data.heading,utm_data.speed_kn);
    Serial.print(text);
#endif

    if (++waypoint >= WAYPOINTS) {
      waypoint = 1;
    }
  }

  if ((msecs - last_update) > 24) {
    last_update = msecs;
    time(&time_2);

    gmt = gmtime(&time_2);

    utm_data.seconds = gmt->tm_sec;
    utm_data.minutes = gmt->tm_min;
    utm_data.hours   = gmt->tm_hour;

    squitter.transmit(&utm_data);
  }

  return;
}

//
