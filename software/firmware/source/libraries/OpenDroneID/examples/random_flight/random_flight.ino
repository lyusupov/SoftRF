/* -*- tab-width: 2; mode: c; -*-
 * 
 * A random wander around the BMFA's Buckminster flying centre.
 * 
 */

#pragma GCC diagnostic warning "-Wunused-variable"

#include <Arduino.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#if defined(ARDUINO_ARCH_NRF52)
#include <avr/dtostrf.h>
#endif

#include "id_open.h"

static ID_OpenDrone          squitter;
static UTM_Utilities         utm_utils;

static struct UTM_parameters utm_parameters;
static struct UTM_data       utm_data;

static int    speed_kn = 40;
static float  x = 0.0, y = 0.0, z = 100.0, speed_m_x, max_dir_change = 75.0;
static double deg2rad = 0.0, m_deg_lat = 0.0, m_deg_long = 0.0;

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
  clock_tm.tm_mday  =  16;
  clock_tm.tm_mon   =  11;
  clock_tm.tm_year  = 122;

  tv.tv_sec =
  time_2    = mktime(&clock_tm);
  
#if !defined(ARDUINO_ARCH_NRF52)
  settimeofday(&tv,&utc);
#endif

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

  //  52°46'49.89"N 0°42'26.26"W

  lat_d                   = 
  utm_data.latitude_d     =
  utm_data.base_latitude  = 52.0 + (46.0 / 60.0) + (49.89 / 3600.0);
  long_d                  =
  utm_data.longitude_d    =
  utm_data.base_longitude =  0.0 - (42.0 / 60.0) - (26.26 / 3600.0);
  utm_data.base_alt_m     = 137.0;

  utm_data.alt_msl_m      = utm_data.base_alt_m + z;
  utm_data.alt_agl_m      = z;

  utm_data.speed_kn   = speed_kn;
  utm_data.satellites = 8;
  utm_data.base_valid = 1;

  speed_m_x = ((float) speed_kn) * 0.514444 / 5.0; // Because we update every 200 ms.

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

  int             dir_change;
  char            text[64], lat_s[16], long_s[16];
  float           rads, ran;
  uint32_t        msecs;
  static uint32_t last_update = 0;

  msecs = millis();

  if ((msecs - last_update) > 199) {

    last_update = msecs;

    ran                  = 0.001 * (float) (((int) rand() % 1000) - 500);
    dir_change           = (int) (max_dir_change * ran);
    utm_data.heading     = (utm_data.heading + dir_change + 360) % 360;

    x                   += speed_m_x * sin(rads = (deg2rad * (float) utm_data.heading));
    y                   += speed_m_x * cos(rads);

    utm_data.latitude_d  = utm_data.base_latitude  + (y / m_deg_lat);
    utm_data.longitude_d = utm_data.base_longitude + (x / m_deg_long);

    dtostrf(utm_data.latitude_d,10,7,lat_s);
    dtostrf(utm_data.longitude_d,10,7,long_s);

#if 1
    sprintf(text,"%s,%s,%d,%d,%d\r\n",
            lat_s,long_s,utm_data.heading,utm_data.speed_kn,dir_change);
    Serial.print(text);
#endif

    squitter.transmit(&utm_data);
  }

  return;
}

//
