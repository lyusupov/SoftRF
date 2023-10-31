/* -*- tab-width: 2; mode: c; -*-
 * 
 * Seconds since 1/1/1970 for systems that don't have the unix time() function. 
 * 
 * The Julian day algorithm is from Jean Meeus's Astronomical Algorithms
 * as are the two test dates.
 * 
 */

#if defined(ARDUINO)

#include <Arduino.h>

#else

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>

#endif

#include <math.h>

uint64_t      alt_unix_secs(int,int,int,int,int,int);
static double julian_day(int,int,float,int);


/*
 *
 */

#ifndef ARDUINO

int main(int argc,char *argv[]) {

  uint64_t   alt_secs;
  time_t     unix_secs;
  struct tm *gmt;
  
  time(&unix_secs);

  gmt = gmtime(&unix_secs);
  
  alt_secs = alt_unix_secs(1900 + gmt->tm_year,1 + gmt->tm_mon,gmt->tm_mday,
                           gmt->tm_hour,gmt->tm_min,gmt->tm_sec);
  
  printf("\nunix: %10lu\n",(unsigned long int) unix_secs);
  printf(  "alt:  %10lu\n",(unsigned long int) alt_secs);
  printf(  "      %10d\n",(int) ((int64_t) unix_secs - (int64_t) alt_secs));

  printf("\nJD 27/1/333: %10.2f\n",julian_day(333,1,27.5,0));
  printf(  "JD Sputnik:  %10.2f\n",julian_day(1957,10,4.81,1));

  return 0;
}

#endif

/*
 *
 */

uint64_t alt_unix_secs(int year,int month,int mday,
                       int hour,int minute,int second) {

  uint64_t        secs = 0;
  static uint64_t jd_1970 = 0;

  if (!jd_1970) {

    jd_1970 = (uint64_t) julian_day(1970,1,1,1) * (uint64_t) 86400;
  }
  
  secs  = (uint64_t) julian_day(year,month,mday,1) * (uint64_t) 86400;
  secs += (uint64_t) (((uint32_t) hour   * 3600) +
                      ((uint32_t) minute *   60) +
                      ((uint32_t) second));
  secs -= jd_1970;

  return secs;
}


/*
 *
 */

double julian_day(int year,int month,float mday,int gregorian) {

  int      a;
  double   y, m, d, jday = 0.0, b = 0.0;

  if (month < 3) {

    --year;
    month += 12;
  }

  if (gregorian) {
    a = year / 100;
    b = 2.0 - (double) a + (double) (a/4);
  }

  y = (double) (year  + 4716);
  m = (double) (month + 1);
  d = (double) mday;
  
  jday = floor(365.25  * y) +
         floor(30.6001 * m) +
         d + b - 1524.5;
  
  return jday;
}
  
/*
 *
 */

