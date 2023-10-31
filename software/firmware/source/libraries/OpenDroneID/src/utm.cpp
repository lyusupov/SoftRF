/* -*- tab-width: 2; mode: c; -*-
 * 
 * UTM/eID utility functions.
 *
 * Copyright (c) 2020, Steve Jack.
 *
 * Notes
 *
 * 
 */

#pragma GCC diagnostic warning "-Wunused-variable"

#define DIAGNOSTICS  1

#include <Arduino.h>

#include "utm.h"

/*
 *
 */

UTM_Utilities::UTM_Utilities() {

  memset(s,0,sizeof(s));

  return;
}

/*
 *
 */

void UTM_Utilities::calc_m_per_deg(double lat_d,double long_d,double *m_deg_lat,double *m_deg_long) {

  calc_m_per_deg(lat_d,m_deg_lat,m_deg_long);

  return;
}

//

void UTM_Utilities::calc_m_per_deg(double lat_d,double *m_deg_lat,double *m_deg_long) {

  double pi, deg2rad, sin_lat, cos_lat;

  pi          = 4.0 * atan(1.0);
  deg2rad     = pi / 180.0;

  lat_d      *= deg2rad;

  sin_lat     = sin(lat_d);
  cos_lat     = cos(lat_d);

#if 1 // Wikipedia 

  double a = 0.08181922, b, radius;

  b           = a * sin_lat;
  radius      = 6378137.0 * cos_lat / sqrt(1.0 - (b * b));
  *m_deg_long = deg2rad * radius;
  *m_deg_lat  = 111132.954 - (559.822 * cos(2.0 * lat_d)) - 
                (1.175 * cos(4.0 * lat_d));

#else // Astronomical Algorithms

  double a = 6378140.0, c, d, e = 0.08181922, rho, Rp = 0.0, Rm = 0.0;

  rho         = 0.9983271 + (0.0016764 * cos(2.0 * lat_d)) - (0.0000035 * cos(4.0 * lat_d));
  c           = e * sin_lat;
  d           = sqrt(1.0 - (c * c));
  Rp          = a * cos_lat / d;
  *m_deg_long = deg2rad * Rp;
  Rm          = a * (1.0 - (e * e)) / pow(d,3);
  *m_deg_lat  = deg2rad * Rm;

#endif

  return;
}

/*
 *
 */

int UTM_Utilities::check_EU_op_id(const char *id,const char *secret) {

  int  i, j;
  char check;

  if ((strlen(id) != 16)&&(strlen(secret) != 3)) {

    return 0;
  }

  for (i = 0, j = 0; i < 12; ++i) {

    s[j++] = id[i + 3];
  }
  
  for (i = 0; i < 3; ++i) {

    s[j++] = secret[i];
  }

  s[j] = 0;

  check = luhn36_check(s);
  
  return ((id[15] == check) ? 1: 0);
}

/*
 *
 */

char UTM_Utilities::luhn36_check(const char *s) {

  int       sum = 0, factor = 2, l, i, add, rem;
  const int base = 36;

  l = strlen(s);

  for (i = l - 1; i >= 0; --i) {

    add    = luhn36_c2i(s[i]) * factor;
    sum   += (add / base) + (add % base);

    factor = (factor == 2) ? 1: 2;
  }

  rem = sum % base;
  
  return luhn36_i2c(base - rem);
}

/*
 *
 */

int UTM_Utilities::luhn36_c2i(char c) {

  if ((c >= '0')&&(c <= '9')) {

    return (c - '0');

  } else if ((c >= 'a')&&(c <= 'z')) {

    return (10 + (c - 'a'));

  } else if ((c >= 'A')&&(c <= 'Z')) {

    return (10 + (c - 'A'));
  }

  return 0;
}

/*
 *
 */

char UTM_Utilities::luhn36_i2c(int i) {

  if ((i >= 0)&&(i <= 9)) {

    return ('0' + i);
    
  } else if ((i >= 10)&&(i < 36)) {

    return ('a' + i - 10);
  }

  return '0';
}

/*
 *
 */

 
