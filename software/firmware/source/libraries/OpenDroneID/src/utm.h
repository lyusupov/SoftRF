/* -*- tab-width: 2; mode: c; -*-
 *
 * UTM/eID interface structure definition, some defines and an object for 
 * utility functions.
 *
 * Copyright (c) 2020, Steve Jack.
 *
 */

#ifndef UTM_H
#define UTM_H

#if not defined(SATS_LEVEL_1)
#define SATS_LEVEL_1   4
#endif

#if not defined(SATS_LEVEL_2)
#define SATS_LEVEL_2   7
#endif

#if not defined(SATS_LEVEL_3)
#define SATS_LEVEL_3  10
#endif

#define ID_SIZE       24

//

struct UTM_parameters {

  char    UAS_operator[ID_SIZE];
  char    UAV_id[ID_SIZE];
  char    flight_desc[ID_SIZE];
  uint8_t UA_type, ID_type, region, spare1,
          EU_category, EU_class, ID_type2, spare3;
  char    UTM_id[ID_SIZE * 2];
  char    secret[4];
  uint8_t spare[28];
};

//

struct UTM_data {

  int    years;
  int    months;
  int    days;
  int    hours;
  int    minutes;
  int    seconds;
  int    csecs;
  double latitude_d;
  double longitude_d;
  float  alt_msl_m;
  float  alt_agl_m;
  int    speed_kn;
  int    heading;
  char  *hdop_s;
  char  *vdop_s;
  int    satellites;
  double base_latitude;
  double base_longitude;
  float  base_alt_m;
  int    base_valid;
  int    vel_N_cm;
  int    vel_E_cm;
  int    vel_D_cm;
};

/*
 *
 */

class UTM_Utilities {

 public:

       UTM_Utilities(void);

  void calc_m_per_deg(double,double,double *,double *);
  void calc_m_per_deg(double,double *,double *);

  int  check_EU_op_id(const char *,const char *);
  char luhn36_check(const char *);
  int  luhn36_c2i(char);
  char luhn36_i2c(int);

 private:

  char s[20];
};

#endif
