/*
 * NMEAHelper.cpp
 * Copyright (C) 2017-2018 Linar Yusupov
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
#include <nmealib.h>

#include "NMEAHelper.h"
#include "GNSSHelper.h"
#include "RFHelper.h"
#include "SoCHelper.h"
#include "WiFiHelper.h"
#include "EEPROMHelper.h"
#include "TrafficHelper.h"

#if defined(AIRCONNECT_IS_ACTIVE)
WiFiServer AirConnectServer(AIR_CONNECT_PORT);
WiFiClient AirConnectClient[MAX_AIRCONNECT_CLIENTS];
#endif

char NMEABuffer[128]; //buffer for NMEA data
NmeaMallocedBuffer nmealib_buf;

const char *NMEA_CallSign_Prefix[] = {
  [RF_PROTOCOL_LEGACY]    = "FLR",
  [RF_PROTOCOL_OGNTP]     = "OGN",
  [RF_PROTOCOL_P3I]       = "PAW",
  [RF_PROTOCOL_ADSB_1090] = "ADS",
  [RF_PROTOCOL_ADSB_UAT]  = "UAT",
  [RF_PROTOCOL_FANET]     = "FAN"
};

static char *ltrim(char *s)
{
  if(s) {
    while(*s && isspace(*s))
      ++s;
  }

  return s;
}

void NMEA_setup()
{
#if defined(AIRCONNECT_IS_ACTIVE)
  AirConnectServer.begin();
  Serial.print(F("AirConnect server has started at port: "));
  Serial.println(AIR_CONNECT_PORT);

  AirConnectServer.setNoDelay(true);
#endif
  memset(&nmealib_buf, 0, sizeof(nmealib_buf));
}

void NMEA_loop()
{
#if defined(AIRCONNECT_IS_ACTIVE)
  uint8_t i;

  if (AirConnectServer.hasClient()){
    for(i = 0; i < MAX_AIRCONNECT_CLIENTS; i++){
      // find free/disconnected spot
      if (!AirConnectClient[i] || !AirConnectClient[i].connected()){
        if(AirConnectClient[i]) AirConnectClient[i].stop();
        AirConnectClient[i] = AirConnectServer.available();
        break;
      }
    }
    if (i >= MAX_AIRCONNECT_CLIENTS) {
      // no free/disconnected spot so reject
      AirConnectServer.available().stop();
    }
  }
#endif
}

void NMEA_Out(byte *buf, size_t size, bool nl)
{
  Serial.write(buf, size);
  if (nl)
    Serial.write('\n');

  if (SoC->Bluetooth) {
    SoC->Bluetooth->write(buf, size);
    if (nl)
      SoC->Bluetooth->write((byte *) "\n", 1);
  }

  if (settings->nmea_u) {
    size_t udp_size = size;

    if (size >= sizeof(UDPpacketBuffer))
      udp_size = sizeof(UDPpacketBuffer) - 1;
    memcpy(UDPpacketBuffer, buf, udp_size);

    if (nl)
      UDPpacketBuffer[udp_size] = '\n';

    SoC->WiFi_transmit_UDP(NMEA_DST_PORT, (byte *) UDPpacketBuffer,
                            nl ? udp_size + 1 : udp_size);
  }

#if defined(AIRCONNECT_IS_ACTIVE)
  for (uint8_t acc_ndx = 0; acc_ndx < MAX_AIRCONNECT_CLIENTS; acc_ndx++) {
    if (AirConnectClient[acc_ndx] && AirConnectClient[acc_ndx].connected()){
      AirConnectClient[acc_ndx].write(buf, size);
      if (nl)
        AirConnectClient[acc_ndx].write('\n');
    }
  }
#endif
}

void NMEA_Export()
{
    int bearing;
    int alt_diff;
    float distance;
    int total_objects = 0;
    int alarm_level = ALARM_LEVEL_NONE;
    time_t this_moment = now();

    /* High priority object (most relevant target) */
    int HP_bearing = 0;
    int HP_alt_diff = 0;
    int HP_alarm_level = ALARM_LEVEL_NONE;
    float HP_distance = 2147483647;

    /* account for all detected objects at first */
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].addr && (this_moment - Container[i].timestamp) <= EXPORT_EXPIRATION_TIME) {
        total_objects++;
      }
    }

    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].addr && (this_moment - Container[i].timestamp) <= EXPORT_EXPIRATION_TIME) {

#if 0
        Serial.println(fo.addr);
        Serial.println(fo.latitude, 4);
        Serial.println(fo.longitude, 4);
        Serial.println(fo.altitude);
        Serial.println(fo.addr_type);
        Serial.println(fo.vs);
        Serial.println(fo.aircraft_type);
        Serial.println(fo.stealth);
        Serial.println(fo.no_track);
#endif
        if (settings->nmea_l) {
          distance = Container[i].distance;

          if (distance < ALARM_ZONE_NONE) {

            char *csum_ptr;
            unsigned char cs = 0;
            char str_climb_rate[8] = "";

            bearing = Container[i].bearing;
            alarm_level = Container[i].alarm_level;
            alt_diff = (int) (Container[i].altitude - ThisAircraft.altitude);

            if (!Container[i].stealth && !ThisAircraft.stealth) {
              dtostrf(
                constrain(Container[i].vs / (_GPS_FEET_PER_METER * 60.0), -32.7, 32.7),
                5, 1, str_climb_rate);
            }

            snprintf(NMEABuffer, sizeof(NMEABuffer), "$PFLAA,%d,%d,%d,%d,%d,%06X!%s_%06X,%d,,%d,%s,%d*",
                    alarm_level,
                    (int) (distance * cos(radians(bearing))), (int) (distance * sin(radians(bearing))),
                    alt_diff, ADDR_TYPE_FLARM, Container[i].addr,
                    NMEA_CallSign_Prefix[Container[i].protocol], Container[i].addr,
                    (int) Container[i].course, (int) (Container[i].speed * _GPS_MPS_PER_KNOT),
                    ltrim(str_climb_rate), Container[i].aircraft_type);

            //calculate the checksum
            for (unsigned int n = 1; n < strlen(NMEABuffer) - 1; n++) {
              cs ^= NMEABuffer[n];
            }

            csum_ptr = NMEABuffer + strlen(NMEABuffer);
            snprintf(csum_ptr, sizeof(NMEABuffer) - strlen(NMEABuffer), "%02X\r\n", cs);

            NMEA_Out((byte *) NMEABuffer, strlen(NMEABuffer), false);

            /* Most close traffic is treated as highest priority target */
            if (distance < HP_distance) {
              HP_bearing = bearing;
              HP_alt_diff = alt_diff;
              HP_alarm_level = alarm_level;
              HP_distance = distance;
            }

          }
        }
      }
    }

    /* One PFLAU NMEA sentence is mandatory regardless of traffic reception status */
    if (settings->nmea_l) {

      char *csum_ptr;
      unsigned char cs = 0;

      snprintf(NMEABuffer, sizeof(NMEABuffer), "$PFLAU,%d,%d,%d,%d,%d,%d,%d,%d,%u*",
              total_objects, TX_STATUS_ON, GNSS_STATUS_3D_MOVING,
              POWER_STATUS_GOOD, HP_alarm_level,
              (HP_bearing < 180 ? HP_bearing : HP_bearing - 360),
              ALARM_TYPE_AIRCRAFT, HP_alt_diff, (int) HP_distance );

      //calculate the checksum
      for (unsigned int n = 1; n < strlen(NMEABuffer) - 1; n++) {
        cs ^= NMEABuffer[n];
      }

      csum_ptr = NMEABuffer + strlen(NMEABuffer);
      snprintf(csum_ptr, sizeof(NMEABuffer) - strlen(NMEABuffer), "%02X\r\n", cs);

      NMEA_Out((byte *) NMEABuffer, strlen(NMEABuffer), false);
    }
}

void NMEA_Position()
{
  NmeaInfo info;
  size_t i;
  struct timeval tv;

  if (settings->nmea_g) {

    nmeaInfoClear(&info);

    info.sig = NMEALIB_SIG_SENSITIVE;
    info.fix = NMEALIB_FIX_3D;

    tv.tv_sec  = ThisAircraft.timestamp;
    tv.tv_usec = 0;

    nmeaTimeSet(&info.utc, &info.present, &tv);

    info.latitude = ((int) ThisAircraft.latitude) * 100.0;
    info.latitude += (ThisAircraft.latitude - (int) ThisAircraft.latitude) * 60.0;
    info.longitude = ((int) ThisAircraft.longitude) * 100.0;
    info.longitude += (ThisAircraft.longitude - (int) ThisAircraft.longitude) * 60.0;
    info.speed = ThisAircraft.speed * _GPS_KMPH_PER_KNOT;
    info.elevation = ThisAircraft.altitude; /* above MSL */
    info.height = LookupSeparation(ThisAircraft.latitude, ThisAircraft.longitude);
    info.track = ThisAircraft.course;

#if 0
    info.mtrack = 55;
    info.magvar = 55;
#endif
    info.hdop = 2.3;
    info.vdop = 1.2;
    info.pdop = 2.594224354;

    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SIG);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_FIX);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LAT);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LON);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SPEED);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_ELV);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HEIGHT);

    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_TRACK);
#if 0
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_MTRACK);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_MAGVAR);
#endif
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HDOP);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_VDOP);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_PDOP);

#if 0
    info.satellites.inUseCount = NMEALIB_MAX_SATELLITES;
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINUSECOUNT);
    for (i = 0; i < NMEALIB_MAX_SATELLITES; i++) {
      info.satellites.inUse[i] = (unsigned int) (i + 1);
    }
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINUSE);

    info.satellites.inViewCount = NMEALIB_MAX_SATELLITES;
    for (i = 0; i < NMEALIB_MAX_SATELLITES; i++) {
      info.satellites.inView[i].prn = (unsigned int) i + 1;
      info.satellites.inView[i].elevation = (int) ((i * 10) % 90);
      info.satellites.inView[i].azimuth = (unsigned int) (i + 1);
      info.satellites.inView[i].snr = 99 - (unsigned int) i;
    }
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINVIEWCOUNT);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINVIEW);
#endif

    size_t gen_sz = nmeaSentenceFromInfo(&nmealib_buf, &info, (NmeaSentence)
      (NMEALIB_SENTENCE_GPGGA | NMEALIB_SENTENCE_GPGSA | NMEALIB_SENTENCE_GPRMC));

    if (ThisAircraft.pressure_altitude != 0.0 &&
        /* Assume that space of 24 bytes is sufficient for PGRMZ */
        nmealib_buf.bufferSize - gen_sz > 24) {

      int altitude = constrain(
                    (int) (ThisAircraft.pressure_altitude * _GPS_FEET_PER_METER),
                    -1000, 60000);

      snprintf(nmealib_buf.buffer + gen_sz, 24, "$PGRMZ,%d,f,3*",
              altitude ); /* feet , 3D fix */

      size_t sentence_size = strlen(nmealib_buf.buffer + gen_sz);

      //calculate the checksum
      unsigned char cs = 0;
      for (unsigned int n = 1; n < sentence_size - 1; n++) {
        cs ^= nmealib_buf.buffer[gen_sz + n];
      }

      char *csum_ptr = nmealib_buf.buffer + (gen_sz + sentence_size);
      snprintf(csum_ptr, 8, "%02X\r\n", cs);

      gen_sz += sentence_size + strlen(csum_ptr);
    }

    if (gen_sz) {
      NMEA_Out((byte *) nmealib_buf.buffer, gen_sz, false);
    }
  }
}

void NMEA_GGA()
{
  NmeaInfo info;

  float latitude = gnss.location.lat();
  float longitude = gnss.location.lng();

  nmeaInfoClear(&info);

  info.utc.hour = gnss.time.hour();
  info.utc.min = gnss.time.minute();
  info.utc.sec = gnss.time.second();
  info.utc.hsec = gnss.time.centisecond();

  info.latitude = ((int) latitude) * 100.0;
  info.latitude += (latitude - (int) latitude) * 60.0;
  info.longitude = ((int) longitude) * 100.0;
  info.longitude += (longitude - (int) longitude) * 60.0;

  info.sig = (NmeaSignal) gnss.location.Quality();
  info.satellites.inViewCount = gnss.satellites.value();

  info.hdop = gnss.hdop.hdop();

  info.elevation = gnss.altitude.meters(); /* above MSL */
  info.height = gnss.separation.meters();

  if (info.height == 0.0) {
    info.height = LookupSeparation(latitude, longitude);
    info.elevation -= info.height;
  }

  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_UTCTIME);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LAT);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LON);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SIG);
  /* Should be SATINUSECOUNT, but it seems to be a bug in NMEALib */
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINVIEWCOUNT);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HDOP);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_ELV);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HEIGHT);

  size_t gen_sz = nmeaSentenceFromInfo(&nmealib_buf, &info, (NmeaSentence)
                                        NMEALIB_SENTENCE_GPGGA );

  if (gen_sz) {
    NMEA_Out((byte *) nmealib_buf.buffer, gen_sz, false);
  }
}
