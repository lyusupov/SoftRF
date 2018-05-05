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

#if defined(AIRCONNECT_IS_ACTIVE)
WiFiServer AirConnectServer(AIR_CONNECT_PORT);
WiFiClient AirConnectClient;
#endif

char NMEABuffer[128]; //buffer for NMEA data
NmeaMallocedBuffer nmealib_buf;

extern ufo_t fo, Container[MAX_TRACKING_OBJECTS];
extern ufo_t ThisAircraft;

const char *NMEA_CallSign_Prefix[] = {
  [RF_PROTOCOL_LEGACY]    = "FLR",
  [RF_PROTOCOL_OGNTP]     = "OGN",
  [RF_PROTOCOL_P3I]       = "PAW",
  [RF_PROTOCOL_ADSB_1090] = "ADS",
  [RF_PROTOCOL_ADSB_UAT]  = "UAT",
  [RF_PROTOCOL_FANET]     = "FAN"
};

//convert degrees to radians
double dtor(double fdegrees)
{
  return(fdegrees * PI / 180);
}

//Convert radians to degrees
double rtod(double fradians)
{
  return(fradians * 180.0 / PI);
}

/*
 * arbarnhart
 * 
 * http://forum.arduino.cc/index.php?topic=45760.msg332012#msg332012
 */
//Calculate bearing from lat1/lon1 to lat2/lon2
//Note lat1/lon1/lat2/lon2 must be in radians
//Returns bearing in degrees
int CalcBearing(double lat1, double lon1, double lat2, double lon2)
{
  lat1 = dtor(lat1);
  lon1 = dtor(lon1);
  lat2 = dtor(lat2);
  lon2 = dtor(lon2);
  
  //determine angle
  double bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)));
  //convert to degrees
  bearing = rtod(bearing);
  //use mod to turn -90 = 270
  //bearing = fmod((bearing + 360.0), 360);
  //return (int) bearing + 0.5;
  return ((int) bearing + 360) % 360;
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
  if (AirConnectServer.hasClient()){
    if (!AirConnectClient || !AirConnectClient.connected()){
      if(AirConnectClient) AirConnectClient.stop();
      AirConnectClient = AirConnectServer.available();
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

    /* account for all detected objects at first */
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].addr && (this_moment - Container[i].timestamp) <= EXPORT_EXPIRATION_TIME) {
        total_objects++;
      }
    }

    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].addr && (this_moment - Container[i].timestamp) <= EXPORT_EXPIRATION_TIME) {

        char *csum_ptr;
        unsigned char cs = 0; //clear any old checksum

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
          distance = gnss.distanceBetween(ThisAircraft.latitude,
            ThisAircraft.longitude,
            Container[i].latitude,
            Container[i].longitude);

          if (distance < EXPORT_DISTANCE_FAR) {

            if (distance < EXPORT_DISTANCE_CLOSE) {
              alarm_level = ALARM_LEVEL_URGENT;
            } else if (distance < EXPORT_DISTANCE_NEAR) {
              alarm_level = ALARM_LEVEL_IMPORTANT;
            } else {
              alarm_level = ALARM_LEVEL_LOW;
            }

            bearing = gnss.courseTo(ThisAircraft.latitude, ThisAircraft.longitude,
                                Container[i].latitude, Container[i].longitude);
            alt_diff = (int) (Container[i].altitude - ThisAircraft.altitude);
            snprintf(NMEABuffer, sizeof(NMEABuffer), "$PFLAU,%d,%d,%d,%d,%d,%d,%d,%d,%u*",
                    total_objects, TX_STATUS_ON, GNSS_STATUS_3D_MOVING,
                    POWER_STATUS_GOOD, alarm_level,
                    (bearing < 180 ? bearing : bearing - 360),
                    ALARM_TYPE_AIRCRAFT, alt_diff, (int) distance );

            //calculate the checksum
            for (unsigned int n = 1; n < strlen(NMEABuffer) - 1; n++) {
              cs ^= NMEABuffer[n];
            }

            csum_ptr = NMEABuffer + strlen(NMEABuffer);
            snprintf(csum_ptr, sizeof(NMEABuffer) - strlen(NMEABuffer), "%02X\r\n", cs);

            Serial.write((uint8_t *) NMEABuffer, strlen(NMEABuffer));
            SoC->BltnBT_write((uint8_t *) NMEABuffer, strlen(NMEABuffer));

            if (settings->nmea_u) {
              SoC->WiFi_transmit_UDP(NMEA_DST_PORT, (byte *) NMEABuffer, strlen(NMEABuffer));
            }

#if defined(AIRCONNECT_IS_ACTIVE)
            if (AirConnectClient && AirConnectClient.connected()){
              AirConnectClient.write(NMEABuffer, strlen(NMEABuffer));
            }
#endif
            snprintf(NMEABuffer, sizeof(NMEABuffer), "$PFLAA,%d,%d,%d,%d,%d,%X!%s_%X,%d,,%d,,%d*",
                    alarm_level,
                    (int) (distance * cos(dtor(bearing))), (int) (distance * sin(dtor(bearing))),
                    alt_diff, ADDR_TYPE_FLARM, Container[i].addr,
                    NMEA_CallSign_Prefix[Container[i].protocol], Container[i].addr,
                    (int) Container[i].course, (int) (Container[i].speed * _GPS_MPS_PER_KNOT),
                    AIRCRAFT_TYPE_GLIDER);

            cs = 0; //clear any old checksum
            for (unsigned int n = 1; n < strlen(NMEABuffer) - 1; n++) {
              cs ^= NMEABuffer[n]; //calculates the checksum
            }

            csum_ptr = NMEABuffer + strlen(NMEABuffer);
            snprintf(csum_ptr, sizeof(NMEABuffer) - strlen(NMEABuffer), "%02X\r\n", cs);

            Serial.write((uint8_t *) NMEABuffer, strlen(NMEABuffer));
            SoC->BltnBT_write((uint8_t *) NMEABuffer, strlen(NMEABuffer));

            if (settings->nmea_u) {
              SoC->WiFi_transmit_UDP(NMEA_DST_PORT, (byte *) NMEABuffer, strlen(NMEABuffer));
            }

#if defined(AIRCONNECT_IS_ACTIVE)
            if (AirConnectClient && AirConnectClient.connected()){
              AirConnectClient.write(NMEABuffer, strlen(NMEABuffer));
            }
#endif
          }
        }
      }
    }
}

void NMEA_Position()
{
  NmeaInfo info;
  size_t i;
  IPAddress broadcastIP = SoC->WiFi_get_broadcast();
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
    info.elevation = ThisAircraft.altitude;
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
      Serial.write((uint8_t *) nmealib_buf.buffer, gen_sz);
      SoC->BltnBT_write((uint8_t *) nmealib_buf.buffer, gen_sz);

      if (settings->nmea_u) {
        SoC->WiFi_transmit_UDP(NMEA_DST_PORT, (byte *) nmealib_buf.buffer, gen_sz);
      }

#if defined(AIRCONNECT_IS_ACTIVE)
      if (AirConnectClient && AirConnectClient.connected()){
        AirConnectClient.write((char *) nmealib_buf.buffer, gen_sz);
      }
#endif
    }
  }
}
