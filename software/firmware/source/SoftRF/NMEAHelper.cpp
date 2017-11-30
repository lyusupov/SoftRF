/*
 * NMEAHelper.cpp
 * Copyright (C) 2017 Linar Yusupov
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
#include <WiFiUdp.h>

#include "NMEAHelper.h"
#include "GNSSHelper.h"
#include "RFHelper.h"
#include "WiFiHelper.h"

char NMEABuffer[128]; //buffer for NMEA data

extern ufo_t fo, Container[MAX_TRACKING_OBJECTS];
extern ufo_t ThisAircraft;

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

void NMEA_Export()
{
    int bearing;
    int alt_diff;
    float distance;
    int total_objects = 0;
    int alarm_level = ALARM_LEVEL_NONE;
    time_t this_moment = now();
    IPAddress broadcastIP = WiFi_get_broadcast();

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
          distance = gnss.distanceBetween(ThisAircraft.latitude, ThisAircraft.longitude, Container[i].latitude, Container[i].longitude);

          if (distance < EXPORT_DISTANCE_FAR) {

            if (distance < EXPORT_DISTANCE_CLOSE) {
              alarm_level = ALARM_LEVEL_URGENT;
            } else if (distance < EXPORT_DISTANCE_NEAR) {
              alarm_level = ALARM_LEVEL_IMPORTANT;
            } else {
              alarm_level = ALARM_LEVEL_LOW;
            }

            bearing = gnss.courseTo(ThisAircraft.latitude, ThisAircraft.longitude, Container[i].latitude, Container[i].longitude);
            alt_diff = (int) (Container[i].altitude - ThisAircraft.altitude);
            snprintf(NMEABuffer, sizeof(NMEABuffer), "$PFLAU,%d,%d,%d,%d,%d,%d,%d,%d,%u*",
                    total_objects, TX_STATUS_ON, GNSS_STATUS_3D_MOVING, POWER_STATUS_GOOD, alarm_level,
                    (bearing < 180 ? bearing : bearing - 360), ALARM_TYPE_AIRCRAFT, alt_diff, (int) distance );
    
            //calculate the checksum
            for (unsigned int n = 1; n < strlen(NMEABuffer) - 1; n++) {
              cs ^= NMEABuffer[n]; 
            }
    
            csum_ptr = NMEABuffer + strlen(NMEABuffer);
            snprintf(csum_ptr, sizeof(NMEABuffer) - strlen(NMEABuffer), "%02X", cs);
    
            Serial.println(NMEABuffer);

            Uni_Udp.beginPacket(broadcastIP, NMEA_DST_PORT);
            Uni_Udp.write(NMEABuffer, strlen(NMEABuffer));
            Uni_Udp.write('\r'); Uni_Udp.write('\n');
            Uni_Udp.endPacket();
    
            snprintf(NMEABuffer, sizeof(NMEABuffer), "$PFLAA,%d,%d,%d,%d,%d,%X,,,,,%d*",
                    alarm_level,
                    (int) (distance * cos(dtor(bearing))), (int) (distance * sin(dtor(bearing))), alt_diff,
                    ADDR_TYPE_FLARM, Container[i].addr, AIRCRAFT_TYPE_GLIDER );
    
            cs = 0; //clear any old checksum
            for (unsigned int n = 1; n < strlen(NMEABuffer) - 1; n++) {
              cs ^= NMEABuffer[n]; //calculates the checksum
            }
    
            csum_ptr = NMEABuffer + strlen(NMEABuffer);
            snprintf(csum_ptr, sizeof(NMEABuffer) - strlen(NMEABuffer), "%02X", cs);
    
            Serial.println(NMEABuffer);

            Uni_Udp.beginPacket(broadcastIP, NMEA_DST_PORT);
            Uni_Udp.write(NMEABuffer, strlen(NMEABuffer));
            Uni_Udp.write('\r'); Uni_Udp.write('\n');
            Uni_Udp.endPacket();

          }
        }
      }
    }
#if 0
  uint8_t i;
  //check if there are any new clients
  if (GNSSserver.hasClient()) {
    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      //find free/disconnected spot
      if (!serverClients[i] || !serverClients[i].connected()) {
        if (serverClients[i]) serverClients[i].stop();
        serverClients[i] = GNSSserver.available();
        Serial.print("New client: "); Serial.print(i);
        //serverClients[i].setNoDelay(true);
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient serverClient = GNSSserver.available();
    serverClient.stop();
  }
#endif

}

