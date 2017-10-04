/*
 * Library.cpp
 * Copyright (C) 2016-2017 Linar Yusupov
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

//#include <TimeLib.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

#include "GNSSHelper.h"
#include "RFHelper.h"
#include "WiFiHelper.h"

#include "SoftRF.h"

#if LOGGER_IS_ENABLED
#include "LogHelper.h"
#endif /* LOGGER_IS_ENABLED */

extern WiFiUDP Udp;

#define TypeADSB  1

char UDPpacketBuffer[512]; //buffer to hold incoming and outgoing packets

void WiFi_forward_to_argus(void);
void WiFi_forward_to_xcsoar(void);
void WiFi_forward_to_cloud(void);
double dtor(double fdegrees);
void *WiFi_relay_from_android(void);

ufo_t fo, Container[MAX_TRACKING_OBJECTS], EmptyFO;
extern ufo_t ThisAircraft;

#if DEBUG
String TxDataTemplate = "0282dd204901f981798a85b69764bdf99ed77fd3c2300000";
#endif

char NMEABuffer[128]; //buffer to NMEA data


bool Import()
{
  void *answer = WiFi_relay_from_android();
  if (answer != NULL)
  {
    memcpy(RxBuffer, (unsigned char*) answer, PKT_SIZE);
    return true;
  } else {
    return false;
  }
}

#define EXPORT_DISTANCE_CLOSE  500
#define EXPORT_DISTANCE_NEAR   1500
#define EXPORT_DISTANCE_FAR    10000

void Export()
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
        Serial.println(fo.distance);
        Serial.println(fo.vs);
        Serial.println(fo.type);
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
            alt_diff = Container[i].altitude - ThisAircraft.altitude;
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

void ParseData()
{

#if DEBUG
    Hex2Bin(TxDataTemplate, RxBuffer);
#endif

    fo.raw = Bin2Hex(RxBuffer);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,")); StdOut.print(now()); StdOut.print(F(",")); StdOut.println(fo.raw);
    }

    if (legacy_decode((legacy_packet *) RxBuffer, &ThisAircraft, &fo)) {            

      for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
        int max_dist_ndx = 0;
        
        if (Container[i].addr == fo.addr) {
          Container[i] = fo;   
           break;  
        } else {
          if (now() - Container[i].timestamp > 5000) {
            Container[i] = fo;
            break;  
          }
#if 0
          if  (Container[i].distance > Container[max_dist_ndx].distance)  {
            max_dist_ndx = i;  
          }
          if ((i == (MAX_TRACKING_OBJECTS-1)) && (Container[max_dist_ndx].distance > fo.distance) ) {
            Container[max_dist_ndx] = fo; 
          }
#endif
        }
      }
    }
}

void Misc_info()
{
  Serial.println("\r\n");
  Serial.print("Chip ID: 0x");
  Serial.println(ESP.getChipId(), HEX);

  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  Serial.printf("Flash real id:   %08X\n", ESP.getFlashChipId());
  Serial.printf("Flash real size: %u\n\n", realSize);

  Serial.printf("Flash ide  size: %u\n", ideSize);
  Serial.printf("Flash ide speed: %u\n", ESP.getFlashChipSpeed());
  Serial.printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));

  if (ideSize != realSize) {
    Serial.println("Flash Chip configuration wrong!\n");
  } else {
    Serial.println("Flash Chip configuration ok.\n");
  }
}

void ClearExpired()
{
  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr && (now() - Container[i].timestamp) > LED_EXPIRATION_TIME) {
      Container[i] = EmptyFO;
    }
  }
}

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

void WiFi_forward_to_xcsoar()
{
    float dist = gnss.distanceBetween(ThisAircraft.latitude, ThisAircraft.longitude, fo.latitude, fo.longitude);
    int bearing = CalcBearing(fo.latitude, fo.longitude, ThisAircraft.latitude, ThisAircraft.longitude);
    int alt_diff = fo.altitude - ThisAircraft.altitude;
    char *csum_ptr;
    unsigned char cs = 0; //clear any old checksum

    Udp.beginPacket(XCSOAR_HOSTNAME, XCSOAR_PORT);
    snprintf(UDPpacketBuffer, sizeof(UDPpacketBuffer), "$PFLAU,1,1,2,1,1,%d,2,%d,%u*", \
      bearing < 180 ? bearing : bearing - 360, alt_diff, (int) dist );

    //calculate the checksum
    for (unsigned int n = 1; n < strlen(UDPpacketBuffer) - 1; n++) {
      cs ^= UDPpacketBuffer[n]; //calculates the checksum
    }

    csum_ptr = UDPpacketBuffer + strlen(UDPpacketBuffer);
    snprintf(csum_ptr, sizeof(UDPpacketBuffer) - strlen(UDPpacketBuffer), "%02X\n", cs);

#ifdef SERIAL_VERBOSE
    Serial.println(UDPpacketBuffer);
#endif

    Udp.write(UDPpacketBuffer, strlen(UDPpacketBuffer));
    Udp.endPacket();

    Udp.beginPacket(XCSOAR_HOSTNAME, XCSOAR_PORT);
    snprintf(UDPpacketBuffer, sizeof(UDPpacketBuffer), "$PFLAA,1,%d,%d,%d,2,%X,,,,,1*",   \
      (int) (dist * cos(dtor(bearing))), (int) (dist * sin(dtor(bearing))), \
      alt_diff, fo.addr );

    cs = 0; //clear any old checksum
    for (unsigned int n = 1; n < strlen(UDPpacketBuffer) - 1; n++) {
      cs ^= UDPpacketBuffer[n]; //calculates the checksum
    }

    csum_ptr = UDPpacketBuffer + strlen(UDPpacketBuffer);
    snprintf(csum_ptr, sizeof(UDPpacketBuffer) - strlen(UDPpacketBuffer), "%02X\n", cs);

#ifdef SERIAL_VERBOSE
    Serial.println(UDPpacketBuffer);
#endif

    Udp.write(UDPpacketBuffer, strlen(UDPpacketBuffer));
    Udp.endPacket();
}

#define take_degrees(x) ( (int) x )
#define take_minutes(x) ( fabs(x - (float) take_degrees(x)) * 60.00)

char * dtostrf_workaround(double number, signed char width, unsigned char prec, char *s) {
  char * rval = dtostrf(number, width, prec, s);
  if (number < 10.0) {
    s[0] = '0';
  }
  return rval;
}

void *WiFi_relay_from_android()
{
  int noBytes = Udp.parsePacket();
  if ( noBytes ) {
#if 0
    Serial.print(millis() / 1000);
    Serial.print(":Packet of ");
    Serial.print(noBytes);
    Serial.print(" received from ");
    Serial.print(Udp.remoteIP());
    Serial.print(":");
    Serial.println(Udp.remotePort());
#endif
    // We've received a packet, read the data from it
    Udp.read(UDPpacketBuffer,noBytes); // read the packet into the buffer
#if 0
    // display the packet contents in HEX
    for (int i=1;i<=noBytes;i++){
      Serial.print(UDPpacketBuffer[i-1],HEX);
      if (i % 32 == 0){
        Serial.println();
      }
      else Serial.print(' ');
    } // end for
    Serial.println();
#endif
    return UDPpacketBuffer;
  } else {
    return NULL;
  }  // end if
}

char misc_hexdata[2 * PKT_SIZE + 1] ;
void WiFi_relay_to_android()
{
    Udp.beginPacket(WiFi_get_broadcast(), RELAY_PORT);
    fo.raw.toCharArray(misc_hexdata, sizeof(misc_hexdata));
    snprintf(UDPpacketBuffer, sizeof(UDPpacketBuffer), "%s\n", misc_hexdata );

#if 0
    Serial.println(UDPpacketBuffer);
#endif

    Udp.write(UDPpacketBuffer, strlen(UDPpacketBuffer));
    Udp.endPacket();
}