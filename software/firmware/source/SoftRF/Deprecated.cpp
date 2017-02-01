/*
 * Deprecated.cpp
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

#if 0
//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 1
WiFiServer GNSSserver(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];
#endif


#if 0
void WiFi_forward_to_argus()
{
    char str_lat[16];
    char str_lon[16];

    dtostrf(fo.latitude, 8, 4, str_lat);
    dtostrf(fo.longtitude, 8, 4, str_lon);

    Udp.beginPacket(ARGUS_HOSTNAME, ARGUS_PORT);

    snprintf(UDPpacketBuffer, sizeof(UDPpacketBuffer), "%u %X %s %s %d %s %s %u", \
      fo.timestamp * 1000, fo.addr, str_lat, str_lon, fo.altitude, "0" , "0" , TypeADSB );

#ifdef SERIAL_VERBOSE
    Serial.println(UDPpacketBuffer);
#endif

    Udp.write(UDPpacketBuffer, strlen(UDPpacketBuffer));
    Udp.endPacket();
}
#endif

#if 0
// FLRDDE626>APRS,qAS,EGHL:/074548h5111.32N/00102.04W'086/007/A=000607 id0ADDE626 -019fpm +0.0rot 5.5dB 3e -4.3kHz
void WiFi_forward_to_cloud() {
  tmElements_t tm;
  char str_lat[8];
  char str_lon[8];

  breakTime(fo.timestamp, tm);

  dtostrf_workaround(take_minutes(fo.latitude), 5, 2, str_lat);
  dtostrf_workaround(take_minutes(fo.longtitude), 5, 2, str_lon);

  //Serial.print(fo.latitude); Serial.print(" "); Serial.println(fo.longtitude); 

  snprintf(UDPpacketBuffer, sizeof(UDPpacketBuffer), \
    "FLR%X>APRS,qAS,%s:/%02d%02d%02dh%02d%s%s/%03d%s%s/A=%05u TS:%d RAW:%s", \
    fo.addr, STATION_ID, tm.Hour, tm.Minute, tm.Second, \
    abs(take_degrees(fo.latitude)), str_lat, (fo.latitude < 0 ? "S" : "N"), \
    abs(take_degrees(fo.longtitude)), str_lon, (fo.longtitude < 0 ? "W" : "E"), \
    fo.altitude, fo.timestamp, fo.raw.c_str() );

#ifdef SERIAL_VERBOSE
  Serial.println(UDPpacketBuffer);
#endif

  client.println(UDPpacketBuffer);
}
#endif
