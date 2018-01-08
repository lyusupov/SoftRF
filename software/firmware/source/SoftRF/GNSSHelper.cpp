/*
 * GNSSHelper.cpp
 * Copyright (C) 2016-2018 Linar Yusupov
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
#include <SoftwareSerial.h>

#include "GNSSHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"
#include "NMEAHelper.h"

#include "SoftRF.h"

unsigned long GNSSTimeSyncMarker = 0;
extern SoftwareSerial swSer;

byte gnss_set_sucess = 0 ;
TinyGPSPlus gnss;  // Create an Instance of the TinyGPS++ object called gnss

uint8_t GNSSbuf[128];
int GNSS_cnt = 0;

#if 0
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for (int i = 0; i < len; i++) {
    swSer.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  swSer.println();
}


// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B

  // Calculate the checksums
  for (uint8_t i = 2; i < 8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      Serial.println(" (FAILED!)");
      return false;
    }

    // Make sure data is available to read
    if (swSer.available()) {
      b = swSer.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
        Serial.print(b, HEX);
      }
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }

    }
  }
}
#endif


void GNSS_setup() {

  swSer.begin(9600);

#if 0
  gnss_set_sucess = 0;
  Serial.println("Switching off NMEA GLL: ");
  uint8_t setGLL[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B
  };
  while (!gnss_set_sucess)
  {
    sendUBX(setGLL, sizeof(setGLL) / sizeof(uint8_t));
    gnss_set_sucess = getUBX_ACK(setGLL);
  }
  gnss_set_sucess = 0;
  Serial.println("Switching off NMEA GSA: ");
  uint8_t setGSA[] = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32
  };
  while (!gnss_set_sucess)
  {
    sendUBX(setGSA, sizeof(setGSA) / sizeof(uint8_t));
    gnss_set_sucess = getUBX_ACK(setGSA);
  }
  gnss_set_sucess = 0;
#endif
}

void GNSSTimeSync()
{
  if (GNSSTimeSyncMarker == 0 && gnss.time.isValid() && gnss.time.isUpdated()) {
      setTime(gnss.time.hour(), gnss.time.minute(), gnss.time.second(), gnss.date.day(), gnss.date.month(), gnss.date.year());
      GNSSTimeSyncMarker = millis();
  } else {

    if ((millis() - GNSSTimeSyncMarker > 60000) /* 1m */ && gnss.time.isValid() &&
        gnss.time.isUpdated() && (gnss.time.age() <= 1000) /* 1s */ ) {
  #if 0
      Serial.print("Valid: ");
      Serial.println(gnss.time.isValid());
      Serial.print("isUpdated: ");
      Serial.println(gnss.time.isUpdated());
      Serial.print("age: ");
      Serial.println(gnss.time.age());
  #endif
      setTime(gnss.time.hour(), gnss.time.minute(), gnss.time.second(), gnss.date.day(), gnss.date.month(), gnss.date.year());
      GNSSTimeSyncMarker = millis();
    }  
  }
}

void PickGNSSFix()
{
  bool isValidSentence = false;
  IPAddress broadcastIP = WiFi_get_broadcast();

  //check UART for data
  while (swSer.available() > 0) {
    GNSSbuf[GNSS_cnt] = swSer.read();
    isValidSentence = gnss.encode(GNSSbuf[GNSS_cnt]);
    if (settings->nmea_g && GNSSbuf[GNSS_cnt] == '\r' && isValidSentence) {

      Serial.write((uint8_t *) &GNSSbuf[0], GNSS_cnt+1);
      Serial.write('\n');

      if (settings->nmea_u) {
        Uni_Udp.beginPacket(broadcastIP, NMEA_DST_PORT);
        Uni_Udp.write(&GNSSbuf[0], GNSS_cnt + 1);
        Uni_Udp.write('\n');
        Uni_Udp.endPacket();
      }

#if defined(AIRCONNECT_IS_ACTIVE)
    if (AirConnectClient && AirConnectClient.connected()){
      AirConnectClient.write(&GNSSbuf[0], GNSS_cnt + 1);
      AirConnectClient.write('\n');
    }
#endif

    }
    if (GNSSbuf[GNSS_cnt] == '\n' || GNSS_cnt == sizeof(GNSSbuf)-1) {
      GNSS_cnt = 0;
    } else {
      GNSS_cnt++;
      delay(0);
    }
  }
}
