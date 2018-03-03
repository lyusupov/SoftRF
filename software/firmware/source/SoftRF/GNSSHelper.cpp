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

#include <Arduino.h>
#include <TimeLib.h>

#include "GNSSHelper.h"
#include "EEPROMHelper.h"
#include "NMEAHelper.h"
#include "SoCHelper.h"
#include "WiFiHelper.h"

#include "SoftRF.h"

#if !defined(DO_GNSS_DEBUG)
#define GNSS_DEBUG_PRINT
#define GNSS_DEBUG_PRINTLN
#else
#define GNSS_DEBUG_PRINT    Serial.print
#define GNSS_DEBUG_PRINTLN  Serial.println
#endif

extern char UDPpacketBuffer[256];

unsigned long GNSSTimeSyncMarker = 0;

boolean gnss_set_sucess = false ;
TinyGPSPlus gnss;  // Create an Instance of the TinyGPS++ object called gnss

uint8_t GNSSbuf[160]; // 2 lines of 80 characters each
int GNSS_cnt = 0;

uint8_t UBXbuf[32];

 /* CFG-MSG */
const uint8_t setGLL[] = {0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
const uint8_t setGSV[] = {0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
const uint8_t setVTG[] = {0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
#if !defined(AIRCONNECT_IS_ACTIVE)
const uint8_t setGSA[] = {0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
#endif
 /* CFG-PRT */
uint8_t setBR[] = {0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96,
                   0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t makeUBXCFG(uint8_t cl, uint8_t id, uint8_t msglen, const uint8_t *msg)
{
  if (msglen > (sizeof(UBXbuf) - 8) ) {
    msglen = sizeof(UBXbuf) - 8;
  }

  // Construct the UBX packet
  UBXbuf[0] = 0xB5;   // header
  UBXbuf[1] = 0x62;   // header
  UBXbuf[2] = cl;  // class
  UBXbuf[3] = id;     // id
  UBXbuf[4] = msglen; // length
  UBXbuf[5] = 0x00;

  UBXbuf[6+msglen] = 0x00; // CK_A
  UBXbuf[7+msglen] = 0x00; // CK_B

  for (int i = 2; i < 6; i++) {
    UBXbuf[6+msglen] += UBXbuf[i];
    UBXbuf[7+msglen] += UBXbuf[6+msglen];
  }

  for (int i = 0; i < msglen; i++) {
    UBXbuf[6+i] = msg[i];
    UBXbuf[6+msglen] += UBXbuf[6+i];
    UBXbuf[7+msglen] += UBXbuf[6+msglen];
  }
  return (msglen + 8);
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(const uint8_t *MSG, uint8_t len) {
  for (int i = 0; i < len; i++) {
    swSer.write( MSG[i]);
    GNSS_DEBUG_PRINT(MSG[i], HEX);
  }
//  swSer.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t cl, uint8_t id) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[2] = {cl, id};
  unsigned long startTime = millis();
  GNSS_DEBUG_PRINT(F(" * Reading ACK response: "));

  // Construct the expected ACK packet
  makeUBXCFG(0x05, 0x01, 2, ackPacket);

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      GNSS_DEBUG_PRINTLN(F(" (SUCCESS!)"));
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      GNSS_DEBUG_PRINTLN(F(" (FAILED!)"));
      return false;
    }

    // Make sure data is available to read
    if (swSer.available()) {
      b = swSer.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == UBXbuf[ackByteID]) {
        ackByteID++;
        GNSS_DEBUG_PRINT(b, HEX);
      }
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
    }
    yield();
  }
}

static void setup_UBX()
{
  uint8_t msglen;
  unsigned int baudrate = 38400;

  setBR[ 8] = (baudrate      ) & 0xFF;
  setBR[ 9] = (baudrate >>  8) & 0xFF;
  setBR[10] = (baudrate >> 16) & 0xFF;

  SoC->swSer_begin(9600);

  Serial.print(F("Switching baud rate onto "));
  Serial.println(baudrate);

  msglen = makeUBXCFG(0x06, 0x00, sizeof(setBR), setBR);
  sendUBX(UBXbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x00);

  if (!gnss_set_sucess) {
    Serial.print(F("WARNING: Unable to set baud rate onto "));
    Serial.println(baudrate); 
  }
  swSer.flush();
  SoC->swSer_begin(baudrate);

  GNSS_DEBUG_PRINTLN(F("Switching off NMEA GLL: "));

  msglen = makeUBXCFG(0x06, 0x01, sizeof(setGLL), setGLL);
  sendUBX(UBXbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x01);

  if (!gnss_set_sucess) {
    Serial.println(F("WARNING: Unable to disable NMEA GLL."));
  }

  GNSS_DEBUG_PRINTLN(F("Switching off NMEA GSV: "));

  msglen = makeUBXCFG(0x06, 0x01, sizeof(setGSV), setGSV);
  sendUBX(UBXbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x01);

  if (!gnss_set_sucess) {
    Serial.println(F("WARNING: Unable to disable NMEA GSV."));
  }

  GNSS_DEBUG_PRINTLN(F("Switching off NMEA VTG: "));

  msglen = makeUBXCFG(0x06, 0x01, sizeof(setVTG), setVTG);
  sendUBX(UBXbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x01);

  if (!gnss_set_sucess) {
    Serial.println(F("WARNING: Unable to disable NMEA VTG."));
  }

#if !defined(AIRCONNECT_IS_ACTIVE)

  GNSS_DEBUG_PRINTLN(F("Switching off NMEA GSA: "));

  msglen = makeUBXCFG(0x06, 0x01, sizeof(setGSA), setGSA);
  sendUBX(UBXbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x01);

  if (!gnss_set_sucess) {
    Serial.println(F("WARNING: Unable to disable NMEA GSA."));
  }

#endif
}

static void setup_NMEA()
{
  SoC->swSer_begin(9600);

  //swSer.write("$PUBX,41,1,0007,0003,9600,0*10\r\n");
  swSer.write("$PUBX,41,1,0007,0003,38400,0*20\r\n");

  swSer.flush();
  SoC->swSer_begin(38400);

  // Turning off some GPS NMEA strings on the uBlox modules
  swSer.write("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  swSer.write("$PUBX,40,GSV,0,0,0,0*59\r\n");
  swSer.write("$PUBX,40,VTG,0,0,0,0*5E\r\n");
#if !defined(AIRCONNECT_IS_ACTIVE)
  swSer.write("$PUBX,40,GSA,0,0,0,0*4E\r\n");
#endif
}

void GNSS_setup() {
  //setup_UBX();
  //setup_NMEA();
  SoC->swSer_begin(9600);
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
  int ndx;

  /*
   * Check SW, HW and BT UARTs for data
   * WARNING! Make use only one input source at a time.
   */
  while (swSer.available() > 0   ||
         Serial.available() > 0  ||
         SoC->BltnBT_available() > 0) {

    if (swSer.available() > 0) {
      GNSSbuf[GNSS_cnt] = swSer.read();
    } else if (Serial.available()) {
      GNSSbuf[GNSS_cnt] = Serial.read();
    } else {
      GNSSbuf[GNSS_cnt] = SoC->BltnBT_read();
    }

    isValidSentence = gnss.encode(GNSSbuf[GNSS_cnt]);
    if (settings->nmea_g && GNSSbuf[GNSS_cnt] == '\r' && isValidSentence) {
      for (ndx = GNSS_cnt - 4; ndx >= 0; ndx--) { // skip CS and *
        if ((GNSSbuf[ndx] == '$') && (GNSSbuf[ndx+1] == 'G')) {

          Serial.write((uint8_t *) &GNSSbuf[ndx], GNSS_cnt - ndx + 1);
          Serial.write('\n');

          SoC->BltnBT_write((uint8_t *) &GNSSbuf[ndx], GNSS_cnt - ndx + 1);
          SoC->BltnBT_write((uint8_t *) "\n", 1);

          if (settings->nmea_u) {
            size_t num = GNSS_cnt - ndx + 1;
            // ASSERT(sizeof(UDPpacketBuffer) > sizeof(GNSSbuf))
            memcpy(UDPpacketBuffer, &GNSSbuf[ndx], num);
            UDPpacketBuffer[num] = '\n';
            SoC->WiFi_transmit_UDP(NMEA_DST_PORT, (byte *)UDPpacketBuffer, num + 1);
          }

#if defined(AIRCONNECT_IS_ACTIVE)
          if (AirConnectClient && AirConnectClient.connected()){
            AirConnectClient.write(&GNSSbuf[ndx], GNSS_cnt - ndx + 1);
            AirConnectClient.write('\n');
          }
#endif
          break;
        }
      }
    }
    if (GNSSbuf[GNSS_cnt] == '\n' || GNSS_cnt == sizeof(GNSSbuf)-1) {
      GNSS_cnt = 0;
    } else {
      GNSS_cnt++;
      yield();
    }
  }
}
