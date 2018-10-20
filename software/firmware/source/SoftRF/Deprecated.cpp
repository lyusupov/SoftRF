/*
 * Deprecated.cpp
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
    dtostrf(fo.longitude, 8, 4, str_lon);

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
  dtostrf_workaround(take_minutes(fo.longitude), 5, 2, str_lon);

  //Serial.print(fo.latitude); Serial.print(" "); Serial.println(fo.longitude); 

  snprintf(UDPpacketBuffer, sizeof(UDPpacketBuffer), \
    "FLR%X>APRS,qAS,%s:/%02d%02d%02dh%02d%s%s/%03d%s%s/A=%05u TS:%d RAW:%s", \
    fo.addr, STATION_ID, tm.Hour, tm.Minute, tm.Second, \
    abs(take_degrees(fo.latitude)), str_lat, (fo.latitude < 0 ? "S" : "N"), \
    abs(take_degrees(fo.longitude)), str_lon, (fo.longitude < 0 ? "W" : "E"), \
    fo.altitude, fo.timestamp, fo.raw.c_str() );

#ifdef SERIAL_VERBOSE
  Serial.println(UDPpacketBuffer);
#endif

  client.println(UDPpacketBuffer);
}
#endif

#if 0
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
#endif

#if 0
void *WiFi_relay_from_android()
{
  int noBytes = Uni_Udp.parsePacket();
  if ( noBytes ) {
#if 0
    Serial.print(millis() / 1000);
    Serial.print(":Packet of ");
    Serial.print(noBytes);
    Serial.print(" received from ");
    Serial.print(Uni_Udp.remoteIP());
    Serial.print(":");
    Serial.println(Udp.remotePort());
#endif
    // We've received a packet, read the data from it
    Uni_Udp.read(UDPpacketBuffer,noBytes); // read the packet into the buffer
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
#endif


/* bridge_loop */
#if 0
  void *answer = WiFi_relay_from_android();
  if ((answer != NULL) && (settings->txpower != RF_TX_POWER_OFF) )
  {
    memcpy(TxBuffer, (unsigned char*) answer, PKT_SIZE);

    // Make data
    char *data = (char *) TxBuffer;

    // Set address of device to send to
    byte addr[] = TXADDR;
    nRF905_setTXAddress(addr);

    // Set payload data
    nRF905_setData(data, NRF905_PAYLOAD_SIZE );

    // Send payload (send fails if other transmissions are going on, keep trying until success)
    while (!nRF905_send()) {
      yield();
    } ;
    if (settings->nmea_p) {
      StdOut.print(F("$PSRFO,")); StdOut.print(now()); StdOut.print(F(",")); StdOut.println(Bin2Hex((byte *) data));
    }

    tx_packets_counter++;
    TxTimeMarker = millis();
  }
#endif


#if 0
unsigned int pos_ndx = 0;
unsigned long TxPosUpdMarker = 0;

void tx_test_loop()
{
  bool success = false;
#if DEBUG_TIMING
  unsigned long tx_start_ms, tx_end_ms, rx_start_ms, rx_end_ms;
#endif
  ThisAircraft.timestamp = now();

  if (TxPosUpdMarker == 0 || (millis() - TxPosUpdMarker) > 4000 ) {
    ThisAircraft.latitude =  pgm_read_float( &tx_test_positions[pos_ndx][0]);
    ThisAircraft.longitude =  pgm_read_float( &tx_test_positions[pos_ndx][1]);
    pos_ndx = (pos_ndx + 1) % TX_TEST_NUM_POSITIONS;
    TxPosUpdMarker = millis();
  }
  ThisAircraft.altitude = TEST_ALTITUDE;
  ThisAircraft.course = TEST_COURSE;
  ThisAircraft.speed = TEST_SPEED;
#if DEBUG_TIMING
  tx_start_ms = millis();
#endif
  RF_Transmit(RF_Encode());
#if DEBUG_TIMING
  tx_end_ms = millis();
  rx_start_ms = millis();
#endif
  success = RF_Receive();
#if DEBUG_TIMING
  rx_end_ms = millis();

  Serial.print(F("TX start: "));
  Serial.print(tx_start_ms);
  Serial.print(F(" TX stop: "));
  Serial.print(tx_end_ms);
  Serial.print(F(" RX start: "));
  Serial.print(rx_start_ms);
  Serial.print(F(" RX stop: "));
  Serial.println(rx_end_ms);
#endif

  if(success)
  {
    fo.raw = Bin2Hex(RxBuffer);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,")); StdOut.print(now()); StdOut.print(F(",")); StdOut.println(fo.raw);
    }
  }

  if (isTimeToExport()) {
    ExportTimeMarker = millis();
  }
}

const float rx_test_positions[][2] PROGMEM = { { 56.0092, 38.3710 } };

void rx_test_loop()
{
  bool success = false;

  ThisAircraft.timestamp = now();

  ThisAircraft.latitude = pgm_read_float( &rx_test_positions[0][0]);;
  ThisAircraft.longitude = pgm_read_float( &rx_test_positions[0][1]);
  ThisAircraft.altitude = TEST_ALTITUDE;
  ThisAircraft.course = TEST_COURSE;
  ThisAircraft.speed = TEST_SPEED;

  //RF_Transmit(RF_Encode());

  success = RF_Receive();

  if (success) ParseData();

  if (isTimeToDisplay()) {
    LED_DisplayTraffic();
    LEDTimeMarker = millis();
  }

  if (isTimeToExport()) {
    NMEA_Position();
    NMEA_Export();
    GDL90_Export();
    D1090_Export();
    ExportTimeMarker = millis();
  }

  ClearExpired();
}
#endif

#if 0
#define take_degrees(x) ( (int) x )
#define take_minutes(x) ( fabs(x - (float) take_degrees(x)) * 60.00)

char * dtostrf_workaround(double number, signed char width, unsigned char prec, char *s) {
  char * rval = dtostrf(number, width, prec, s);
  if (number < 10.0) {
    s[0] = '0';
  }
  return rval;
}
#endif

#if 0

void Misc_info()
{
  Serial.println("\r\n");
  Serial.print(F("Chip ID: 0x"));
  Serial.println(SoC->getChipId(), HEX);

  uint32_t realSize = SoC->getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  Serial.printf("Flash real id:   %08X\n", SoC->getFlashChipId());
  Serial.printf("Flash real size: %u\n\n", realSize);

  Serial.printf("Flash ide  size: %u\n", ideSize);
  Serial.printf("Flash ide speed: %u\n", ESP.getFlashChipSpeed());
  Serial.printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));

  if (ideSize != realSize) {
    Serial.println(F("Flash Chip configuration wrong!\n"));
  } else {
    Serial.println(F("Flash Chip configuration ok.\n"));
  }
}

#endif
