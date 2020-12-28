/*
 * NMEAHelper.cpp
 * Copyright (C) 2019-2021 Linar Yusupov
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

#include <TinyGPS++.h>
#include <TimeLib.h>

#include "SoCHelper.h"
#include "NMEAHelper.h"
#include "TrafficHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"

TinyGPSPlus nmea;

TinyGPSCustom T_AlarmLevel      (nmea, "PFLAA", 1);
TinyGPSCustom T_RelativeNorth   (nmea, "PFLAA", 2);
TinyGPSCustom T_RelativeEast    (nmea, "PFLAA", 3);
TinyGPSCustom T_RelativeVertical(nmea, "PFLAA", 4);
TinyGPSCustom T_IDType          (nmea, "PFLAA", 5);
TinyGPSCustom T_ID              (nmea, "PFLAA", 6);
TinyGPSCustom T_Track           (nmea, "PFLAA", 7);
TinyGPSCustom T_TurnRate        (nmea, "PFLAA", 8);
TinyGPSCustom T_GroundSpeed     (nmea, "PFLAA", 9);
TinyGPSCustom T_ClimbRate       (nmea, "PFLAA", 10);
TinyGPSCustom T_AcftType        (nmea, "PFLAA", 11);

TinyGPSCustom S_RX              (nmea, "PFLAU", 1);
TinyGPSCustom S_TX              (nmea, "PFLAU", 2);
TinyGPSCustom S_GPS             (nmea, "PFLAU", 3);
TinyGPSCustom S_Power           (nmea, "PFLAU", 4);
TinyGPSCustom S_AlarmLevel      (nmea, "PFLAU", 5);
TinyGPSCustom S_RelativeBearing (nmea, "PFLAU", 6);
TinyGPSCustom S_AlarmType       (nmea, "PFLAU", 7);
TinyGPSCustom S_RelativeVertical(nmea, "PFLAU", 8);
TinyGPSCustom S_RelativeDistance(nmea, "PFLAU", 9);
TinyGPSCustom S_ID              (nmea, "PFLAU", 10);
/* SoftRF/S7xG PFLAU NMEA sentence extension(s) */
TinyGPSCustom S_Addr            (nmea, "PFLAU", 11);
TinyGPSCustom S_Protocol        (nmea, "PFLAU", 12);
TinyGPSCustom S_RxCnt           (nmea, "PFLAU", 13);
TinyGPSCustom S_TxCnt           (nmea, "PFLAU", 14);

status_t NMEA_Status;

uint32_t tx_packets_counter = 0;
uint32_t rx_packets_counter = 0;

static unsigned long NMEA_TimeMarker = 0;

static bool RTC_sync = false;

#if defined(NMEA_TCP_SERVICE)
WiFiServer NmeaTCPServer(NMEA_TCP_PORT);
NmeaTCP_t NmeaTCP[MAX_NMEATCP_CLIENTS];
#endif

char NMEABuffer[NMEA_BUFFER_SIZE]; // buffer for NMEA data
int NMEA_cnt = 0;

#define isTimeToPGRMZ() (millis() - PGRMZ_TimeMarker > 1000)
unsigned long PGRMZ_TimeMarker = 0;
char PGRMZBuffer[32];

void NMEA_add_checksum(char *buf, size_t limit)
{
  size_t sentence_size = strlen(buf);

  //calculate the checksum
  unsigned char cs = 0;
  for (unsigned int n = 1; n < sentence_size - 1; n++) {
    cs ^= buf[n];
  }

  char *csum_ptr = buf + sentence_size;
  snprintf_P(csum_ptr, limit, PSTR("%02X\r\n"), cs);
}

static void NMEA_Parse_Character(char c)
{
    int ndx;
    bool isValidSentence;

    if (c == -1) {
      /* retry */
      return;
    }

    if (isPrintable(c) || c == '\r' || c == '\n') {
      NMEABuffer[NMEA_cnt] = c;
    } else {
      /* ignore */
      return;
    }

    isValidSentence = nmea.encode(NMEABuffer[NMEA_cnt]);
    if (isValidSentence) {

      for (ndx = NMEA_cnt - 4; ndx >= 0; ndx--) { // skip CS and *
        if ( NMEABuffer[ndx]   == '$' &&
            (NMEABuffer[ndx+1] == 'G' || NMEABuffer[ndx+1] == 'P')) {

          size_t write_size = NMEA_cnt - ndx + 1;
          NMEA_Out((byte *) &NMEABuffer[ndx], write_size, true);
          break;
        }
      }

      if (nmea.location.isUpdated()) {
        ThisAircraft.latitude  = nmea.location.lat();
        ThisAircraft.longitude = nmea.location.lng();
      }
      if (nmea.altitude.isUpdated()) {
        ThisAircraft.altitude = nmea.altitude.meters();
      }
      if (nmea.course.isUpdated()) {
        ThisAircraft.Track = nmea.course.deg();
      }
      if (nmea.speed.isUpdated()) {
        ThisAircraft.GroundSpeed = nmea.speed.knots();
      }
      if (T_ID.isUpdated()) {
        fo = EmptyFO;

//        Serial.print(F(" ID=")); Serial.print(ID.value());

        fo.ID = strtol(T_ID.value(), NULL, 16);

#if 0
        Serial.print(F(" ID="));
        Serial.print((fo.ID >> 16) & 0xFF, HEX);
        Serial.print((fo.ID >>  8) & 0xFF, HEX);
        Serial.print((fo.ID      ) & 0xFF, HEX);
        Serial.println();
#endif

        if (T_AlarmLevel.isUpdated())
        {
          fo.AlarmLevel = atoi(T_AlarmLevel.value());
        }
        if (T_RelativeNorth.isUpdated())
        {
          fo.RelativeNorth = atoi(T_RelativeNorth.value());
        }
        if (T_RelativeEast.isUpdated())
        {
          fo.RelativeEast = atoi(T_RelativeEast.value());
        }
        if (T_RelativeVertical.isUpdated())
        {
          fo.RelativeVertical = atoi(T_RelativeVertical.value());
        }
        if (T_IDType.isUpdated())
        {
          fo.IDType = atoi(T_IDType.value());
        }
        if (T_Track.isUpdated())
        {
          fo.Track = atoi(T_Track.value());
        }
        if (T_TurnRate.isUpdated())
        {
          fo.TurnRate = atoi(T_TurnRate.value());
        }
        if (T_GroundSpeed.isUpdated())
        {
          fo.GroundSpeed = atoi(T_GroundSpeed.value());
        }
        if (T_ClimbRate.isUpdated())
        {
//          Serial.print(F(" ClimbRate=")); Serial.println(T_ClimbRate.value());
          /* TBD */
        }
        if (T_AcftType.isUpdated())
        {
          fo.AcftType = atoi(T_AcftType.value());
        }

        fo.timestamp = now();

        for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

          if (Container[i].ID == fo.ID) {
            Container[i] = fo;
            break;
          } else {
            if (now() - Container[i].timestamp > ENTRY_EXPIRATION_TIME) {
              Container[i] = fo;
              break;
            }
          }
        }

      } else if (S_RX.isUpdated()) {

        NMEA_Status.timestamp = now();
        NMEA_Status.RX = atoi(S_RX.value());

        if (S_TX.isUpdated())
        {
          NMEA_Status.TX = atoi(S_TX.value());
        }
        if (S_GPS.isUpdated())
        {
          NMEA_Status.GPS = atoi(S_GPS.value());
        }
        if (S_Power.isUpdated())
        {
          NMEA_Status.Power = atoi(S_Power.value());
        }
        if (S_AlarmLevel.isUpdated())
        {
          NMEA_Status.AlarmLevel = atoi(S_AlarmLevel.value());
        }
        if (S_RelativeBearing.isUpdated())
        {
          NMEA_Status.RelativeBearing = atoi(S_RelativeBearing.value());
        }
        if (S_AlarmType.isUpdated())
        {
          NMEA_Status.AlarmType = atoi(S_AlarmType.value());
        }
        if (S_RelativeVertical.isUpdated())
        {
          NMEA_Status.RelativeVertical = atoi(S_RelativeVertical.value());
        }
        if (S_RelativeDistance.isUpdated())
        {
          NMEA_Status.RelativeDistance = strtol(S_RelativeDistance.value(), NULL, 10);
        }
        if (S_ID.isUpdated())
        {
          NMEA_Status.ID = strtol(S_ID.value(), NULL, 16);
        }

        /* SoftRF/S7xG PFLAU NMEA sentence extension(s) */
        if (S_Addr.isUpdated())
        {
          ThisDevice.addr = strtol(S_Addr.value(), NULL, 16);
        }
        if (S_Protocol.isUpdated())
        {
          ThisDevice.protocol = atoi(S_Protocol.value());
        }
        if (S_RxCnt.isUpdated())
        {
          rx_packets_counter = strtol(S_RxCnt.value(), NULL, 10);
        }
        if (S_TxCnt.isUpdated())
        {
          tx_packets_counter = strtol(S_TxCnt.value(), NULL, 10);
        }
      }
    }

    if (NMEABuffer[NMEA_cnt] == '\n' || NMEA_cnt == sizeof(NMEABuffer)-1) {
      NMEA_cnt = 0;
    } else {
      NMEA_cnt++;
    }
}

void NMEA_setup()
{
  if (settings->m.protocol == PROTOCOL_NMEA) {
    switch (settings->m.connection)
    {
    case CON_SERIAL_MAIN:
    case CON_SERIAL_AUX:
      uint32_t SerialBaud;

      switch (settings->m.baudrate)
      {
      case B4800:
        SerialBaud = 4800;
        break;
      case B9600:
        SerialBaud = 9600;
        break;
      case B19200:
        SerialBaud = 19200;
        break;
      case B57600:
        SerialBaud = 57600;
        break;
      case B115200:
        SerialBaud = 115200;
        break;
      case B2000000:
        SerialBaud = 2000000;
        break;
      case B38400:
      default:
        SerialBaud = 38400;
        break;
      }

      SoC->swSer_begin(SerialBaud);
      break;
    case CON_BLUETOOTH:
      if (SoC->Bluetooth) {
        SoC->Bluetooth->setup();
      }
      break;
    case CON_NONE:
    case CON_WIFI_UDP:
    default:
      break;
    }

#if defined(NMEA_TCP_SERVICE)
    if (settings->s.nmea_out == NMEA_TCP) {
      NmeaTCPServer.begin();
      Serial.print(F("NMEA TCP server has started at port: "));
      Serial.println(NMEA_TCP_PORT);

      NmeaTCPServer.setNoDelay(true);
    }
#endif /* NMEA_TCP_SERVICE */

    NMEA_TimeMarker  = millis();
    PGRMZ_TimeMarker = millis();
  }
}

void NMEA_loop()
{
  char c;
  size_t size;

  switch (settings->m.connection)
  {
  case CON_SERIAL_MAIN:
    while (SerialInput.available() > 0) {
      c = SerialInput.read();
//      Serial.print(c);
      NMEA_Parse_Character(c);
      NMEA_TimeMarker = millis();
    }
    break;
  case CON_SERIAL_AUX:
    /* read data from Type-C USB port */
    while (Serial.available() > 0) {
      c = Serial.read();
//        Serial.print(c);
      NMEA_Parse_Character(c);
      NMEA_TimeMarker = millis();
    }
    break;
  case CON_WIFI_UDP:
    size = SoC->WiFi_Receive_UDP((uint8_t *) UDPpacketBuffer, sizeof(UDPpacketBuffer));
    if (size > 0) {
      for (size_t i=0; i < size; i++) {
        Serial.print(UDPpacketBuffer[i]);
        NMEA_Parse_Character(UDPpacketBuffer[i]);
      }
      NMEA_TimeMarker = millis();
    }
    break;
  case CON_BLUETOOTH:
    if (SoC->Bluetooth) {
      while (SoC->Bluetooth->available() > 0) {
        c = SoC->Bluetooth->read();
        Serial.print(c);
        NMEA_Parse_Character(c);
        NMEA_TimeMarker = millis();
      }
    }
    break;
  case CON_NONE:
  default:
    break;
  }

  if (settings->s.nmea_s && ThisDevice.pressure_altitude != 0.0 && isTimeToPGRMZ()) {

    int altitude = constrain(
            (int) (ThisDevice.pressure_altitude * _GPS_FEET_PER_METER),
            -1000, 60000);

    snprintf_P(PGRMZBuffer, sizeof(PGRMZBuffer), PSTR("$PGRMZ,%d,f,3*"),
            altitude ); /* feet , 3D fix */

    NMEA_add_checksum(PGRMZBuffer, sizeof(PGRMZBuffer) - strlen(PGRMZBuffer));

    NMEA_Out((byte *) PGRMZBuffer, strlen(PGRMZBuffer), false);

    PGRMZ_TimeMarker = millis();
  }

#if !defined(EXCLUDE_RTC)
  if (!RTC_sync) {
    if (rtc &&
        nmea.date.isValid()     &&
        nmea.time.isValid()     &&
        nmea.date.year() > 2018 &&
        nmea.date.year() < 2030 ) {
      rtc->setDateTime(nmea.date.year(),   nmea.date.month(),
                       nmea.date.day(),    nmea.time.hour(),
                       nmea.time.minute(), nmea.time.second());
      RTC_sync = true;
    }
  }
#endif /* EXCLUDE_RTC */

#if defined(NMEA_TCP_SERVICE)
  uint8_t i;

  if (settings->s.nmea_out == NMEA_TCP) {

    if (NmeaTCPServer.hasClient()) {
      for(i = 0; i < MAX_NMEATCP_CLIENTS; i++) {
        // find free/disconnected spot
        if (!NmeaTCP[i].client || !NmeaTCP[i].client.connected()) {
          if(NmeaTCP[i].client) {
            NmeaTCP[i].client.stop();
            NmeaTCP[i].connect_ts = 0;
          }
          NmeaTCP[i].client = NmeaTCPServer.available();
          NmeaTCP[i].connect_ts = now();
          NmeaTCP[i].ack = false;
          NmeaTCP[i].client.print(F("PASS?"));
          break;
        }
      }
      if (i >= MAX_NMEATCP_CLIENTS) {
        // no free/disconnected spot so reject
        NmeaTCPServer.available().stop();
      }
    }

    for (i = 0; i < MAX_NMEATCP_CLIENTS; i++) {
      if (NmeaTCP[i].client && NmeaTCP[i].client.connected() &&
         !NmeaTCP[i].ack && NmeaTCP[i].connect_ts > 0 &&
         (now() - NmeaTCP[i].connect_ts) >= NMEATCP_ACK_TIMEOUT) {

          /* Clean TCP input buffer from any pass codes sent by client */
          while (NmeaTCP[i].client.available()) {
            char c = NmeaTCP[i].client.read();
            yield();
          }
          /* send acknowledge */
          NmeaTCP[i].client.print(F("AOK"));
          NmeaTCP[i].ack = true;
      }
    }
  }
#endif
}

bool NMEA_Save_Settings()
{
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRFC,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d*"),
            PSRFC_VERSION, settings->s.mode, settings->s.rf_protocol,
            settings->s.band, settings->s.aircraft_type, settings->s.alarm,
            settings->s.txpower, BUZZER_OFF, LED_OFF,
            settings->s.nmea_g, settings->s.nmea_p, settings->s.nmea_l,
            settings->s.nmea_s, NMEA_UART, GDL90_OFF, D1090_OFF,
            settings->s.stealth, settings->s.no_track, settings->s.power_save );

    NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));

    SerialInput.write((byte *) NMEABuffer, strlen(NMEABuffer));
}

bool NMEA_isConnected()
{
  return (NMEA_TimeMarker > DATA_TIMEOUT &&
         (millis() - NMEA_TimeMarker) < DATA_TIMEOUT);
}

bool NMEA_hasGNSS()
{
  return (nmea.time.isValid() && nmea.time.age() < NMEA_EXP_TIME);
}

bool NMEA_hasFLARM()
{
  return (S_RX.isValid() && S_RX.age() < NMEA_EXP_TIME);
}

bool NMEA_has3DFix()
{
  return (S_GPS.isValid()                       &&
          S_GPS.age() < NMEA_EXP_TIME           &&
          NMEA_Status.GPS == GNSS_STATUS_3D_MOVING);
}

void NMEA_Out(byte *buf, size_t size, bool nl)
{
  switch(settings->s.nmea_out)
  {
  case NMEA_UART:
    {
      Serial.write(buf, size);
      if (nl)
        Serial.write('\n');
    }
    break;
  case NMEA_UDP:
    {
      size_t udp_size = size;

      if (size >= sizeof(UDPpacketBuffer))
        udp_size = sizeof(UDPpacketBuffer) - 1;
      memcpy(UDPpacketBuffer, buf, udp_size);

      if (nl)
        UDPpacketBuffer[udp_size] = '\n';

      SoC->WiFi_transmit_UDP(NMEA_UDP_PORT, (byte *) UDPpacketBuffer,
                              nl ? udp_size + 1 : udp_size);
    }
    break;
  case NMEA_TCP:
    {
#if defined(NMEA_TCP_SERVICE)
      for (uint8_t acc_ndx = 0; acc_ndx < MAX_NMEATCP_CLIENTS; acc_ndx++) {
        if (NmeaTCP[acc_ndx].client && NmeaTCP[acc_ndx].client.connected()){
          if (NmeaTCP[acc_ndx].ack) {
            NmeaTCP[acc_ndx].client.write(buf, size);
            if (nl)
              NmeaTCP[acc_ndx].client.write('\n');
          }
        }
      }
#endif
    }
    break;
  case NMEA_BLUETOOTH:
    {
      if (SoC->Bluetooth) {
        SoC->Bluetooth->write(buf, size);
        if (nl)
          SoC->Bluetooth->write((byte *) "\n", 1);
      }
    }
    break;
  case NMEA_OFF:
  default:
    break;
  }
}

void NMEA_fini()
{
#if defined(NMEA_TCP_SERVICE)
  if (settings->s.nmea_out == NMEA_TCP) {
    NmeaTCPServer.stop();
  }
#endif /* NMEA_TCP_SERVICE */
}
