/*
 * NMEAHelper.cpp
 * Copyright (C) 2019-2022 Linar Yusupov
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
#include "GNSSHelper.h"

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

TinyGPSCustom H_Addr            (nmea, "PSRFH", 1);
TinyGPSCustom H_Protocol        (nmea, "PSRFH", 2);
TinyGPSCustom H_RxCnt           (nmea, "PSRFH", 3);
TinyGPSCustom H_TxCnt           (nmea, "PSRFH", 4);

#if !defined(USE_NMEA_CFG)

TinyGPSCustom C_Version         (nmea, "PSRFC", 1);
TinyGPSCustom C_Mode            (nmea, "PSRFC", 2);
TinyGPSCustom C_Protocol        (nmea, "PSRFC", 3);
TinyGPSCustom C_Band            (nmea, "PSRFC", 4);
TinyGPSCustom C_AcftType        (nmea, "PSRFC", 5);
TinyGPSCustom C_Alarm           (nmea, "PSRFC", 6);
TinyGPSCustom C_TxPower         (nmea, "PSRFC", 7);
TinyGPSCustom C_Volume          (nmea, "PSRFC", 8);
TinyGPSCustom C_Pointer         (nmea, "PSRFC", 9);
TinyGPSCustom C_NMEA_gnss       (nmea, "PSRFC", 10);
TinyGPSCustom C_NMEA_private    (nmea, "PSRFC", 11);
TinyGPSCustom C_NMEA_legacy     (nmea, "PSRFC", 12);
TinyGPSCustom C_NMEA_sensors    (nmea, "PSRFC", 13);
TinyGPSCustom C_NMEA_Output     (nmea, "PSRFC", 14);
TinyGPSCustom C_GDL90_Output    (nmea, "PSRFC", 15);
TinyGPSCustom C_D1090_Output    (nmea, "PSRFC", 16);
TinyGPSCustom C_Stealth         (nmea, "PSRFC", 17);
TinyGPSCustom C_noTrack         (nmea, "PSRFC", 18);
TinyGPSCustom C_PowerSave       (nmea, "PSRFC", 19);

#endif /* USE_NMEA_CFG */

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
          NMEA_Out(settings->m.data_dest, (byte *) &NMEABuffer[ndx], write_size, true);
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

      } else if (H_Addr.isUpdated()) {

        ThisDevice.addr = strtol(H_Addr.value(), NULL, 16);

        if (H_Protocol.isUpdated())
        {
          ThisDevice.protocol = atoi(H_Protocol.value());
        }
        if (H_RxCnt.isUpdated())
        {
          rx_packets_counter = strtol(H_RxCnt.value(), NULL, 10);
        }
        if (H_TxCnt.isUpdated())
        {
          tx_packets_counter = strtol(H_TxCnt.value(), NULL, 10);
        }

#if !defined(USE_NMEA_CFG)
      } else if (C_Version.isUpdated()) {
        if (atoi(C_Version.value()) == PSRFC_VERSION) {
          bool cfg_is_updated = false;

          if (C_Mode.isUpdated())
          {
            settings->s.mode = atoi(C_Mode.value());
//            Serial.print(F("Mode = ")); Serial.println(settings->s.mode);
            cfg_is_updated = true;
          }
          if (C_Protocol.isUpdated())
          {
            settings->s.rf_protocol = atoi(C_Protocol.value());
//            Serial.print(F("Protocol = ")); Serial.println(settings->s.rf_protocol);
            cfg_is_updated = true;
          }
          if (C_Band.isUpdated())
          {
            settings->s.band = atoi(C_Band.value());
//            Serial.print(F("Region = ")); Serial.println(settings->s.band);
            cfg_is_updated = true;
          }
          if (C_AcftType.isUpdated())
          {
            settings->s.aircraft_type = atoi(C_AcftType.value());
//            Serial.print(F("AcftType = ")); Serial.println(settings->s.aircraft_type);
            cfg_is_updated = true;
          }
          if (C_Alarm.isUpdated())
          {
            settings->s.alarm = atoi(C_Alarm.value());
//            Serial.print(F("Alarm = ")); Serial.println(settings->s.alarm);
            cfg_is_updated = true;
          }
          if (C_TxPower.isUpdated())
          {
            settings->s.txpower = atoi(C_TxPower.value());
//            Serial.print(F("TxPower = ")); Serial.println(settings->s.txpower);
            cfg_is_updated = true;
          }
          if (C_Volume.isUpdated())
          {
            settings->s.volume = atoi(C_Volume.value());
//            Serial.print(F("Volume = ")); Serial.println(settings->s.volume);
            cfg_is_updated = true;
          }
           if (C_Pointer.isUpdated())
          {
            settings->s.pointer = atoi(C_Pointer.value());
//            Serial.print(F("Pointer = ")); Serial.println(settings->s.pointer);
            cfg_is_updated = true;
          }
          if (C_NMEA_gnss.isUpdated())
          {
            settings->s.nmea_g = atoi(C_NMEA_gnss.value());
//            Serial.print(F("NMEA_gnss = ")); Serial.println(settings->s.nmea_g);
            cfg_is_updated = true;
          }
          if (C_NMEA_private.isUpdated())
          {
            settings->s.nmea_p = atoi(C_NMEA_private.value());
//            Serial.print(F("NMEA_private = ")); Serial.println(settings->s.nmea_p);
            cfg_is_updated = true;
          }
          if (C_NMEA_legacy.isUpdated())
          {
            settings->s.nmea_l = atoi(C_NMEA_legacy.value());
//            Serial.print(F("NMEA_legacy = ")); Serial.println(settings->s.nmea_l);
            cfg_is_updated = true;
          }
           if (C_NMEA_sensors.isUpdated())
          {
            settings->s.nmea_s = atoi(C_NMEA_sensors.value());
//            Serial.print(F("NMEA_sensors = ")); Serial.println(settings->s.nmea_s);
            cfg_is_updated = true;
          }
          if (C_NMEA_Output.isUpdated())
          {
            settings->s.nmea_out = atoi(C_NMEA_Output.value());
//            Serial.print(F("NMEA_Output = ")); Serial.println(settings->s.nmea_out);
            cfg_is_updated = true;
          }
          if (C_GDL90_Output.isUpdated())
          {
            settings->s.gdl90 = atoi(C_GDL90_Output.value());
//            Serial.print(F("GDL90_Output = ")); Serial.println(settings->s.gdl90);
            cfg_is_updated = true;
          }
          if (C_D1090_Output.isUpdated())
          {
            settings->s.d1090 = atoi(C_D1090_Output.value());
//            Serial.print(F("D1090_Output = ")); Serial.println(settings->s.d1090);
            cfg_is_updated = true;
          }
          if (C_Stealth.isUpdated())
          {
            settings->s.stealth = atoi(C_Stealth.value());
//            Serial.print(F("Stealth = ")); Serial.println(settings->s.stealth);
            cfg_is_updated = true;
          }
          if (C_noTrack.isUpdated())
          {
            settings->s.no_track = atoi(C_noTrack.value());
//            Serial.print(F("noTrack = ")); Serial.println(settings->s.no_track);
            cfg_is_updated = true;
          }
          if (C_PowerSave.isUpdated())
          {
            settings->s.power_save = atoi(C_PowerSave.value());
//            Serial.print(F("PowerSave = ")); Serial.println(settings->s.power_save);
            cfg_is_updated = true;
          }

          if (cfg_is_updated) {
#if 0
            SoC->WDT_fini();
            if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
            EEPROM_store();
            nmea_cfg_restart();
#endif
          }
        }
#endif /* USE_NMEA_CFG */
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
#if 0
      if (SoC->Bluetooth_ops) {
        SoC->Bluetooth_ops->setup();
      }
#endif
      break;
#if defined(CONFIG_IDF_TARGET_ESP32S2)
    case CON_USB:
#if 0
      if (SoC->USB_ops) {
        SoC->USB_ops->setup();
      }
#endif
      break;
#endif /* CONFIG_IDF_TARGET_ESP32S2 */
    case CON_NONE:
    case CON_WIFI_UDP:
    default:
      break;
    }

#if defined(NMEA_TCP_SERVICE)
    if (settings->m.data_dest == NMEA_TCP) {
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
  case CON_USB:
    /* read data from Type-C USB port in Host mode */
    if (SoC->USB_ops) {
      while (SoC->USB_ops->available() > 0) {
        c = SoC->USB_ops->read();
#if defined(ENABLE_USB_HOST_DEBUG)
        if (hw_info.gnss == GNSS_MODULE_NONE) {
          Serial.print(c);
        }
#endif
        NMEA_Parse_Character(c);
        NMEA_TimeMarker = millis();
      }
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
    if (SoC->Bluetooth_ops) {
      while (SoC->Bluetooth_ops->available() > 0) {
        c = SoC->Bluetooth_ops->read();
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

    NMEA_Out(settings->m.data_dest, (byte *) PGRMZBuffer, strlen(PGRMZBuffer), false);

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

  if (settings->m.data_dest == NMEA_TCP) {

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

bool NMEA_Request_Settings()
{
    const char *msg = "$PSRFC,?*47\r\n";

    if (hw_info.model == SOFTRF_MODEL_WEBTOP_USB &&
        settings->m.connection == CON_USB) {
      if (SoC->USB_ops) {
        SoC->USB_ops->write((byte *) msg, strlen(msg));
      }
    } else {
      SerialInput.write(msg);
    }

    return true;
}

bool NMEA_Save_Settings()
{
    int nmea_out = NMEA_UART;

    if (hw_info.model == SOFTRF_MODEL_WEBTOP_USB &&
        settings->m.connection == CON_USB) {
      nmea_out = NMEA_USB;
    }

    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRFC,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d*"),
            PSRFC_VERSION, settings->s.mode, settings->s.rf_protocol,
            settings->s.band, settings->s.aircraft_type, settings->s.alarm,
            settings->s.txpower, BUZZER_OFF, LED_OFF,
            settings->s.nmea_g, settings->s.nmea_p, settings->s.nmea_l,
            settings->s.nmea_s, nmea_out, GDL90_OFF, D1090_OFF,
            settings->s.stealth, settings->s.no_track, settings->s.power_save );

    NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));

    if (hw_info.model == SOFTRF_MODEL_WEBTOP_USB &&
        settings->m.connection == CON_USB) {
      if (SoC->USB_ops) {
        SoC->USB_ops->write((byte *) NMEABuffer, strlen(NMEABuffer));
      }
    } else {
      SerialInput.write((byte *) NMEABuffer, strlen(NMEABuffer));
    }

    return true;
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

void NMEA_Out(uint8_t dest, byte *buf, size_t size, bool nl)
{
  switch (dest)
  {
  case NMEA_UART:
  case NMEA_USB:
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
      if (SoC->Bluetooth_ops) {
        SoC->Bluetooth_ops->write(buf, size);
        if (nl)
          SoC->Bluetooth_ops->write((byte *) "\n", 1);
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
  if (settings->m.data_dest == NMEA_TCP) {
    NmeaTCPServer.stop();
  }
#endif /* NMEA_TCP_SERVICE */
}
