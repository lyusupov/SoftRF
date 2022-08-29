/*
 * NMEAHelper.cpp
 * Copyright (C) 2017-2022 Linar Yusupov
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

#include "NMEA.h"
#include "../../driver/GNSS.h"
#include "../../driver/RF.h"
#include "../../system/SoC.h"
#include "../../driver/WiFi.h"
#include "../../driver/EEPROM.h"
#include "../../driver/Battery.h"
#include "../../driver/Baro.h"
#include "../../TrafficHelper.h"

#define ADDR_TO_HEX_STR(s, c) (s += ((c) < 0x10 ? "0" : "") + String((c), HEX))

#if defined(NMEA_TCP_SERVICE)
WiFiServer NmeaTCPServer(NMEA_TCP_PORT);
NmeaTCP_t NmeaTCP[MAX_NMEATCP_CLIENTS];
#endif

char NMEABuffer[NMEA_BUFFER_SIZE]; //buffer for NMEA data

static char NMEA_Callsign[NMEA_CALLSIGN_SIZE];

#if defined(USE_NMEALIB)
#include <nmealib.h>

NmeaMallocedBuffer nmealib_buf;
#endif /* USE_NMEALIB */

const char *NMEA_CallSign_Prefix[] = {
  [RF_PROTOCOL_LEGACY]    = "FLR",
  [RF_PROTOCOL_OGNTP]     = "OGN",
  [RF_PROTOCOL_P3I]       = "PAW",
  [RF_PROTOCOL_ADSB_1090] = "ADS",
  [RF_PROTOCOL_ADSB_UAT]  = "UAT",
  [RF_PROTOCOL_FANET]     = "FAN"
};

#define isTimeToPGRMZ() (millis() - PGRMZ_TimeMarker > 1000)
unsigned long PGRMZ_TimeMarker = 0;

extern uint32_t tx_packets_counter, rx_packets_counter;

#if defined(ENABLE_AHRS)
#include "../../driver/AHRSHelper.h"

#define isTimeToRPYL()  (millis() - RPYL_TimeMarker > AHRS_INTERVAL)
unsigned long RPYL_TimeMarker = 0;
#endif /* ENABLE_AHRS */

static char *ltrim(char *s)
{
  if(s) {
    while(*s && isspace(*s))
      ++s;
  }

  return s;
}

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

void NMEA_setup()
{
#if defined(NMEA_TCP_SERVICE)
  if (settings->nmea_out == NMEA_TCP) {
    NmeaTCPServer.begin();
    Serial.print(F("NMEA TCP server has started at port: "));
    Serial.println(NMEA_TCP_PORT);

    NmeaTCPServer.setNoDelay(true);
  }
#endif /* NMEA_TCP_SERVICE */

#if defined(USE_NMEALIB)
  memset(&nmealib_buf, 0, sizeof(nmealib_buf));
#endif /* USE_NMEALIB */

  PGRMZ_TimeMarker = millis();

#if defined(ENABLE_AHRS)
  RPYL_TimeMarker = millis();
#endif /* ENABLE_AHRS */
}

void NMEA_loop()
{

  if (settings->nmea_s && ThisAircraft.pressure_altitude != 0.0 && isTimeToPGRMZ()) {

    int altitude = constrain(
            (int) (ThisAircraft.pressure_altitude * _GPS_FEET_PER_METER),
            -1000, 60000);

    snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PGRMZ,%d,f,3*"),
            altitude ); /* feet , 3D fix */

    NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));

    NMEA_Out(settings->nmea_out, (byte *) NMEABuffer, strlen(NMEABuffer), false);

#if !defined(EXCLUDE_LK8EX1)
    char str_Vcc[6];
    dtostrf(Battery_voltage(), 3, 1, str_Vcc);

    snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$LK8EX1,999999,%d,%d,%d,%s*"),
            constrain((int) ThisAircraft.pressure_altitude, -1000, 99998), /* meters */
            (int) ((ThisAircraft.vs * 100) / (_GPS_FEET_PER_METER * 60)),  /* cm/s   */
            constrain((int) Baro_temperature(), -99, 98),                  /* deg. C */
            str_Vcc);

    NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));

    NMEA_Out(settings->nmea_out, (byte *) NMEABuffer, strlen(NMEABuffer), false);
#endif /* EXCLUDE_LK8EX1 */

    PGRMZ_TimeMarker = millis();
  }

#if defined(ENABLE_AHRS)
  if (settings->nmea_s && isTimeToRPYL()) {

    AHRS_NMEA();

    RPYL_TimeMarker = millis();
  }
#endif /* ENABLE_AHRS */

#if defined(NMEA_TCP_SERVICE)
  uint8_t i;

  if (settings->nmea_out == NMEA_TCP) {

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

void NMEA_fini()
{
#if defined(NMEA_TCP_SERVICE)
  if (settings->nmea_out == NMEA_TCP) {
    NmeaTCPServer.stop();
  }
#endif /* NMEA_TCP_SERVICE */
}

void NMEA_Out(uint8_t dest, byte *buf, size_t size, bool nl)
{
  switch (dest)
  {
  case NMEA_UART:
    {
      if (SoC->UART_ops) {
        SoC->UART_ops->write(buf, size);
        if (nl)
          SoC->UART_ops->write((byte *) "\n", 1);
      } else {
        SerialOutput.write(buf, size);
        if (nl)
          SerialOutput.write('\n');
      }
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
  case NMEA_USB:
    {
      if (SoC->USB_ops) {
        SoC->USB_ops->write(buf, size);
        if (nl)
          SoC->USB_ops->write((byte *) "\n", 1);
      }
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

void NMEA_Export()
{
    int bearing;
    int alt_diff;
    float distance;

    int total_objects  = 0;
    int alarm_level    = ALARM_LEVEL_NONE;
    int data_source    = DATA_SOURCE_FLARM;
    time_t this_moment = now();

    /* High priority object (most relevant target) */
    int HP_bearing     = 0;
    int HP_alt_diff    = 0;
    int HP_alarm_level = ALARM_LEVEL_NONE;
    float HP_distance  = 2147483647;
    uint32_t HP_addr   = 0;

    bool has_Fix       = isValidFix() || (settings->mode == SOFTRF_MODE_TXRX_TEST);

    if (has_Fix) {
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

              total_objects++;

              char str_climb_rate[8] = "";
              uint8_t addr_type = Container[i].addr_type > ADDR_TYPE_ANONYMOUS ?
                                  ADDR_TYPE_ANONYMOUS : Container[i].addr_type;

              bearing = Container[i].bearing;
              alarm_level = Container[i].alarm_level;
              alt_diff = (int) (Container[i].altitude - ThisAircraft.altitude);

              if (!Container[i].stealth && !ThisAircraft.stealth) {
                dtostrf(
                  constrain(Container[i].vs / (_GPS_FEET_PER_METER * 60.0), -32.7, 32.7),
                  5, 1, str_climb_rate);
              }

              /*
               * When callsign is available - send it to a NMEA client.
               * If it is not - generate a callsign substitute,
               * based upon a protocol ID and the ICAO address
               */
              memset((void *) NMEA_Callsign, 0, sizeof(NMEA_Callsign));

              if (strnlen((char *) Container[i].callsign, sizeof(Container[i].callsign)) > 0) {
                memcpy(NMEA_Callsign, Container[i].callsign, sizeof(Container[i].callsign));
                for (int j=0; j < sizeof(NMEA_Callsign); j++) {
                  if (NMEA_Callsign[j] == ' ' || NMEA_Callsign[j] == ',' || NMEA_Callsign[j] == '*') {
                    NMEA_Callsign[j] = 0;
                    break;
                  }
                }
              } else {
                memcpy(NMEA_Callsign, NMEA_CallSign_Prefix[Container[i].protocol],
                  strlen(NMEA_CallSign_Prefix[Container[i].protocol]));

                String str = "_";

                ADDR_TO_HEX_STR(str, (Container[i].addr >> 16) & 0xFF);
                ADDR_TO_HEX_STR(str, (Container[i].addr >>  8) & 0xFF);
                ADDR_TO_HEX_STR(str, (Container[i].addr      ) & 0xFF);

                str.toUpperCase();
                memcpy(NMEA_Callsign + strlen(NMEA_CallSign_Prefix[Container[i].protocol]),
                  str.c_str(), str.length());
              }

              data_source = (Container[i].protocol == RF_PROTOCOL_ADSB_UAT ||
                             Container[i].protocol == RF_PROTOCOL_ADSB_1090) ?
                            DATA_SOURCE_ADSB : DATA_SOURCE_FLARM;

              snprintf_P(NMEABuffer, sizeof(NMEABuffer),
                      PSTR("$PFLAA,%d,%d,%d,%d,%d,%06X!%s,%d,,%d,%s,%d" PFLAA_EXT1_FMT "*"),
                      alarm_level,
                      (int) (distance * cos(radians(bearing))), (int) (distance * sin(radians(bearing))),
                      alt_diff, addr_type, Container[i].addr, NMEA_Callsign,
                      (int) Container[i].course, (int) (Container[i].speed * _GPS_MPS_PER_KNOT),
                      ltrim(str_climb_rate), Container[i].aircraft_type
                      PFLAA_EXT1_ARGS );

              NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));

              NMEA_Out(settings->nmea_out, (byte *) NMEABuffer, strlen(NMEABuffer), false);

              /* Most close traffic is treated as highest priority target */
              if (distance < HP_distance && abs(alt_diff) < VERTICAL_VISIBILITY_RANGE) {
                HP_bearing = bearing;
                HP_alt_diff = alt_diff;
                HP_alarm_level = alarm_level;
                HP_distance = distance;
                HP_addr = Container[i].addr;
              }

            }
          }
        }
      }
    }

    /* One PFLAU NMEA sentence is mandatory regardless of traffic reception status */
    if (settings->nmea_l) {
      float voltage    = Battery_voltage();
      int power_status = voltage > BATTERY_THRESHOLD_INVALID &&
                         voltage < Battery_threshold() ?
                         POWER_STATUS_BAD : POWER_STATUS_GOOD;

      if (total_objects > 0) {
        int rel_bearing = HP_bearing - ThisAircraft.course;
        rel_bearing += (rel_bearing < -180 ? 360 : (rel_bearing > 180 ? -360 : 0));

        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
                PSTR("$PFLAU,%d,%d,%d,%d,%d,%d,%d,%d,%u,%06X" PFLAU_EXT1_FMT "*"),
                total_objects,
                settings->txpower == RF_TX_POWER_OFF ? TX_STATUS_OFF : TX_STATUS_ON,
                GNSS_STATUS_3D_MOVING,
                power_status, HP_alarm_level, rel_bearing,
                ALARM_TYPE_AIRCRAFT, HP_alt_diff, (int) HP_distance, HP_addr
                PFLAU_EXT1_ARGS );
      } else {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
                PSTR("$PFLAU,0,%d,%d,%d,%d,,0,,," PFLAU_EXT1_FMT "*"),
                has_Fix && (settings->txpower != RF_TX_POWER_OFF) ?
                  TX_STATUS_ON : TX_STATUS_OFF,
                has_Fix ? GNSS_STATUS_3D_MOVING : GNSS_STATUS_NONE,
                power_status, HP_alarm_level
                PFLAU_EXT1_ARGS );
      }

      NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));

      NMEA_Out(settings->nmea_out, (byte *) NMEABuffer, strlen(NMEABuffer), false);

#if !defined(EXCLUDE_SOFTRF_HEARTBEAT)
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("$PSRFH,%06X,%d,%d,%d,%d*"),
              ThisAircraft.addr,settings->rf_protocol,
              rx_packets_counter,tx_packets_counter,(int)(voltage*100));

      NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));

      NMEA_Out(settings->nmea_out, (byte *) NMEABuffer, strlen(NMEABuffer), false);
#endif /* EXCLUDE_SOFTRF_HEARTBEAT */
    }
}

#if defined(USE_NMEALIB)

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

    if (gen_sz) {
      NMEA_Out(settings->nmea_out, (byte *) nmealib_buf.buffer, gen_sz, false);
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

  if (info.height == 0.0 && info.sig != (NmeaSignal) Invalid) {
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
    NMEA_Out(settings->nmea_out, (byte *) nmealib_buf.buffer, gen_sz, false);
  }
}

#endif /* USE_NMEALIB */

#if defined(USE_NMEA_CFG)

#include "../../driver/Sound.h"
#include "../../driver/LED.h"
#include "GDL90.h"
#include "D1090.h"

#if !defined(SERIAL_FLUSH)
#define SERIAL_FLUSH()       Serial.flush()
#endif

TinyGPSCustom C_Version      (gnss, "PSRFC", 1);
TinyGPSCustom C_Mode         (gnss, "PSRFC", 2);
TinyGPSCustom C_Protocol     (gnss, "PSRFC", 3);
TinyGPSCustom C_Band         (gnss, "PSRFC", 4);
TinyGPSCustom C_AcftType     (gnss, "PSRFC", 5);
TinyGPSCustom C_Alarm        (gnss, "PSRFC", 6);
TinyGPSCustom C_TxPower      (gnss, "PSRFC", 7);
TinyGPSCustom C_Volume       (gnss, "PSRFC", 8);
TinyGPSCustom C_Pointer      (gnss, "PSRFC", 9);
TinyGPSCustom C_NMEA_gnss    (gnss, "PSRFC", 10);
TinyGPSCustom C_NMEA_private (gnss, "PSRFC", 11);
TinyGPSCustom C_NMEA_legacy  (gnss, "PSRFC", 12);
TinyGPSCustom C_NMEA_sensors (gnss, "PSRFC", 13);
TinyGPSCustom C_NMEA_Output  (gnss, "PSRFC", 14);
TinyGPSCustom C_GDL90_Output (gnss, "PSRFC", 15);
TinyGPSCustom C_D1090_Output (gnss, "PSRFC", 16);
TinyGPSCustom C_Stealth      (gnss, "PSRFC", 17);
TinyGPSCustom C_noTrack      (gnss, "PSRFC", 18);
TinyGPSCustom C_PowerSave    (gnss, "PSRFC", 19);

#if defined(USE_OGN_ENCRYPTION)
/* Security and privacy */
TinyGPSCustom S_Version      (gnss, "PSRFS", 1);
TinyGPSCustom S_IGC_Key      (gnss, "PSRFS", 2);
#endif /* USE_OGN_ENCRYPTION */

#if defined(USE_SKYVIEW_CFG)
#include "../../driver/EPD.h"

TinyGPSCustom V_Version      (gnss, "PSKVC", 1);
TinyGPSCustom V_Adapter      (gnss, "PSKVC", 2);
TinyGPSCustom V_Connection   (gnss, "PSKVC", 3);
TinyGPSCustom V_Units        (gnss, "PSKVC", 4);
TinyGPSCustom V_Zoom         (gnss, "PSKVC", 5);
TinyGPSCustom V_Protocol     (gnss, "PSKVC", 6);
TinyGPSCustom V_Baudrate     (gnss, "PSKVC", 7);
TinyGPSCustom V_Server       (gnss, "PSKVC", 8);
TinyGPSCustom V_Key          (gnss, "PSKVC", 9);
TinyGPSCustom V_Rotate       (gnss, "PSKVC", 10);
TinyGPSCustom V_Orientation  (gnss, "PSKVC", 11);
TinyGPSCustom V_AvDB         (gnss, "PSKVC", 12);
TinyGPSCustom V_ID_Pref      (gnss, "PSKVC", 13);
TinyGPSCustom V_VMode        (gnss, "PSKVC", 14);
TinyGPSCustom V_Voice        (gnss, "PSKVC", 15);
TinyGPSCustom V_AntiGhost    (gnss, "PSKVC", 16);
TinyGPSCustom V_Filter       (gnss, "PSKVC", 17);
TinyGPSCustom V_PowerSave    (gnss, "PSKVC", 18);
TinyGPSCustom V_Team         (gnss, "PSKVC", 19);
#endif /* USE_SKYVIEW_CFG */

uint8_t C_NMEA_Source;

static void nmea_cfg_restart()
{
  Serial.println();
  Serial.println(F("Restart is in progress. Please, wait..."));
  Serial.println();
  SERIAL_FLUSH();
  Sound_fini();
  RF_Shutdown();
  SoC->reset();
}

void NMEA_Process_SRF_SKV_Sentences()
{
      if (C_Version.isUpdated()) {
        if (strncmp(C_Version.value(), "RST", 3) == 0) {
            SoC->WDT_fini();
            nmea_cfg_restart();
        } else if (strncmp(C_Version.value(), "OFF", 3) == 0) {
          shutdown(SOFTRF_SHUTDOWN_NMEA);
        } else if (strncmp(C_Version.value(), "?", 1) == 0) {
          char psrfc_buf[MAX_PSRFC_LEN];

          snprintf_P(psrfc_buf, sizeof(psrfc_buf),
              PSTR("$PSRFC,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d*"),
              PSRFC_VERSION,        settings->mode,     settings->rf_protocol,
              settings->band,       settings->aircraft_type, settings->alarm,
              settings->txpower,    settings->volume,   settings->pointer,
              settings->nmea_g,     settings->nmea_p,   settings->nmea_l,
              settings->nmea_s,     settings->nmea_out, settings->gdl90,
              settings->d1090,      settings->stealth,  settings->no_track,
              settings->power_save );

          NMEA_add_checksum(psrfc_buf, sizeof(psrfc_buf) - strlen(psrfc_buf));

#if !defined(USE_NMEA_CFG)
          uint8_t dest = settings->nmea_out;
#else
          uint8_t dest = C_NMEA_Source;
#endif /* USE_NMEA_CFG */

          NMEA_Out(dest, (byte *) psrfc_buf, strlen(psrfc_buf), false);

        } else if (atoi(C_Version.value()) == PSRFC_VERSION) {
          bool cfg_is_updated = false;

          if (C_Mode.isUpdated())
          {
            settings->mode = atoi(C_Mode.value());
            Serial.print(F("Mode = ")); Serial.println(settings->mode);
            cfg_is_updated = true;
          }
          if (C_Protocol.isUpdated())
          {
            settings->rf_protocol = atoi(C_Protocol.value());
            Serial.print(F("Protocol = ")); Serial.println(settings->rf_protocol);
            cfg_is_updated = true;
          }
          if (C_Band.isUpdated())
          {
            settings->band = atoi(C_Band.value());
            Serial.print(F("Region = ")); Serial.println(settings->band);
            cfg_is_updated = true;
          }
          if (C_AcftType.isUpdated())
          {
            settings->aircraft_type = atoi(C_AcftType.value());
            Serial.print(F("AcftType = ")); Serial.println(settings->aircraft_type);
            cfg_is_updated = true;
          }
          if (C_Alarm.isUpdated())
          {
            settings->alarm = atoi(C_Alarm.value());
            Serial.print(F("Alarm = ")); Serial.println(settings->alarm);
            cfg_is_updated = true;
          }
          if (C_TxPower.isUpdated())
          {
            settings->txpower = atoi(C_TxPower.value());
            Serial.print(F("TxPower = ")); Serial.println(settings->txpower);
            cfg_is_updated = true;
          }
          if (C_Volume.isUpdated())
          {
            settings->volume = atoi(C_Volume.value());
            Serial.print(F("Volume = ")); Serial.println(settings->volume);
            cfg_is_updated = true;
          }
           if (C_Pointer.isUpdated())
          {
            settings->pointer = atoi(C_Pointer.value());
            Serial.print(F("Pointer = ")); Serial.println(settings->pointer);
            cfg_is_updated = true;
          }
          if (C_NMEA_gnss.isUpdated())
          {
            settings->nmea_g = atoi(C_NMEA_gnss.value());
            Serial.print(F("NMEA_gnss = ")); Serial.println(settings->nmea_g);
            cfg_is_updated = true;
          }
          if (C_NMEA_private.isUpdated())
          {
            settings->nmea_p = atoi(C_NMEA_private.value());
            Serial.print(F("NMEA_private = ")); Serial.println(settings->nmea_p);
            cfg_is_updated = true;
          }
          if (C_NMEA_legacy.isUpdated())
          {
            settings->nmea_l = atoi(C_NMEA_legacy.value());
            Serial.print(F("NMEA_legacy = ")); Serial.println(settings->nmea_l);
            cfg_is_updated = true;
          }
           if (C_NMEA_sensors.isUpdated())
          {
            settings->nmea_s = atoi(C_NMEA_sensors.value());
            Serial.print(F("NMEA_sensors = ")); Serial.println(settings->nmea_s);
            cfg_is_updated = true;
          }
          if (C_NMEA_Output.isUpdated())
          {
            settings->nmea_out = atoi(C_NMEA_Output.value());
            Serial.print(F("NMEA_Output = ")); Serial.println(settings->nmea_out);
            cfg_is_updated = true;
          }
          if (C_GDL90_Output.isUpdated())
          {
            settings->gdl90 = atoi(C_GDL90_Output.value());
            Serial.print(F("GDL90_Output = ")); Serial.println(settings->gdl90);
            cfg_is_updated = true;
          }
          if (C_D1090_Output.isUpdated())
          {
            settings->d1090 = atoi(C_D1090_Output.value());
            Serial.print(F("D1090_Output = ")); Serial.println(settings->d1090);
            cfg_is_updated = true;
          }
          if (C_Stealth.isUpdated())
          {
            settings->stealth = atoi(C_Stealth.value());
            Serial.print(F("Stealth = ")); Serial.println(settings->stealth);
            cfg_is_updated = true;
          }
          if (C_noTrack.isUpdated())
          {
            settings->no_track = atoi(C_noTrack.value());
            Serial.print(F("noTrack = ")); Serial.println(settings->no_track);
            cfg_is_updated = true;
          }
          if (C_PowerSave.isUpdated())
          {
            settings->power_save = atoi(C_PowerSave.value());
            Serial.print(F("PowerSave = ")); Serial.println(settings->power_save);
            cfg_is_updated = true;
          }

          if (cfg_is_updated) {
            SoC->WDT_fini();
            if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
            EEPROM_store();
            nmea_cfg_restart();
          }
        }
      }

#if defined(USE_OGN_ENCRYPTION)
      if (S_Version.isUpdated()) {
        if (strncmp(S_Version.value(), "?", 1) == 0) {
          char psrfs_buf[MAX_PSRFS_LEN];

          snprintf_P(psrfs_buf, sizeof(psrfs_buf),
              PSTR("$PSRFS,%d,%08X%08X%08X%08X*"),
              PSRFS_VERSION, settings->igc_key[0], settings->igc_key[1],
                             settings->igc_key[2], settings->igc_key[3]);

          NMEA_add_checksum(psrfs_buf, sizeof(psrfs_buf) - strlen(psrfs_buf));

          NMEA_Out(C_NMEA_Source, (byte *) psrfs_buf, strlen(psrfs_buf), false);

        } else if (atoi(S_Version.value()) == PSRFS_VERSION) {
          bool cfg_is_updated = false;

          if (S_IGC_Key.isUpdated())
          {
            char buf[32 + 1];

            strncpy(buf, S_IGC_Key.value(), sizeof(buf));

            settings->igc_key[3] = strtoul(buf + 24, NULL, 16);
            buf[24] = 0;
            settings->igc_key[2] = strtoul(buf + 16, NULL, 16);
            buf[16] = 0;
            settings->igc_key[1] = strtoul(buf +  8, NULL, 16);
            buf[ 8] = 0;
            settings->igc_key[0] = strtoul(buf +  0, NULL, 16);

            snprintf_P(buf, sizeof(buf),
              PSTR("%08X%08X%08X%08X"),
              settings->igc_key[0], settings->igc_key[1],
              settings->igc_key[2], settings->igc_key[3]);

            Serial.print(F("IGC Key = ")); Serial.println(buf);
            cfg_is_updated = true;
          }
          if (cfg_is_updated) {
            SoC->WDT_fini();
            if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
            EEPROM_store();
            nmea_cfg_restart();
          }
        }
      }
#endif /* USE_OGN_ENCRYPTION */

#if defined(USE_SKYVIEW_CFG)
      if (V_Version.isUpdated()) {
        if (strncmp(V_Version.value(), "?", 1) == 0) {
          char pskvc_buf[MAX_PSKVC_LEN];

          snprintf_P(pskvc_buf, sizeof(pskvc_buf),
              PSTR("$PSKVC,%d,%d,%d,%d,%d,%d,%d,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%X*"),
              PSKVC_VERSION,  ui->adapter,      ui->connection,
              ui->units,      ui->zoom,         ui->protocol,
              ui->baudrate,   ui->server,       ui->key,
              ui->rotate,     ui->orientation,  ui->adb,
              ui->idpref,     ui->vmode,        ui->voice,
              ui->aghost,     ui->filter,       ui->power_save,
              ui->team);

          NMEA_add_checksum(pskvc_buf, sizeof(pskvc_buf) - strlen(pskvc_buf));

          NMEA_Out(C_NMEA_Source, (byte *) pskvc_buf, strlen(pskvc_buf), false);

        } else if (atoi(V_Version.value()) == PSKVC_VERSION) {
          bool cfg_is_updated = false;

          if (V_Adapter.isUpdated())
          {
            ui->adapter = atoi(V_Adapter.value());
            Serial.print(F("Adapter = ")); Serial.println(ui->adapter);
            cfg_is_updated = true;
          }
          if (V_Connection.isUpdated())
          {
            ui->connection = atoi(V_Connection.value());
            Serial.print(F("Connection = ")); Serial.println(ui->connection);
            cfg_is_updated = true;
          }
          if (V_Units.isUpdated())
          {
            ui->units = atoi(V_Units.value());
            Serial.print(F("Units = ")); Serial.println(ui->units);
            cfg_is_updated = true;
          }
          if (V_Zoom.isUpdated())
          {
            ui->zoom = atoi(V_Zoom.value());
            Serial.print(F("Zoom = ")); Serial.println(ui->zoom);
            cfg_is_updated = true;
          }
          if (V_Protocol.isUpdated())
          {
            ui->protocol = atoi(V_Protocol.value());
            Serial.print(F("Protocol = ")); Serial.println(ui->protocol);
            cfg_is_updated = true;
          }
          if (V_Baudrate.isUpdated())
          {
            ui->baudrate = atoi(V_Baudrate.value());
            Serial.print(F("Baudrate = ")); Serial.println(ui->baudrate);
            cfg_is_updated = true;
          }
          if (V_Server.isUpdated())
          {
            strncpy(ui->server, V_Server.value(), sizeof(ui->server));
            Serial.print(F("Server = ")); Serial.println(ui->server);
            cfg_is_updated = true;
          }
           if (V_Key.isUpdated())
          {
            strncpy(ui->key, V_Key.value(), sizeof(ui->key));
            Serial.print(F("Key = ")); Serial.println(ui->key);
            cfg_is_updated = true;
          }
          if (V_Rotate.isUpdated())
          {
            ui->rotate = atoi(V_Rotate.value());
            Serial.print(F("Rotation = ")); Serial.println(ui->rotate);
            cfg_is_updated = true;
          }
          if (V_Orientation.isUpdated())
          {
            ui->orientation = atoi(V_Orientation.value());
            Serial.print(F("Orientation = ")); Serial.println(ui->orientation);
            cfg_is_updated = true;
          }
          if (V_AvDB.isUpdated())
          {
            ui->adb = atoi(V_AvDB.value());
            Serial.print(F("AvDB = ")); Serial.println(ui->adb);
            cfg_is_updated = true;
          }
          if (V_ID_Pref.isUpdated())
          {
            ui->idpref = atoi(V_ID_Pref.value());
            Serial.print(F("ID_Pref = ")); Serial.println(ui->idpref);
            cfg_is_updated = true;
          }
           if (V_VMode.isUpdated())
          {
            ui->vmode = atoi(V_VMode.value());
            Serial.print(F("VMode = ")); Serial.println(ui->vmode);
            cfg_is_updated = true;
          }
          if (V_Voice.isUpdated())
          {
            ui->voice = atoi(V_Voice.value());
            Serial.print(F("Voice = ")); Serial.println(ui->voice);
            cfg_is_updated = true;
          }
          if (V_AntiGhost.isUpdated())
          {
            ui->aghost = atoi(V_AntiGhost.value());
            Serial.print(F("AntiGhost = ")); Serial.println(ui->aghost);
            cfg_is_updated = true;
          }
          if (V_Filter.isUpdated())
          {
            ui->filter = atoi(V_Filter.value());
            Serial.print(F("Filter = ")); Serial.println(ui->filter);
            cfg_is_updated = true;
          }
          if (V_PowerSave.isUpdated())
          {
            ui->power_save = atoi(V_PowerSave.value());
            Serial.print(F("PowerSave = ")); Serial.println(ui->power_save);
            cfg_is_updated = true;
          }
          if (V_Team.isUpdated())
          {
            ui->team = strtoul(V_Team.value(), NULL, 16);
            Serial.print(F("Team = ")); Serial.println(ui->team, HEX);
            cfg_is_updated = true;
          }

          if (cfg_is_updated) {
            SoC->WDT_fini();
            if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
            EEPROM_store();
            nmea_cfg_restart();
          }
        }
      }
#endif /* USE_SKYVIEW_CFG */
}
#endif /* USE_NMEA_CFG */
