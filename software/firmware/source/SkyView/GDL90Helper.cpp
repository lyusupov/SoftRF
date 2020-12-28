/*
 * GDL90Helper.cpp
 * Copyright (C) 2019-2021 Linar Yusupov
 *
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
#include "GDL90Helper.h"
#include "EEPROMHelper.h"
#include "NMEAHelper.h"
#include "TrafficHelper.h"
#include "WiFiHelper.h"

#include "SkyView.h"

extern "C" {
#include <gdl90.h>
}

#define GDL90_RINGBUF_SIZE  sizeof(gdl_message_escaped_t)

static unsigned long GDL90_Data_TimeMarker = 0;
static unsigned long GDL90_HeartBeat_TimeMarker = 0;
static unsigned long GDL90_OwnShip_TimeMarker = 0;
static unsigned char gdl90_ringbuf[GDL90_RINGBUF_SIZE];
static unsigned int gdl90buf_head = 0;
static unsigned char prev_c = 0;

gdl_message_t message;

gdl90_msg_heartbeat heartbeat;
gdl90_msg_traffic_report_t gdl_traffic;
gdl90_msg_traffic_report_t ownship;
gdl90_msg_ownship_geo_altitude geo_altitude;

const uint8_t gdl90_to_aircraft_type[] PROGMEM = {
	AIRCRAFT_TYPE_UNKNOWN,
	AIRCRAFT_TYPE_POWERED,
	AIRCRAFT_TYPE_POWERED,
	AIRCRAFT_TYPE_JET,
	AIRCRAFT_TYPE_JET,
	AIRCRAFT_TYPE_JET,
	AIRCRAFT_TYPE_POWERED,
	AIRCRAFT_TYPE_HELICOPTER,
	AIRCRAFT_TYPE_RESERVED,
	AIRCRAFT_TYPE_GLIDER,
	AIRCRAFT_TYPE_BALLOON,
	AIRCRAFT_TYPE_PARACHUTE,
	AIRCRAFT_TYPE_HANGGLIDER,
	AIRCRAFT_TYPE_RESERVED,
	AIRCRAFT_TYPE_UAV,
	AIRCRAFT_TYPE_RESERVED
};

static void GDL90_Parse_Character(char c)
{
    unsigned int gdl90buf_tail;
    size_t msg_size;

    if (c == GDL90_CONTROL_ESCAPE) {
      prev_c = c;
      return;
    } else if (prev_c == GDL90_CONTROL_ESCAPE) {
      prev_c = c;
      c ^= GDL90_ESCAPE_BYTE;
    } else {
      prev_c = c;
    }

    gdl90_ringbuf[gdl90buf_head % GDL90_RINGBUF_SIZE] = c;
    gdl90buf_head++;

    msg_size = 1 /* flag */ + 1 /* id */ + GDL90_MSG_LEN_HEARTBEAT + 2 /* FC */ + 1 /* flag */;
    gdl90buf_tail = gdl90buf_head - msg_size;
    if (c == GDL90_FLAG_BYTE && 
        gdl90_ringbuf[ gdl90buf_tail    % GDL90_RINGBUF_SIZE] == GDL90_FLAG_BYTE &&
        gdl90_ringbuf[(gdl90buf_tail+1) % GDL90_RINGBUF_SIZE] == MSG_ID_HEARTBEAT) {

      unsigned char *buf = (unsigned char *) &message;
      for (uint8_t i=0; i < msg_size; i++) {
          buf[i] = gdl90_ringbuf[(gdl90buf_tail + i) % GDL90_RINGBUF_SIZE];
      }

      if (decode_gdl90_heartbeat(&message, &heartbeat)) {

//      print_gdl90_heartbeat(&heartbeat);

        GDL90_HeartBeat_TimeMarker = millis();
      }
    }

    msg_size = 1 /* flag */ + 1 /* id */ + GDL90_MSG_LEN_OWNSHIP_GEOMETRIC + 2 /* FC */ + 1 /* flag */;
    gdl90buf_tail = gdl90buf_head - msg_size;
    if (c == GDL90_FLAG_BYTE && 
        gdl90_ringbuf[ gdl90buf_tail    % GDL90_RINGBUF_SIZE] == GDL90_FLAG_BYTE &&
        gdl90_ringbuf[(gdl90buf_tail+1) % GDL90_RINGBUF_SIZE] == MSG_ID_OWNSHIP_GEOMETRIC) {

      unsigned char *buf = (unsigned char *) &message;
      for (uint8_t i=0; i < msg_size; i++) {
          buf[i] = gdl90_ringbuf[(gdl90buf_tail + i) % GDL90_RINGBUF_SIZE];
      }

      decode_gdl90_ownship_geo_altitude(&message, &geo_altitude);
//    print_gdl90_ownship_geo_altitude(&geo_altitude);
    }

    msg_size = 1 /* flag */ + 1 /* id */ + GDL90_MSG_LEN_TRAFFIC_REPORT + 2 /* FC */ + 1 /* flag */;
    gdl90buf_tail = gdl90buf_head - msg_size;
    if (c == GDL90_FLAG_BYTE && 
        gdl90_ringbuf[ gdl90buf_tail    % GDL90_RINGBUF_SIZE] == GDL90_FLAG_BYTE &&
        gdl90_ringbuf[(gdl90buf_tail+1) % GDL90_RINGBUF_SIZE] == MSG_ID_TRAFFIC_REPORT) {

      unsigned char *buf = (unsigned char *) &message;
      for (uint8_t i=0; i < msg_size; i++) {
          buf[i] = gdl90_ringbuf[(gdl90buf_tail + i) % GDL90_RINGBUF_SIZE];
      }

      if (decode_gdl90_traffic_report(&message, &gdl_traffic)) {

//      print_gdl90_traffic_report(&gdl_traffic);

        fo = EmptyFO;

        fo.ID          = gdl_traffic.address;
        fo.IDType      = gdl_traffic.addressType == ADS_B_WITH_ICAO_ADDRESS ?
                                          ADDR_TYPE_ICAO : ADDR_TYPE_ANONYMOUS;

        fo.latitude    = gdl_traffic.latitude;
        fo.longitude   = gdl_traffic.longitude;
        fo.altitude    = gdl_traffic.altitude  / _GPS_FEET_PER_METER;

        fo.AlarmLevel  = gdl_traffic.trafficAlertStatus == TRAFFIC_ALERT ?
                                            ALARM_LEVEL_LOW : ALARM_LEVEL_NONE;
        fo.Track       = gdl_traffic.trackOrHeading;           // degrees
        fo.ClimbRate   = gdl_traffic.verticalVelocity/ (_GPS_FEET_PER_METER * 60.0);
        fo.TurnRate    = 0;
        fo.GroundSpeed = gdl_traffic.horizontalVelocity * _GPS_MPS_PER_KNOT;
        fo.AcftType    = GDL90_TO_AT(gdl_traffic.emitterCategory);

        memcpy(fo.callsign, gdl_traffic.callsign, sizeof(fo.callsign));

        fo.timestamp   = now();

        Traffic_Update(&fo);
        Traffic_Add();
      }
    }

    msg_size = 1 /* flag */ + 1 /* id */ + GDL90_MSG_LEN_OWNSHIP_REPORT + 2 /* FC */ + 1 /* flag */;
    gdl90buf_tail = gdl90buf_head - msg_size;
    if (c == GDL90_FLAG_BYTE && 
        gdl90_ringbuf[ gdl90buf_tail    % GDL90_RINGBUF_SIZE] == GDL90_FLAG_BYTE &&
        gdl90_ringbuf[(gdl90buf_tail+1) % GDL90_RINGBUF_SIZE] == MSG_ID_OWNSHIP_REPORT) {

      unsigned char *buf = (unsigned char *) &message;
      for (uint8_t i=0; i < msg_size; i++) {
          buf[i] = gdl90_ringbuf[(gdl90buf_tail + i) % GDL90_RINGBUF_SIZE];
      }

      if (decode_gdl90_traffic_report(&message, &ownship)) {

//      print_gdl90_traffic_report(&ownship);

        ThisAircraft.ID          = ownship.address;
        ThisAircraft.IDType      = ownship.addressType == ADS_B_WITH_ICAO_ADDRESS ?
                                          ADDR_TYPE_ICAO : ADDR_TYPE_ANONYMOUS;

        ThisAircraft.latitude    = ownship.latitude;
        ThisAircraft.longitude   = ownship.longitude;

        if (ownship.altitude != 101375 /* 0xFFF */ ) {
          ThisAircraft.altitude  = ownship.altitude / _GPS_FEET_PER_METER;
        } else if (geo_altitude.ownshipGeoAltitude != 0) {
          ThisAircraft.altitude  = geo_altitude.ownshipGeoAltitude / _GPS_FEET_PER_METER;
        }

        ThisAircraft.AlarmLevel  = ownship.trafficAlertStatus == TRAFFIC_ALERT ?
                                            ALARM_LEVEL_LOW : ALARM_LEVEL_NONE;
        ThisAircraft.Track       = ownship.trackOrHeading;           // degrees
        ThisAircraft.ClimbRate   = ownship.verticalVelocity/ (_GPS_FEET_PER_METER * 60.0);
        ThisAircraft.TurnRate    = 0;
        ThisAircraft.GroundSpeed = ownship.horizontalVelocity * _GPS_MPS_PER_KNOT;
        ThisAircraft.AcftType    = GDL90_TO_AT(ownship.emitterCategory);

        memcpy(ThisAircraft.callsign, ownship.callsign, sizeof(ThisAircraft.callsign));

        ThisAircraft.timestamp   = now();

        GDL90_OwnShip_TimeMarker = millis();
      }
    }
}

void GDL90_setup()
{
  if (settings->protocol == PROTOCOL_GDL90) {

    gdl90_crcInit();

    switch (settings->connection)
    {
    case CON_SERIAL:
      uint32_t SerialBaud;

      switch (settings->baudrate)
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
    case CON_BLUETOOTH_SPP:
    case CON_BLUETOOTH_LE:
      if (SoC->Bluetooth) {
        SoC->Bluetooth->setup();
      }
      break;
    case CON_NONE:
    case CON_WIFI_UDP:
    default:
      break;
    }

    GDL90_HeartBeat_TimeMarker = GDL90_Data_TimeMarker = millis();
  }
}

void GDL90_loop()
{
  size_t size;

  switch (settings->connection)
  {
  case CON_SERIAL:
    while (SerialInput.available() > 0) {
      char c = SerialInput.read();
//      Serial.print(c);
      GDL90_Parse_Character(c);
      GDL90_Data_TimeMarker = millis();
    }
    /* read data from microUSB port */
#if !defined(RASPBERRY_PI)
    if (Serial != SerialInput)
#endif
    {
      while (Serial.available() > 0) {
        char c = Serial.read();
//        Serial.print(c);
        GDL90_Parse_Character(c);
        GDL90_Data_TimeMarker = millis();
      }
    }
    break;
  case CON_WIFI_UDP:
    size = SoC->WiFi_Receive_UDP((uint8_t *) UDPpacketBuffer, sizeof(UDPpacketBuffer));
    if (size > 0) {
      for (size_t i=0; i < size; i++) {
//        Serial.print(UDPpacketBuffer[i]);
        GDL90_Parse_Character(UDPpacketBuffer[i]);
      }
      GDL90_Data_TimeMarker = millis();
    }
    break;
  case CON_BLUETOOTH_SPP:
  case CON_BLUETOOTH_LE:
    if (SoC->Bluetooth) {
      while (SoC->Bluetooth->available() > 0) {
        char c = SoC->Bluetooth->read();
//        Serial.print(c);
        GDL90_Parse_Character(c);
        GDL90_Data_TimeMarker = millis();
      }
    }
    break;
  case CON_NONE:
  default:
    break;
  }
}

bool GDL90_isConnected()
{
  return (GDL90_Data_TimeMarker > DATA_TIMEOUT &&
         (millis() - GDL90_Data_TimeMarker) < DATA_TIMEOUT);
}

bool GDL90_hasHeartBeat()
{
  return (GDL90_HeartBeat_TimeMarker > GDL90_EXP_TIME &&
         (millis() - GDL90_HeartBeat_TimeMarker) < GDL90_EXP_TIME);
}

bool GDL90_hasOwnShip()
{
  return (GDL90_OwnShip_TimeMarker > GDL90_EXP_TIME &&
         (millis() - GDL90_OwnShip_TimeMarker) < GDL90_EXP_TIME);
}
