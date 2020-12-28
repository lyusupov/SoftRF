/*
 * GDL90Helper.cpp
 * Copyright (C) 2016-2021 Linar Yusupov
 *
 * Inspired by Eric's Dey Python GDL-90 encoder:
 * https://github.com/etdey/gdl90
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
#include <lib_crc.h>
#include <protocol.h>

#include "../../system/SoC.h"
#include "GDL90.h"
#include "../../driver/GNSS.h"
#include "../../driver/EEPROM.h"
#include "../../driver/WiFi.h"
#include "../../TrafficHelper.h"
#include "../radio/Legacy.h"
#include "NMEA.h"

#if defined(ENABLE_AHRS)
#include "../../AHRS.h"
#endif /* ENABLE_AHRS */

#define ADDR_TO_HEX_STR(s, c) (s += ((c) < 0x10 ? "0" : "") + String((c), HEX))

static GDL90_Msg_HeartBeat_t HeartBeat;
static GDL90_Msg_Traffic_t Traffic;
static GDL90_Msg_OwnershipGeometricAltitude_t GeometricAltitude;

const char *GDL90_CallSign_Prefix[] = {
  [RF_PROTOCOL_LEGACY]    = "FL",
  [RF_PROTOCOL_OGNTP]     = "OG",
  [RF_PROTOCOL_P3I]       = "PA",
  [RF_PROTOCOL_ADSB_1090] = "AD",
  [RF_PROTOCOL_ADSB_UAT]  = "UA",
  [RF_PROTOCOL_FANET]     = "FA"
};

const uint8_t aircraft_type_to_gdl90[] PROGMEM = {
	GDL90_EMITTER_CATEGORY_NONE,
	GDL90_EMITTER_CATEGORY_GLIDER,
	GDL90_EMITTER_CATEGORY_LIGHT,
	GDL90_EMITTER_CATEGORY_ROTORCRAFT,
	GDL90_EMITTER_CATEGORY_SKYDIVER,
	GDL90_EMITTER_CATEGORY_LIGHT,
	GDL90_EMITTER_CATEGORY_ULTRALIGHT,
	GDL90_EMITTER_CATEGORY_ULTRALIGHT,
	GDL90_EMITTER_CATEGORY_LIGHT,
	GDL90_EMITTER_CATEGORY_SMALL,
	GDL90_EMITTER_CATEGORY_MANEUVERABLE,
	GDL90_EMITTER_CATEGORY_BALLOON,
	GDL90_EMITTER_CATEGORY_BALLOON,
	GDL90_EMITTER_CATEGORY_UAV,
	GDL90_EMITTER_CATEGORY_UNASSIGNED1,
	GDL90_EMITTER_CATEGORY_NONE
};

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

#if defined(DO_GDL90_FF_EXT)
/*
 * See https://www.foreflight.com/connect/spec/ for details
 */
const GDL90_Msg_FF_ID_t msgFFid = {
  .Sub_Id       = 0, /* 0 for ID, 1 for AHRS */
  .Version      = 1, /* Must be 1 */
  /* Device serial number is 0xFFFFFFFFFFFFFFFF for invalid */
  .SerialNum    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  .ShortName    = {'S', 'o', 'f', 't', 'R', 'F', ' ', ' ' },
  .LongName     = {'S', 'o', 'f', 't', 'R', 'F',
                    ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ' },
  .Capabilities = {0x00, 0x00, 0x00, 0x01}, /* MSL altitude for Ownship Geometric report */
};
#endif

/* convert a signed latitude to 2s complement ready for 24-bit packing */
static uint32_t makeLatitude(float latitude)
{
    int32_t int_lat;

    if (latitude > 90.0) {
      latitude = 90.0;
    }

    if (latitude < -90.0) {
      latitude = -90.0;
    }

    int_lat = (int) (latitude * (0x800000 / 180.0));

    if (int_lat < 0) {
      int_lat = (0x1000000 + int_lat) & 0xffffff;  /* 2s complement */  
    }

    return(int_lat);    
}

/* convert a signed longitude to 2s complement ready for 24-bit packing */
static uint32_t makeLongitude(float longitude)
{
    int32_t int_lon;

    if (longitude > 180.0) {
      longitude = 180.0;
    }

    if (longitude < -180.0) {
      longitude = -180.0;
    }

    int_lon = (int) (longitude * (0x800000 / 180.0));

    if (int_lon < 0) {
      int_lon = (0x1000000 + int_lon) & 0xffffff;  /* 2s complement */  
    }

    return(int_lon);    
}


static uint32_t pack24bit(uint32_t num)
{
  return( ((num & 0xff0000) >> 16) | (num & 0x00ff00) | ((num & 0xff) << 16) );
}

uint16_t GDL90_calcFCS(uint8_t msg_id, uint8_t *msg, int size)
{
  uint16_t crc16 = 0x0000;  /* seed value */

  crc16 = update_crc_gdl90(crc16, msg_id);

  for (int i=0; i < size; i++)
  {
    crc16 = update_crc_gdl90(crc16, msg[i]);
  }    

  return(crc16);
}

uint8_t *GDL90_EscapeFilter(uint8_t *buf, uint8_t *p, int size)
{
  while (size--) {
    if (*p != 0x7D && *p != 0x7E) {
      *buf++ = *p++;
    } else {
      *buf++ = 0x7D;
      *buf++ = *p++ ^ 0x20;
    }   
  }

  return (buf);
}

static void *msgHeartbeat()
{
  time_t ts = elapsedSecsToday(now());

  /* Status Byte 1 */
  HeartBeat.gnss_pos_valid  = isValidFix() ;
  HeartBeat.maint_reqd      = 0;
  HeartBeat.ident           = 0;
  HeartBeat.addr_type       = 0;
  HeartBeat.gnss_bat_low    = 0;
  HeartBeat.ratcs           = 0;
//HeartBeat.reserved1       = 0;
  HeartBeat.uat_init        = 1;

  /* Status Byte 2 */
  HeartBeat.time_stamp_ms   = (ts >> 16) & 1;
  HeartBeat.csa_req         = 0;
  HeartBeat.csa_not_avail   = 0;
//HeartBeat.reserved2       = 0;
//HeartBeat.reserved3       = 0;
//HeartBeat.reserved4       = 0;
//HeartBeat.reserved5       = 0;
  HeartBeat.utc_ok          = 0;

  HeartBeat.time_stamp      = (ts & 0xFFFF);   // LSB first
  HeartBeat.message_counts  = 0;

  return (&HeartBeat);
}

static void *msgType10and20(ufo_t *aircraft)
{
  int altitude;

  /*
   * The Altitude field "ddd" contains the pressure altitude
   * (referenced to 29.92 inches Hg), encoded using 25-foot resolution,
   * offset by 1,000 feet.
   * The 0xFFF value represents that the pressure altitude is invalid.
   * The minimum altitude that can be represented is -1,000 feet.
   * The maximum valid altitude is +101,350 feet.
   */

  /* If the aircraft's data has standard pressure altitude - make use it */
  if (aircraft->pressure_altitude != 0.0) {
    altitude = (int)(aircraft->pressure_altitude * _GPS_FEET_PER_METER);
  } else if (ThisAircraft.pressure_altitude != 0.0) {
    /* If this SoftRF unit is equiped with baro sensor - try to make an adjustment */
    float altDiff = ThisAircraft.pressure_altitude - ThisAircraft.altitude;
    altitude = (int)((aircraft->altitude + altDiff) * _GPS_FEET_PER_METER);
  } else {
    /* If there are no any choice - report GNSS AMSL altitude as pressure altitude */
    altitude = (int)(aircraft->altitude * _GPS_FEET_PER_METER);
  }
  altitude = (altitude + 1000) / 25; /* Resolution = 25 feet */

  int trackHeading = (int)(aircraft->course / (360.0 / 256)); /* convert to 1.4 deg single byte */

  if (altitude < 0) {
    altitude = 0;  
  }
  if (altitude > 0xffe) {
    altitude = 0xffe;  
  }
 
  uint8_t misc = 9;
  //altitude = 0x678;
  
  uint16_t horiz_vel = (uint16_t) aircraft->speed /* 0x123 */ ; /*  in knots */
  uint16_t vert_vel = (uint16_t) ((int16_t) (aircraft->vs / 64.0)) /* 0x456 */; /* in units of 64 fpm */

  Traffic.alert_status  = 0 /* 0x1 */;
  Traffic.addr_type     = 0 /* 0x2 */;
  Traffic.addr          = pack24bit(aircraft->addr) /* pack24bit(0x345678) */;
  Traffic.latitude      = pack24bit(makeLatitude(aircraft->latitude)) /* pack24bit(0x9abcde) */;
  Traffic.longitude     = pack24bit(makeLongitude(aircraft->longitude)) /* pack24bit(0xf12345) */;

  /*
   * workaround against "implementation dependant"
   * XTENSA's GCC bitmap layout in structures
   */
  Traffic.altitude      = ((altitude >> 4) & 0xFF) | (misc << 8);
  Traffic.misc          = (altitude & 0x00F);

  Traffic.nic           = 8 /* 0xa */;
  Traffic.nacp          = 8 /* 0xb */;

  /*
   * workaround against "implementation dependant"
   * XTENSA's GCC bitmap layout in structures
   */

  Traffic.horiz_vel = (((vert_vel >> 8) & 0xF) << 8)| (((horiz_vel >> 8) & 0xF) << 4) | ((horiz_vel >> 4) & 0xF) ;
  Traffic.vert_vel =  (((vert_vel >> 4) & 0xF) << 8) | ((vert_vel & 0xF) << 4) | (horiz_vel & 0xF) ;

  Traffic.track         = (trackHeading & 0xFF) /* 0x03 */;
  Traffic.emit_cat      = AT_TO_GDL90(aircraft->aircraft_type) /* 0x4 */;

  /*
   * When callsign is available - send it to a GDL90 client.
   * If it is not - generate a callsign substitute,
   * based upon a protocol ID and the ICAO address
   */
  if (strnlen((char *) aircraft->callsign, sizeof(aircraft->callsign)) > 0) {
    memcpy(Traffic.callsign, aircraft->callsign, sizeof(Traffic.callsign));
  } else {
    memcpy((char *)Traffic.callsign, GDL90_CallSign_Prefix[aircraft->protocol],
      strlen(GDL90_CallSign_Prefix[aircraft->protocol]));

    String str = "";

    ADDR_TO_HEX_STR(str, (aircraft->addr >> 16) & 0xFF);
    ADDR_TO_HEX_STR(str, (aircraft->addr >>  8) & 0xFF);
    ADDR_TO_HEX_STR(str, (aircraft->addr      ) & 0xFF);

    str.toUpperCase();
    memcpy((char *)Traffic.callsign + strlen(GDL90_CallSign_Prefix[aircraft->protocol]),
      str.c_str(), str.length());
  }

  Traffic.emerg_code    = 0 /* 0x5 */;
//Traffic.reserved      = 0;

  return (&Traffic);
}

static void *msgOwnershipGeometricAltitude(ufo_t *aircraft)
{
  uint16_t vfom = 0x000A;

#if 0
  /*
   * The Geo Altitude field is a 16-bit signed integer that represents
   * the geometric altitude (height above WGS-84 ellipsoid),
   * encoded using 5-foot resolution
   */
  uint16_t altitude = (int16_t)((aircraft->altitude + aircraft->geoid_separation) *
                        _GPS_FEET_PER_METER / 5);
#else
  /*
   * Vast majority of EFBs deviates from Rev A of GDL90 ICD (2007) specs
   * and uses MSL altitude here.
   * SkyDemon is the only known exception which uses WGS-84 altitude still.
   */
  uint16_t altitude = (int16_t)(aircraft->altitude * _GPS_FEET_PER_METER / 5);
#endif

  GeometricAltitude.geo_altitude  = ((altitude & 0x00FF) << 8) | ((altitude & 0xFF00) >> 8) ;
  GeometricAltitude.VFOM          = ((vfom & 0x00FF) << 8) | ((vfom & 0xFF00) >> 8);
  GeometricAltitude.vert_warning  = 0;

  return (&GeometricAltitude);
}

static size_t makeHeartbeat(uint8_t *buf)
{
  uint8_t *ptr = buf;
  uint8_t *msg = (uint8_t *) msgHeartbeat();
  uint16_t fcs = GDL90_calcFCS(GDL90_HEARTBEAT_MSG_ID, msg,
                               sizeof(GDL90_Msg_HeartBeat_t));
  uint8_t fcs_lsb, fcs_msb;
  
  fcs_lsb = fcs        & 0xFF;
  fcs_msb = (fcs >> 8) & 0xFF;

  *ptr++ = 0x7E; /* Start flag */
  *ptr++ = GDL90_HEARTBEAT_MSG_ID;
  ptr = GDL90_EscapeFilter(ptr, msg, sizeof(GDL90_Msg_HeartBeat_t));
  ptr = GDL90_EscapeFilter(ptr, &fcs_lsb, 1);
  ptr = GDL90_EscapeFilter(ptr, &fcs_msb, 1);
  *ptr++ = 0x7E; /* Stop flag */

  return(ptr-buf);
}

static size_t makeType10and20(uint8_t *buf, uint8_t id, ufo_t *aircraft)
{
  uint8_t *ptr = buf;
  uint8_t *msg = (uint8_t *) msgType10and20(aircraft);
  uint16_t fcs = GDL90_calcFCS(id, msg, sizeof(GDL90_Msg_Traffic_t));
  uint8_t fcs_lsb, fcs_msb;
  
  fcs_lsb = fcs        & 0xFF;
  fcs_msb = (fcs >> 8) & 0xFF;

  *ptr++ = 0x7E; /* Start flag */
  *ptr++ = id;
  ptr = GDL90_EscapeFilter(ptr, msg, sizeof(GDL90_Msg_Traffic_t));
  ptr = GDL90_EscapeFilter(ptr, &fcs_lsb, 1);
  ptr = GDL90_EscapeFilter(ptr, &fcs_msb, 1);
  *ptr++ = 0x7E; /* Stop flag */

  return(ptr-buf);
}

static size_t makeGeometricAltitude(uint8_t *buf, ufo_t *aircraft)
{
  uint8_t *ptr = buf;
  uint8_t *msg = (uint8_t *) msgOwnershipGeometricAltitude(aircraft);
  uint16_t fcs = GDL90_calcFCS(GDL90_OWNGEOMALT_MSG_ID, msg,
                               sizeof(GDL90_Msg_OwnershipGeometricAltitude_t));
  uint8_t fcs_lsb, fcs_msb;

  fcs_lsb = fcs        & 0xFF;
  fcs_msb = (fcs >> 8) & 0xFF;

  *ptr++ = 0x7E; /* Start flag */
  *ptr++ = GDL90_OWNGEOMALT_MSG_ID;
  ptr = GDL90_EscapeFilter(ptr, msg, sizeof(GDL90_Msg_OwnershipGeometricAltitude_t));
  ptr = GDL90_EscapeFilter(ptr, &fcs_lsb, 1);
  ptr = GDL90_EscapeFilter(ptr, &fcs_msb, 1);
  *ptr++ = 0x7E; /* Stop flag */

  return(ptr-buf);
}

#if defined(DO_GDL90_FF_EXT)

static size_t makeFFid(uint8_t *buf)
{
  uint8_t *ptr = buf;
  uint8_t *msg = (uint8_t *) &msgFFid;
  uint16_t fcs = GDL90_calcFCS(GDL90_FFEXT_MSG_ID, msg, sizeof(GDL90_Msg_FF_ID_t));
  uint8_t fcs_lsb, fcs_msb;

  fcs_lsb = fcs        & 0xFF;
  fcs_msb = (fcs >> 8) & 0xFF;

  *ptr++ = 0x7E; /* Start flag */
  *ptr++ = GDL90_FFEXT_MSG_ID;
  ptr = GDL90_EscapeFilter(ptr, msg, sizeof(GDL90_Msg_FF_ID_t));
  ptr = GDL90_EscapeFilter(ptr, &fcs_lsb, 1);
  ptr = GDL90_EscapeFilter(ptr, &fcs_msb, 1);
  *ptr++ = 0x7E; /* Stop flag */

  return(ptr-buf);
}
#endif

#define makeOwnershipReport(b,a)  makeType10and20(b, GDL90_OWNSHIP_MSG_ID, a)
#define makeTrafficReport(b,a)    makeType10and20(b, GDL90_TRAFFIC_MSG_ID, a)

static void GDL90_Out(byte *buf, size_t size)
{
  if (size > 0) {
    switch(settings->gdl90)
    {
    case GDL90_UART:
      if (SoC->UART_ops) {
        SoC->UART_ops->write(buf, size);
      } else {
        SerialOutput.write(buf, size);
      }
      break;
    case GDL90_UDP:
      {
        SoC->WiFi_transmit_UDP(GDL90_DST_PORT, buf, size);
      }
      break;
    case GDL90_USB:
      {
        if (SoC->USB_ops) {
          SoC->USB_ops->write(buf, size);
        }
      }
      break;
    case GDL90_BLUETOOTH:
      {
        if (SoC->Bluetooth_ops) {
          SoC->Bluetooth_ops->write(buf, size);
        }
      }
      break;
    case GDL90_TCP:
    case GDL90_OFF:
    default:
      break;
    }
  }
}

void GDL90_Export()
{
  size_t size;
  float distance;
  time_t this_moment = now();
  uint8_t *buf = (uint8_t *) (sizeof(UDPpacketBuffer) < UDP_PACKET_BUFSIZE ?
                              NMEABuffer : UDPpacketBuffer);

  if (settings->gdl90 != GDL90_OFF) {
    size = makeHeartbeat(buf);
    GDL90_Out(buf, size);

#if defined(DO_GDL90_FF_EXT)
    size = makeFFid(buf);
    GDL90_Out(buf, size);
#endif /* DO_GDL90_FF_EXT */

#if defined(ENABLE_AHRS)
    size = AHRS_GDL90(buf);
    GDL90_Out(buf, size);
#endif /* ENABLE_AHRS */

    if (isValidFix()) {
      size = makeOwnershipReport(buf, &ThisAircraft);
      GDL90_Out(buf, size);

      size = makeGeometricAltitude(buf, &ThisAircraft);
      GDL90_Out(buf, size);

      for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (Container[i].addr &&
           (this_moment - Container[i].timestamp) <= EXPORT_EXPIRATION_TIME) {

          distance = Container[i].distance;

          if (distance < ALARM_ZONE_NONE) {
            size = makeTrafficReport(buf, &Container[i]);
            GDL90_Out(buf, size);
          }
        }
      }
    }
  }
}
