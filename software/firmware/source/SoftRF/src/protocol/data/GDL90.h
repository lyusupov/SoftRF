/*
 * GDL90Helper.h
 * Copyright (C) 2016-2021 Linar Yusupov
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

#ifndef GDL90HELPER_H
#define GDL90HELPER_H

#include "../../../SoftRF.h"

enum
{
	GDL90_EMITTER_CATEGORY_NONE,
	GDL90_EMITTER_CATEGORY_LIGHT,
	GDL90_EMITTER_CATEGORY_SMALL,
	GDL90_EMITTER_CATEGORY_LARGE,
	GDL90_EMITTER_CATEGORY_HVLARGE,
	GDL90_EMITTER_CATEGORY_HEAVY,
	GDL90_EMITTER_CATEGORY_MANEUVERABLE,
	GDL90_EMITTER_CATEGORY_ROTORCRAFT,
	GDL90_EMITTER_CATEGORY_UNASSIGNED1,
	GDL90_EMITTER_CATEGORY_GLIDER,
	GDL90_EMITTER_CATEGORY_BALLOON,
	GDL90_EMITTER_CATEGORY_SKYDIVER,
	GDL90_EMITTER_CATEGORY_ULTRALIGHT,
	GDL90_EMITTER_CATEGORY_UNASSIGNED2,
	GDL90_EMITTER_CATEGORY_UAV,
	GDL90_EMITTER_CATEGORY_SPACESHIP
};

enum
{
	GDL90_OFF,
	GDL90_UART,
	GDL90_UDP,
	GDL90_TCP,
	GDL90_USB,
	GDL90_BLUETOOTH
};

typedef struct GDL90_Message {
  uint8_t   flag_start;
  uint8_t   message_id;
  void      *message_body;
  uint16_t  fcs;                 /* LSB first */
  uint8_t   flag_stop;
} GGDL90_Message_t;

#define GDL90_HEARTBEAT_MSG_ID  0

typedef struct GDL90_Msg_HeartBeat {

  /* Status Byte 1 */
  unsigned int uat_init:1;
  unsigned int reserved1:1;
  unsigned int ratcs:1;
  unsigned int gnss_bat_low:1;
  unsigned int addr_type:1;
  unsigned int ident:1;
  unsigned int maint_reqd:1;
  unsigned int gnss_pos_valid:1;

  /* Status Byte 2 */
  unsigned int utc_ok:1;
  unsigned int reserved5:1;
  unsigned int reserved4:1;
  unsigned int reserved3:1;
  unsigned int reserved2:1;
  unsigned int csa_not_avail:1;
  unsigned int csa_req:1;
  unsigned int time_stamp_ms:1;

  unsigned int time_stamp:16;    // LSB first

  unsigned int message_counts:16;

} __attribute__((packed)) GDL90_Msg_HeartBeat_t;


#define GDL90_OWNSHIP_MSG_ID  10
#define GDL90_TRAFFIC_MSG_ID  20

typedef struct GDL90_Msg_Traffic {

  unsigned int addr_type:4;
  unsigned int alert_status:4;

  unsigned int addr:24;        // MSB first
  unsigned int latitude:24;    // MSB first
  unsigned int longitude:24;   // MSB first

  unsigned int altitude:12;
  unsigned int misc:4;

  unsigned int nacp:4;
  unsigned int nic:4;

  unsigned int horiz_vel:12; /* in knots */
  unsigned int vert_vel:12;  /* x 64 fpm */

  unsigned int track:8;
  unsigned int emit_cat:8;

  uint8_t      callsign[8];

  unsigned int reserved:4;
  unsigned int emerg_code:4;

} __attribute__((packed)) GDL90_Msg_Traffic_t;

#define GDL90_OWNGEOMALT_MSG_ID  11

typedef struct GDL90_Msg_OwnershipGeometricAltitude {

  unsigned int geo_altitude :16;  // MSB first

  unsigned int VFOM         :15;
  unsigned int vert_warning :1;

} __attribute__((packed)) GDL90_Msg_OwnershipGeometricAltitude_t;

#if defined(DO_GDL90_FF_EXT)

#define GDL90_FFEXT_MSG_ID  0x65

typedef struct GDL90_Msg_FF_ID {

  uint8_t   Sub_Id; /* 0 for ID, 1 for AHRS */
  uint8_t   Version;

  uint8_t   SerialNum[8];
  char      ShortName[8];
  uint8_t   LongName[16];

  uint8_t   Capabilities[4];
} GDL90_Msg_FF_ID_t;

#endif

#define AT_TO_GDL90(x)  ((x) > 15 ? \
   GDL90_EMITTER_CATEGORY_NONE : pgm_read_byte(&aircraft_type_to_gdl90[(x)]))

#define GDL90_TO_AT(x)  ((x) > 15 ? \
   AIRCRAFT_TYPE_UNKNOWN : pgm_read_byte(&gdl90_to_aircraft_type[(x)]))

extern const uint8_t aircraft_type_to_gdl90[] PROGMEM;
extern const uint8_t gdl90_to_aircraft_type[] PROGMEM;
extern const char *GDL90_CallSign_Prefix[];

void GDL90_Export(void);
uint16_t GDL90_calcFCS(uint8_t, uint8_t *, int);
uint8_t *GDL90_EscapeFilter(uint8_t *, uint8_t *, int);

#endif /* GDL90HELPER_H */