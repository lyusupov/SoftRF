/*
 * legacy_codec.h
 * Copyright (C) 2014-2015 Stanislaw Pusep
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

#ifndef LEGACY_CODEC

#define LEGACY_CODEC

#define LEGACY_KEY1 { 0xe43276df, 0xdca83759, 0x9802b8ac, 0x4675a56b }
#define LEGACY_KEY2 0x045d9f3b
#define LEGACY_KEY3 0x87b562f4

enum
{
	ADDR_TYPE_RANDOM,
	ADDR_TYPE_ICAO,
	ADDR_TYPE_FLARM,
	ADDR_TYPE_OGN
};

enum
{
	AIRCRAFT_TYPE_UNKNOWN,
	AIRCRAFT_TYPE_GLIDER,
	AIRCRAFT_TYPE_TOWPLANE,
	AIRCRAFT_TYPE_HELICOPTER,
	AIRCRAFT_TYPE_PARACHUTE,
	AIRCRAFT_TYPE_DROPPLANE,
	AIRCRAFT_TYPE_HANGGLIDER,
	AIRCRAFT_TYPE_PARAGLIDER,
	AIRCRAFT_TYPE_POWERED,
	AIRCRAFT_TYPE_JET,
	AIRCRAFT_TYPE_UFO,
	AIRCRAFT_TYPE_BALLOON,
	AIRCRAFT_TYPE_ZEPPELIN,
	AIRCRAFT_TYPE_UAV,
	AIRCRAFT_TYPE_RESERVED,
	AIRCRAFT_TYPE_STATIC
};

enum
{
	ALARM_LEVEL_NONE,
	ALARM_LEVEL_LOW,
	ALARM_LEVEL_IMPORTANT,
	ALARM_LEVEL_URGENT
};

enum
{
	ALARM_TYPE_TRAFFIC,
	ALARM_TYPE_SILENT,
	ALARM_TYPE_AIRCRAFT,
	ALARM_TYPE_OBSTACLE
};

enum
{
	GNSS_STATUS_NONE,
	GNSS_STATUS_3D_GROUND,
	GNSS_STATUS_3D_MOVING
};

enum
{
	POWER_STATUS_BAD,
	POWER_STATUS_GOOD
};

enum
{
	TX_STATUS_OFF,
	TX_STATUS_ON
};

typedef struct {
    /********************/
    unsigned int addr:24;
    unsigned int _unk0:1;
    unsigned int parity:3;
    unsigned int addr_type:3;
    unsigned int _unk1:1;
    // unsigned int magic:8;
    /********************/
    int vs:10;
    unsigned int _unk2:3;
    unsigned int stealth:1;
    unsigned int no_track:1;
    unsigned int _unk3:1;
    unsigned int gps:12;
    unsigned int aircraft_type:4;
    /********************/
    unsigned int lat:19;
    unsigned int alt:13;
    /********************/
    unsigned int lon:20;
    unsigned int _unk4:10;
    unsigned int vsmult:2;
    /********************/
    int8_t ns[4];
    int8_t ew[4];
    /********************/
} legacy_packet;

//#define MYADDR  Device_Id

bool legacy_decode(legacy_packet *pkt, float ref_lat, float ref_lon,
  int16_t ref_alt, uint32_t timestamp, ufo_t *fop);
legacy_packet *legacy_encode(legacy_packet *pkt, uint32_t id, float ref_lat,
  float ref_lon, int16_t ref_alt, uint32_t timestamp, unsigned int aircraft_type);

#endif
