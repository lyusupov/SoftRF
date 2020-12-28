/*
 * Protocol_FANET.cpp
 *
 * Encoder and decoder for open FANET radio protocol
 *
 * URL:
 *    Development -  https://github.com/3s1d/fanet-stm32
 *    Deprecated  -  https://github.com/3s1d/fanet
 *
 * Copyright (C) 2017-2021 Linar Yusupov
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

#include <stdint.h>

#include <protocol.h>
#include <TimeLib.h>

#include "../../../SoftRF.h"
#include "../../driver/RF.h"

const rf_proto_desc_t fanet_proto_desc = {
  "FANET",
  .type             = RF_PROTOCOL_FANET,
  .modulation_type  = RF_MODULATION_TYPE_LORA,
  .preamble_type    = 0 /* INVALID FOR LORA */,
  .preamble_size    = 0 /* INVALID FOR LORA */,
#if defined(FANET_DEPRECATED)
  .syncword         = { 0x12 },  // sx127x default value, valid for FANET
#else
  .syncword         = { 0xF1 },  // FANET+
#endif
  .syncword_size    = 1,
  .net_id           = 0x0000, /* not in use */
  .payload_type     = RF_PAYLOAD_DIRECT,
  .payload_size     = FANET_PAYLOAD_SIZE,
  .payload_offset   = 0,
  .crc_type         = RF_CHECKSUM_TYPE_NONE, /* LoRa packet has built-in CRC */
  .crc_size         = 0 /* INVALID FOR LORA */,
  .bitrate          = DR_SF7B /* CR_5 BW_250 SF_7 */,

  .deviation        = 0 /* INVALID FOR LORA */,
  .whitening        = RF_WHITENING_NONE,
  .bandwidth        = 0, /* INVALID FOR LORA */

  .tx_interval_min  = FANET_TX_INTERVAL_MIN,
  .tx_interval_max  = FANET_TX_INTERVAL_MAX
};

const uint8_t aircraft_type_to_fanet[] PROGMEM = {
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_GLIDER,
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_HELICOPTER,
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_HANGGLIDER,
	FANET_AIRCRAFT_TYPE_PARAGLIDER,
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_BALLOON,
	FANET_AIRCRAFT_TYPE_BALLOON,
	FANET_AIRCRAFT_TYPE_UAV,
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_OTHER
};

const uint8_t aircraft_type_from_fanet[] PROGMEM = {
	AIRCRAFT_TYPE_UNKNOWN,
	AIRCRAFT_TYPE_PARAGLIDER,
	AIRCRAFT_TYPE_HANGGLIDER,
	AIRCRAFT_TYPE_BALLOON,
	AIRCRAFT_TYPE_GLIDER,
	AIRCRAFT_TYPE_POWERED,
	AIRCRAFT_TYPE_HELICOPTER,
	AIRCRAFT_TYPE_UAV
};

#define AT_TO_FANET(x)  (x > 15 ? \
   FANET_AIRCRAFT_TYPE_OTHER : pgm_read_byte(&aircraft_type_to_fanet[x]))

#define AT_FROM_FANET(x)  (x > 7 ? \
   AIRCRAFT_TYPE_UNKNOWN : pgm_read_byte(&aircraft_type_from_fanet[x]))

#if defined(FANET_DEPRECATED)
/* ------------------------------------------------------------------------- */
/*
 *
 *  Created on: 30 Sep 2016
 *      Author: sid
 */
static uint16_t coord2payload_compressed(float deg)
{
	float deg_round =  roundf(deg);
	bool deg_odd = ((int)deg_round) & 1;
	const float decimal = deg-deg_round;
	const int dec_int = constrain((int)(decimal*32767), -16383, 16383);

	return ((dec_int&0x7FFF) | (!!deg_odd<<15));
}
/* ------------------------------------------------------------------------- */
/*
 *  Created on: 06 Dec 2017
 *      Author: Linar Yusupov
 */
static float payload_compressed2coord(uint16_t payload, float ref_deg)
{
  float deg;
  bool deg_odd = (payload >> 15) & 0x1;
  unsigned int dec_uint = payload & 0x7FFF;

  float ref_deg_round =  roundf(ref_deg);
  bool ref_deg_odd = ((int)ref_deg_round) & 1;

  int dec_int = 0;
  if (dec_uint <= 0x3FFF) {
      dec_int = (int) dec_uint;
  } else if (dec_uint >= 0x4001) {
    dec_int = (int)-1 - (int) (dec_uint ^ 0x7FFF);
  }

  float decimal = (float) dec_int / 32767.0;

  if (deg_odd == ref_deg_odd) {
    deg = ref_deg_round + decimal;
  } else {
    if (decimal < 0) {
      deg = ref_deg_round + 1 + decimal;
    } else {
      deg = ref_deg_round - 1 + decimal;
    }
  }

  return deg;
}
/* ------------------------------------------------------------------------- */
#else

/* ------------------------------------------------------------------------- */
/*
 *
 *  Created on: 30 Sep 2016
 *      Author: sid
 */
static void coord2payload_absolut(float lat, float lon, uint8_t *buf)
{
	if (buf == NULL)
		return;

	int32_t lat_i = roundf(lat * 93206.0f);
	int32_t lon_i = roundf(lon * 46603.0f);

	buf[0] = ((uint8_t*)&lat_i)[0];
	buf[1] = ((uint8_t*)&lat_i)[1];
	buf[2] = ((uint8_t*)&lat_i)[2];

	buf[3] = ((uint8_t*)&lon_i)[0];
	buf[4] = ((uint8_t*)&lon_i)[1];
	buf[5] = ((uint8_t*)&lon_i)[2];
}
/* ------------------------------------------------------------------------- */
/*
 *  Created on: 06 Dec 2017
 *      Author: Linar Yusupov
 */
static void payload_absolut2coord(float *lat, float *lon, uint8_t *buf)
{
  int32_t lat_i = 0;
  int32_t lon_i = 0;

  if (buf == NULL || lat == NULL || lon == NULL)
    return;

  ((uint8_t*)&lat_i)[0] = buf[0];
  ((uint8_t*)&lat_i)[1] = buf[1];
  ((uint8_t*)&lat_i)[2] = buf[2];
  ((uint8_t*)&lat_i)[3] = buf[2] & 0x80 ? 0xFF : 0x00;

  ((uint8_t*)&lon_i)[0] = buf[3];
  ((uint8_t*)&lon_i)[1] = buf[4];
  ((uint8_t*)&lon_i)[2] = buf[5];
  ((uint8_t*)&lon_i)[3] = buf[5] & 0x80 ? 0xFF : 0x00;

  *lat = (float) lat_i / 93206.0f;
  *lon = (float) lon_i / 46603.0f;
}

/* ------------------------------------------------------------------------- */
#endif

bool fanet_decode(void *fanet_pkt, ufo_t *this_aircraft, ufo_t *fop) {

  fanet_packet_t *pkt = (fanet_packet_t *) fanet_pkt;
  unsigned int altitude;
  uint8_t speed_byte, climb_byte, offset_byte;
  int speed_int, climb_int, offset_int;
  bool rval = false;

  if (pkt->ext_header == 0 && pkt->type == 1 ) {  /* Tracking  */

    /* ignore this device own (relayed) packets */
    if (pkt->vendor  == SOFRF_FANET_VENDOR_ID &&
        pkt->address == (this_aircraft->addr & 0xFFFF) /* && */
        /* pkt->forward == 1 */) {
      return rval;
    }

    fop->protocol = RF_PROTOCOL_FANET;
    fop->addr     = (pkt->vendor << 16) | pkt->address;

#if defined(FANET_DEPRECATED)
    fop->latitude  = payload_compressed2coord(pkt->latitude, this_aircraft->latitude);
    fop->longitude = payload_compressed2coord(pkt->longitude, this_aircraft->longitude);
#else
    payload_absolut2coord(&(fop->latitude), &(fop->longitude),
      ((uint8_t *) pkt) + FANET_HEADER_SIZE);
#endif

    altitude = ((pkt->altitude_msb << 8) | pkt->altitude_lsb);
    if (pkt->altitude_scale) {
      altitude = altitude * 4 /* -2 */;
    }
    fop->altitude = (float) altitude;

    fop->aircraft_type = AT_FROM_FANET(pkt->aircraft_type);
    fop->course = (float) pkt->heading * 360.0 / 256.0;

    speed_byte = pkt->speed;
    speed_int = (int) (speed_byte | (speed_byte & (1<<6) ? 0xFFFFFF80U : 0));

    if (pkt->speed_scale) {
      speed_int *= 5 /* -2 */;
    }
    fop->speed = ((float) speed_int) / (2 * _GPS_KMPH_PER_KNOT);

    climb_byte = pkt->climb;
    climb_int = (int) (climb_byte | (climb_byte & (1<<6) ? 0xFFFFFF80U : 0));

    if (pkt->climb_scale) {
      climb_int *= 5 /* +-2 */;
    }
    fop->vs = ((float)climb_int) * (_GPS_FEET_PER_METER * 6.0);

#if defined(FANET_NEXT)
    offset_byte = pkt->qne_offset;
    offset_int = (int) (offset_byte | (offset_byte & (1<<6) ? 0xFFFFFF80U : 0));

    if (pkt->qne_scale) {
      offset_int *= 4;
    }

    fop->pressure_altitude = fop->altitude + (float) offset_int;
#endif

    fop->addr_type = ADDR_TYPE_FANET;
    fop->timestamp = this_aircraft->timestamp;

    fop->stealth = 0;
    fop->no_track = !(pkt->track_online);

    fop->ns[0] = 0; fop->ns[1] = 0;
    fop->ns[2] = 0; fop->ns[3] = 0;
    fop->ew[0] = 0; fop->ew[1] = 0;
    fop->ew[2] = 0; fop->ew[3] = 0;
#if 0
    Serial.print(fop->addr, HEX);
    Serial.print(',');
    Serial.print(fop->aircraft_type, HEX);
    Serial.print(',');
    Serial.print(fop->latitude, 6);
    Serial.print(',');
    Serial.print(fop->longitude, 6);
    Serial.print(',');
    Serial.print(fop->altitude);
    Serial.print(',');
    Serial.print(fop->speed);
    Serial.print(',');
    Serial.print(fop->course);
    Serial.println();
    Serial.flush();
#endif
    rval = true;
  }

  return rval;
}

size_t fanet_encode(void *fanet_pkt, ufo_t *this_aircraft) {

  uint32_t id = this_aircraft->addr;
  float lat = this_aircraft->latitude;
  float lon = this_aircraft->longitude;
  int16_t alt = (int16_t) this_aircraft->altitude;
  unsigned int aircraft_type =  this_aircraft->aircraft_type;
  float speed = this_aircraft->speed * _GPS_KMPH_PER_KNOT;
  float climb = this_aircraft->vs / (_GPS_FEET_PER_METER * 60.0);
  float heading = this_aircraft->course;
  float turnrate = 0;
  int16_t alt_diff = this_aircraft->pressure_altitude == 0 ? 0 :
          (int16_t) (this_aircraft->pressure_altitude - this_aircraft->altitude);

  fanet_packet_t *pkt = (fanet_packet_t *) fanet_pkt;

  pkt->ext_header     = 0;
  pkt->forward        = 1;
  pkt->type           = 1;  /* Tracking  */

  pkt->vendor         = SOFRF_FANET_VENDOR_ID;
  pkt->address        = id & 0xFFFF;

#if defined(FANET_DEPRECATED)
  pkt->latitude       = coord2payload_compressed(lat);
  pkt->longitude      = coord2payload_compressed(lon);
#else
  coord2payload_absolut(lat, lon, ((uint8_t *) pkt) + FANET_HEADER_SIZE);
#endif

  pkt->track_online   = (this_aircraft->no_track ? 0 : 1);
  pkt->aircraft_type  = AT_TO_FANET(aircraft_type);

  int altitude        = constrain(alt, 0, 8190);
  pkt->altitude_scale = altitude > 2047 ? (altitude = (altitude + 2) / 4, 1) : 0;
  pkt->altitude_msb   = (altitude & 0x700) >> 8;
  pkt->altitude_lsb   = (altitude & 0x0FF);

  int speed2          = constrain((int)roundf(speed * 2.0f), 0, 635);
  if(speed2 > 127) {
    pkt->speed_scale  = 1;
    pkt->speed        = ((speed2 + 2) / 5);
  } else {
    pkt->speed_scale  = 0;
    pkt->speed        = speed2 & 0x7F;
  }

  int climb10         = constrain((int)roundf(climb * 10.0f), -315, 315);
  if(climb10 > 63) {
    pkt->climb_scale  = 1;
    pkt->climb        = ((climb10 + (climb10 >= 0 ? 2 : -2)) / 5);
  } else {
    pkt->climb_scale  = 0;
    pkt->climb        = climb10 & 0x7F;
  }

  pkt->heading        = constrain((int)roundf(heading * 256.0f)/360.0f, 0, 255);

  int turnr4          = constrain((int)roundf(turnrate * 4.0f), -255, 255);
  if(abs(turnr4) > 63) {
    pkt->turn_scale   = 1;
    pkt->turn_rate    = ((turnr4 + (turnr4 >= 0 ? 2 : -2)) / 4);
  } else {
    pkt->turn_scale   = 0;
    pkt->turn_rate    = turnr4 & 0x7F;
  }

#if defined(FANET_NEXT)
  int16_t offset      = constrain(alt_diff, -254, 254);
  if(abs(offset) > 63) {
    pkt->qne_scale    = 1;
    pkt->qne_offset   = ((offset + (offset >= 0 ? 2 : -2)) / 4);
  } else {
    pkt->qne_scale    = 0;
    pkt->qne_offset   = offset & 0x7F;
  }
#endif

  return sizeof(fanet_packet_t);
}
