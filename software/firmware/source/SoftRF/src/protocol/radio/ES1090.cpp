/*
 *
 * Protocol_ES1090.cpp
 * Decoder for Extended Squitter 1090 MHz ADS-B radio protocol
 * Copyright (C) 2021-2022 Linar Yusupov
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

#include "../../../SoftRF.h"
#include "../../driver/RF.h"
#include "../data/GDL90.h"

const rf_proto_desc_t es1090_proto_desc = {
  "ES",
  .type            = RF_PROTOCOL_ADSB_1090,
  .modulation_type = RF_MODULATION_TYPE_PPM,
  .preamble_type   = ES1090_PREAMBLE_TYPE,
  .preamble_size   = ES1090_PREAMBLE_SIZE,
  .syncword        = ES1090_SYNCWORD,
  .syncword_size   = ES1090_SYNCWORD_SIZE,
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_DIRECT,
  .payload_size    = ES1090_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = ES1090_CRC_TYPE,
  .crc_size        = ES1090_CRC_SIZE,

  .bitrate         = RF_BITRATE_1042KBPS,
  .deviation       = RF_FREQUENCY_DEVIATION_NONE,
  .whitening       = RF_WHITENING_NONE,
  .bandwidth       = RF_RX_BANDWIDTH_SS_1567KHZ,

  .air_time        = ES1090_AIR_TIME,

  .tm_type         = RF_TIMING_INTERVAL,
  .tx_interval_min = ES1090_TX_INTERVAL_MIN,
  .tx_interval_max = ES1090_TX_INTERVAL_MAX,
  .slot0           = {0, 0},
  .slot1           = {0, 0}
};

bool es1090_decode(void *pkt, ufo_t *this_aircraft, ufo_t *fop) {

  struct mode_s_aircraft *a = (struct mode_s_aircraft *) pkt;

  fop->protocol  = RF_PROTOCOL_ADSB_1090;

  fop->addr      = a->addr;
  fop->latitude  = a->lat;
  fop->longitude = a->lon;

  if (a->unit == MODE_S_UNIT_FEET) {
    fop->pressure_altitude = a->altitude / _GPS_FEET_PER_METER;
  } else {
    fop->pressure_altitude = a->altitude;
  }

  if (this_aircraft->pressure_altitude != 0.0) {
    fop->altitude = fop->pressure_altitude -
                    this_aircraft->pressure_altitude +
                    this_aircraft->altitude;
  } else {
    fop->altitude = fop->pressure_altitude;
  }

  fop->aircraft_type = GDL90_TO_AT(a->aircraft_type);
  fop->course = a->track;
  fop->speed = a->speed;
//  fop->vs = a->vert_rate;                                     /* TBD */
  fop->hdop = 0;                                                /* TBD */

  fop->addr_type = ADDR_TYPE_ICAO;
  fop->timestamp = this_aircraft->timestamp;

  fop->stealth = false;
  fop->no_track = false;
  fop->ns[0] = 0; fop->ns[1] = 0;
  fop->ns[2] = 0; fop->ns[3] = 0;
  fop->ew[0] = 0; fop->ew[1] = 0;
  fop->ew[2] = 0; fop->ew[3] = 0;

  /* sizeof(mdb.callsign) = 9 ; sizeof(fop->callsign) = 8 */
  memcpy(fop->callsign, a->flight, sizeof(fop->callsign));

  return true;
}

size_t es1090_encode(void *pkt, ufo_t *this_aircraft) {

  /* No Tx on 1090 MHz until the system will eventually become certified */

  return (0);
}
