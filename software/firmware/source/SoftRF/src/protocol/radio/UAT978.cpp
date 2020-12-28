/*
 *
 * Protocol_UAT978.cpp
 * Decoder for UAT 978 MHz ADS-B radio protocol
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

#include <stdint.h>

#include <protocol.h>

#include "../../../SoftRF.h"
#include "../../driver/RF.h"
#include "../data/GDL90.h"

const rf_proto_desc_t uat978_proto_desc = {
  "UAT",
  .type            = RF_PROTOCOL_ADSB_UAT,
  .modulation_type = RF_MODULATION_TYPE_2FSK,
  .preamble_type   = UAT978_PREAMBLE_TYPE,
  .preamble_size   = UAT978_PREAMBLE_SIZE,
  .syncword        = UAT978_SYNCWORD,
  .syncword_size   = UAT978_SYNCWORD_SIZE,
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_DIRECT,
  .payload_size    = UAT978_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = UAT978_CRC_TYPE,
  .crc_size        = UAT978_CRC_SIZE,

  .bitrate         = RF_BITRATE_1042KBPS,
  .deviation       = RF_FREQUENCY_DEVIATION_625KHZ,
  .whitening       = RF_WHITENING_NONE,
  .bandwidth       = RF_RX_BANDWIDTH_SS_1567KHZ,

  .tx_interval_min  = UAT978_TX_INTERVAL_MIN,
  .tx_interval_max  = UAT978_TX_INTERVAL_MAX
};

static struct uat_adsb_mdb mdb;

bool uat978_decode(void *pkt, ufo_t *this_aircraft, ufo_t *fop) {

  uat_decode_adsb_mdb((uint8_t *) pkt, &mdb);

  fop->protocol = RF_PROTOCOL_ADSB_UAT;

  fop->addr = mdb.address;
  fop->latitude = mdb.lat;
  fop->longitude = mdb.lon;

  if (mdb.altitude_type == ALT_GEO) {
    fop->altitude = mdb.altitude / _GPS_FEET_PER_METER;          /* TBD */
  }
  if (mdb.altitude_type == ALT_BARO) {
    fop->pressure_altitude = mdb.altitude / _GPS_FEET_PER_METER; /* TBD */
  }

  fop->aircraft_type = GDL90_TO_AT(mdb.emitter_category);
  fop->course = mdb.track;
  fop->speed = mdb.speed;
  fop->vs = mdb.vert_rate;
  fop->hdop = 0;                                                 /* TBD */

  fop->addr_type = ADDR_TYPE_ICAO;
  fop->timestamp = this_aircraft->timestamp;

  fop->stealth = false;
  fop->no_track = false;
  fop->ns[0] = 0; fop->ns[1] = 0;
  fop->ns[2] = 0; fop->ns[3] = 0;
  fop->ew[0] = 0; fop->ew[1] = 0;
  fop->ew[2] = 0; fop->ew[3] = 0;

  /* sizeof(mdb.callsign) = 9 ; sizeof(fop->callsign) = 8 */
  memcpy(fop->callsign, mdb.callsign, sizeof(fop->callsign));

  return true;
}

size_t uat978_encode(void *pkt, ufo_t *this_aircraft) {

  /* No Tx on 978 MHz until the system will eventually become certified */

  return (0);
}
