/*
 *
 * Protocol_APRS.cpp
 * Encoder for Automatic Packet Reporting System radio protocol
 * Copyright (C) 2023 Linar Yusupov
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
#include <LibAPRSesp.h>

#include "../../../SoftRF.h"
#include "../../driver/RF.h"

const rf_proto_desc_t aprs_proto_desc = {
  "APRS",
  .type            = RF_PROTOCOL_APRS,
  .modulation_type = RF_MODULATION_TYPE_2FSK, /* Bell 202 AFSK */
  .preamble_type   = APRS_PREAMBLE_TYPE,
  .preamble_size   = APRS_PREAMBLE_SIZE,
  .syncword        = APRS_SYNCWORD,
  .syncword_size   = APRS_SYNCWORD_SIZE,
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_DIRECT,
  .payload_size    = APRS_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = APRS_CRC_TYPE,
  .crc_size        = APRS_CRC_SIZE,

  .bitrate         = RF_BITRATE_38400, /* 1200 */
  .deviation       = RF_FREQUENCY_DEVIATION_9_6KHZ, /* TBD */
  .whitening       = RF_WHITENING_NONE, /* TBD */
  .bandwidth       = RF_RX_BANDWIDTH_SS_50KHZ, /* TBD */

  .air_time        = APRS_AIR_TIME,

  .tm_type         = RF_TIMING_INTERVAL,
  .tx_interval_min = APRS_TX_INTERVAL_MIN,
  .tx_interval_max = APRS_TX_INTERVAL_MAX,
  .slot0           = {0, 0},
  .slot1           = {0, 0}
};

bool aprs_decode(void *pkt, ufo_t *this_aircraft, ufo_t *fop) {

  /* N/A */

  return false;
}

size_t aprs_encode(void *pkt, ufo_t *this_aircraft) {

  // Let's first set our latitude and longtitude.
  // These should be in NMEA format!
  APRS_setLat("5530.80N");
  APRS_setLon("01143.89E");
  
  // We can optionally set power/height/gain/directivity
  // information. These functions accept ranges
  // from 0 to 10, directivity 0 to 9.
  // See this site for a calculator:
  // http://www.aprsfl.net/phgr.php
  // LibAPRS will only add PHG info if all four variables
  // are defined!
  APRS_setPower(2);
  APRS_setHeight(4);
  APRS_setGain(7);
  APRS_setDirectivity(0);

  strcpy((char *) pkt, "NOT IN USE");

  return aprs_proto_desc.payload_size;
}
