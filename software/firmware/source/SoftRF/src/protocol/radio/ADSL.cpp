/*
 * Protocol_ADSL.cpp
 *
 * Encoder and decoder for ADS-L SRD-860 radio protocol
 * Copyright (C) 2024 Linar Yusupov
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

/*
 * Issue 1:
 * https://www.easa.europa.eu/sites/default/files/dfu/ads-l_4_srd860_issue_1.pdf
 */

#include <stdint.h>

#include <protocol.h>
#include <TimeLib.h>

#include "../../../SoftRF.h"
#include "../../driver/RF.h"
#include "../../driver/EEPROM.h"

const rf_proto_desc_t adsl_proto_desc = {
  .name            = {'A','D','S','-','L', 0},
  .type            = RF_PROTOCOL_ADSL_860,
  .modulation_type = RF_MODULATION_TYPE_2FSK,
  .preamble_type   = ADSL_PREAMBLE_TYPE,
  .preamble_size   = ADSL_PREAMBLE_SIZE,
  .syncword        = ADSL_SYNCWORD,
  .syncword_size   = ADSL_SYNCWORD_SIZE,
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_INVERTED,
  .payload_size    = ADSL_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = ADSL_CRC_TYPE,
  .crc_size        = ADSL_CRC_SIZE,

  .bitrate         = RF_BITRATE_100KBPS,
  .deviation       = RF_FREQUENCY_DEVIATION_50KHZ,
  .whitening       = RF_WHITENING_MANCHESTER,
  .bandwidth       = RF_RX_BANDWIDTH_SS_125KHZ,

  .air_time        = ADSL_AIR_TIME,

#if defined(USE_TIME_SLOTS)
  .tm_type         = RF_TIMING_2SLOTS_PPS_SYNC,
#else
  .tm_type         = RF_TIMING_INTERVAL,
#endif
  .tx_interval_min = ADSL_TX_INTERVAL_MIN,
  .tx_interval_max = ADSL_TX_INTERVAL_MAX,
  .slot0           = {400,  800},
  .slot1           = {800, 1200}
};

const rf_proto_desc_t adsl_proto_desc_oband = {
  .name            = {'A','D','S','-','L', 0},
  .type            = RF_PROTOCOL_ADSL_860,
  .modulation_type = RF_MODULATION_TYPE_2FSK,
  .preamble_type   = P3I_PREAMBLE_TYPE,
  .preamble_size   = P3I_PREAMBLE_SIZE,
  .syncword        = {0x2d, 0xd4},
  .syncword_size   = P3I_SYNCWORD_SIZE,
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_DIRECT,
  .payload_size    = ADSL_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = ADSL_CRC_TYPE,
  .crc_size        = ADSL_CRC_SIZE,

  .bitrate         = RF_BITRATE_38400,
  .deviation       = P3I_FDEV,
  .whitening       = RF_WHITENING_NONE,
  .bandwidth       = P3I_BANDWIDTH,

  .air_time        = P3I_AIR_TIME,

  .tm_type         = RF_TIMING_INTERVAL,
  .tx_interval_min = P3I_TX_INTERVAL_MIN,
  .tx_interval_max = P3I_TX_INTERVAL_MAX,
  .slot0           = {0, 0},
  .slot1           = {0, 0}
};

static GPS_Position pos;
static ADSL_Packet  r __attribute__((aligned(sizeof(uint32_t)))); /* Rx */
static ADSL_Packet  t __attribute__((aligned(sizeof(uint32_t)))); /* Tx */

void adsl_init()
{
  pos.Clear();
}

void adsl_fini()
{

}

bool adsl_decode(void *pkt, ufo_t *this_aircraft, ufo_t *fop) {

  uint8_t *ptr = (uint8_t *) pkt;

  r.Init();
  r.Version = *ptr;
  ptr += sizeof(ADSL_Packet::Version);

  for (int Idx=0; Idx<5; Idx++) {
    r.Word[Idx] = r.get4bytes(ptr + Idx * sizeof(r.Word[0]));
  }

  r.Descramble();

  fop->protocol  = RF_PROTOCOL_ADSL_860;

  fop->addr      = r.getAddress();
  fop->latitude  = r.FNTtoFloat(r.getLat());
  fop->longitude = r.FNTtoFloat(r.getLon());
  fop->altitude  = (float) r.getAlt();
  fop->pressure_altitude = 0; /* TBD */
  fop->aircraft_type = r.getAcftTypeOGN();
  fop->course    = (45.0/0x40) * r.getTrack();
  fop->speed     = (0.25 * r.getSpeed()) / _GPS_MPS_PER_KNOT;
  fop->vs        = (0.125 * r.getClimb()) * (_GPS_FEET_PER_METER * 60.0);
  fop->hdop      = r.getHorAccur();

  fop->addr_type = r.getAddrTypeOGN();
  fop->timestamp = this_aircraft->timestamp;

  fop->stealth   = 0;
  fop->no_track  = 0;
  fop->ns[0] = 0; fop->ns[1] = 0;
  fop->ns[2] = 0; fop->ns[3] = 0;
  fop->ew[0] = 0; fop->ew[1] = 0;
  fop->ew[2] = 0; fop->ew[3] = 0;

  return true;
}

size_t adsl_encode(void *pkt, ufo_t *this_aircraft) {

  pos.Latitude  = (int32_t) (this_aircraft->latitude * 600000);
  pos.Longitude = (int32_t) (this_aircraft->longitude * 600000);
  pos.Altitude  = (int32_t) (this_aircraft->altitude * 10);
  if (this_aircraft->pressure_altitude != 0.0) {
    pos.StdAltitude = (int32_t) (this_aircraft->pressure_altitude * 10);
    pos.ClimbRate = this_aircraft->stealth ?
                    0 : (int32_t) (this_aircraft->vs / (_GPS_FEET_PER_METER * 6.0));
  }
  pos.Heading = (int16_t) (this_aircraft->course * 10);
  pos.Speed   = (int16_t) (this_aircraft->speed * 10 * _GPS_MPS_PER_KNOT);
  pos.HDOP    = (uint8_t) (this_aircraft->hdop / 10);

  pos.Sec     = second();
  // pos.Sec     = gnss.time.second();
  pos.FracSec = gnss.time.centisecond();

  t.Init();
  t.setAddress(this_aircraft->addr);
#if !defined(SOFTRF_ADDRESS)
  t.setAddrTypeOGN(ADDR_TYPE_ANONYMOUS);
#else
  t.setAddrTypeOGN(this_aircraft->addr == SOFTRF_ADDRESS ?
                   ADDR_TYPE_ICAO : ADDR_TYPE_ANONYMOUS);
#endif
  t.setRelay(0);
  t.setAcftTypeOGN((int16_t) this_aircraft->aircraft_type);
  pos.Encode(t);
  t.Scramble();
  t.setCRC();

  memcpy((void *) pkt, &t.Version, t.Length);

  return (t.Length);
}
