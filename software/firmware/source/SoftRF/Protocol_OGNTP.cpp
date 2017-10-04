/*
 *
 * Protocol_OGNTP.cpp
 * Encoder and decoder for Open Glider Network tracker radio protocol
 * Copyright (C) 2017 Linar Yusupov
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

#include "SoftRF.h"
#include "Protocol_OGNTP.h"
#include "Protocol_Legacy.h"

static GPS_Position pos;
static OGN_TxPacket ogn_pkt;

void ogntp_init()
{
  pos.Clear();
}

void ogntp_fini()
{

}

bool ogntp_decode(ogntp_packet *pkt, ufo_t *this_aircraft, ufo_t *fop) {

  fop->addr = ogn_pkt.Packet.Header.Address;
  fop->latitude = ogn_pkt.Packet.DecodeLatitude();
  fop->longitude = ogn_pkt.Packet.DecodeLongitude();
  fop->altitude = ogn_pkt.Packet.DecodeAltitude();
  fop->aircraft_type = ogn_pkt.Packet.Position.AcftType;

  fop->addr_type = ogn_pkt.Packet.Header.AddrType;
  fop->timestamp = this_aircraft->timestamp;

  fop->vs = 0;
  fop->stealth = ogn_pkt.Packet.Position.Stealth;
  fop->no_track = 0;
  fop->ns[0] = 0; fop->ns[1] = 0;
  fop->ns[2] = 0; fop->ns[3] = 0;
  fop->ew[0] = 0; fop->ew[1] = 0;
  fop->ew[2] = 0; fop->ew[3] = 0;

  return true;
}

ogntp_packet *ogntp_encode(ogntp_packet *pkt, ufo_t *this_aircraft) {

  pos.Latitude = this_aircraft->latitude;
  pos.Longitude = this_aircraft->longitude;
  pos.Altitude = (int16_t) this_aircraft->altitude;
  pos.Heading = (int16_t) this_aircraft->course;
  pos.Encode(ogn_pkt.Packet);

  ogn_pkt.Packet.Header.Address = this_aircraft->addr;
  ogn_pkt.Packet.Header.AddrType = ADDR_TYPE_OGN;

  ogn_pkt.Packet.Position.AcftType = (int16_t) this_aircraft->aircraft_type;
  ogn_pkt.Packet.Position.Stealth = (int16_t) this_aircraft->stealth;

  ogn_pkt.calcFEC();
}