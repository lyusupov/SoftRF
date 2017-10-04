/*
 *
 * Protocol_P3I.cpp
 * Encoder and decoder for PilotAware P3I radio protocol
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
#include "Protocol_P3I.h"

bool p3i_decode(p3i_packet *pkt, ufo_t *this_aircraft, ufo_t *fop) {

  uint32_t timestamp = (uint32_t) this_aircraft->timestamp;

  fop->addr = pkt->icao;
  fop->latitude = pkt->latitude;
  fop->longitude = pkt->longitude;
  fop->altitude = pkt->altitude;
  fop->aircraft_type = pkt->aircraft;

  fop->addr_type = ADDR_TYPE_PILOTAWARE;
  fop->timestamp = timestamp;

  fop->vs = 0;
  fop->stealth = 0;
  fop->no_track = 0;
  fop->ns[0] = 0; fop->ns[1] = 0;
  fop->ns[2] = 0; fop->ns[3] = 0;
  fop->ew[0] = 0; fop->ew[1] = 0;
  fop->ew[2] = 0; fop->ew[3] = 0;

  return true;
}

p3i_packet *p3i_encode(p3i_packet *pkt, ufo_t *this_aircraft) {

  uint32_t id = this_aircraft->addr;
  float lat = this_aircraft->latitude;
  float lon = this_aircraft->longitude;
  int16_t alt = (int16_t) this_aircraft->altitude;
  uint32_t timestamp = (uint32_t) this_aircraft->timestamp;
  unsigned int aircraft_type =  this_aircraft->aircraft_type;

  pkt->sync = '$'; 
  pkt->icao = id & 0x00FFFFFF;
  pkt->longitude = lon;
  pkt->latitude = lat;
  pkt->altitude = (uint16_t) alt;
  
  pkt->track = (uint16_t) this_aircraft->course;
  pkt->msd[0] = 0;
  pkt->msd[1] = 0;
  pkt->msd[2] = 0;
  pkt->msd[3] = 0;
  pkt->knots  = 0;    
  pkt->aircraft = aircraft_type;
  
  pkt->crc = 0;
}
