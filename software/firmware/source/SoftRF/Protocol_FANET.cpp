/*
 *
 * Protocol_FANET.cpp
 *
 * Encoder and decoder for open FANET radio protocol
 * URL: https://github.com/3s1d/fanet-stm32/tree/master/Src/fanet
 *
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

#include <protocol.h>
#include <Time.h>

#include "SoftRF.h"
#include "Protocol_FANET.h"
#include "RFHelper.h"

const rf_proto_desc_t fanet_proto_desc = {
  .type           = RF_PROTOCOL_FANET,
  .preamble_type  = FANET_PREAMBLE_TYPE,
  .preamble_size  = FANET_PREAMBLE_SIZE,
  .syncword       = FANET_SYNCWORD,
  .syncword_size  = FANET_SYNCWORD_SIZE,
  .net_id         = 0x0000, /* not in use */
  .payload_type   = 0 /* TBD */,
  .payload_size   = FANET_PAYLOAD_SIZE,
  .payload_offset = 0,
  .crc_type       = FANET_CRC_TYPE,
  .crc_size       = FANET_CRC_SIZE,

  .bitrate        = 0 /* TBD */,
  .deviation      = 0 /* TBD */,
  .whitening      = 0 /* TBD */,
  .bandwidth      = 0 /* TBD */
};

App app = App();

void fanet_init()
{

}

void fanet_fini()
{

}

bool fanet_decode(void *pkt, ufo_t *this_aircraft, ufo_t *fop) {

#if 0
  fop->addr = ogn_rx_pkt.Packet.Header.Address;
  fop->latitude = ogn_rx_pkt.Packet.DecodeLatitude() * 0.0001/60;
  fop->longitude = ogn_rx_pkt.Packet.DecodeLongitude() * 0.0001/60;
  fop->altitude = ogn_rx_pkt.Packet.DecodeAltitude();
  fop->aircraft_type = ogn_rx_pkt.Packet.Position.AcftType;
  fop->course = ogn_rx_pkt.Packet.Position.Heading * 0.1;
  fop->speed = (ogn_rx_pkt.Packet.Position.Speed * 0.1) / _GPS_MPS_PER_KNOT;

  fop->addr_type = ogn_rx_pkt.Packet.Header.AddrType;
  fop->timestamp = this_aircraft->timestamp;

  fop->vs = 0;
  fop->stealth = ogn_rx_pkt.Packet.Position.Stealth;
  fop->no_track = 0;
  fop->ns[0] = 0; fop->ns[1] = 0;
  fop->ns[2] = 0; fop->ns[3] = 0;
  fop->ew[0] = 0; fop->ew[1] = 0;
  fop->ew[2] = 0; fop->ew[3] = 0;
#endif

  return true;
}

size_t fanet_encode(void *fanet_pkt, ufo_t *this_aircraft) {

  size_t size;
	Frame* frm;

  fanet_packet_t *pkt = (fanet_packet_t *) fanet_pkt;

  app.set( this_aircraft->latitude, this_aircraft->longitude,
    this_aircraft->altitude, this_aircraft->speed * _GPS_MPS_PER_KNOT,
    0, this_aircraft->course, 0
  );

  frm = app.get_frame();
	if(frm == NULL)
		return (0);

  uint8_t* buffer;
  size = frm->serialize(buffer);

  delete frm;

  if (size > 0) {
    memcpy((void *) pkt, buffer, size);
  	delete[] buffer;
  } else {
    size = 0;
  }

  return (size);
}
