/*
 * Protocol_FANET.cpp
 *
 * Encoder and decoder for open FANET radio protocol
 *
 * URL:
 *    Development -  https://github.com/3s1d/fanet-stm32
 *    Deprecated  -  https://github.com/3s1d/fanet
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
  .preamble_type  = 0 /* INVALID FOR LORA */,
  .preamble_size  = 0 /* INVALID FOR LORA */,
  .syncword       = {0} /* INVALID FOR LORA */,
  .syncword_size  = 0 /* INVALID FOR LORA */,
  .net_id         = 0x0000, /* not in use */
  .payload_type   = 0 /* INVALID FOR LORA */,
  .payload_size   = FANET_PAYLOAD_SIZE,
  .payload_offset = 0,
  .crc_type       = 0 /* INVALID FOR LORA */,
  .crc_size       = 0 /* INVALID FOR LORA */,

  .bitrate        = 0 /* INVALID FOR LORA */,
  .deviation      = 0 /* INVALID FOR LORA */,
  .whitening      = 0 /* INVALID FOR LORA */,
  .bandwidth      = 0 /* INVALID FOR LORA */
};

App app = App();

void fanet_init()
{

}

void fanet_fini()
{

}

bool fanet_decode(void *pkt, ufo_t *this_aircraft, ufo_t *fop) {

  /* deserialize packet */
  Frame *frm = new Frame(sizeof(fanet_packet_t), (uint8_t *) pkt);


	/* simply print frame */

	Serial.print(F(FANET_CMD_START CMD_RX_FRAME " "));

	/* src_manufacturer,src_id,broadcast,signature,type,payloadlength,payload */
	Serial.print(frm->src.manufacturer, HEX);
	Serial.print(',');
	Serial.print(frm->src.id, HEX);
	Serial.print(',');
	Serial.print(frm->dest == MacAddr());	//broadcast
	Serial.print(',');
	Serial.print(frm->signature, HEX);
	Serial.print(',');
	Serial.print(frm->type, HEX);
	Serial.print(',');
	Serial.print(frm->payload_length, HEX);
	Serial.print(',');
	for(int i=0; i<frm->payload_length; i++)
	{
		char buf[8];
		sprintf(buf, "%02X", frm->payload[i]);
		Serial.print(buf);
	}

	Serial.println();
	Serial.flush();

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

  delete frm;

  return false; // true;
}

size_t fanet_encode(void *fanet_pkt, ufo_t *this_aircraft) {

  size_t size;
  Frame* frm;

  fanet_packet_t *pkt = (fanet_packet_t *) fanet_pkt;

  app.set( this_aircraft->latitude, this_aircraft->longitude,
    this_aircraft->altitude, this_aircraft->speed * _GPS_KMPH_PER_KNOT,
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
