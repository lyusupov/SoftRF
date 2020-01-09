/*
 *
 * Protocol_OGNTP.cpp
 * Encoder and decoder for Open Glider Network tracker radio protocol
 * Copyright (C) 2017-2020 Linar Yusupov
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

#include "SoftRF.h"
#include "Protocol_OGNTP.h"
#include "Protocol_Legacy.h"
#include "RFHelper.h"

const rf_proto_desc_t ogntp_proto_desc = {
  "OGNTP",
  .type            = RF_PROTOCOL_OGNTP,
  .modulation_type = RF_MODULATION_TYPE_2FSK,
  .preamble_type   = OGNTP_PREAMBLE_TYPE,
  .preamble_size   = OGNTP_PREAMBLE_SIZE,
  .syncword        = OGNTP_SYNCWORD,
  .syncword_size   = OGNTP_SYNCWORD_SIZE,
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_INVERTED,
  .payload_size    = OGNTP_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = OGNTP_CRC_TYPE,
  .crc_size        = OGNTP_CRC_SIZE,

  .bitrate         = RF_BITRATE_100KBPS,
  .deviation       = RF_FREQUENCY_DEVIATION_50KHZ,
  .whitening       = RF_WHITENING_MANCHESTER,
  .bandwidth       = RF_RX_BANDWIDTH_SS_125KHZ,

  .tx_interval_min  = OGNTP_TX_INTERVAL_MIN,
  .tx_interval_max  = OGNTP_TX_INTERVAL_MAX
};

static GPS_Position pos;
static OGN_TxPacket ogn_tx_pkt;
static OGN_RxPacket ogn_rx_pkt;

void ogntp_init()
{
  pos.Clear();
  ogn_rx_pkt.Clear();
}

void ogntp_fini()
{

}

bool ogntp_decode(void *pkt, ufo_t *this_aircraft, ufo_t *fop) {

  ogn_rx_pkt.recvBytes((uint8_t *) pkt);

/* that has been alreay done by RFHelper */
//  if (ogn_rx_pkt.checkFEC()) {
//    return false;
//  }

  if ( ogn_rx_pkt.Packet.Header.Other || ogn_rx_pkt.Packet.Header.Encrypted ) {
    return false;
  }

#if !defined(SOFTRF_ADDRESS)
  uint8_t addr_type = ADDR_TYPE_ANONYMOUS;
#else
  uint8_t addr_type = (this_aircraft->addr == SOFTRF_ADDRESS ?
                        ADDR_TYPE_ICAO : ADDR_TYPE_ANONYMOUS);
#endif

  /* ignore this device own (relayed) packets */
  if ((ogn_rx_pkt.Packet.Header.Address    == this_aircraft->addr) &&
      (ogn_rx_pkt.Packet.Header.AddrType   == addr_type          ) &&
      (ogn_rx_pkt.Packet.Header.RelayCount > 0 )) {
    return false;
  }

  ogn_rx_pkt.Packet.Dewhiten();

  fop->protocol = RF_PROTOCOL_OGNTP;

  fop->addr = ogn_rx_pkt.Packet.Header.Address;
  fop->latitude = ogn_rx_pkt.Packet.DecodeLatitude() * 0.0001/60;
  fop->longitude = ogn_rx_pkt.Packet.DecodeLongitude() * 0.0001/60;
  fop->altitude = (float) ogn_rx_pkt.Packet.DecodeAltitude();
  fop->pressure_altitude = (float) ogn_rx_pkt.Packet.DecodeStdAltitude();
  fop->aircraft_type = ogn_rx_pkt.Packet.Position.AcftType;
  fop->course = ogn_rx_pkt.Packet.DecodeHeading() * 0.1;
  fop->speed = (ogn_rx_pkt.Packet.DecodeSpeed() * 0.1) / _GPS_MPS_PER_KNOT;
  fop->vs = (ogn_rx_pkt.Packet.DecodeClimbRate() * 0.1) * (_GPS_FEET_PER_METER * 60.0);
  fop->hdop = (ogn_rx_pkt.Packet.DecodeDOP() + 10) * 10;

  fop->addr_type = ogn_rx_pkt.Packet.Header.AddrType;
  fop->timestamp = this_aircraft->timestamp;

  fop->stealth = ogn_rx_pkt.Packet.Position.Stealth;
  fop->no_track = 0;
  fop->ns[0] = 0; fop->ns[1] = 0;
  fop->ns[2] = 0; fop->ns[3] = 0;
  fop->ew[0] = 0; fop->ew[1] = 0;
  fop->ew[2] = 0; fop->ew[3] = 0;

  return true;
}

size_t ogntp_encode(void *pkt, ufo_t *this_aircraft) {

  pos.Latitude = (int32_t) (this_aircraft->latitude * 600000);
  pos.Longitude = (int32_t) (this_aircraft->longitude * 600000);
  pos.Altitude = (int32_t) (this_aircraft->altitude * 10);
  if (this_aircraft->pressure_altitude != 0.0) {
    pos.StdAltitude = (int32_t) (this_aircraft->pressure_altitude * 10);
    pos.ClimbRate = (int32_t) (this_aircraft->vs / (_GPS_FEET_PER_METER * 6.0));
  }
  pos.Heading = (int16_t) (this_aircraft->course * 10);
  pos.Speed = (int16_t) (this_aircraft->speed * 10 * _GPS_MPS_PER_KNOT);
  pos.HDOP = (uint8_t) (this_aircraft->hdop / 10);

  pos.Encode(ogn_tx_pkt.Packet);

  ogn_tx_pkt.Packet.HeaderWord=0;
  ogn_tx_pkt.Packet.Header.Address = this_aircraft->addr;

#if !defined(SOFTRF_ADDRESS)
  ogn_tx_pkt.Packet.Header.AddrType = ADDR_TYPE_ANONYMOUS;
#else
  ogn_tx_pkt.Packet.Header.AddrType = (this_aircraft->addr == SOFTRF_ADDRESS ?
                                      ADDR_TYPE_ICAO : ADDR_TYPE_ANONYMOUS);
#endif

  ogn_tx_pkt.Packet.calcAddrParity();

  ogn_tx_pkt.Packet.Position.AcftType = (int16_t) this_aircraft->aircraft_type;
  ogn_tx_pkt.Packet.Position.Stealth = (int16_t) this_aircraft->stealth;
  ogn_tx_pkt.Packet.Position.Time = second();

  ogn_tx_pkt.Packet.Whiten();
  ogn_tx_pkt.calcFEC();

  memcpy((void *) pkt,  ogn_tx_pkt.Byte(), ogn_tx_pkt.Bytes);
  return (ogn_tx_pkt.Bytes);
}
