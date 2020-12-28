/*
 *
 * Protocol_FANET.h
 *
 * Encoder and decoder for open FANET radio protocol
 * URL: https://github.com/3s1d/fanet-stm32/tree/master/Src/fanet
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

#ifndef PROTOCOL_FANET_H
#define PROTOCOL_FANET_H

/*
 * FANET uses LoRa modulation
 * FANET+ uses both LoRa (FANET) and FSK(FLARM)
 *
 * Freq: 868.2 [ 869.525 ] MHz
 * Modulation: LoRa (TM)
 * Parameters: BW_250 SF_7 CR_5
 */

#define SOFRF_FANET_VENDOR_ID       0x07

//#define FANET_NEXT

enum
{
	FANET_AIRCRAFT_TYPE_OTHER,
	FANET_AIRCRAFT_TYPE_PARAGLIDER,
	FANET_AIRCRAFT_TYPE_HANGGLIDER,
	FANET_AIRCRAFT_TYPE_BALLOON,
	FANET_AIRCRAFT_TYPE_GLIDER,
	FANET_AIRCRAFT_TYPE_POWERED,
	FANET_AIRCRAFT_TYPE_HELICOPTER,
	FANET_AIRCRAFT_TYPE_UAV
};

/*
 * Tracking frame type (#1),
 * Standard header,
 * No signature,
 * Broadcast
 */
typedef struct {
  unsigned int type           :6;
  unsigned int forward        :1;
  unsigned int ext_header     :1;

  unsigned int vendor         :8;
  unsigned int address        :16;

#if defined(FANET_DEPRECATED)
  unsigned int latitude       :16;
  unsigned int longitude      :16;
#else
  unsigned int latitude       :24;
  unsigned int longitude      :24;
#endif

	/* units are degrees, seconds, and meter */
  unsigned int altitude_lsb   :8; /* FANET+ reported alt. comes from ext. source */
  unsigned int altitude_msb   :3; /* I assume that it is geo (GNSS) altitude */
  unsigned int altitude_scale :1;
  unsigned int aircraft_type  :3;
  unsigned int track_online   :1;

  unsigned int speed          :7;
  unsigned int speed_scale    :1;

  unsigned int climb          :7;
  unsigned int climb_scale    :1;

  unsigned int heading        :8;

  unsigned int turn_rate      :7;
  unsigned int turn_scale     :1;

#if defined(FANET_NEXT)
  unsigned int qne_offset     :7;
  unsigned int qne_scale      :1;
#endif
} __attribute__((packed)) fanet_packet_t;

#define FANET_PAYLOAD_SIZE    sizeof(fanet_packet_t)
#define FANET_HEADER_SIZE     4

/* Declared air time of FANET+ is 20-40 ms */
#define FANET_TX_INTERVAL_MIN 2500 /* in ms */
#define FANET_TX_INTERVAL_MAX 3500

extern const rf_proto_desc_t fanet_proto_desc;

bool fanet_decode(void *, ufo_t *, ufo_t *);
size_t fanet_encode(void *, ufo_t *);

#endif /* PROTOCOL_FANET_H */
