/*
 *
 * Protocol_APRS.h
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

#ifndef PROTOCOL_APRS_H
#define PROTOCOL_APRS_H

#include <protocol.h>

#define APRS_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_AA /* TBD */
#define APRS_PREAMBLE_SIZE   1                   /* TBD */

#define APRS_SYNCWORD        { 0x00 }            /* TBD */
#define APRS_SYNCWORD_SIZE   1
#define APRS_PAYLOAD_SIZE    AX25_MIN_FRAME_LEN
#define APRS_CRC_TYPE        RF_CHECKSUM_TYPE_CCITT_FFFF
#define APRS_CRC_SIZE        2

#define APRS_AIR_TIME        3000  /* 3 s */

#define APRS_TX_INTERVAL_MIN 60000 /* in ms, no SB support at this time */
#define APRS_TX_INTERVAL_MAX 80000

typedef struct {

  /* Dummy type definition. */

} aprs_packet_t;

extern const rf_proto_desc_t aprs_proto_desc;

bool   aprs_decode(void *, ufo_t *, ufo_t *);
size_t aprs_encode(void *, ufo_t *);

#if defined(ENABLE_PROL)
/*
 * APRS-over-LoRa
 */
#define PROL_PAYLOAD_SIZE    AX25_MIN_FRAME_LEN /* TBD */

#define PROL_AIR_TIME        3000  /* 3 s */

#define PROL_TX_INTERVAL_MIN 60000 /* in ms, no SB support at this time */
#define PROL_TX_INTERVAL_MAX 80000

extern const rf_proto_desc_t prol_proto_desc;

bool   prol_decode(void *, ufo_t *, ufo_t *);
size_t prol_encode(void *, ufo_t *);

#endif /* ENABLE_PROL */
#endif /* PROTOCOL_APRS_H */
