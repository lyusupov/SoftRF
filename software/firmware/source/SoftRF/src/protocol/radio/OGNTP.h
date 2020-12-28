/*
 *
 * Protocol_OGNTP.h
 * Encoder and decoder for Open Glider Network tracker radio protocol
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

#ifndef PROTOCOL_OGNTP_H
#define PROTOCOL_OGNTP_H

#define OGNTP_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_AA
#define OGNTP_PREAMBLE_SIZE   1 /* Warmup: 6 bits, preamble: 8 bits, value:  0xAA */
/* IEEE  Manchester(0AF3656C) = AA 66 55 A5 96 99 96 5A */
#define OGNTP_SYNCWORD        {0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A}
#define OGNTP_SYNCWORD_SIZE   8
#define OGNTP_PAYLOAD_SIZE    20
#define OGNTP_CRC_TYPE        RF_CHECKSUM_TYPE_GALLAGER
#define OGNTP_CRC_SIZE        6

#define OGNTP_TX_INTERVAL_MIN 600 /* in ms */
#define OGNTP_TX_INTERVAL_MAX 1400

#include "ogn.h"

typedef struct {

  /* Dummy type definition. Actual Tx/Rx packet format is defined in ogn.h */

} ogntp_packet_t;

extern const rf_proto_desc_t ogntp_proto_desc;

bool ogntp_decode(void *, ufo_t *, ufo_t *);
size_t ogntp_encode(void *, ufo_t *);

#endif /* PROTOCOL_OGNTP_H */