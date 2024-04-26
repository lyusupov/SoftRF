/*
 * Protocol_ADSL.h
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

#ifndef PROTOCOL_ADSL_H
#define PROTOCOL_ADSL_H

#define ADSL_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_AA
#define ADSL_PREAMBLE_SIZE   1 /* Warmup: 6 bits, preamble: 8 bits, value:  0xAA */
/* IEEE  Manchester(0AF3656C) = AA 66 55 A5 96 99 96 5A */
#define ADSL_SYNCWORD        {0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A}
#define ADSL_SYNCWORD_SIZE   8
#define ADSL_PAYLOAD_SIZE    20
#define ADSL_CRC_TYPE        RF_CHECKSUM_TYPE_GALLAGER
#define ADSL_CRC_SIZE        6

#define ADSL_AIR_TIME        6 /* in ms */

#define ADSL_TX_INTERVAL_MIN 600 /* in ms */
#define ADSL_TX_INTERVAL_MAX 1400

#include <ads-l.h>

typedef struct {

  /* Dummy type definition. Actual Tx/Rx packet format is defined in ads-l.h */

} adsl_packet_t;

extern const rf_proto_desc_t adsl_proto_desc;

bool adsl_decode(void *, ufo_t *, ufo_t *);
size_t adsl_encode(void *, ufo_t *);

#endif /* PROTOCOL_ADSL_H */
