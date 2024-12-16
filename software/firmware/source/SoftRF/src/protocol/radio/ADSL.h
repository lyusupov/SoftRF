/*
 * Protocol_ADSL.h
 *
 * Encoder and decoder for ADS-L SRD-860 radio protocol
 * Copyright (C) 2024-2025 Linar Yusupov
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

#define ADSL_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_55
#define ADSL_PREAMBLE_SIZE   1

/* IEEE  Manchester(F5724B18) = 55 99 95 A6 9A 65 A9 6A */
#define ADSL_SYNCWORD        {0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65, 0xA9, 0x6A}
#define ADSL_SYNCWORD_SIZE   8
#define ADSL_PAYLOAD_SIZE    21
#define ADSL_CRC_TYPE        RF_CHECKSUM_TYPE_CRC_MODES
#define ADSL_CRC_SIZE        3

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
