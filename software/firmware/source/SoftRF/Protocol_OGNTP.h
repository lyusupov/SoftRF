/*
 *
 * Protocol_OGNTP.h
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

#ifndef PROTOCOL_OGNTP_H
#define PROTOCOL_OGNTP_H

#define OGNTP_PREAMBLE_SIZE   1 /* Warmup: 6 bits, preamble: 8 bits, value:  0xAA */
#define OGNTP_SYNCWORD        {0x0A, 0xF3, 0x65, 0x6C}
#define OGNTP_PAYLOAD_SIZE    20
#define OGNTP_CRC_SIZE        6             
#define OGNTP_CRC_TYPE        2             /* Gallager */

#include "ogn.h"

typedef struct {

  /* Dummy type definition. Actual Tx/Rx packet format is defined in ogn.h */ 

} ogntp_packet;

#endif /* PROTOCOL_OGNTP_H */