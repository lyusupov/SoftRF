/*
 * Protocol_P3I.h
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

#ifndef PROTOCOL_P3I_H
#define PROTOCOL_P3I_H

/*
 *
 * http://www.pilotaware.com/wp-content/uploads/2017/03/Protocol.pdf
 *
 * 24 byte packet
 * data rate of 38.4kb/s 
 */

#define P3I_PREAMBLE_SIZE   1             /* TBD */
#define P3I_SYNCWORD        {0, 0, 0, 0}  /* TBD */
#define P3I_PAYLOAD_SIZE    24
#define P3I_CRC_SIZE        0             /* TBD */
#define P3I_CRC_TYPE        0             /* TBD */

#define ADDR_TYPE_PILOTAWARE  0

typedef struct {
    uint8_t  sync;      // $
    uint8_t  icao:24;   // 24bit identifier 
    /* floats alignment is TBD */
    float    longitude; // IEEE-754 
    float    latitude;  // IEEE-754 
    uint16_t altitude;  // metres
    uint16_t track;     // degrees Relative to true north
    uint8_t  msd[4];    // sequencer
    uint16_t knots;     //  ground speed of the aircraft in knots
    uint8_t  aircraft;  //  aircraft type
    uint8_t  crc;
} p3i_packet;

#endif /* PROTOCOL_P3I_H */