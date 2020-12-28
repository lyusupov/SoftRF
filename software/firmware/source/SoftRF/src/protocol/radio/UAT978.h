/*
 *
 * Protocol_UAT978.h
 * Decoder for UAT 978 MHz ADS-B radio protocol
 * Copyright (C) 2019-2021 Linar Yusupov
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

#ifndef PROTOCOL_UAT978_H
#define PROTOCOL_UAT978_H

#include <uat.h>
#include <uat_decode.h>

#define UAT978_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_AA /* TBD */
#define UAT978_PREAMBLE_SIZE   4                   /* TBD */

#define UAT978_SYNCWORD        { 0xAC, 0xDD, 0xA4, 0xE2 }
#define UAT978_SYNCWORD_SIZE   4
#define UAT978_PAYLOAD_SIZE    LONG_FRAME_DATA_BYTES
#define UAT978_CRC_TYPE        RF_CHECKSUM_TYPE_RS
#define UAT978_CRC_SIZE        (LONG_FRAME_BYTES - LONG_FRAME_DATA_BYTES)

#define UAT978_TX_INTERVAL_MIN 900 /* in ms */ /* TBD */
#define UAT978_TX_INTERVAL_MAX 1000            /* TBD */

#define STRATUX_UATRADIO_MAGIC_1   0x0a
#define STRATUX_UATRADIO_MAGIC_2   0xb0
#define STRATUX_UATRADIO_MAGIC_3   0xcd
#define STRATUX_UATRADIO_MAGIC_4   0xe0

typedef struct __attribute__ ((packed)) Stratux_LPUATRadio_UART_frame {
  byte      magic1;
  byte      magic2;
  byte      magic3;
  byte      magic4;

  uint16_t  msgLen;

  int8_t    rssi;
  uint32_t  timestamp;

  uint8_t   data[LONG_FRAME_BYTES];
} Stratux_frame_t;

typedef struct {

  /* Dummy type definition. Actual Rx packet format is defined in uat_decode.h */

} uat978_packet_t;

extern const rf_proto_desc_t uat978_proto_desc;

bool   uat978_decode(void *, ufo_t *, ufo_t *);
size_t uat978_encode(void *, ufo_t *);

#endif /* PROTOCOL_UAT978_H */