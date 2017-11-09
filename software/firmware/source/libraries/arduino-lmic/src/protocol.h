/*
 * Protocol.h
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

#ifndef PROTOCOL_H
#define PROTOCOL_H

enum
{
	RF_PREAMBLE_TYPE_55,
	RF_PREAMBLE_TYPE_AA
};

enum
{
	RF_CHECKSUM_TYPE_CCITT_FFFF,
	RF_CHECKSUM_TYPE_CCITT_0000,
	RF_CHECKSUM_TYPE_CCITT_1D02,
	RF_CHECKSUM_TYPE_GALLAGER
};

enum
{
	RF_BITRATE_100KBPS,
	RF_BITRATE_38400
};

enum
{
	RF_FREQUENCY_DEVIATION_25KHZ,
	RF_FREQUENCY_DEVIATION_50KHZ
};

enum
{
	RF_WHITENING_NONE,
	RF_WHITENING_MANCHESTER,
	RF_WHITENING_PN9,
	RF_WHITENING_NICERF
};

enum
{
	RF_PAYLOAD_DIRECT,
	RF_PAYLOAD_INVERTED
};

enum
{
	RF_RX_BANDWIDTH_SS_100KHZ,
	RF_RX_BANDWIDTH_SS_125KHZ,
	RF_RX_BANDWIDTH_SS_166KHZ
};

#define RF_MAX_SYNC_WORD_SIZE  8

typedef struct RF_PROTOCOL {
    uint8_t   type;
    uint8_t   preamble_type;
    uint8_t   preabmble_size;
    uint8_t   syncword[RF_MAX_SYNC_WORD_SIZE];
    uint8_t   syncword_size;
    uint32_t  net_id;
    uint8_t   payload_type;
    uint8_t   payload_size;
    uint8_t   payload_offset;
    uint8_t   crc_type;
    uint8_t   crc_size;

    uint8_t   bitrate;
    uint8_t   deviation;
    uint8_t   whitening;   
    uint8_t   bandwidth;                    
} rf_proto_desc_t;


#endif /* PROTOCOL_H */

