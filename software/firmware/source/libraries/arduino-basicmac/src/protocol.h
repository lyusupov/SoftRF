/*
 * Protocol.h
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

#ifndef PROTOCOL_H
#define PROTOCOL_H

enum
{
	RF_PROTOCOL_LEGACY,    /* Air V6 */
	RF_PROTOCOL_OGNTP,     /* Open Glider Network tracker */
	RF_PROTOCOL_P3I,       /* PilotAware */
	RF_PROTOCOL_ADSB_1090, /* ADS-B 1090ES */
	RF_PROTOCOL_ADSB_UAT,  /* ADS-B UAT */
	RF_PROTOCOL_FANET,     /* Skytraxx */
	/* Volunteer contributors are welcome */
	RF_PROTOCOL_EID,       /* UAS eID */
	RF_PROTOCOL_GOTENNA    /* goTenna Mesh */
};

enum
{
	RF_MODULATION_TYPE_2FSK,
	RF_MODULATION_TYPE_LORA,
	RF_MODULATION_TYPE_PPM
};

enum
{
	RF_PREAMBLE_TYPE_55,
	RF_PREAMBLE_TYPE_AA
};

enum
{
	RF_CHECKSUM_TYPE_NONE,
	RF_CHECKSUM_TYPE_CCITT_FFFF,
	RF_CHECKSUM_TYPE_CCITT_0000,
	RF_CHECKSUM_TYPE_CCITT_1D02,
	RF_CHECKSUM_TYPE_GALLAGER,
	RF_CHECKSUM_TYPE_CRC8_107,
	RF_CHECKSUM_TYPE_RS
};

enum
{
	RF_BITRATE_100KBPS,
	RF_BITRATE_38400,
	RF_BITRATE_1042KBPS
};

enum
{
	RF_FREQUENCY_DEVIATION_19_2KHZ,
	RF_FREQUENCY_DEVIATION_25KHZ,
	RF_FREQUENCY_DEVIATION_50KHZ,
	RF_FREQUENCY_DEVIATION_625KHZ
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
	RF_RX_BANDWIDTH_SS_50KHZ,
	RF_RX_BANDWIDTH_SS_100KHZ,
	RF_RX_BANDWIDTH_SS_125KHZ,
	RF_RX_BANDWIDTH_SS_166KHZ,
	RF_RX_BANDWIDTH_SS_200KHZ,
	RF_RX_BANDWIDTH_SS_250KHZ,
	RF_RX_BANDWIDTH_SS_1567KHZ
};

#define RF_MAX_SYNC_WORD_SIZE  8

typedef struct RF_PROTOCOL {
    const char name[10];
    uint8_t    type;
    uint8_t    modulation_type;
    uint8_t    preamble_type;
    uint8_t    preamble_size;
    uint8_t    syncword[RF_MAX_SYNC_WORD_SIZE];
    uint8_t    syncword_size;
    uint32_t   net_id;
    uint8_t    payload_type;
    uint8_t    payload_size;
    uint8_t    payload_offset;
    uint8_t    crc_type;
    uint8_t    crc_size;

    uint8_t    bitrate;
    uint8_t    deviation;
    uint8_t    whitening;
    uint8_t    bandwidth;

    uint16_t   tx_interval_min;
    uint16_t   tx_interval_max;
} rf_proto_desc_t;

#endif /* PROTOCOL_H */
