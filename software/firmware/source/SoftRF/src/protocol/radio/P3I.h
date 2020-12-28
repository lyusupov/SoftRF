/*
 * Protocol_P3I.h
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

#ifndef PROTOCOL_P3I_H
#define PROTOCOL_P3I_H

/*
 *  Initially, the PilotAware were using Ciseco ARF module.
 *  The module is based on Ti's CC1110 RF+MCU combo.
 *  The Ciseco went out of buisness but some ARF firmare is still available
 *  on GitHub: https://github.com/CisecoPlc/XRF-Firmware-downloads
 *  
 *  Current PAW RF hardware design is "PilotAware Bridge".
 *  The bridge contains:
 *  --------------------
 *  - NiceRF SV650 module (Si4432/63 RF + R5R0C002SN MCU + Tx amp.)
 *  - MPL3115A2 I2C baro sensor  
 *
 *  The bridge wiring:
 *  ------------------
 *  GPIO02 - MPL3115A2 SDA
 *  GPIO03 - MPL3115A2 SCL
 *  GPOI14 - SV650 Rx
 *  GPIO15 - SV650 Tx
 *  GPIO17 - SV650 SET
 *  GPIO18 - SV650 CS
 *
 * Settings:
 * ---------
 * Raw:
 * 0xaa, 0xfa, 0x03, 0x15, 0x03, 0x06, 0x02, 0x07, 0x02, 0x01, 0x01,
 * 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
 *
 * Channel: 21 (869.52 MHZ), Band: 868, RF rate: 38400, Tx power: 27.6 dBm
 * UART: 57600,8N1, Net ID: 0x00000000, Node Id: 0x0000
 *
 */

/*
 *
 * http://www.pilotaware.com/wp-content/uploads/2017/03/Protocol.pdf
 *
 * 24 byte packet
 * data rate of 38.4kb/s
 */

#if !defined(TEST_PAW_ON_NICERF_SV610_FW466)
/* Valid for NiceRF SV650 firmware revision: "SV650_PA_VER1.1" */
/*
 *  RF frame:
 *  ---------
 * +--------------+-------------------+------------------+
 * | Size (bits)  |   Description     |     Value        |
 * +--------------+-------------------+------------------+
 * |      2       |      Warmup       |                  |
 * +--------------+-------------------+------------------+
 * |      80      |    Preamble       |  0xAA,...,0xAA   |
 * +--------------+-------------------+------------------+
 * |      16      |    Syncword       |    0xb4, 0x2b    |
 * +--------------+-------------------+------------------+
 * |      32      |     Net ID        |  0x00,...,0x00   |
 * +--------------+-------------------+------------------+
 * |      8       |  Payload length   |     0x18 (24)    |
 * +--------------+-------------------+------------------+
 * |      8       |  CRC seed value   |        0x71      |
 * +--------------+-------------------+------------------+
 * |      192     | "White" payload   |                  |
 * +--------------+-------------------+------------------+
 * |      8       |  CRC-8, POLY_107  |                  |
 * +--------------+-------------------+------------------+
 * |      4       |      Cooldown     |                  |
 * +--------------+-------------------+------------------+
 */

#define P3I_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_AA
#define P3I_PREAMBLE_SIZE   10
#define P3I_SYNCWORD        {0xb4, 0x2b}
#define P3I_SYNCWORD_SIZE   2
#define P3I_NET_ID          0x00000000
#define P3I_PAYLOAD_SIZE    24
#define P3I_PAYLOAD_OFFSET  6
#define P3I_CRC_TYPE        RF_CHECKSUM_TYPE_CRC8_107
#define P3I_CRC_SIZE        1

#define P3I_FDEV            RF_FREQUENCY_DEVIATION_19_2KHZ
#define P3I_BANDWIDTH       RF_RX_BANDWIDTH_SS_50KHZ

#else

/* Valid for NiceRF SV610 firmware ver. 4.66 */
/*
 *  RF frame:
 *  ---------  
 * +--------------+-------------------+------------------+
 * | Size (bits)  |   Description     |     Value        |
 * +--------------+-------------------+------------------+
 * |      2       |      Warmup       |                  |
 * +--------------+-------------------+------------------+
 * |      40      |    Preamble       |  0x55,...,0x55   |
 * +--------------+-------------------+------------------+
 * |      16      |    Syncword       |  0x2d, 0xd4      |
 * +--------------+-------------------+------------------+
 * |      32      |     Net ID        |  0x00,...,0x00   |
 * +--------------+-------------------+------------------+
 * |      8       |  Payload length   |     0x18 (24)    |
 * +--------------+-------------------+------------------+
 * |      192     | "White" payload   |                  |
 * +--------------+-------------------+------------------+
 * |      16      | CRC-16,CCITT-0000 |                  |
 * +--------------+-------------------+------------------+
 * |      2       |      Cooldown     |                  |
 * +--------------+-------------------+------------------+
 */
#define P3I_PREAMBLE_TYPE   RF_PREAMBLE_TYPE_55
#define P3I_PREAMBLE_SIZE   5
#define P3I_SYNCWORD        {0x2d, 0xd4}
#define P3I_SYNCWORD_SIZE   2
#define P3I_NET_ID          0x00000000
#define P3I_PAYLOAD_SIZE    24
#define P3I_PAYLOAD_OFFSET  5
#define P3I_CRC_TYPE        RF_CHECKSUM_TYPE_CCITT_0000
#define P3I_CRC_SIZE        2

#define P3I_FDEV            RF_FREQUENCY_DEVIATION_50KHZ
#define P3I_BANDWIDTH       RF_RX_BANDWIDTH_SS_100KHZ

#endif

#define P3I_TX_INTERVAL_MIN 1600 /* in ms */
#define P3I_TX_INTERVAL_MAX 1800

/*
 * Example:
 * --------
 * icao=374780 type=1f (GS) lat=56.50000 lon=38.90000 alt=138
 * 
 * Raw:
 * ----
 * 0x24,
 * 0x80, 0x47, 0x37,
 * 0x9a, 0x99, 0x1b, 0x42,    0x00, 0x00, 0x62, 0x42,    0x8a, 0x00,
 * 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 * 0x1f,
 * 0x3b
 */
typedef struct {
    uint8_t  sync;      // $
    uint32_t  icao:24;   // 24bit identifier 
    float    longitude; // IEEE-754 
    float    latitude;  // IEEE-754 
    uint16_t altitude;  // Geo altitude (GNSS) in metres
    uint16_t track;     // degrees Relative to true north
    uint8_t  msd[4];    // sequencer
    uint16_t knots;     //  ground speed of the aircraft in knots
    uint8_t  aircraft;  //  aircraft type
    uint8_t  crc;
    /* No vertical speed available */
} __attribute__((packed)) p3i_packet_t;

extern const rf_proto_desc_t p3i_proto_desc;
extern const uint8_t whitening_pattern[] PROGMEM;

bool p3i_decode(void *, ufo_t *, ufo_t *);
size_t p3i_encode(void *, ufo_t *);

#endif /* PROTOCOL_P3I_H */
