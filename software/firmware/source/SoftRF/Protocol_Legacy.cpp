/*
 * Protocol_Legacy, decoder for legacy radio protocol
 * Copyright (C) 2014-2015 Stanislaw Pusep
 *
 * Protocol_Legacy, encoder for legacy radio protocol
 * Copyright (C) 2016-2018 Linar Yusupov
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

#include <math.h>
#include <stdint.h>

#include <protocol.h>

#include "SoftRF.h"
#include "RFHelper.h"
#include "Protocol_Legacy.h"
#include "EEPROMHelper.h"

const rf_proto_desc_t legacy_proto_desc = {
  .type            = RF_PROTOCOL_LEGACY,
  .modulation_type = RF_MODULATION_TYPE_2FSK,
  .preamble_type   = LEGACY_PREAMBLE_TYPE,
  .preamble_size   = LEGACY_PREAMBLE_SIZE,
  .syncword        = LEGACY_SYNCWORD,
  .syncword_size   = LEGACY_SYNCWORD_SIZE,
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_INVERTED,
  .payload_size    = LEGACY_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = LEGACY_CRC_TYPE,
  .crc_size        = LEGACY_CRC_SIZE,

  .bitrate         = RF_BITRATE_100KBPS,
  .deviation       = RF_FREQUENCY_DEVIATION_50KHZ,
  .whitening       = RF_WHITENING_MANCHESTER,
  .bandwidth       = RF_RX_BANDWIDTH_SS_125KHZ,

  .tx_interval_min  = LEGACY_TX_INTERVAL_MIN,
  .tx_interval_max  = LEGACY_TX_INTERVAL_MAX
};

/* http://en.wikipedia.org/wiki/XXTEA */
void btea(uint32_t *v, int8_t n, const uint32_t key[4]) {
    uint32_t y, z, sum;
    uint32_t p, rounds, e;

    #define DELTA 0x9e3779b9
    // #define ROUNDS (6 + 52 / n)
    #define ROUNDS 6
    #define MX (((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + (key[(p & 3) ^ e] ^ z)))

    if (n > 1) {
        /* Coding Part */
        rounds = ROUNDS;
        sum = 0;
        z = v[n - 1];
        do {
            sum += DELTA;
            e = (sum >> 2) & 3;
            for (p = 0; p < n - 1; p++) {
                y = v[p + 1];
                z = v[p] += MX;
            }
            y = v[0];
            z = v[n - 1] += MX;
        } while (--rounds);
    } else if (n < -1) {
        /* Decoding Part */
        n = -n;
        rounds = ROUNDS;
        sum = rounds * DELTA;
        y = v[0];
        do {
            e = (sum >> 2) & 3;
            for (p = n - 1; p > 0; p--) {
                z = v[p - 1];
                y = v[p] -= MX;
            }
            z = v[n - 1];
            y = v[0] -= MX;
            sum -= DELTA;
        } while (--rounds);
    }
}

/* http://pastebin.com/YK2f8bfm */
long obscure(uint32_t key, uint32_t seed) {
    uint32_t m1 = seed * (key ^ (key >> 16));
    uint32_t m2 = (seed * (m1 ^ (m1 >> 16)));
    return m2 ^ (m2 >> 16);
}

void make_key(uint32_t key[4], uint32_t timestamp, uint32_t address) {
    static const uint32_t table[8] = LEGACY_KEY1;
    int8_t i, ndx;
    for (i = 0; i < 4; i++) {
        ndx = ((timestamp >> 23) & 1) ? i+4 : i ;
        key[i] = obscure(table[ndx] ^ ((timestamp >> 6) ^ address), LEGACY_KEY2) ^ LEGACY_KEY3;
    }
}

bool legacy_decode(void *legacy_pkt, ufo_t *this_aircraft, ufo_t *fop) {

    legacy_packet_t *pkt = (legacy_packet_t *) legacy_pkt;

    float ref_lat = this_aircraft->latitude;
    float ref_lon = this_aircraft->longitude;
    float geo_separ = this_aircraft->geoid_separation;
    uint32_t timestamp = (uint32_t) this_aircraft->timestamp;

    uint32_t key[4];
    int ndx;
    uint8_t pkt_parity=0;

    make_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
    btea((uint32_t *) pkt + 1, -5, key);

    for (ndx = 0; ndx < sizeof (legacy_packet_t); ndx++) {
      pkt_parity += parity(*(((unsigned char *) pkt) + ndx));
    }
    if (pkt_parity % 2) {
        if (settings->nmea_p) {
          StdOut.print(F("$PSRFE,bad parity of decoded packet: "));
          StdOut.println(pkt_parity % 2, HEX);
        }
        return false;
    }

    int32_t round_lat = (int32_t) (ref_lat * 1e7) >> 7;
    int32_t lat = (pkt->lat - round_lat) % (uint32_t) 0x080000;
    if (lat >= 0x040000) lat -= 0x080000;
    lat = ((lat + round_lat) << 7) /* + 0x40 */;

    int32_t round_lon = (int32_t) (ref_lon * 1e7) >> 7;
    int32_t lon = (pkt->lon - round_lon) % (uint32_t) 0x100000;
    if (lon >= 0x080000) lon -= 0x100000;
    lon = ((lon + round_lon) << 7) /* + 0x40 */;

    int32_t vs = pkt->vs * (1 << pkt->vsmult);

    int16_t alt = pkt->alt ; /* relative to WGS84 ellipsoid */

    fop->protocol = RF_PROTOCOL_LEGACY;

    fop->addr = pkt->addr;
    fop->addr_type = pkt->addr_type;
    fop->timestamp = timestamp;
    fop->latitude = (float)lat / 1e7;
    fop->longitude = (float)lon / 1e7;
    fop->altitude = (float) alt - geo_separ;
    fop->vs = 0;
    fop->aircraft_type = pkt->aircraft_type;
    fop->stealth = pkt->stealth;
    fop->no_track = pkt->no_track;
    fop->ns[0] = pkt->ns[0]; fop->ns[1] = pkt->ns[1];
    fop->ns[2] = pkt->ns[2]; fop->ns[3] = pkt->ns[3];
    fop->ew[0] = pkt->ew[0]; fop->ew[1] = pkt->ew[1];
    fop->ew[2] = pkt->ew[2]; fop->ew[3] = pkt->ew[3];

    return true;
}

extern String Bin2Hex(byte *);
size_t legacy_encode(void *legacy_pkt, ufo_t *this_aircraft) {

    legacy_packet_t *pkt = (legacy_packet_t *) legacy_pkt;

    int ndx;
    uint8_t pkt_parity=0;
    uint32_t key[4];

    uint32_t id = this_aircraft->addr;
    float lat = this_aircraft->latitude;
    float lon = this_aircraft->longitude;
    int16_t alt = (int16_t) (this_aircraft->altitude + this_aircraft->geoid_separation);
    uint32_t timestamp = (uint32_t) this_aircraft->timestamp;

    pkt->addr = id & 0x00FFFFFF;

#if !defined(SOFTRF_ADDRESS)
    pkt->addr_type = ADDR_TYPE_FLARM; /* ADDR_TYPE_ANONYMOUS */
#else
    pkt->addr_type = (pkt->addr == SOFTRF_ADDRESS ?
                      ADDR_TYPE_ICAO : ADDR_TYPE_FLARM); /* ADDR_TYPE_ANONYMOUS */
#endif

    pkt->parity = 0;
    pkt->vs = 0;
    pkt->vsmult = 0;
    pkt->stealth = this_aircraft->stealth;
    pkt->no_track = this_aircraft->no_track;

    pkt->aircraft_type = this_aircraft->aircraft_type;

    pkt->gps = 323;

    pkt->lat = (uint32_t ( lat * 1e7) >> 7) & 0x7FFFF;
    pkt->lon = (uint32_t ( lon * 1e7) >> 7) & 0xFFFFF;
    pkt->alt = alt ;

    pkt->_unk0 = 0;
    pkt->_unk1 = 0;
    pkt->_unk2 = 0;
    pkt->_unk3 = 0;
    pkt->_unk4 = 0;
    pkt->ns[0] = 0; pkt->ns[1] = 0; pkt->ns[2] = 0; pkt->ns[3] = 0;
    pkt->ew[0] = 0; pkt->ew[1] = 0; pkt->ew[2] = 0; pkt->ew[3] = 0;

    for (ndx = 0; ndx < sizeof (legacy_packet_t); ndx++) {
      pkt_parity += parity(*(((unsigned char *) pkt) + ndx));
    }
     
    pkt->parity = (pkt_parity % 2);

    //Serial.println(Bin2Hex((byte *) pkt));
    make_key(key, timestamp , (pkt->addr << 8) & 0xffffff);

#if 0
    Serial.print(key[0]);   Serial.print(", ");
    Serial.print(key[1]);   Serial.print(", ");
    Serial.print(key[2]);   Serial.print(", ");
    Serial.println(key[3]);
#endif    
    btea((uint32_t *) pkt + 1, 5, key);

    return (sizeof(legacy_packet_t));
}

