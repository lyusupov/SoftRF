/*
 * Protocol_Legacy, decoder for legacy radio protocol
 * Copyright (C) 2014-2015 Stanislaw Pusep
 *
 * Protocol_Legacy, encoder for legacy radio protocol
 * Copyright (C) 2016-2025 Linar Yusupov
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

#include "../../../SoftRF.h"
#include "../../driver/RF.h"
#include "../../driver/EEPROM.h"

const rf_proto_desc_t legacy_proto_desc = {
  .name            = {'L','e','g','a','c','y', 0},
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

  .air_time        = LEGACY_AIR_TIME,

#if defined(USE_TIME_SLOTS)
  .tm_type         = RF_TIMING_2SLOTS_PPS_SYNC,
#else
  .tm_type         = RF_TIMING_INTERVAL,
#endif
  .tx_interval_min = LEGACY_TX_INTERVAL_MIN,
  .tx_interval_max = LEGACY_TX_INTERVAL_MAX,
  .slot0           = {400,  800},
  .slot1           = {800, 1200}
};

/*
 * FTD-014. Speed threshold in m/sec.
 * The aircraft is treated as on ground if its horizontal
 * velocity is below this value.
 */
static const uint8_t legacy_GS_threshold[] PROGMEM = {
        0,  /* OTHER      */
        2,  /* GLIDER     */
        10, /* TOWPLANE   */
        2,  /* HELICOPTER */
        5,  /* PARACHUTE  */
        10, /* DROPPLANE  */
        2,  /* HANGGLIDER */
        2,  /* PARAGLIDER */
        10, /* POWERED    */
        10, /* JET        */
        0,  /* UFO        */
        0,  /* BALLOON    */
        0,  /* ZEPPELIN   */
        2,  /* UAV        */
        0,  /* RESERVED   */
        0   /* STATIC     */
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

static const uint32_t table[12] = LEGACY_KEY1;

void make_v6_key(uint32_t key[4], uint32_t timestamp, uint32_t address) {
    int8_t i, ndx;
    for (i = 0; i < 4; i++) {
        ndx = ((timestamp >> 23) & 1) ? i+4 : i ;
        key[i] = obscure(table[ndx] ^ ((timestamp >> 6) ^ address), LEGACY_KEY2) ^ LEGACY_KEY3;
    }
}

void make_v7_key(uint32_t key[4]) {
  uint8_t *bkeys;
  int p, q, x, y, z, sum;

  bkeys = (uint8_t *) &key[0];
  x = bkeys[15];
  sum = 0;
  q = 2;

  do {
    sum += DELTA;
    for (p=0; p<16; p++) {
      z = x & 0xFF;
      y = bkeys[(p+1) % 16];
      x = bkeys[p];
      x += ((((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^ (sum ^ y));
      bkeys[p] = (uint8_t)x;
    }
  } while (--q > 0);
}

#if !defined(EXCLUDE_AIR6)

static bool legacy_v6_decode(void *legacy_pkt, ufo_t *this_aircraft, ufo_t *fop) {

    legacy_v6_packet_t *pkt = (legacy_v6_packet_t *) legacy_pkt;

    if (pkt->type != 0) { /* not Air V6 position */
        return false;
    }

    float ref_lat = this_aircraft->latitude;
    float ref_lon = this_aircraft->longitude;
    float geo_separ = this_aircraft->geoid_separation;
    uint32_t timestamp = (uint32_t) this_aircraft->timestamp;

    uint32_t key[4];
    int ndx;
    uint8_t pkt_parity=0;

    make_v6_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
    btea((uint32_t *) pkt + 1, -5, key);

    for (ndx = 0; ndx < sizeof (legacy_v6_packet_t); ndx++) {
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

    int32_t ns = (pkt->ns[0] + pkt->ns[1] + pkt->ns[2] + pkt->ns[3]) / 4;
    int32_t ew = (pkt->ew[0] + pkt->ew[1] + pkt->ew[2] + pkt->ew[3]) / 4;
    float speed4 = sqrtf(ew * ew + ns * ns) * (1 << pkt->smult);

    float direction = 0;
    if (speed4 > 0) {
      direction = atan2f(ns,ew) * 180.0 / PI;  /* -180 ... 180 */
      /* convert from math angle into course relative to north */
      direction = (direction <= 90.0 ? 90.0 - direction :
                                      450.0 - direction);
    }

    uint16_t vs_u16 = pkt->vs;
    int16_t vs_i16 = (int16_t) (vs_u16 | (vs_u16 & (1<<9) ? 0xFC00U : 0));
    int16_t vs10 = vs_i16 << pkt->smult;

    int16_t alt = pkt->alt ; /* relative to WGS84 ellipsoid */

    fop->protocol = RF_PROTOCOL_LEGACY;

    fop->addr = pkt->addr;
    fop->addr_type = pkt->addr_type;
    fop->timestamp = timestamp;
    fop->latitude = (float)lat / 1e7;
    fop->longitude = (float)lon / 1e7;
    fop->altitude = (float) alt - geo_separ;
    fop->speed = speed4 / (4 * _GPS_MPS_PER_KNOT);
    fop->course = direction;
    fop->vs = ((float) vs10) * (_GPS_FEET_PER_METER * 6.0);
    fop->aircraft_type = pkt->aircraft_type;
    fop->stealth = pkt->stealth;
    fop->no_track = pkt->no_track;
    fop->ns[0] = pkt->ns[0]; fop->ns[1] = pkt->ns[1];
    fop->ns[2] = pkt->ns[2]; fop->ns[3] = pkt->ns[3];
    fop->ew[0] = pkt->ew[0]; fop->ew[1] = pkt->ew[1];
    fop->ew[2] = pkt->ew[2]; fop->ew[3] = pkt->ew[3];

    return true;
}

static size_t legacy_v6_encode(void *legacy_pkt, ufo_t *this_aircraft) {

    legacy_v6_packet_t *pkt = (legacy_v6_packet_t *) legacy_pkt;

    int ndx;
    uint8_t pkt_parity=0;
    uint32_t key[4];

    uint32_t id = this_aircraft->addr;
    uint8_t acft_type = this_aircraft->aircraft_type > AIRCRAFT_TYPE_STATIC ?
            AIRCRAFT_TYPE_UNKNOWN : this_aircraft->aircraft_type;

    int32_t lat = (int32_t) (this_aircraft->latitude  * 1e7);
    int32_t lon = (int32_t) (this_aircraft->longitude * 1e7);
    int16_t alt = (int16_t) (this_aircraft->altitude  + this_aircraft->geoid_separation);
    uint32_t timestamp = (uint32_t) this_aircraft->timestamp;

    float course = this_aircraft->course;
    float speedf = this_aircraft->speed * _GPS_MPS_PER_KNOT; /* m/s */
    float vsf = this_aircraft->vs / (_GPS_FEET_PER_METER * 60.0); /* m/s */

    uint16_t speed4 = (uint16_t) roundf(speedf * 4.0f);
    if (speed4 > 0x3FF) {
      speed4 = 0x3FF;
    }

    if        (speed4 & 0x200) {
      pkt->smult = 3;
    } else if (speed4 & 0x100) {
      pkt->smult = 2;
    } else if (speed4 & 0x080) {
      pkt->smult = 1;
    } else {
      pkt->smult = 0;
    }

    uint8_t speed = speed4 >> pkt->smult;

    int8_t ns = (int8_t) (speed * cosf(radians(course)));
    int8_t ew = (int8_t) (speed * sinf(radians(course)));

    int16_t vs10 = (int16_t) roundf(vsf * 10.0f);
    pkt->vs = this_aircraft->stealth ? 0 : vs10 >> pkt->smult;

    pkt->type = 0; /* Air V6 position */
    pkt->addr = id & 0x00FFFFFF;

#if !defined(SOFTRF_ADDRESS)
    pkt->addr_type = ADDR_TYPE_FLARM; /* ADDR_TYPE_ANONYMOUS */
#else
    pkt->addr_type = (pkt->addr == SOFTRF_ADDRESS ?
                      ADDR_TYPE_ICAO : ADDR_TYPE_FLARM); /* ADDR_TYPE_ANONYMOUS */
#endif

    pkt->parity = 0;

    pkt->stealth  = this_aircraft->stealth;
    pkt->no_track = this_aircraft->no_track;

    pkt->aircraft_type = acft_type;

    pkt->gps = 323;

    pkt->lat = ((lat >> 7) + (lat & 0x40 ? (lat < 0 ? -1 : 1) : 0)) & 0x7FFFF;
    pkt->lon = ((lon >> 7) + (lon & 0x40 ? (lon < 0 ? -1 : 1) : 0)) & 0xFFFFF;

    pkt->alt = alt < 0 ? 0 : alt;

    pkt->airborne = ((int) speedf) >= legacy_GS_threshold[acft_type] ? 1 : 0;

    pkt->ns[0] = ns; pkt->ns[1] = ns; pkt->ns[2] = ns; pkt->ns[3] = ns;
    pkt->ew[0] = ew; pkt->ew[1] = ew; pkt->ew[2] = ew; pkt->ew[3] = ew;

    pkt->_unk1 = 0;
    pkt->_unk2 = 0;
    pkt->_unk3 = 0;
//    pkt->_unk4 = 0;

    for (ndx = 0; ndx < sizeof (legacy_v6_packet_t); ndx++) {
      pkt_parity += parity(*(((unsigned char *) pkt) + ndx));
    }

    pkt->parity = (pkt_parity % 2);

    make_v6_key(key, timestamp , (pkt->addr << 8) & 0xffffff);

#if 0
    Serial.print(key[0]);   Serial.print(", ");
    Serial.print(key[1]);   Serial.print(", ");
    Serial.print(key[2]);   Serial.print(", ");
    Serial.println(key[3]);
#endif
    btea((uint32_t *) pkt + 1, 5, key);

    return (sizeof(legacy_v6_packet_t));
}

#endif /* EXCLUDE_AIR6 */

#if defined(EXCLUDE_AIR7)

bool legacy_decode(void *legacy_pkt, ufo_t *this_aircraft, ufo_t *fop) {

    return legacy_v6_decode(legacy_pkt, this_aircraft, fop);
}

size_t legacy_encode(void *legacy_pkt, ufo_t *this_aircraft) {

    return legacy_v6_encode(legacy_pkt, this_aircraft);
}

#else
/*
 * https://pastebin.com/YB1ppAbt
 */

#define USE_INTERLEAVING
//#define EXCLUDE_AIR6

static const uint16_t lon_div_table[] = {
   53,  53,  54,  54,  55,  55,
   56,  56,  57,  57,  58,  58,  59,  59,  60,  60,
   61,  61,  62,  62,  63,  63,  64,  64,  65,  65,
   67,  68,  70,  71,  73,  74,  76,  77,  79,  80,
   82,  83,  85,  86,  88,  89,  91,  94,  98, 101,
  105, 108, 112, 115, 119, 122, 126, 129, 137, 144,
  152, 159, 167, 174, 190, 205, 221, 236, 252,
  267, 299, 330, 362, 425, 489, 552, 616, 679, 743, 806, 806
};

static int descale(unsigned int value, unsigned int mbits, unsigned int ebits)
{
    unsigned int offset   = (1 << mbits);
    unsigned int signbit  = (offset << ebits);
    unsigned int negative = (value & signbit);

    value &= (signbit - 1);

    if (value >= offset) {
        unsigned int exp = value >> mbits;
        value &= (offset - 1);
        value += offset;
        value <<= exp;
        value -= offset;
    }

    return (negative ? -(int)value : value);
}

static unsigned int enscale_unsigned(unsigned int value,
                                     unsigned int mbits,
                                     unsigned int ebits)
{
    unsigned int offset  = (1 << mbits);
    unsigned int max_val = (offset << ebits) - 1;

    if (value >= offset) {
      unsigned int e      = 0;
      unsigned int m      = offset + value;
      unsigned int mlimit = offset + offset - 1;

      while (m > mlimit) {
          m >>= 1;
          e += offset;
          if (e > max_val) {
              return (max_val);
          }
      }
      m -= offset;
      value = (e | m);
    }

    return (value);
}

static unsigned int enscale_signed(  signed int value,
                                   unsigned int mbits,
                                   unsigned int ebits)
{
    unsigned int offset  = (1 << mbits);
    unsigned int signbit = (offset << ebits);
    unsigned int max_val = signbit - 1;
    unsigned int sign    = 0;

    if (value < 0) {
      value = -value;
      sign  = signbit;
    }

    unsigned int rval;
    if (value >= offset) {
      unsigned int e      = 0;
      unsigned int m      = offset + (unsigned int) value;
      unsigned int mlimit = offset + offset - 1;

      while (m > mlimit) {
        m >>= 1;
        e += offset;
        if (e > max_val) {
          return (sign | max_val);
        }
      }
      m -= offset;
      rval = (sign | e | m);
    } else {
      rval = (sign | (unsigned int) value);
    }

    return (rval);
}

bool legacy_decode(void *legacy_pkt, ufo_t *this_aircraft, ufo_t *fop) {
    const uint32_t xxtea_key[4] = LEGACY_KEY5;
    uint32_t key_v7[4];

    legacy_v7_packet_t *pkt = (legacy_v7_packet_t *) legacy_pkt;

#if !defined(EXCLUDE_AIR6)
    if (pkt->type == 0) { /* Air V6 position */
      return legacy_v6_decode(legacy_pkt, this_aircraft, fop);
    }
#endif /* EXCLUDE_AIR6 */

    if (pkt->type != 2) { /* not Air V7 position */
        return false;
    }

    uint32_t *wpkt     = (uint32_t *) legacy_pkt;
    uint32_t timestamp = (uint32_t) this_aircraft->timestamp;

    btea(&wpkt[2], -4, xxtea_key);

    key_v7[0]          = wpkt[0];
    key_v7[1]          = wpkt[1];
    key_v7[2]          = timestamp >> 4;
    key_v7[3]          = LEGACY_KEY4;

    make_v7_key(key_v7);

    wpkt[2] ^= key_v7[0];
    wpkt[3] ^= key_v7[1];
    wpkt[4] ^= key_v7[2];
    wpkt[5] ^= key_v7[3];

    fop->protocol      = RF_PROTOCOL_LEGACY;

    fop->addr          = pkt->addr;
    fop->addr_type     = pkt->addr_type;
    fop->timestamp     = timestamp;

    fop->stealth       = pkt->stealth;
    fop->no_track      = pkt->no_track;
    fop->aircraft_type = pkt->aircraft_type;

    float ref_lat      = this_aircraft->latitude;
    float ref_lon      = this_aircraft->longitude;
    float geo_separ    = this_aircraft->geoid_separation;

    int16_t alt = descale(pkt->alt, 12, 1) - 1000 ; /* relative to WGS84 ellipsoid */
    fop->altitude      = (float) alt - geo_separ;

    int32_t round_lat  = (int32_t) (ref_lat * 1e7) / 52;
    int32_t lat        = (pkt->lat - round_lat) % (uint32_t) 0x100000;
    if (lat >= 0x080000) lat -= 0x100000;
    lat                = ((lat + round_lat) * 52) /* + 0x40 */;
    fop->latitude      = (float)lat / 1e7;

    int ilat           = (int)fabs(fop->latitude);
    if (ilat > 89) { ilat = 89; }
    int32_t lon_div    = (ilat < 14) ? 52 : lon_div_table[ilat-14];

    int32_t round_lon  = (int32_t) (ref_lon * 1e7) / lon_div;
    int32_t lon        = (pkt->lon - round_lon) % (uint32_t) 0x100000;
    if (lon >= 0x080000) lon -= 0x100000;
    lon                = ((lon + round_lon) * lon_div) /* + 0x40 */;
    fop->longitude     = (float)lon / 1e7;

    uint16_t speed10   = (uint16_t) descale(pkt->hs, 8, 2);
    fop->speed         = speed10 / (10 * _GPS_MPS_PER_KNOT);

    int16_t vs10       = (int16_t) descale(pkt->vs, 6, 2);
    fop->vs            = ((float) vs10) * (_GPS_FEET_PER_METER * 6.0);

    float course       = pkt->course;
    fop->course        = course / 2;

    /* TODO */

    return true;
}

#if defined(USE_INTERLEAVING)
static bool use_v6_on_tx = true;
#endif /* USE_INTERLEAVING */

size_t legacy_encode(void *legacy_pkt, ufo_t *this_aircraft) {

#if !defined(EXCLUDE_AIR6) && defined(USE_INTERLEAVING)
    if (use_v6_on_tx) {
      use_v6_on_tx = false;
      return legacy_v6_encode(legacy_pkt, this_aircraft);
    }

    use_v6_on_tx = true;
#endif /* USE_INTERLEAVING */

    const uint32_t xxtea_key[4] = LEGACY_KEY5;
    uint32_t key_v7[4];

    legacy_v7_packet_t *pkt = (legacy_v7_packet_t *) legacy_pkt;
    uint32_t *wpkt     = (uint32_t *) legacy_pkt;

    uint32_t id        = this_aircraft->addr;
    uint8_t acft_type  = this_aircraft->aircraft_type > AIRCRAFT_TYPE_STATIC ?
                         AIRCRAFT_TYPE_UNKNOWN : this_aircraft->aircraft_type;

    int32_t lat        = (int32_t) (this_aircraft->latitude  * 1e7);
    int32_t lon        = (int32_t) (this_aircraft->longitude * 1e7);
    int16_t alt = (int16_t) (this_aircraft->altitude  + this_aircraft->geoid_separation);
    uint32_t timestamp = (uint32_t) this_aircraft->timestamp;

    float course       = this_aircraft->course;
    float speedf       = this_aircraft->speed * _GPS_MPS_PER_KNOT; /* m/s */
    float vsf          = this_aircraft->vs / (_GPS_FEET_PER_METER * 60.0); /* m/s */

    pkt->addr          = id & 0x00FFFFFF;
    pkt->type          = 2; /* Air V7 position */

#if !defined(SOFTRF_ADDRESS)
    pkt->addr_type     = ADDR_TYPE_FLARM; /* ADDR_TYPE_ANONYMOUS */
#else
    pkt->addr_type     = (pkt->addr == SOFTRF_ADDRESS ?
                          ADDR_TYPE_ICAO : ADDR_TYPE_FLARM); /* ADDR_TYPE_ANONYMOUS */
#endif

    pkt->stealth       = this_aircraft->stealth;
    pkt->no_track      = this_aircraft->no_track;

    pkt->tstamp        = timestamp & 0xF;
    pkt->aircraft_type = acft_type;

    alt += 1000;
    if (alt < 0) { alt = 0; }
    pkt->alt           = enscale_unsigned(alt, 12, 1); /* 0 ... 12286 */

    pkt->lat = ((lat / 52) + (lat & 0x40 /* TBD */ ? (lat < 0 ? -1 : 1) : 0)) & 0xFFFFF;

    int ilat           = (int)fabs(this_aircraft->latitude);
    if (ilat > 89) { ilat = 89; }
    int32_t lon_div    = (ilat < 14) ? 52 : lon_div_table[ilat-14];

    pkt->lon           = (lon / lon_div) & 0xFFFFF;

    pkt->turn          = 0; /* TBD */

    uint16_t speed10   = (uint16_t) roundf(speedf * 10.0f);
    pkt->hs            = enscale_unsigned(speed10, 8, 2); /* 0 ... 3832 */

    int16_t vs10       = (int16_t) roundf(vsf * 10.0f);
    pkt->vs = this_aircraft->stealth ? 0 : enscale_signed(vs10, 6, 2); /* 0 ... 952 */

    pkt->course        = (int) (course * 2);
    pkt->airborne      = ((int) speedf) >= legacy_GS_threshold[acft_type] ? 2 : 1;

/*
 * TODO
 * Volunteer contributors are welcome:
 * https://pastebin.com/YB1ppAbt
 */
    pkt->_unk1         = 0;
    pkt->_unk2         = 0;
    pkt->_unk3         = 3;
    pkt->_unk4         = 0;
    pkt->_unk5         = 3;
    pkt->_unk6         = 0;
    pkt->_unk7         = 0;
    pkt->_unk8         = 0;
    pkt->_unk9         = 0; /* TBD */
    pkt->_unk10        = 0;

    key_v7[0]          = wpkt[0];
    key_v7[1]          = wpkt[1];
    key_v7[2]          = timestamp >> 4;
    key_v7[3]          = LEGACY_KEY4;

    make_v7_key(key_v7);

    wpkt[2] ^= key_v7[0];
    wpkt[3] ^= key_v7[1];
    wpkt[4] ^= key_v7[2];
    wpkt[5] ^= key_v7[3];

    btea(&wpkt[2], 4, xxtea_key);

    return (sizeof(legacy_v7_packet_t));
}

#endif /* EXCLUDE_AIR7 */
