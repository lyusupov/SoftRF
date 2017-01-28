/* flarm_decode, decoder for FLARM radio protocol
 * Copyright (C) 2014 Stanislaw Pusep
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

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#include "flarm_codec.h"

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

/* https://metacpan.org/source/GRAY/Geo-Distance-XS-0.13/XS.xs */
const float DEG_RADS = M_PI / 180.;
const float KILOMETER_RHO = 6371.64;
float haversine(float lat1, float lon1, float lat2, float lon2) {
    lat1 *= DEG_RADS; lon1 *= DEG_RADS;
    lat2 *= DEG_RADS; lon2 *= DEG_RADS;
    float a = sin(0.5 * (lat2 - lat1));
    float b = sin(0.5 * (lon2 - lon1));
    float c = a * a + cos(lat1) * cos(lat2) * b * b;
    float d = 2. * atan2(sqrt(c), sqrt(fabs(1. - c)));
    return d;
}

/* http://pastebin.com/YK2f8bfm */
long obscure(uint32_t key, uint32_t seed) {
    uint32_t m1 = seed * (key ^ (key >> 16));
    uint32_t m2 = (seed * (m1 ^ (m1 >> 16)));
    return m2 ^ (m2 >> 16);
}

void make_key(uint32_t key[4], uint32_t timestamp, uint32_t address) {
    static const uint32_t table[4] = FLARM_KEY1;
    int8_t i;
    for (i = 0; i < 4; i++)
        key[i] = obscure(table[i] ^ ((timestamp >> 6) ^ address), FLARM_KEY2) ^ FLARM_KEY3;
}

char *flarm_decode(flarm_packet *pkt, float ref_lat, float ref_lon, int16_t ref_alt, double timestamp, float rssi, int16_t channel) {
    if (!(pkt->magic == 0x10 || pkt->magic == 0x20)) {
        Serial.print(F("bad packet signature: "));
        Serial.println(pkt->magic, HEX);
        return NULL;
    }
    char tmp[32];
    static char out[512];
    uint32_t key[4];

    char str_lat[16];
    char str_lon[16];
    char str_dist[16];
    
    make_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
    btea((uint32_t *) pkt + 1, -5, key);

    int32_t round_lat = (int32_t) (ref_lat * 1e7) >> 7;
    int32_t lat = (pkt->lat - round_lat) % (uint32_t) 0x080000;
    if (lat >= 0x040000) lat -= 0x080000;
    lat = ((lat + round_lat) << 7) + 0x40;

    int32_t round_lon = (int32_t) (ref_lon * 1e7) >> 7;
    int32_t lon = (pkt->lon - round_lon) % (uint32_t) 0x100000;
    if (lon >= 0x080000) lon -= 0x100000;
    lon = ((lon + round_lon) << 7) + 0x40;

    int32_t vs = pkt->vs * (1 << pkt->vsmult);

    float dist = haversine(ref_lat, ref_lon, (float)lat / 1e7, (float)lon / 1e7) * KILOMETER_RHO * 1000;
    int16_t alt = pkt->alt - ref_alt;

    out[0] = '\0';

    #define json_concat(...)                        \
        snprintf(tmp, sizeof(tmp), ##__VA_ARGS__);  \
        strncat(out, tmp, sizeof(out) - sizeof(tmp) - 1);

    json_concat("{\"addr\":\"%u\",", pkt->addr); // Arduino's JSON has issues with HEX parsing
    if (timestamp > 1.4e9) {
        json_concat("\"time\":%d,", (uint32_t) timestamp);
    }
    if (fabs(rssi) > 0.01) {
        json_concat("\"rssi\":%.01f,", rssi);
    }
    if (channel > 0) {
        json_concat("\"channel\":%d,", channel);
    }

    dtostrf((float)lat / 1e7, 8, 4, str_lat);
    dtostrf((float)lon / 1e7, 8, 4, str_lon);
    dtostrf(sqrt(pow(dist, 2) + pow(alt, 2)), 8, 2, str_dist);

    json_concat("\"lat\":%s,", str_lat);
    json_concat("\"lon\":%s,", str_lon);
    json_concat("\"dist\":%s,", str_dist);
    json_concat("\"alt\":%d,", alt);
    json_concat("\"vs\":%d,", vs);
    json_concat("\"type\":%d,", pkt->type);
    json_concat("\"stealth\":%d,", pkt->stealth);
    json_concat("\"no_track\":%d,", pkt->no_track);
    json_concat("\"ns\":[%d,%d,%d,%d],", pkt->ns[0], pkt->ns[1], pkt->ns[2], pkt->ns[3]);
    json_concat("\"ew\":[%d,%d,%d,%d]}", pkt->ew[1], pkt->ew[1], pkt->ew[2], pkt->ew[3]);

    return out;
}
