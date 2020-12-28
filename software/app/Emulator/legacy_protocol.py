#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Copyright 2014 Clayton Smith.
#  Copyright (C) 2016-2021 Linar Yusupov
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

import numpy

def u32(x):
    return x & 0xffffffff

def obscure_key(key, seed):
    # print key, seed
    m1 = u32(seed * (key ^ (key >> 16)))
    m2 = u32(seed * (m1 ^ (m1 >> 16)))
    return m2 ^ (m2 >> 16)

def make_key(time, address):
    if (time >> 23) & 1 == 0:
        table = [ 0xe43276df, 0xdca83759, 0x9802b8ac, 0x4675a56b ]
    else:
        table = [ 0xfc78ea65, 0x804b90ea, 0xb76542cd, 0x329dfa32 ]
    return [obscure_key(word ^ ((time>>6) ^ address), 0x045D9F3B) ^ 0x87B562F4 for word in table]

# Adapted from https://github.com/andersekbom/prycut/blob/master/pyxxtea.py
#
# Pure Python (2.x) implementation of the XXTEA cipher
# (c) 2009. Ivan Voras <ivoras@gmail.com>
# Released under the BSD License.
#
def raw_xxtea(v, n, k):
    def MX():
        return ((z>>5)^(y<<2)) + ((y>>3)^(z<<4))^(sum^y) + (k[(p & 3)^e]^z)

    y = v[0]
    sum = 0
    DELTA = 0x9e3779b9
    if n > 1:       # Encoding
        z = v[n-1]
        q = 6 # + 52 / n
        while q > 0:
            q -= 1
            sum = u32(sum + DELTA)
            e = u32(sum >> 2) & 3
            p = 0
            while p < n - 1:
                y = v[p+1]
                z = v[p] = u32(v[p] + MX())
                p += 1
            y = v[0]
            z = v[n-1] = u32(v[n-1] + MX())
        return 0
    elif n < -1:    # Decoding
        n = -n
        q = 6 # + 52 / n
        sum = u32(q * DELTA)
        while sum != 0:
            e = u32(sum >> 2) & 3
            p = n - 1
            while p > 0:
                z = v[p-1]
                y = v[p] = u32(v[p] - MX())
                p -= 1
            z = v[n-1]
            y = v[0] = u32(v[0] - MX())
            sum = u32(sum - DELTA)
        return 0
    return 1

def hex_to_bits(hex_string):
    bytes = []
    for x in range(0, len(hex_string), 2):
        bytes.append(int(hex_string[x:x+2], 16))
    bytes = numpy.array(bytes, dtype=numpy.uint8)
    return numpy.unpackbits(bytes)

def parityOf(int_type):
    parity = 0
    while (int_type):
      parity = ~parity
      int_type = int_type & (int_type - 1)
    return(parity)

def crc16(message):
    poly = 0x1021
    reg = 0xffff
    for byte in message:
        mask = 0x80
        while mask != 0:
            reg <<= 1
            if byte & mask:
                reg ^= 1
            mask >>= 1
            if reg & 0x10000 != 0:
                reg &= 0xffff
                reg ^= poly
    reg ^= 0x9335
    return reg

def decrypt_packet(bytes, key):
    v = []
    result = list(bytes)
    #print "Bytes" , bytes
    for x in range(5):
        v.append(((bytes[4*x+7] << 24) & 0xff000000) | ((bytes[4*x+6] << 16) & 0x00ff0000) | ((bytes[4*x+5] << 8) &0x0000ff00) | (bytes[4*x+4] & 0xff))
    #print "V " , v
    raw_xxtea(v, -5, key)
    for x in range(5):
        result[4*x+7]  = (v[x] >> 24) & 0xff
        result[4*x+6]  = (v[x] >> 16) & 0xff
        result[4*x+5]  = (v[x] >> 8) & 0xff
        result[4*x+4]  =  v[x] & 0xff
    return result

def encrypt_packet(bytes, key):
    v = []
    result = list(bytes)
    #print "Bytes" , bytes
    for x in range(5):
        v.append(((bytes[4*x+7] << 24) & 0xff000000) | ((bytes[4*x+6] << 16) & 0x00ff0000) | ((bytes[4*x+5] << 8) &0x0000ff00) | (bytes[4*x+4] & 0xff))
    #print "V " , v
    raw_xxtea(v, 5, key)
    for x in range(5):
        result[4*x+7]  = (v[x] >> 24) & 0xff
        result[4*x+6]  = (v[x] >> 16) & 0xff
        result[4*x+5]  = (v[x] >> 8) & 0xff
        result[4*x+4]  =  v[x] & 0xff
    return result

def extract_values(bytes):
    icao = "{0:02x}{1:02x}{2:02x}".format(bytes[2], bytes[1], bytes[0])
    vs = ((bytes[5] & 0b00000011) << 8) | bytes[4]
    status = ((bytes[7] & 0b00001111) << 8) | bytes[6]
    typ = ((bytes[7] & 0b11110000) >> 4)
    lat = ((bytes[10] & 0b00000111) << 16) | (bytes[9] << 8) | bytes[8]
    lon = ((bytes[14] & 0b00001111) << 16) | (bytes[13] << 8) | bytes[12]
    alt = (bytes[11] << 5) | ((bytes[10] & 0b11111000) >> 3)
    vsmult = ((bytes[15] & 0b11000000) >> 6)
    if vs < 0x200:
        vs = (vs << vsmult)
    else:
        vs -= 0x400
    no_track = bool(bytes[5] & 0b01000000)
    stealth  = bool(bytes[5] & 0b00100000)
    ns = [b if b < 0x80 else (b - 0x100) for b in bytes[16:20]]
    ew = [b if b < 0x80 else (b - 0x100) for b in bytes[20:24]]
    unk = ((bytes[5] & 0b00011100) << 8) | ((bytes[14] & 0b11110000) << 2) | (bytes[15] & 0b00111111)
    return icao, lat, lon, alt, vs, no_track, stealth, typ, ns, ew, status, unk

def pack_values(icao, vs, status, typ, lat, lon, alt, vsmult, no_track, stealth):
    #icao = "{0:02x}{1:02x}{2:02x}".format(bytes[2], bytes[1], bytes[0])
    icao_bytes = numpy.packbits(hex_to_bits(icao))
    #icao_bytes = numpy.packbits(icao)
    lat = pack_lat(lat)
    lon = pack_lon(lon)
    alt = int(alt)
    
    bytes = []
    bytes.append(icao_bytes[2]) # 0
    bytes.append(icao_bytes[1]) # 1
    bytes.append(icao_bytes[0]) # 2
    bytes.append(0x20)          # 3
    # vs = ((bytes[5] & 0b00000011) << 8) | bytes[4]
    bytes.append(vs & 0xff)     # 4
    bytes.append(((vs & 0x300) >> 8) | (no_track << 6) | (stealth << 5)) # 5
    # status = ((bytes[7] & 0b00001111) << 8) | bytes[6]
    bytes.append(status & 0xff) # 6
    bytes.append(((status & 0xf00) >> 8) | (typ << 4)) # 7
    # typ = ((bytes[7] & 0b11110000) >> 4)
    # lat = ((bytes[10] & 0b00000111) << 16) | (bytes[9] << 8) | bytes[8]
    bytes.append(lat & 0x00ff)  # 8
    bytes.append((lat & 0xff00) >> 8) # 9
    bytes.append(((lat & 0x70000) >> 16) | ((alt & 0b11111) << 3)) # 10
    bytes.append((alt & 0b1111111100000) >> 5) # 11
    # lon = ((bytes[14] & 0b00001111) << 16) | (bytes[13] << 8) | bytes[12]
    bytes.append(lon & 0x00ff) # 12
    bytes.append((lon & 0xff00) >> 8) # 13
    bytes.append((lon & 0xf0000) >> 16) # 14
    # alt = (bytes[11] << 5) | ((bytes[10] & 0b11111000) >> 3)
    # vsmult = ((bytes[15] & 0b11000000) >> 6)
    bytes.append((vsmult & 0b11) << 6) # 15
    #if vs < 0x200:
    #    vs = (vs << vsmult)
    #else:
    #    vs -= 0x400
    # no_track = bool(bytes[5] & 0b01000000)
    # stealth  = bool(bytes[5] & 0b00100000)
    #ns = [b if b < 0x80 else (b - 0x100) for b in bytes[16:20]]
    #ew = [b if b < 0x80 else (b - 0x100) for b in bytes[20:24]]
    #unk = ((bytes[5] & 0b00011100) << 8) | ((bytes[14] & 0b11110000) << 2) | (bytes[15] & 0b00111111)
    bytes.extend([0,0,0,0,0,0,0,0])
    parity = 0
    for sym in bytes:
      parity ^= parityOf(sym)
    if (parity & 0x1) == 1:
      bytes[3] = bytes[3] | (1 << 1)
    return bytes

def pack_lat(float_lat):
    return (int(float_lat * 1e7) >> 7) & 0x7FFFF

def pack_lon(float_lon):
    return (int(float_lon * 1e7) >> 7) & 0xFFFFF

def recover_lat(recv_lat, reflat):
    round_lat = reflat >> 7
    lat = (recv_lat - round_lat) % 0x80000
    if lat >= 0x40000: lat -= 0x80000
    lat = ((lat + round_lat) << 7) + 0x40
    return lat

def recover_lon(recv_lon, reflon):
    round_lon = reflon >> 7
    lon = (recv_lon - round_lon) % 0x100000
    if lon >= 0x80000: lon -= 0x100000
    lon = ((lon + round_lon) << 7) + 0x40
    return lon
