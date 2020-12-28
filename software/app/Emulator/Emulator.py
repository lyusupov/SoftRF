#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Emulator.py
#  Copyright (C) 2016-2021 Linar Yusupov
# 
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

from sys import argv
from socket import socket, gethostname, gethostbyname, AF_INET, SOCK_DGRAM, \
      SOL_SOCKET, SO_REUSEADDR, SO_BROADCAST
from socket import inet_ntoa, inet_aton, error
from time import time, sleep

import platform
import errno
import numpy

if platform.system() == "Linux":
  if platform.dist()[0] == 'debian':
    #
    # Raspbian
    #
    #from gps import gps
    from Linux import platform_init, platform_get_fix, platform_fini, platform_name

  else:
    #
    # Android
    #  
    
    from Android import platform_init, platform_get_fix, platform_fini, platform_name

from legacy_protocol import pack_values, make_key, encrypt_packet, \
     hex_to_bits, decrypt_packet, extract_values, parityOf, recover_lat, \
     recover_lon

from NMEA import export_nmea
from GDL90 import Encoder
from math import isnan

#DEF_SEND_ADDR="255.255.255.255"
DEF_SEND_ADDR="192.168.1.255"
DEF_SEND_PORT=4000

#Snippet for getting the default gateway on Linux
#No dependencies beyond Python stdlib

import struct

def get_default_gateway_linux():
    """Read the default gateway directly from /proc."""
    with open("/proc/net/route") as fh:
        for line in fh:
            fields = line.strip().split()
            if fields[1] != '00000000' or not int(fields[3], 16) & 2:
                continue

            return inet_ntoa(struct.pack("=L", int(fields[2], 16)))

class legacy_emulator:

    def __init__(self, bridge_host='192.168.1.255', bridge_port=12390, \
      xcsoar_host='localhost' , xcsoar_port=10110):
      print 'legacy_emulator: bridge_host =' , bridge_host , ', bridge_port =' , bridge_port , \
        ', xcsoar_host =' , xcsoar_host, ', xcsoar_port =' , xcsoar_port
      if bridge_host:
        self.src_host = gethostbyname(bridge_host)        
      else:
        myip = gethostbyname(gethostname())
        broadip = inet_ntoa( inet_aton(myip)[:3] + b'\xff' )
        self.src_host = gethostbyname(broadip)
      self.src_port = bridge_port
      #self.dst_host = gethostbyname(get_default_gateway_linux())
      self.dst_host = gethostbyname('192.168.1.1')
      self.dst_port = bridge_port - 1
      self.xcsoar_host = gethostbyname(xcsoar_host)
      self.xcsoar_port = xcsoar_port
      self.NorthUp = True
      self.s = socket(AF_INET, SOCK_DGRAM)
      self.s.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
      self.s.bind((self.src_host, bridge_port))
      self.s.setblocking(0)
      self.d = socket(AF_INET, SOCK_DGRAM)
      self.x = socket(AF_INET, SOCK_DGRAM)
      self.g = socket(AF_INET, SOCK_DGRAM)
      self.g.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
      self.g.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
      self.gdl90_encoder = Encoder()
      self.tx_cnt = 0
      self.rx_cnt = 0
      self.mylat = 0
      self.mylon = 0
      self.myalt = 0
      self.mytrk = 0
      self.mytstamp = 0
      self.myId = "deadbf"
      self.refalt = 0

    def process_d(self):

        try:
          message = self.s.recv(8192)

          timestamp_d = time()
          
          page = message.split("\n")
          for record in page:
            if len(record) > 0:
              bits = hex_to_bits(record)
            
              in_bytes = numpy.packbits(bits)
      
              raw_hex = "".join(["{0:02x}".format(byte) for byte in in_bytes])
              if (in_bytes[3] == 0x20 or (in_bytes[3] == 0x22)):
                  key = make_key(int(timestamp_d), (in_bytes[1] << 16) | (in_bytes[0] << 8))
                  #print "Key: " + str(key)
                  bytes = decrypt_packet(in_bytes, key)
                  #print bytes
                  icao, lat, lon, alt, vs, no_track, stealth, typ, ns, ew, status, unk = extract_values(bytes[0:24])

                  parity = 0
                  for sym in bytes:
                    parity ^= parityOf(sym)
                  if (parity & 0x1) == 0:
                    lat = recover_lat(lat, int(self.mylat * 1e7))
                    lon = recover_lon(lon, int(self.mylon * 1e7))
                    if False: # True: 
                      print "Timestamp: " + str(timestamp_d),
                      print "ICAO: " + icao,
                      print "Lat: " + str(lat),
                      print "Lon: " + str(lon),
                      print "Alt: " + str(alt) + "m",
                      print "VS: " + str(vs),
                      print "No-track: " + str(no_track),
                      print "Stealth: " + str(stealth),
                      print "Type: " + str(typ),
                      print "GPS status: " + str(status),
                      print "North/South speeds: {0},{1},{2},{3}".format(*ns),
                      print "East/West speeds: {0},{1},{2},{3}".format(*ew),
                      print "Unknown: {0:04x}".format(unk),
                      print "Raw: {0:02x}".format(bytes[3]),
                      print "{0:02x}{1:02x}{2:02x}{3:02x}{4:02x}{5:02x}{6:02x}{7:02x}".format(*bytes[4:12]),
                      print "{0:02x}{1:02x}{2:02x}{3:02x}{4:02x}{5:02x}{6:02x}{7:02x}".format(*bytes[12:20]),
                      print "{0:02x}{1:02x}{2:02x}{3:02x}".format(*bytes[20:24]),

                      print
                    #else:
                    #  print raw_hex
  
                    trlat = lat / 1e7
                    trlgt = lon / 1e7
                    return (icao, trlat, trlgt, alt)
      
              else:
                  print "Don't know how to decrypt packet type {0:02x}".format(in_bytes[3])
                  icao, _, _, _, _, _, _, _, _, _, _, _ = extract_values(in_bytes[0:24])
                  lat, lon, alt = -1, -1, -1

              return (0,0,0,0) # ""

        except error, e:
          if e.args[0] == errno.EWOULDBLOCK: 
            #print 'EWOULDBLOCK'
            #time.sleep(1)           # short delay, no tight loops
            return (0,0,0,0) # ""
          else:
            print e
          return (0,0,0,0) # "" # "$FLM\r\n" 

    def process_e(self):
        type = 1
        status = 323
        vs = vsmult = no_track = stealth = 0
        out_bytes = pack_values(self.myId, vs, status, type, self.mylat, self.mylon, self.myalt, vsmult, no_track, stealth)
        #print timestamp, out_bytes
        key = make_key(self.mytstamp, (out_bytes[1] << 16) | (out_bytes[0] << 8))
        enc_bytes = encrypt_packet(out_bytes, key)
        self.d.sendto(bytearray(enc_bytes), (self.dst_host, self.dst_port))

if __name__ == "__main__":

    opts = { }
    argc = len(argv)
    if argc > 1:
      opts["bridge_host"]    = argv[1]
  
    if argc > 2:
      opts["bridge_port"]    = int(argv[2])
  
    if argc > 3:
      opts["xcsoar_host"]    = argv[3]
    
    if argc > 4:
      opts["xcsoar_port"]    = int(argv[4])


    session = legacy_emulator(**opts)
    
    platform_init(session)


    destAddr = DEF_SEND_ADDR
    destPort = int(DEF_SEND_PORT)
    
    try:
      while True:
  
        # Heartbeat Message
        buf = session.gdl90_encoder.msgHeartbeat()
        if platform_name() != 'Android':  # workaround against broadcast UDP packets issue
          session.g.sendto(buf, (destAddr, destPort))

        result = platform_get_fix(session)
        if result:

          if isnan(session.mytrk):
            heading = 0
          else:
            heading = int(session.mytrk)

          if isnan(session.myalt):
            session.myalt = 0

          # Demo data
          # session.mylat = 43.9790152
          # session.mylon = -88.5559553
	
          # Ownership Report
          buf = session.gdl90_encoder.msgOwnershipReport(latitude=session.mylat, longitude=session.mylon, altitude=session.myalt, trackHeading=heading, callSign=session.myId)
          if platform_name() != 'Android':  # workaround against broadcast UDP packets issue
            session.g.sendto(buf, (destAddr, destPort))

          print "S", session.tx_cnt, session.mytstamp, session.myId, "%.4f" % session.mylat, "%.4f" % session.mylon, int(session.myalt)
          session.process_e()
          session.tx_cnt = session.tx_cnt + 1

        (icao, lat, lon, alt) = session.process_d()
        if icao != 0:
          print  "R", session.rx_cnt, session.mytstamp, icao, "%.4f" % lat, "%.4f" % lon, int(alt)

          export_nmea(session, icao, lat, lon, alt)

          # Demo data
          # lat = 43.97446
          # lon = -88.6032945
          # alt = 1000

          tcall ='FLARM'
          icao_bytes = numpy.packbits(hex_to_bits(icao))
          taddr = (icao_bytes[0]<<16) | (icao_bytes[1]<<8) | icao_bytes[2] 
 
          # Traffic Report
          buf = session.gdl90_encoder.msgTrafficReport(latitude=lat, longitude=lon, altitude=alt, callSign=tcall, address=taddr)
          if platform_name() != 'Android':  # workaround against broadcast UDP packets issue
            session.g.sendto(buf, (destAddr, destPort))

          session.rx_cnt = session.rx_cnt + 1

    except KeyboardInterrupt:
      # Avoid garble on ^C
      print ""
      
    platform_fini(session)    