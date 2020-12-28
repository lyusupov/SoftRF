#
#  NMEA.py
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


from misc import EarthDistance, MeterOffset, Rad2Deg
from math import atan2
from math import sin, cos, radians

import re

DISTANCE_CLOSE = 500
DISTANCE_NEAR  = 1500
DISTANCE_FAR   = 5000
ALTITUDE_FAR   = 2000

ALARM_LEVEL_LOW       = 1
ALARM_LEVEL_IMPORTANT = 2
ALARM_LEVEL_URGENT    = 3

""" Calculate  the checksum for NMEA sentence 
    from a GPS device. An NMEA sentence comprises
    a number of comma separated fields followed by
    a checksum (in hex) after a "*". An example
    of NMEA sentence with a correct checksum (of
    0x76) is:
    
      GPGSV,3,3,10,26,37,134,00,29,25,136,00*76"
"""

def checksum(sentence):
    """ Remove leading $ """

    sentence = sentence.lstrip('$')
    nmeadata,cksum = re.split('\*', sentence)
    #print nmeadata

    calc_cksum = 0
    for s in nmeadata:
        calc_cksum ^= ord(s)

    return calc_cksum

def export_nmea(emu, icao, trlat, trlgt, tralt):

    distance = EarthDistance((trlat, trlgt), (emu.mylat, emu.mylon))
    alt_diff = tralt - emu.refalt
    if (distance < DISTANCE_FAR and alt_diff < ALTITUDE_FAR): # Filter out unrealistic readings
      (dx, dy) = MeterOffset((trlat, trlgt), (emu.mylat, emu.mylon))
      
      math_angle = Rad2Deg(atan2(dy , dx))
      
      azim = 90 - math_angle
      if emu.NorthUp is False:
        azim = azim - emu.mytrk
      azim = (azim + 360) % 360
      
      bearing = int(azim)
      if bearing > 180:
        bearing = bearing - 360
      #print distance , azimuth

      if (distance < DISTANCE_CLOSE):
        alarm_level = ALARM_LEVEL_URGENT
      else:
        if (distance < DISTANCE_NEAR):
          alarm_level = ALARM_LEVEL_IMPORTANT
        else:
          alarm_level = ALARM_LEVEL_LOW          

      str1 = "$PFLAU,%d,1,2,1,%d,%d,2,%d,%u*" % \
        ( 1, alarm_level, bearing, int(alt_diff), int(distance) )
      csum = checksum(str1)
      str1 += "%02x\r\n" % csum
      #print str1
      #self.tty.write(str1)
      emu.x.sendto(bytearray(str1), (emu.xcsoar_host, emu.xcsoar_port))
      # XCSoar ignores warning data from PFLAU
      azim_rad = radians(azim)
      str2 = "$PFLAA,%d,%d,%d,%d,2,%s,,,,,1*" % \
        ( alarm_level, int(distance * cos(azim_rad)), \
          int(distance * sin(azim_rad)), int(alt_diff), icao )
      csum = checksum(str2)
      str2 += "%02x\r\n" % csum
      #print str2
      #self.tty.write(str2)
      emu.x.sendto(bytearray(str2), (emu.xcsoar_host, emu.xcsoar_port))
