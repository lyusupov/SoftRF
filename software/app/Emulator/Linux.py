#
#  Linux.py
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


from gps import gps
from time import mktime
from calendar import timegm

import iso8601
import os

WATCH_ENABLE	= 0x000001	# enable streaming
WATCH_DISABLE	= 0x000002	# disable watching        
WATCH_DEVICE	= 0x000800	# watch specific device
WATCH_NEWSTYLE	= 0x010000	# force JSON streaming


def hostid():
    return os.popen("hostid").read().strip()

def platform_name():
    return 'Linux'

def platform_init(emu):

    emu.gpsd = gps()
    device = "/dev/ttyUSB0" # device
    emu.myId = format(int(hostid(),16) & 0xffffff, 'x')

    #gpsd.stream(WATCH_DEVICE, device)
    emu.gpsd.stream(WATCH_ENABLE)

def platform_get_fix(emu):

    gpsd = emu.gpsd
    gpsd.read()

    if gpsd.data.get("class") == "TPV":

      if False:
        print
        print ' GPS reading' , gpsd.data.get("class") 
        print '----------------------------------------'
        print 'fix         ' , ("NO_FIX","FIX","DGPS_FIX")[gpsd.fix.mode - 1]
        print 'latitude    ' , gpsd.fix.latitude
        print 'longitude   ' , gpsd.fix.longitude
        print 'time utc    ' , gpsd.utc # , session.fix.time
        print 'altitude    ' , gpsd.fix.altitude
        print 'track       ' , gpsd.fix.track
        print 'speed       ' , gpsd.fix.speed
        print

      if (gpsd.utc != '') and (gpsd.utc != None):
        dt = iso8601.parse_date(gpsd.utc)
        emu.mytstamp = int(long(timegm(dt.timetuple())*1000 + dt.microsecond/1000) / 1000)

      if gpsd.fix.mode > 1:
        emu.mylat = gpsd.fix.latitude
        emu.mylon = gpsd.fix.longitude
        emu.myalt = gpsd.fix.altitude
        emu.mytrk = gpsd.fix.track
        return True

    return False  

def platform_fini():

    return
