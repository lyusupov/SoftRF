#
#  Android.py
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

from time import sleep
import androidhelper
import os
import hashlib


def hostid():
    #return os.popen("settings get secure android_id").read().strip()
    serial = os.popen("getprop ro.serialno").read().strip()
    hash_object = hashlib.md5(serial)
    return hash_object.hexdigest()

def platform_name():
    return 'Android'

def platform_init(emu):

    emu.myId = format(int(hostid(),16) & 0xffffff, 'x')
    emu.droid = androidhelper.Android()
    emu.droid.startLocating(1000, 0)

def platform_get_fix(emu):

    emu.droid.eventWaitFor('location')
    loc = emu.droid.readLocation().result
    # print loc
    if loc != {}:
     try:
       n = loc['gps']
     except KeyError:
       n = loc['network'] 
     timestamp = n['time']
     lat = n['latitude'] 
     lon = n['longitude']
     alt = n['altitude']
     bearing = n['bearing'] 
     speed = n['speed'] 
     accuracy = n['accuracy']
     prov = n['provider']
    
     #print
     #print cnt
     #print "timestamp: ", timestamp
     #print "latitude:  ", lat
     #print "longtitude:", lon
     #print "altitude:  ", alt
     #print "bearing:   ", bearing
     #print "speed:     ", speed
     #print "accuracy:  ", accuracy
     #print "provider:  ", prov
     #print "S", cnt, int(timestamp / 1000), "%.4f" % lat, "%.4f" % lon, int(alt)
     #result = session.process_e(timestamp, lat, lon, alt)
     #cnt = cnt + 1
     emu.mytstamp = int(timestamp / 1000)
     emu.mylat = lat
     emu.mylon = lon
     emu.myalt = int(alt)
     emu.mytrk = bearing
     sleep(1)
     return True

def platform_fini(emu):
    emu.droid.stopLocating()
    return
