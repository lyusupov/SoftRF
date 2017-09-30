#!/usr/bin/env python
#
#  Copyright (c) 2016 Eric Dey
#  
#  Permission is hereby granted, free of charge, to any person obtaining
#  a copy of this software and associated documentation files (the "Software"),
#  to deal in the Software without restriction, including without limitation
#  the rights to use, copy, modify, merge, publish, distribute, sublicense,
#  and/or sell copies of the Software, and to permit persons to whom
#  the Software is furnished to do so, subject to the following conditions:
#  
#  The above copyright notice and this permission notice shall be included
#  in all copies or substantial portions of the Software.
#  
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
#  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
#  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
#  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
#  DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
#  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
#  OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#
# fcs.py
#

"""GDL frame check sequence functions"""

CRC16Table = (
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
)

def crcCompute(data):
    mask16bit = 0xffff
    crcArray = bytearray()
    
    crc = 0
    i = 0
    for c in data:
        m = (crc << 8) & mask16bit
        crc = CRC16Table[(crc >> 8)] ^ m ^ c
    
    crcArray.append(chr((crc & 0x00ff)))
    crcArray.append(chr((crc & 0xff00) >> 8))
    return crcArray

#
# encoder.py
#

import sys
import datetime
import struct

class Encoder(object):
    """GDL-90 data link interface decoder class"""

    def __init__(self):
        pass
    
    
    def _addCrc(self, msg):
        """compute the CRC for msg and append CRC bytes to msg"""
        crcBytes = crcCompute(msg)
        msg.extend(crcBytes)
    
    
    def _escape(self, msg):
        """escape 0x7d and 0x7e characters"""
        msgNew = bytearray()
        escapeChar = chr(0x7d)
        charsToEscape = (chr(0x7d), chr(0x7e))
        
        for i in range(len(msg)):
            c = chr(msg[i])
            if c in charsToEscape:
                msgNew.append(escapeChar)
                msgNew.append(chr(ord(c) ^ 0x20))
            else:
                msgNew.append(c)
        
        return(msgNew)
    
    
    def _preparedMessage(self, msg):
        """returns a prepared a message with CRC, escapes it, adds begin/end markers"""
        self._addCrc(msg)
        newMsg = self._escape(msg)
        newMsg.insert(0,chr(0x7e))
        newMsg.append(chr(0x7e))
        return(newMsg)
    
    
    def _pack24bit(self, num):
        """make a 24 bit packed array (MSB) from an unsigned number"""
        a = bytearray()
        a.append((num & 0xff0000) >> 16)
        a.append((num & 0x00ff00) >> 8)
        a.append(num & 0xff)
        return(a)


    def _makeLatitude(self, latitude):
        """convert a signed integer latitude to 2s complement ready for 24-bit packing"""
        if latitude > 90.0:  latitude = 90.0
        if latitude < -90.0:  latitude = -90.0
        latitude = int(latitude * (0x800000 / 180.0))
        if latitude < 0:
            latitude = (0x1000000 + latitude) & 0xffffff  # 2s complement
        return(latitude)


    def _makeLongitude(self, longitude):
        """convert a signed integer longitude to 2s complement ready for 24-bit packing"""
        if longitude > 180.0:  longitude = 180.0
        if longitude < -180.0:  longitude = -180.0
        longitude = int(longitude * (0x800000 / 180.0))
        if longitude < 0:
            longitude = (0x1000000 + longitude) & 0xffffff  # 2s complement
        return(longitude)
    
    
    def msgHeartbeat(self, st1=0x81, st2=0x00, ts=None, mc=0x0000):
        """message ID #0"""
        # Auto-fill timestamp if not provided
        if ts is None:
            dt = datetime.datetime.utcnow()
            ts = (dt.hour * 3600) + (dt.minute * 60) + dt.second
        
        # Move 17-bit into status byte 2 if necessary
        if (ts & 0x10000) != 0:
            ts = ts & 0x0ffff
            st2 = st2 | 0x80
        
        msg = bytearray(chr(0x00))
        fmt = '>BBHH'
        msg.extend(struct.pack(fmt,st1,st2,ts,mc))
        
        return(self._preparedMessage(msg))
    
    
    def msgOwnershipReport(self, status=0, addrType=0, address=0, latitude=0.0, longitude=0.0, altitude=0, misc=9, navIntegrityCat=8, navAccuracyCat=8, hVelocity=None, vVelocity=None, trackHeading=0, emitterCat=1, callSign='', code=0):
        """message ID #10"""
        return(self._msgType10and20(10, status, addrType, address, latitude, longitude, altitude, misc, navIntegrityCat, navAccuracyCat, hVelocity, vVelocity, trackHeading, emitterCat, callSign, code))
    
    
    def msgTrafficReport(self, status=0, addrType=0, address=0, latitude=0.0, longitude=0.0, altitude=0, misc=9, navIntegrityCat=8, navAccuracyCat=8, hVelocity=None, vVelocity=None, trackHeading=0, emitterCat=1, callSign='', code=0):
        """message ID #20"""
        return(self._msgType10and20(20, status, addrType, address, latitude, longitude, altitude, misc, navIntegrityCat, navAccuracyCat, hVelocity, vVelocity, trackHeading, emitterCat, callSign, code))
    
    
    def _msgType10and20(self, msgid, status, addrType, address, latitude, longitude, altitude, misc, navIntegrityCat, navAccuracyCat, hVelocity, vVelocity, trackHeading, emitterCat, callSign, code):
        """construct message ID 10 or 20"""
        msg = bytearray(chr(msgid))
        
        b = ((status & 0xf) << 4) | (addrType & 0xf)
        msg.append(b)
        
        msg.extend(self._pack24bit(address))
        
        msg.extend(self._pack24bit(self._makeLatitude(latitude)))
        
        msg.extend(self._pack24bit(self._makeLongitude(longitude)))
        
        altitude = (int(altitude) + 1000) / 25
        if altitude < 0:  altitude = 0
        if altitude > 0xffe:  altitude = 0xffe
        
        # altitude is bits 15-4, misc code is bits 3-0
        msg.append((altitude & 0xff0) >> 4)  # top 8 bits of altitude
        msg.append( ((altitude & 0xf) << 4) | (misc & 0xf) )
        
        # nav int cat is top 4 bits, acc cat is bottom 4 bits
        msg.append( ((navIntegrityCat & 0xf) << 4) | (navAccuracyCat & 0xf) )
        
        if hVelocity is None:
            hVelocity = 0xfff
        elif hVelocity < 0:
            hVelocity = 0
        elif hVelocity > 0xffe:
            hVelocity = 0xffe
        
        if vVelocity is None:
            vVelocity = 0x800
        else:
            if vVelocity > 32576:
                vVelocity = 0x1fe
            elif vVelocity < -32576:
                vVelocity = 0xe02
            else:
                vVelocity = int(vVelocity / 64)  # convert to 64fpm increments
                if vVelocity < 0:
                    vVelocity = (0x1000000 + vVelocity) & 0xffffff # 2s complement
        
        # packing hVelocity, vVelocity into 3 bytes:  hh hv vv
        msg.append((hVelocity & 0xff0) >> 4)
        msg.append( ((hVelocity & 0xf) << 4) | ((vVelocity & 0xf00) >> 8) )
        msg.append(vVelocity & 0xff)
        
        trackHeading = int(trackHeading / (360. / 256)) # convert to 1.4 deg single byte
        msg.append(trackHeading & 0xff)
        
        msg.append(emitterCat & 0xff)
        
        callSign = str(callSign + " "*8)[:8]
        msg.extend(callSign)
        
        # code is top 4 bits, bottom 4 bits are 'spare'
        msg.append((code & 0xf) << 4)
        
        return(self._preparedMessage(msg))
    
    
    def msgOwnershipGeometricAltitude(self, altitude=0, merit=50, warning=False):
        """message ID #11"""
        msg = bytearray(chr(0x0b))
        
        # Convert altitude to 5ft increments
        altitude = int(altitude / 5)
        if altitude < 0:
            altitude = (0x10000 + altitude) & 0xffff  # 2s complement
        msg.extend(struct.pack('>H', altitude))  # 16-bit big endian
        
        if merit is None:
            merit = 0x7fff
        elif merit > 32766:
            merit = 0x7ffe
        
        # MSB is warning bit, 6-0 bits are MSB of merit value
        b = (merit & 0x7f00) >> 8  # top 7 bits of merit value
        if warning:
            b = b | 0x80  # set MSB to 1
        msg.append(b)
        msg.append(merit & 0xff)  # bottom 8 bits of merit value
        
        return(self._preparedMessage(msg))
    
    
    def msgGpsTime(self, count=0, quality=2, hour=None, minute=None):
        """message ID #101 for Skyradar"""
        msg = bytearray(chr(0x65))
        
        msg.append(0x2a) # firmware version
        msg.append(0) # debug data
        msg.append(chr((0x30 + quality) & 0xff))  # GPS quality: '0'=no fix, '1'=regular, '2'=DGPS (WAAS)
        msg.extend(struct.pack('<I',count)[:-1])  # use first three LSB bytes only
        
        if hour is None or minute is None:
             # Auto-fill timestamp if not provided
                dt = datetime.datetime.utcnow()
                hour = dt.hour
                minute = dt.minute
        msg.append(hour & 0xff)
        msg.append(minute & 0xff)
        
        msg.append(0); msg.append(0)  # debug data
        msg.append(4) # hardware version
        
        return(self._preparedMessage(msg))
    
    
    def msgStratuxHeartbeat(self, st1=0x02, ver=1):
        """message ID #204 for Stratux heartbeat"""
        msg = bytearray(chr(0xcc))
        
        fmt = '>B'
        data = st1 & 0x03  # lower two bits only
        data += ((ver & 0x3f) << 2)  # lower 6 bits of version packed into upper 6 of data
        msg.extend(struct.pack(fmt,data))
        
        return(self._preparedMessage(msg))
    
    
    def msgSXHeartbeat(self, fv=0x0001, hv=0x0001, st1=0x02, st2=0x01, satLock=0, satConn=0, num978=0, num1090=0, rate978=0, rate1090=0, cpuTemp=0, towers=[]):
        """message ID #29 for Hiltonsoftware SX heartbeat"""
        
        msg = bytearray(chr(0x1d))
        fmt = '>ccBBLLHHBBHHHHHB'
        msg.extend(struct.pack(fmt,'S','X',1,1,fv,hv,st1,st2,satLock,satConn,num978,num1090,rate978,rate1090,cpuTemp,len(towers)))

        for tower in towers:
            (lat, lon) = tower[0:2]
            msg.extend(self._pack24bit(self._makeLatitude(lat)))
            msg.extend(self._pack24bit(self._makeLongitude(lon)))
        
        return(self._preparedMessage(msg))

