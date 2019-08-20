#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  iLogger.py (main.py)
#  Copyright (C) 2019 Linar Yusupov
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

import sys
import network
import datetime
import machine, display, time
import bme280
import uos
import errno

INTERVAL     = const(1)

I2C0_PIN_SCL = const(22)
I2C0_PIN_SDA = const(21)
I2C1_PIN_SCL = const(26)
I2C1_PIN_SDA = const(25)

GNSS_PIN_RX  = const(34)
GNSS_PIN_TX  = const(33)

TFT_PIN_MISO = const(2)
TFT_PIN_MOSI = const(19)
TFT_PIN_CLK  = const(18)
TFT_PIN_SS   = const(5)
TFT_PIN_DC   = const(27)
TFT_PIN_BL   = const(12)

ST7789_INVON = const(0x21)

AXP202_present = False
TFT_present    = False
GNSS_present   = True
BME_present    = False
SD_present     = False
RTC_present    = False

WiFi_active    = False

ICON_LOGO      = 'icons/logo.jpg'
ICON_NOSD      = 'icons/nosd.jpg'
ICON_NOFIX     = 'icons/nofix.jpg'
ICON_REC       = 'icons/rec.jpg'
ICON_OFF       = 'icons/off.jpg'
ICON_LOWBAT    = 'icons/lowbat.jpg'

from settings import info
from settings import wifi
from AXP202   import axp202
from time     import sleep
from bme280   import BME280_I2CADDR
from aerofiles.igc.writer import Writer
from AXP202.constants     import AXP202_SLAVE_ADDRESS
 
def do_connect():
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print('connecting to network...')
        sta_if.active(True)
        sta_if.connect(wifi['ssid'], wifi['psk'])
        while not sta_if.isconnected():
            pass
    print('network config:', sta_if.ifconfig())

def bmevalues():
    t, p, h = bme.read_compensated_data()

    p = p // 256
    pi = p // 100
    pd = p - pi * 100

    hi = h // 1024
    hd = h * 100 // 1024 - hi * 100

    return "[{}] T={}C  ".format(time.strftime("%H:%M:%S",time.localtime()), t / 100) + "P={}.{:02d}hPa  ".format(pi, pd) + "H={}.{:02d}%".format(hi, hd)

def bl_range(start, end, step):
    while start <= end:
        yield start
        start += step

def pressure_altitude(pressure, qnh=1013.25):
    altitude = 44330.0 * (1.0 - pow(pressure / qnh, (1.0 / 5.255)))
    return altitude

i2c0=machine.I2C(id=0, scl=machine.Pin(I2C0_PIN_SCL),sda=machine.Pin(I2C0_PIN_SDA),speed=400000)
if i2c0.is_ready(AXP202_SLAVE_ADDRESS):
    AXP202_present = True
    TFT_present    = True
    RTC_present    = True

i2c0.deinit()

i2c1=machine.I2C(id=1, scl=machine.Pin(I2C1_PIN_SCL),sda=machine.Pin(I2C1_PIN_SDA),speed=400000)
if i2c1.is_ready(BME280_I2CADDR):
    BME_present = True

i2c1.deinit()

uos.sdconfig(uos.SDMODE_1LINE)
SD_present = True
try:
  uos.mountsd()
except OSError as e:
  if e.args[0] == errno.EIO:
    SD_present = False
  pass

# SD is temporary not in use yet
if SD_present:
    uos.umountsd()
    SD_present = False

# with open('/sd/sample.igc', 'w') as fp:
# igc_writer = Writer(fp)

igc_writer = Writer(sys.stdout)

if AXP202_present:
    a = axp202.PMU()
    a.setChgLEDMode(axp202.AXP20X_LED_BLINK_1HZ)
    # power-up TFT
    a.enablePower(axp202.AXP202_LDO2)
    a.setLDO2Voltage(3300)
    # power-up GNSS
    a.setLDO3Mode(1)
    a.enablePower(axp202.AXP202_LDO3)
    a.setLDO3Voltage(3300)

if TFT_present:
    pwm = machine.PWM(TFT_PIN_BL, freq=12000, duty=0)

    tft = display.TFT()
    tft.init(
      tft.ST7789, miso=TFT_PIN_MISO, mosi=TFT_PIN_MOSI,
      clk=TFT_PIN_CLK, cs=TFT_PIN_SS, dc=TFT_PIN_DC,
      rot=tft.PORTRAIT_FLIP, color_bits=tft.COLOR_BITS16,
      splash=False, height=240
    )

    tft.tft_writecmd(ST7789_INVON)

    tft.clear(tft.NAVY)

    tft.image(0, 0, ICON_LOGO)
    width, height = tft.screensize()
    tft.rect(1, 1, width-1, height-1, tft.WHITE)

    for level in bl_range(0, 100, 10):        
      # print("level = ", level)
      pwm.duty(level)
      time.sleep_ms(100)

    # tft.font(tft.FONT_Tooney, rotate=0)
    # tft.text(tft.CENTER, tft.CENTER, "SoftRF", tft.WHITE, transparent=True)
    sleep(3)

if wifi['ssid'] != '':
    WiFi_active = True
    do_connect()
    network.ftp.start()

if RTC_present:
    rtc  = machine.RTC()
    # print("RTC = ", rtc.now())

uart = machine.UART(1, tx=GNSS_PIN_TX, rx=GNSS_PIN_RX, timeout=1000,  buffer_size=256, baudrate=9600)

if GNSS_present:
    gps  = machine.GPS(uart)
    gps.startservice()

if BME_present:
    i2c1=machine.I2C(id=1, scl=machine.Pin(26),sda=machine.Pin(25),speed=400000)
    bme=bme280.BME280(i2c=i2c1)

# Waiting for valid GNSS fix
if TFT_present:
    tft.image(0, 0, ICON_NOFIX)
    width, height = tft.screensize()
    tft.rect(1, 1, width-1, height-1, tft.WHITE)

while True:
    if GNSS_present:
      gnss_data = gps.getdata()

      gnss_year    = gnss_data[0][0]
      gnss_month   = gnss_data[0][1]
      gnss_mday    = gnss_data[0][2]
      gnss_hour    = gnss_data[0][3]
      gnss_minutes = gnss_data[0][4]
      gnss_seconds = gnss_data[0][5]

      gnss_lat     = gnss_data[1]
      gnss_lon     = gnss_data[2]
      gnss_alt     = gnss_data[3]
    
      gnss_nsats   = gnss_data[4]
      gnss_quality = gnss_data[5]

      gnss_speed   = gnss_data[6]
      gnss_course  = gnss_data[7]

      gnss_dop     = gnss_data[8]

      if gnss_quality > 0:
        if RTC_present:
          rtc.init((gnss_year, gnss_month, gnss_mday, gnss_hour, gnss_minutes, gnss_seconds))

        igc_writer.write_headers({
            'manufacturer_code': info['manufacturer'],
            'logger_id': info['id'],
            'date': datetime.date(gnss_year, gnss_month, gnss_mday),
            'fix_accuracy': 50,
            'pilot': info['pilot'],
            'copilot': info['copilot'],
            'glider_type': info['glider']['type'],
            'glider_id': info['glider']['id'],
            'firmware_version': info['version']['firmware'],
            'hardware_version': info['version']['hardware'],
            'logger_type': info['type'],
            'gps_receiver': info['gps'],
            'pressure_sensor': info['pressure'],
            'competition_id': info['competition']['id'],
            'competition_class': info['competition']['_class'],
        })
        break
    else:
      break
    #str = gps.read()
    #if str != '':
    #  print(str, end = '')
    print("Waiting for very first valid GNSS fix...")
    sleep(1)

# Make periodic position reports
if TFT_present:
    tft.image(0, 0, ICON_REC)
    width, height = tft.screensize()
    tft.rect(1, 1, width-1, height-1, tft.WHITE)

while True:
    if GNSS_present:
      gnss_data = gps.getdata()

      gnss_year    = gnss_data[0][0]
      gnss_month   = gnss_data[0][1]
      gnss_mday    = gnss_data[0][2]
      gnss_hour    = gnss_data[0][3]
      gnss_minutes = gnss_data[0][4]
      gnss_seconds = gnss_data[0][5]

      gnss_lat     = gnss_data[1]
      gnss_lon     = gnss_data[2]
      gnss_alt     = gnss_data[3]
    
      gnss_nsats   = gnss_data[4]
      gnss_quality = gnss_data[5]

      gnss_speed   = gnss_data[6]
      gnss_course  = gnss_data[7]

      gnss_dop     = gnss_data[8]

      if gnss_quality > 0:

        if BME_present:
          t, p, h = bme.read_compensated_data()
          baro_alt = pressure_altitude(p)
        else:
          baro_alt = 0

        igc_writer.write_fix(
            datetime.time(gnss_hour, gnss_minutes, gnss_seconds),
            latitude     = gnss_lat,
            longitude    = gnss_lon,
            valid        = True,
            pressure_alt = baro_alt,
            gps_alt      = gnss_alt,
        )

    else:
      break
    #if BME_present:
    #  print(bmevalues())

    sleep(INTERVAL)

if TFT_present:
    tft.image(0, 0, ICON_OFF)
    width, height = tft.screensize()
    tft.rect(1, 1, width-1, height-1, tft.WHITE)
    sleep(5)
    tft.clear()
    tft.deinit()

if SD_present:
    uos.umountsd()

if WiFi_active:
    sta_if = network.WLAN(network.STA_IF)
    if sta_if.isconnected():
      network.ftp.stop()
      sta_if.disconnect()
    sta_if.active(False)
