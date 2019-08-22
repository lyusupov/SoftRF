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

INTERVAL     = const(1)
BL_OFF_TIME  = const(30000)

I2C0_PIN_SCL = const(22)
I2C0_PIN_SDA = const(21)
I2C1_PIN_SCL = const(26)
I2C1_PIN_SDA = const(25)

PMU_PIN_IRQ  = const(35)
BUTTON_PIN   = const(36)

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
Power_Button   = False
Battery_Low    = False
Backlight      = False

ICON_LOGO      = 'icons/logo.bmp'
ICON_NOSD      = 'icons/nosd.jpg'
ICON_NOFIX     = 'icons/nofix.jpg'
ICON_REC1      = 'icons/rec1.bmp'
ICON_REC2      = 'icons/rec2.bmp'
ICON_OFF       = 'icons/off.jpg'
ICON_LOWBAT    = 'icons/lowbat.jpg'

from machine import I2C, Pin, PWM, Timer
from display import TFT
from time    import sleep, sleep_ms

from AXP202.axp202    import PMU
from AXP202.constants import AXP202_SLAVE_ADDRESS, AXP202_PEK_SHORTPRESS_IRQ
from AXP202.constants import AXP20X_LED_OFF, AXP202_LDO2
from AXP202.constants import AXP202_LDO3, AXP202_ADC1, AXP202_BATT_VOL_ADC1

i2c0=I2C(id=0, scl=Pin(I2C0_PIN_SCL),sda=Pin(I2C0_PIN_SDA),speed=400000)
if i2c0.is_ready(AXP202_SLAVE_ADDRESS):
    AXP202_present = True
    TFT_present    = True
    RTC_present    = True

i2c0.deinit()

def backlight(bl):
    if bl:
      bl_range = list(range(10, 101, 10))
    else:
      bl_range = list(range(90, -1, -10))
    for level in bl_range:
      # print("level = ", level)
      pwm.duty(level)
      sleep_ms(100)

def power_button_handler(pin):
    global Power_Button, a, bl_timer, Backlight
    if pin.irqvalue() == 0:
      if Backlight:
        Power_Button = True
      else:
        a.clearIRQ()
        bl_timer.reshoot()
        Backlight = True
        backlight(Backlight)

#def user_button_handler(pin):
#    global Backlight, bl_timer
#    if pin.irqvalue() == 0 and TFT_present:
#      if not Backlight:
#        bl_timer.reshoot()
#      Backlight = not Backlight
#      backlight(Backlight)

def bl_timer_cb(timer):
    global Backlight
    # print(timer)
    if Backlight:
      Backlight = False
      backlight(Backlight)

if AXP202_present:
    a = PMU()
    a.disableIRQ(AXP202_PEK_SHORTPRESS_IRQ)
    a.clearIRQ()
    a.setChgLEDMode(AXP20X_LED_OFF)
    # power-up TFT
    a.enablePower(AXP202_LDO2)
    a.setLDO2Voltage(3300)
    # power-up GNSS
    a.setLDO3Mode(1)
    a.enablePower(AXP202_LDO3)
    a.setLDO3Voltage(3300)
    pmu_pin = Pin(
      PMU_PIN_IRQ, Pin.IN, trigger=Pin.IRQ_FALLING, handler=power_button_handler
    )
    #user_button = Pin(
    #  BUTTON_PIN, Pin.IN, trigger=Pin.IRQ_FALLING, handler=user_button_handler
    #)
    a.enableADC(AXP202_ADC1, AXP202_BATT_VOL_ADC1)
    a.enableIRQ(AXP202_PEK_SHORTPRESS_IRQ)

if TFT_present:
    pwm = PWM(TFT_PIN_BL, freq=12000, duty=0)

    tft = TFT()
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

    Backlight = True
    backlight(Backlight)

    bl_timer = Timer(2)
    bl_timer.init(period=BL_OFF_TIME, mode=bl_timer.ONE_SHOT, callback=bl_timer_cb)

    # tft.font(tft.FONT_Tooney, rotate=0)
    # tft.text(tft.CENTER, tft.CENTER, "SoftRF", tft.WHITE, transparent=True)
    # sleep(3)

from sys import stdout
import network
import datetime

import bme280
import uos
import errno

from settings import info
from settings import wifi

from machine  import RTC, UART, GPS, deepsleep
from bme280   import BME280_I2CADDR
from aerofiles.igc.writer import Writer
from AXP202.constants     import AXP202_SLAVE_ADDRESS

BATTERY_CUTOFF_LIPO = 3.2

def do_connect():
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print('connecting to network...')
        sta_if.active(True)
        sta_if.connect(wifi['ssid'], wifi['psk'])
        while not sta_if.isconnected():
            pass
    print('network config:', sta_if.ifconfig())


def pressure_altitude(pressure, qnh=1013.25):
    altitude = 44330.0 * (1.0 - pow(pressure / qnh, (1.0 / 5.255)))
    return altitude

i2c1=I2C(id=1, scl=Pin(I2C1_PIN_SCL),sda=Pin(I2C1_PIN_SDA),speed=400000)
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

igc_writer = Writer(stdout)

if wifi['ssid'] != '':
    WiFi_active = True
    do_connect()
    network.ftp.start()

if RTC_present:
    rtc  = RTC()
    # print("RTC = ", rtc.now())

uart = UART(1, tx=GNSS_PIN_TX, rx=GNSS_PIN_RX, timeout=1000,  buffer_size=256, baudrate=9600)

if GNSS_present:
    gps  = GPS(uart)
    gps.startservice()

if BME_present:
    i2c1=I2C(id=1, scl=Pin(I2C1_PIN_SCL),sda=Pin(I2C1_PIN_SDA),speed=400000)
    bme=bme280.BME280(i2c=i2c1)

if TFT_present and GNSS_present:
    # wait for some NMEA data to enter into GPS buffer
    sleep_ms(1500)

    gnss_data = gps.getdata()
    gnss_quality = gnss_data[5]
    if gnss_quality == 0:
      tft.image(0, 0, ICON_NOFIX)
      width, height = tft.screensize()
      tft.rect(1, 1, width-1, height-1, tft.WHITE)

# Waiting for valid GNSS fix
print("")
print("Waiting for very first valid GNSS fix...", end = '')

while True:
    if Power_Button:
      break

    if AXP202_present:
      voltage = a.getBattVoltage() / 1000
      if voltage < BATTERY_CUTOFF_LIPO:
        Battery_Low = True
        break

    if GNSS_present:
      gnss_data = gps.getdata()

      gnss_quality = gnss_data[5]

      if gnss_quality > 0:

        print(" done")
        print("")

        gnss_year    = gnss_data[0][0]
        gnss_month   = gnss_data[0][1]
        gnss_mday    = gnss_data[0][2]
        gnss_hour    = gnss_data[0][3]
        gnss_minutes = gnss_data[0][4]
        gnss_seconds = gnss_data[0][5]

        if RTC_present:
          rtc.init((gnss_year, gnss_month, gnss_mday, gnss_hour, gnss_minutes, gnss_seconds))

        if BME_present:
          p_sensor = info['pressure']
        else:
          p_sensor = 'NA'

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
            'pressure_sensor': p_sensor,
            'competition_id': info['competition']['id'],
            'competition_class': info['competition']['_class'],
        })
        break
    else:
      break
    #str = gps.read()
    #if str != '':
    #  print(str, end = '')
    print(".", end = '')
    sleep(1)

# Make periodic position reports
flip = True
while True:

    if Power_Button or Battery_Low:
      break

    if AXP202_present:
      voltage = a.getBattVoltage() / 1000
      # print(voltage)
      if voltage < BATTERY_CUTOFF_LIPO:
        Battery_Low = True
        break

    if GNSS_present:
      gnss_data = gps.getdata()

      gnss_quality = gnss_data[5]

      if gnss_quality > 0:

        gnss_year    = gnss_data[0][0]
        gnss_month   = gnss_data[0][1]
        gnss_mday    = gnss_data[0][2]
        gnss_hour    = gnss_data[0][3]
        gnss_minutes = gnss_data[0][4]
        gnss_seconds = gnss_data[0][5]

        gnss_lat     = gnss_data[1]
        gnss_lon     = gnss_data[2]
        gnss_alt     = gnss_data[3]

        gnss_speed   = gnss_data[6]
        gnss_course  = gnss_data[7]

        gnss_nsats   = gnss_data[4]
        gnss_dop     = gnss_data[8]

        if BME_present:
          t, p, h = bme.read_compensated_data()
          baro_alt = pressure_altitude(float(p) / 25600)
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

        if TFT_present:
            if flip:
              recfile = ICON_REC1
            else:
              recfile = ICON_REC2
            flip = not flip
            tft.image(0, 0, recfile)
    else:
      break

    sleep(INTERVAL)

if SD_present:
    uos.umountsd()

if TFT_present:
    bl_timer.deinit()
    if Battery_Low:
      offname = ICON_LOWBAT
    else:
      offname = ICON_OFF
    tft.image(0, 0, offname)
    width, height = tft.screensize()
    tft.rect(1, 1, width-1, height-1, tft.WHITE)
    if not Backlight:
      backlight(True)
    sleep(2)
    backlight(False)
    tft.clear()
    tft.deinit()

if WiFi_active:
    sta_if = network.WLAN(network.STA_IF)
    if sta_if.isconnected():
      network.ftp.stop()
      sta_if.disconnect()
    sta_if.active(False)

if AXP202_present:
    a.clearIRQ()
    # power-down TFT
    a.disablePower(AXP202_LDO2)
    # power-down GNSS
    a.disablePower(AXP202_LDO3)
    sleep_ms(20);

if RTC_present:
    rtc.wake_on_ext0(PMU_PIN_IRQ, 0)

deepsleep()
