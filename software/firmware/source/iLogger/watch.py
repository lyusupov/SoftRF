#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Flight Recorder for TTGO T-Watch 2019
#
#  File name: watch.py
#  Copyright (C) 2019-2021 Linar Yusupov
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

MF_CODE      = 'XSR'
FR_FW_ID     = '0.9'
RF_HW_ID     = '1.0'
FR_TYPE      = 'SOFTRF,LOGGER'
FR_GPS       = 'uBLOX,NEO-M8N,72,50000,GPS,GLO'
FR_PRESSURE  = 'BOSCH,BMP280,9163'

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

CPU_FREQ_240 = const(240000000)
CPU_FREQ_80  = const(80000000)

AXP202_present = False
TFT_present    = False
BME_present    = False
SD_present     = False
GNSS_present   = True
RTC_present    = True

WiFi_active    = False
Power_Button   = False
Battery_Low    = False
Backlight      = False

DebugMode      = False

SDFolder       = '/sd'
LogFolder      = '/Flights'
IGCSUFFIX      = '.IGC'

SETTINGS       = 'settings.py'

ICON_LOGO      = '/flash/icons/logo.bmp'
ICON_NOSD      = '/flash/icons/nosd.jpg'
ICON_NOFIX     = '/flash/icons/nofix.jpg'
ICON_REC1      = '/flash/icons/rec1.bmp'
ICON_REC2      = '/flash/icons/rec2.bmp'
ICON_OFF       = '/flash/icons/off.jpg'
ICON_LOWBAT    = '/flash/icons/lowbat.jpg'

from machine import I2C, Pin, PWM, Timer, freq
from display import TFT
from time    import sleep, sleep_ms

from AXP202.axp202    import PMU
from AXP202.constants import AXP202_SLAVE_ADDRESS, AXP202_PEK_SHORTPRESS_IRQ
from AXP202.constants import AXP20X_LED_OFF, AXP202_LDO2
from AXP202.constants import AXP202_LDO3, AXP202_ADC1, AXP202_BATT_VOL_ADC1

print("CPU frequency =", freq(), "Hz")
if freq() != CPU_FREQ_240:
    freq(CPU_FREQ_240)

i2c0=I2C(id=0, scl=Pin(I2C0_PIN_SCL),sda=Pin(I2C0_PIN_SDA),speed=400000)
if i2c0.is_ready(AXP202_SLAVE_ADDRESS):
    AXP202_present = True
    TFT_present    = True
else:
    print("No AXP202 PMU detected")

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
    a.setLDO3Voltage(3000)
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
else:
    print("No TFT")

from sys import path, stdout
import network
import datetime

import bme280
import uos
import errno

from machine  import RTC, UART, GPS, deepsleep, unique_id
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

if RTC_present:
    rtc  = RTC()
    # print("RTC = ", rtc.now())
else:
    print("No RTC")

uart = UART(1, tx=GNSS_PIN_TX, rx=GNSS_PIN_RX, timeout=1000,  buffer_size=256, baudrate=9600)

# Max Performance Mode (default)
ubx_cont = bytearray([0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91])

# Power Save Mode
ubx_psm  = bytearray([0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92])

# NEO-6 'Eco' Mode
ubx_eco  = bytearray([0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x04, 0x25, 0x95])

# NMEA filters
ubx_gsv_off = bytearray([0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39])
ubx_vtg_off = bytearray([0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46])
ubx_gll_off = bytearray([0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B])

nmea_gll_off = "$PUBX,40,GLL,0,0,0,0*5C\r\n"
nmea_gsv_off = "$PUBX,40,GSV,0,0,0,0*59\r\n"
nmea_vtg_off = "$PUBX,40,VTG,0,0,0,0*5E\r\n"
nmea_gsa_off = "$PUBX,40,GSA,0,0,0,0*4E\r\n"

if GNSS_present:
    sleep_ms(100)
    uart.write(nmea_gsv_off)
    sleep_ms(200)
    uart.write(nmea_gll_off)
    sleep_ms(200)
    uart.write(nmea_vtg_off)
    sleep_ms(200)
    uart.write(nmea_gsa_off)
    sleep_ms(200)
    uart.write(ubx_psm)
    sleep_ms(200)

    gps  = GPS(uart)
    gps.startservice()

if BME_present:
    i2c1=I2C(id=1, scl=Pin(I2C1_PIN_SCL),sda=Pin(I2C1_PIN_SDA),speed=400000)
    bme=bme280.BME280(i2c=i2c1)
else:
    print("No BMx280 sensor")

uos.sdconfig(uos.SDMODE_1LINE)
SD_present = True
try:
  uos.mountsd()
except OSError as e:
  if e.args[0] == errno.EIO:
    SD_present = False
  pass

DefaultSettings = True
if SD_present:
    DefaultSettings = False
    settings_file = SDFolder + '/' + SETTINGS
    try:
      uos.stat(settings_file)
    except OSError as e:
      if e.args[0] == errno.ENOENT:
        DefaultSettings = True
      pass

if not DefaultSettings:
    path[0] = SDFolder
    uos.chdir(SDFolder)
else:
    print("No settings found on SD card. Loading defaults...")

from settings import wifi

if wifi['ssid'] != '':
    WiFi_active = True
    do_connect()
    network.ftp.start()
else:
    print("No WiFi")

from settings import DebugMode

if TFT_present and GNSS_present:
    # wait for some NMEA data to enter into GPS buffer
    sleep_ms(1500)

    gnss_data = gps.getdata()
    gnss_quality = gnss_data[5]
    if gnss_quality == 0 and SD_present:
      bl_timer.reshoot()
      tft.image(0, 0, ICON_NOFIX)
      width, height = tft.screensize()
      tft.rect(1, 1, width-1, height-1, tft.WHITE)

if not SD_present:
    print("No SD card")
    if TFT_present:
        bl_timer.reshoot()
        tft.image(0, 0, ICON_NOSD)
        width, height = tft.screensize()
        tft.rect(1, 1, width-1, height-1, tft.WHITE)
        if not Backlight:
          Backlight = True
          backlight(Backlight)
        sleep(5)
    if not DebugMode:
        Power_Button = True
else:
    dirname = SDFolder + LogFolder
    try:
      uos.stat(dirname)
    except OSError as e:
      if e.args[0] == errno.ENOENT:
        uos.mkdir(dirname)
      pass

# Waiting for valid GNSS fix
if not Power_Button:
    print("")
    print("Waiting for very first valid GNSS fix...", end = '')

Serial_writer = Writer(stdout)

uid_raw = unique_id()
uid = "%03X" % (((uid_raw[4] & 0xf) << 8) | uid_raw[5])

# Slow down to 80 MHz
freq(CPU_FREQ_80)

from settings import info

while True:
    if Power_Button:
      break

    if AXP202_present:
      voltage = a.getBattVoltage() / 1000
      if voltage < BATTERY_CUTOFF_LIPO:
        Battery_Low = True
        break
      #if DebugMode:
      #  print("ACInC =", a.getAcinCurrent(),
      #    "VbusC =", a.getVbusCurrent(),
      #    "BchgC =", a.getBattChargeCurrent())

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
          p_sensor = FR_PRESSURE
          prev_p = 0
        else:
          p_sensor = 'NA'

        if DebugMode:
          Serial_writer.write_headers({
              'manufacturer_code': MF_CODE,
              'logger_id': uid,
              'date': datetime.date(gnss_year, gnss_month, gnss_mday),
              'fix_accuracy': 50,
              'pilot': info['pilot'],
              'copilot': info['copilot'],
              'glider_type': info['glider']['type'],
              'glider_id': info['glider']['id'],
              'firmware_version': FR_FW_ID,
              'hardware_version': RF_HW_ID,
              'logger_type': FR_TYPE,
              'gps_receiver': FR_GPS,
              'pressure_sensor': p_sensor,
              'competition_id': info['competition']['id'],
              'competition_class': info['competition']['_class'],
          })

        pre_filename  = SDFolder + LogFolder + '/'
        pre_filename += str(gnss_year) + '-' + str(gnss_month) + '-' + str(gnss_mday)
        pre_filename += '-' + MF_CODE + '-' + uid + '-'
        for n in list(range(1, 100, 1)):
          file = pre_filename + "%02d" % n + IGCSUFFIX
          try:
            uos.stat(file)
          except OSError as e:
            if e.args[0] == errno.ENOENT:
              break
            pass
        # print("File = ", file)

        if not DebugMode:
          with open(file, 'w') as fp:
            SD_writer = Writer(fp)
            SD_writer.write_headers({
                'manufacturer_code': MF_CODE,
                'logger_id': uid,
                'date': datetime.date(gnss_year, gnss_month, gnss_mday),
                'fix_accuracy': 50,
                'pilot': info['pilot'],
                'copilot': info['copilot'],
                'glider_type': info['glider']['type'],
                'glider_id': info['glider']['id'],
                'firmware_version': FR_FW_ID,
                'hardware_version': RF_HW_ID,
                'logger_type': FR_TYPE,
                'gps_receiver': FR_GPS,
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
      #if DebugMode:
      #  print("ACInC =", a.getAcinCurrent(),
      #    "VbusC =", a.getVbusCurrent(),
      #    "Bvolt =", a.getBattVoltage(),
      #    "BdisC =", a.getBattDischargeCurrent(),
      #    "BchgC =", a.getBattChargeCurrent())

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

          try:
            t, p, h = bme.read_compensated_data()
          except OSError as e:
            if e.args[0] == 263 or e.args[0] == 6: # I2C bus error
              if DebugMode:
                print("I2C bus error ", e.args[0])
              t, p, h = 0, prev_p, 0
            pass

          # sanity check
          if p > 0:
            baro_alt = pressure_altitude(float(p) / 25600)
            prev_p = p
          else:
            baro_alt = 0

        else:
          baro_alt = 0

        if DebugMode:
          Serial_writer.write_fix(
              datetime.time(gnss_hour, gnss_minutes, gnss_seconds),
              latitude     = gnss_lat,
              longitude    = gnss_lon,
              valid        = True,
              pressure_alt = baro_alt,
              gps_alt      = gnss_alt,
          )

        if not DebugMode:
          with open(file, 'a') as fp:
            SD_writer = Writer(fp)
            SD_writer.write_fix(
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

print("Shutdown")

if SD_present:
    # uos.sync()
    uos.umountsd()

if GNSS_present:
    gps.stopservice()

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
    sleep_ms(20)
    if RTC_present:
      rtc.wake_on_ext0(PMU_PIN_IRQ, 0)

deepsleep()
