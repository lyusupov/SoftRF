#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Flight Recorder for SoftRF Badge Edition
#
#  File name: badge.py
#  Copyright (C) 2021 Linar Yusupov
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

INTERVAL     = 1.0
BL_OFF_TIME  = const(30000)

MF_CODE      = 'XSR'
FR_FW_ID     = '0.9'
RF_HW_ID     = '2.0'
FR_TYPE      = 'SOFTRF,LOGGER'
FR_GPS       = 'Quectel,L76K,32,18000,GPS,GLO'
FR_PRESSURE  = 'BOSCH,BME280,9163'

import supervisor
supervisor.disable_autoreload()
from microcontroller import cpu
if supervisor.runtime.serial_connected:
  print('CPU:')
  print(" frequency  =", cpu.frequency,   "Hz")
  print("temperature =", cpu.temperature, "Celsius")
  print("    voltage =", cpu.voltage,     "Volt")

#import gc

#print('free =', gc.mem_free())

from aerofiles.igc.writer import Writer

#print('free =', gc.mem_free())

import board

SOC_GPIO_PIN_IO_PWR   = board.P0_12
#SOC_GPIO_PIN_3V3_PWR  = board.P0_13
# Modded REV_1 3V3 power
SOC_GPIO_PIN_3V3_PWR  = board.P1_01

SOC_GPIO_LED_GREEN    = board.P0_15
SOC_GPIO_LED_RED      = board.P0_13
SOC_GPIO_LED_BLUE     = board.P0_14

SOC_GPIO_PIN_SS       = board.P0_24
SOC_GPIO_PIN_RST      = board.P0_25
SOC_GPIO_PIN_BUSY     = board.P0_17
SOC_GPIO_PIN_DIO1     = board.P0_20

SOC_GPIO_PIN_SWSER_RX = board.P1_09
SOC_GPIO_PIN_SWSER_TX = board.P1_08

SOC_GPIO_PIN_EPD_MISO = board.P1_07
SOC_GPIO_PIN_EPD_MOSI = board.P0_29
SOC_GPIO_PIN_EPD_SCK  = board.P0_31
SOC_GPIO_PIN_EPD_SS   = board.P0_30
SOC_GPIO_PIN_EPD_DC   = board.P0_28
SOC_GPIO_PIN_EPD_RST  = board.P0_02
SOC_GPIO_PIN_EPD_BUSY = board.P0_03
SOC_GPIO_PIN_EPD_BLGT = board.P1_11

SOC_GPIO_PIN_SDA      = board.P0_26
SOC_GPIO_PIN_SCL      = board.P0_27

SOC_GPIO_PIN_BUTTON   = board.P1_10

SOC_GPIO_PIN_SFL_MOSI = board.P1_12
SOC_GPIO_PIN_SFL_MISO = board.P1_13
SOC_GPIO_PIN_SFL_SCK  = board.P1_14
SOC_GPIO_PIN_SFL_SS   = board.P1_15
SOC_GPIO_PIN_SFL_HOLD = board.P0_05
SOC_GPIO_PIN_SFL_WP   = board.P0_07

from digitalio import DigitalInOut, Direction, Pull
from time   import sleep, monotonic

io_pwr = DigitalInOut(SOC_GPIO_PIN_IO_PWR)
io_pwr.direction = Direction.OUTPUT
io_pwr.value = True

sleep(0.2)

EPD_present    = True
GNSS_present   = True

from os import statvfs, stat, mkdir, sync
fstat = statvfs('/')
FLASH_present  = True if fstat[0] == 512 and fstat[2] > 2048 else False 

#from busio import I2C, SPI, UART
from busio import I2C, UART
from bitbangio import SPI
i2c = I2C(SOC_GPIO_PIN_SCL, SOC_GPIO_PIN_SDA)
i2c.try_lock()
i2c_devs = i2c.scan()
i2c.unlock()

from pcf8563 import PCF8563_SLAVE_ADDRESS
RTC_present    = True if PCF8563_SLAVE_ADDRESS in i2c_devs else False

BME280_ADDRESS = const(0x77)
BME_present    = True if BME280_ADDRESS in i2c_devs else False

button          = DigitalInOut(SOC_GPIO_PIN_BUTTON)
button.direction = Direction.INPUT
button.pull    = Pull.UP

BLE_active     = False
Power_Button   = not button.value
Battery_Low    = False
Backlight      = False

DebugMode      = False

FlashFolder    = ''
LogFolder      = '/Flights'
IGCSUFFIX      = '.IGC'

SETTINGS       = 'mysettings.py'

if EPD_present:
    from adafruit_epd.epd import Adafruit_EPD
    from adafruit_epd.ssd1681 import Adafruit_SSD1681

    epd_spi  = SPI(SOC_GPIO_PIN_EPD_SCK, MOSI=SOC_GPIO_PIN_EPD_MOSI,\
                                               MISO=SOC_GPIO_PIN_EPD_MISO)
    ecs      = DigitalInOut(SOC_GPIO_PIN_EPD_SS)
    dc       = DigitalInOut(SOC_GPIO_PIN_EPD_DC)
    rst      = DigitalInOut(SOC_GPIO_PIN_EPD_RST)
    busy     = DigitalInOut(SOC_GPIO_PIN_EPD_BUSY)
    srcs     = None

    display = Adafruit_SSD1681(200, 200, epd_spi, cs_pin=ecs, dc_pin=dc,\
      sramcs_pin=srcs, rst_pin=rst, busy_pin=busy)

    #while not epd_spi.try_lock():
    #  sleep(0.01)
    #epd_spi.configure(baudrate=250000, phase=0, polarity=0, bits=8)
    #epd_spi.unlock()

    display.rotation = 1
    display.fill(Adafruit_EPD.WHITE)
    x = 30
    y = 50
    display.text('SoftRF', x, y, Adafruit_EPD.BLACK, size=4)
    x = 45
    y = 125
    display.text('FLIGHT', x, y, Adafruit_EPD.BLACK, size=3)
    x = 30
    y = 150
    display.text('RECORDER', x, y, Adafruit_EPD.BLACK, size=3)
    display.display()
else:
    if supervisor.runtime.serial_connected:
      print("No EPD")

from sys    import path, stdout
import errno

BATTERY_CUTOFF_LIPO = 3.2

#import pcf8563
#if RTC_present:
#    rtc = pcf8563.PCF8563(i2c)
    # print("RTC = ", rtc.now())
#else:
#    if supervisor.runtime.serial_connected:
#      print("No RTC")

if GNSS_present:
    from adafruit_gps import GPS
    uart_gps = UART(SOC_GPIO_PIN_SWSER_TX, SOC_GPIO_PIN_SWSER_RX, baudrate=9600, timeout=30)
    gps      = GPS(uart_gps, debug=False)
    # RMC + GGA
    gps.send_command(b"PGKC242,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
    sleep(0.2)
    gps.send_command(b"PGKC115,1,1,0,0")   # GPS + GLONASS
    # gps.send_command(b"PGKC115,1,0,1,0")   # GPS + BEIDOU

import adafruit_bme280
if BME_present:
    bme = adafruit_bme280.Adafruit_BME280_I2C(i2c)
else:
    if supervisor.runtime.serial_connected:
      print("No BMx280 sensor")

DefaultSettings = True

if FLASH_present:
    DefaultSettings = False
    settings_file = FlashFolder + '/' + SETTINGS
    try:
      stat(settings_file)
    except OSError as e:
      if e.args[0] == errno.ENOENT:
        DefaultSettings = True
      pass

if DefaultSettings:
    if supervisor.runtime.serial_connected:
      print("No settings found on FLASH filesystem. Loading defaults...")

from mysettings import DebugMode

time_marker = monotonic()

# wait for some NMEA data to enter into GPS buffer
if GNSS_present:
    Power_Button = not button.value
    while monotonic() - time_marker < 2.0 and not Power_Button:
      gps.update()

if EPD_present and GNSS_present and not gps.has_fix and not Power_Button:
    display.fill(Adafruit_EPD.WHITE)
    x = 30
    y = 80
    display.text('NO FIX', x, y, Adafruit_EPD.BLACK, size=4)
    display.display()

if FLASH_present:
    dirname = FlashFolder + LogFolder
    try:
      stat(dirname)
    except OSError as e:
      if e.args[0] == errno.ENOENT:
        #if not supervisor.runtime.usb_connected:
        mkdir(dirname)
      pass

# Waiting for valid GNSS fix
if not Power_Button:
    if supervisor.runtime.serial_connected:
      print("")
      print("Waiting for very first valid GNSS fix...", end = '')

Serial_writer = Writer(stdout)

uid_raw = cpu.uid
uid = "%03X" % (((uid_raw[4] & 0xf) << 8) | uid_raw[5])

from mysettings import info
import adafruit_datetime as datetime

time_marker = monotonic()

while True:
    if Power_Button:
      break

    Power_Button = not button.value

    if cpu.voltage < BATTERY_CUTOFF_LIPO:
      Battery_Low = True
      break

    if GNSS_present:
      gps.update()

      if gps.has_fix:

        gnss_year    = gps.timestamp_utc.tm_year
        gnss_month   = gps.timestamp_utc.tm_mon
        gnss_mday    = gps.timestamp_utc.tm_mday
        gnss_hour    = gps.timestamp_utc.tm_hour
        gnss_minutes = gps.timestamp_utc.tm_min
        gnss_seconds = gps.timestamp_utc.tm_sec

        if gnss_year == 0 or gnss_month == 0 or gnss_mday == 0:
          continue

        if supervisor.runtime.serial_connected:
          print(" done")
          print("")

        #if RTC_present:
        #  rtc.write_all((gnss_year, gnss_month, gnss_mday, gnss_hour, gnss_minutes, gnss_seconds))

        if BME_present:
          p_sensor = FR_PRESSURE
          prev_p = 0
        else:
          p_sensor = 'NA'

        igc_header = {
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
          }

        if DebugMode:
          Serial_writer.write_headers(igc_header)

        pre_filename  = FlashFolder + LogFolder + '/'
        pre_filename += str(gnss_year) + '-' + str(gnss_month) + '-' + str(gnss_mday)
        pre_filename += '-' + MF_CODE + '-' + uid + '-'
        for n in list(range(1, 100, 1)):
          file = pre_filename + "%02d" % n + IGCSUFFIX
          try:
            stat(file)
          except OSError as e:
            if e.args[0] == errno.ENOENT:
              break
            pass
        # print("File = ", file)

        if not DebugMode:
          with open(file, 'w') as fp:
            Flash_writer = Writer(fp)
            Flash_writer.write_headers(igc_header)

        break

    else:
      break

    if monotonic() - time_marker > INTERVAL:
      if supervisor.runtime.serial_connected:
        print(".", end = '')
      time_marker = monotonic()

#print('free =', gc.mem_free())

if EPD_present and not Power_Button and not Battery_Low:
    display.fill(Adafruit_EPD.WHITE)
    x = 60
    y = 80
    display.text('REC', x, y, Adafruit_EPD.BLACK, size=5)
    display.display()

# Make periodic position reports
while True:

    if Power_Button or Battery_Low:
      break

    if cpu.voltage < BATTERY_CUTOFF_LIPO:
      Battery_Low = True
      break

    Power_Button = not button.value

    if GNSS_present:
      gps.update()

      if gps.has_fix and monotonic() - time_marker > INTERVAL:

        gnss_year    = gps.timestamp_utc.tm_year
        gnss_month   = gps.timestamp_utc.tm_mon
        gnss_mday    = gps.timestamp_utc.tm_mday
        gnss_hour    = gps.timestamp_utc.tm_hour
        gnss_minutes = gps.timestamp_utc.tm_min
        gnss_seconds = gps.timestamp_utc.tm_sec

        gnss_lat     = gps.latitude
        gnss_lon     = gps.longitude
        gnss_alt     = gps.altitude_m

        if BME_present:
          baro_alt = bme.altitude
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
            Flash_writer = Writer(fp)
            Flash_writer.write_fix(
                datetime.time(gnss_hour, gnss_minutes, gnss_seconds),
                latitude     = gnss_lat,
                longitude    = gnss_lon,
                valid        = True,
                pressure_alt = baro_alt,
                gps_alt      = gnss_alt,
            )

        time_marker = monotonic()
    else:
      break

while not button.value:
    sleep(0.1)

print("Shutdown")

if FLASH_present:
    sync()

if EPD_present:
    display.fill(Adafruit_EPD.WHITE)
    if Battery_Low:
      x = 60
      y = 40
      display.text('LOW', x, y, Adafruit_EPD.BLACK, size=5)
      x = 60
      y = 120
      display.text('BAT', x, y, Adafruit_EPD.BLACK, size=5)
    else:
      x = 60
      y = 80
      display.text('OFF', x, y, Adafruit_EPD.BLACK, size=5)

    display.display()

    display.power_down()

io_pwr.value = False

v33_pwr = DigitalInOut(SOC_GPIO_PIN_3V3_PWR)
v33_pwr.direction = Direction.OUTPUT
v33_pwr.value = False

from microcontroller import delay_us, reset

while button.value:
    #sleep(0.1)
    delay_us(100000)

v33_pwr.value = True
io_pwr.value  = True

while not button.value:
    sleep(0.1)

reset()
