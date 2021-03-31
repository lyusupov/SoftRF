# This file is executed on every boot (including wake-boot from deepsleep)
import sys

if sys.platform == 'nRF52840':
  from storage import remount
  #remount('/', readonly=False, disable_concurrent_write_protection=True)
else:
  sys.path[1] = '/flash/lib'
