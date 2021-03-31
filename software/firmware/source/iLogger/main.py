from sys import platform

if platform == 'esp32':
  import watch

if platform == 'nRF52840':
  import badge
