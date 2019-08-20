#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  settings.py


wifi = dict(
  ssid = '',
  psk  = '',
)

info = dict(
  manufacturer = 'SRF',
  id = 'LRY',
  pilot = 'John Doe',
  copilot = 'Jessica Alba',
  glider = dict(
    type = 'Duo Discus',
    id = 'RF-012345',
  ),
  version = dict (
    firmware = '0.9',
    hardware = '0.9',
  ),
  type = 'SOFTRF,LOGGER',
  gps = 'uBLOX NEO-8',
  pressure = 'BMP280',
  competition = dict(
    id = 'LY',
    _class = 'Doubleseater',
  ),
)
