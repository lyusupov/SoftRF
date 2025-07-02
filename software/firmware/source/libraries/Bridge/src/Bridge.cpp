/*
  Copyright (c) 2013 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include "Bridge.h"

//BridgeClass::BridgeClass(Stream &_stream){}
BridgeClass::BridgeClass(SerialSimulator &_serial){}

void BridgeClass::begin() {}

void BridgeClass::put(const char *key, const char *value) {}

unsigned int BridgeClass::get(const char *key, uint8_t *value, unsigned int maxlen) {
  return 0;
}

void checkForRemoteSketchUpdate(uint8_t pin) {}

SerialSimulator TestSerial;
SerialBridgeClass Bridge(TestSerial);

