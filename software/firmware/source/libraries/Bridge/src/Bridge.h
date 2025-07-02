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

#ifndef BRIDGE_H_
#define BRIDGE_H_

#if defined(ARDUINO)
#include <Arduino.h>
#include <Stream.h>
#endif /* ARDUINO */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)
#include <raspi/raspi.h>
#include <raspi/Stream.h>
#endif /* RASPBERRY_PI */

class BridgeClass {
  public:
//    BridgeClass(Stream &_stream);
    BridgeClass(SerialSimulator &_serial);
    void begin();

    // Methods to handle key/value datastore
    void put(const char *key, const char *value);
    void put(const String &key, const String &value){
      put(key.c_str(), value.c_str());
    }
    unsigned int get(const char *key, uint8_t *buff, unsigned int size);
    unsigned int get(const char *key, char *value, unsigned int maxlen){
      return get(key, reinterpret_cast<uint8_t *>(value), maxlen);
    }
    uint16_t getBridgeVersion(){
      return 0;
    }
};

// This subclass uses a serial port Stream
class SerialBridgeClass : public BridgeClass {
  public:
//    SerialBridgeClass(Stream &_serial) : BridgeClass(_serial) {
//      // Empty
//    }
    SerialBridgeClass(SerialSimulator &_serial) : BridgeClass(_serial) {
      // Empty
    }
    void begin(unsigned long baudrate=0) {
      BridgeClass::begin();
    }
};

extern SerialBridgeClass Bridge;
extern void checkForRemoteSketchUpdate(uint8_t pin=0);

#endif /* BRIDGE_H_ */

#include <Process.h>
#include <BridgeServer.h>
