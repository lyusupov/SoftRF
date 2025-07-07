/*
  I2C Master library for Raspberry Pi Arduino.
  Copyright (c) 2015 by Hristo Gochkov

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
}

#if defined(ARDUINO)
#include "Arduino.h"
#include "Wire.h"
#endif /* ARDUINO */

#if defined(RASPBERRY_PI) || defined(LUCKFOX_LYRA)
#include <raspi/raspi.h>
#include <raspi/Wire.h>
#endif /* RASPBERRY_PI */

#if defined(USE_LGPIO)

uint8_t TwoWire::rxBuffer[BUFFER_LENGTH];
uint8_t TwoWire::rxBufferIndex = 0;
uint8_t TwoWire::rxBufferLength = 0;

uint8_t TwoWire::txAddress = 0;
uint8_t TwoWire::txBuffer[BUFFER_LENGTH];
uint8_t TwoWire::txBufferIndex = 0;
uint8_t TwoWire::txBufferLength = 0;

TwoWire::TwoWire(uint8_t i2cDevice) : _i2cDevice(i2cDevice) { }

void TwoWire::begin(void) {
  rxBufferIndex = 0;
  rxBufferLength = 0;

  txBufferIndex = 0;
  txBufferLength = 0;
}

void TwoWire::end(void){ }

void TwoWire::setClock(uint32_t frequency){ }

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
  if (quantity > BUFFER_LENGTH) quantity = BUFFER_LENGTH;

#if 0
  uint8_t read = twi_readFrom(address, rxBuffer, quantity, sendStop);
#else
  uint8_t read = 1;

  int i2cHandle = -1;
  if ((i2cHandle = lgI2cOpen(_i2cDevice, address, 0)) < 0) {
    fprintf(stderr, "Could not open I2C handle on 0: %s\n", lguErrorText(i2cHandle));
  } else {
    if (lgI2cReadDevice(i2cHandle, (char *) rxBuffer, quantity) > 0) {
      read = 0;
    }
    lgI2cClose(i2cHandle);
  }
#endif

  rxBufferIndex = 0;
  rxBufferLength = (read == 0) ? quantity:0;

  return rxBufferLength;
}

void TwoWire::beginTransmission(uint8_t address) {
  txAddress = address;
  txBufferIndex = 0;
  txBufferLength = 0;
}

uint8_t TwoWire::endTransmission(uint8_t sendStop) {
#if 0
  uint8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 0, sendStop);
#else
  uint8_t ret = 0; // success

  int i2cHandle = -1;
  if ((i2cHandle = lgI2cOpen(_i2cDevice, txAddress, 0)) < 0) {
    fprintf(stderr, "Could not open I2C handle on 0: %s\n", lguErrorText(i2cHandle));
  } else {
    if (lgI2cWriteDevice(i2cHandle, (char *) txBuffer, txBufferLength) < 0) {
      ret = 4; // other error
    }
    lgI2cClose(i2cHandle);
  }
#endif

  txBufferIndex = 0;
  txBufferLength = 0;

  return ret;
}

uint8_t TwoWire::endTransmission(void) {
  return endTransmission(true);
}

size_t TwoWire::write(uint8_t data) {
  if (txBufferLength >= BUFFER_LENGTH) return 0;

  txBuffer[txBufferIndex] = data;
  ++txBufferIndex;
  txBufferLength = txBufferIndex;

  return 1;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity) {
  for (size_t i = 0; i < quantity; ++i) {
    write(data[i]);
  }

  return quantity;
}

int TwoWire::available(void) {
  return rxBufferLength - rxBufferIndex;
}

int TwoWire::read(void) {
  int value = -1;

  if (rxBufferIndex < rxBufferLength) {
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}

int TwoWire::peek(void) {
  int value = -1;

  if (rxBufferIndex < rxBufferLength) {
    value = rxBuffer[rxBufferIndex];
  }

  return value;
}

void TwoWire::begin(uint8_t address) {
  begin();
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity) {
  return requestFrom(address, quantity, (uint8_t)true);
}

TwoWire Wire = TwoWire();

#endif /* USE_LGPIO */
#endif // RASPBERRY_PI
