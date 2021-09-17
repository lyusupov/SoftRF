/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach and Dean Miller for Adafruit Industries LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Adafruit_SPIFlash.h"

#ifdef USE_TINYUSB
#include "Adafruit_TinyUSB.h"
#endif

#if SPIFLASH_DEBUG
#define SPIFLASH_LOG(_block, _count)                                           \
  do {                                                                         \
    Serial.print(__FUNCTION__);                                                \
    Serial.print(": lba = ");                                                  \
    Serial.print(_block);                                                      \
    if (_count) {                                                              \
      Serial.print(" count = ");                                               \
      Serial.print(_count);                                                    \
    }                                                                          \
    Serial.println();                                                          \
  } while (0)
#else
#define SPIFLASH_LOG(_sector, _count)
#endif

Adafruit_SPIFlash::Adafruit_SPIFlash() : Adafruit_SPIFlashBase() {
  _cache = NULL;
}

Adafruit_SPIFlash::Adafruit_SPIFlash(Adafruit_FlashTransport *transport)
    : Adafruit_SPIFlashBase(transport) {
  _cache = NULL;
}

bool Adafruit_SPIFlash::begin(SPIFlash_Device_t const *flash_devs,
                              size_t count) {
  bool ret = Adafruit_SPIFlashBase::begin(flash_devs, count);

  // Use cache if not FRAM
  if (_flash_dev && !_flash_dev->is_fram) {
    // new cache object if not already
    if (!_cache) {
      _cache = new Adafruit_FlashCache();
    }
  }

  return ret;
}

//--------------------------------------------------------------------+
// SdFat BaseBlockDRiver API
// A block is 512 bytes
//--------------------------------------------------------------------+
bool Adafruit_SPIFlash::readBlock(uint32_t block, uint8_t *dst) {
  SPIFLASH_LOG(block, 1);

  if (_flash_dev->is_fram) {
    // FRAM does not need caching
    return this->readBuffer(block * 512, dst, 512) > 0;
  } else {
    return _cache->read(this, block * 512, dst, 512);
  }
}

bool Adafruit_SPIFlash::syncBlocks() {
  SPIFLASH_LOG(0, 0);

  if (_flash_dev->is_fram) {
    return true;
  } else {
    return _cache->sync(this);
  }
}

bool Adafruit_SPIFlash::writeBlock(uint32_t block, const uint8_t *src) {
  SPIFLASH_LOG(block, 1);

  if (_flash_dev->is_fram) {
    return this->writeBuffer(block * 512, src, 512) > 0;
  } else {
    return _cache->write(this, block * 512, src, 512);
  }
}

bool Adafruit_SPIFlash::readBlocks(uint32_t block, uint8_t *dst, size_t nb) {
  SPIFLASH_LOG(block, nb);

  if (_flash_dev->is_fram) {
    return this->readBuffer(block * 512, dst, 512 * nb) > 0;
  } else {
    return _cache->read(this, block * 512, dst, 512 * nb);
  }
}

bool Adafruit_SPIFlash::writeBlocks(uint32_t block, const uint8_t *src,
                                    size_t nb) {
  SPIFLASH_LOG(block, nb);
  if (_flash_dev->is_fram) {
    return this->writeBuffer(block * 512, src, 512 * nb) > 0;
  } else {
    return _cache->write(this, block * 512, src, 512 * nb);
  }
}
