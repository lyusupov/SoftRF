/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 hathach for Adafruit Industries
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

#include "Adafruit_FlashTransport.h"

#ifdef ARDUINO_ARCH_RP2040

#include <hardware/flash.h>

// symbol from linker that matches 'Tools->Flash Size' menu selection for
// filesystem
extern uint8_t _FS_start;
extern uint8_t _FS_end;

// FS Size determined by menu selection
#define MENU_FS_SIZE ((uint32_t)(&_FS_end - &_FS_start))

// CircuitPython partition scheme with start adress = 1 MB, the rest is for
// FileSystem
// + 4KB since CPY does not reserve EEPROM from arduino core
#define CPY_START_ADDR (1 * 1024 * 1024)
#define CPY_SIZE (((uint32_t)&_FS_end) - (XIP_BASE + CPY_START_ADDR) + 4096)

static inline void fl_lock(void) {
  noInterrupts();
#if !defined(ARDUINO_ARCH_MBED)
  rp2040.idleOtherCore();
#endif /* ARDUINO_ARCH_MBED */
}

static inline void fl_unlock(void) {
#if !defined(ARDUINO_ARCH_MBED)
  rp2040.resumeOtherCore();
#endif /* ARDUINO_ARCH_MBED */
  interrupts();
}

Adafruit_FlashTransport_RP2040::Adafruit_FlashTransport_RP2040(void)
    : Adafruit_FlashTransport_RP2040(CPY_START_ADDR, CPY_SIZE) {}

Adafruit_FlashTransport_RP2040::Adafruit_FlashTransport_RP2040(
    uint32_t start_addr, uint32_t size) {
  _cmd_read = SFLASH_CMD_READ;
  _addr_len = 3; // work with most device if not set

  _start_addr = start_addr;
  _size = size;

  memset(&_flash_dev, 0, sizeof(_flash_dev));
}

void Adafruit_FlashTransport_RP2040::begin(void) {
  // auto detect start address
  if (!_start_addr) {
    _start_addr = (uint32_t)&_FS_start - XIP_BASE;
  }

  // auto detect size
  if (!_size) {
    _size = MENU_FS_SIZE;
  }
  _flash_dev.total_size = _size;

#if SPIFLASH_DEBUG
  if (Serial) {
    Serial.print("Start = 0x");
    Serial.println(_start_addr, HEX);

    Serial.print("Size = ");
    Serial.println(_size);
  }
#endif

  // Read the RDID register to get the flash capacity.
  uint8_t const cmd[] = {
      0x9f,
      0,
      0,
      0,
  };
  uint8_t data[4];
  fl_lock();
#if !defined(ARDUINO_ARCH_MBED)
  flash_do_cmd(cmd, data, 5);
#endif /* ARDUINO_ARCH_MBED */
  fl_unlock();

  uint8_t *jedec_ids = data + 1;

  _flash_dev.manufacturer_id = jedec_ids[0];
  _flash_dev.memory_type = jedec_ids[1];
  _flash_dev.capacity = jedec_ids[2];
}

void Adafruit_FlashTransport_RP2040::end(void) {
  _cmd_read = SFLASH_CMD_READ;
  _addr_len = 3; // work with most device if not set
}

SPIFlash_Device_t *Adafruit_FlashTransport_RP2040::getFlashDevice(void) {
  return &_flash_dev;
}

void Adafruit_FlashTransport_RP2040::setClockSpeed(uint32_t write_hz,
                                                   uint32_t read_hz) {
  // do nothing, just use current configured clock
  (void)write_hz;
  (void)read_hz;
}

bool Adafruit_FlashTransport_RP2040::runCommand(uint8_t command) {
  switch (command) {
  case SFLASH_CMD_ERASE_CHIP:
    fl_lock();
    flash_range_erase(_start_addr, _size);
    fl_unlock();
    break;

  // do nothing, mostly write enable
  default:
    break;
  }

  return true;
}

bool Adafruit_FlashTransport_RP2040::readCommand(uint8_t command,
                                                 uint8_t *response,
                                                 uint32_t len) {
  // mostly is Read STATUS, just fill with 0x0
  (void)command;
  memset(response, 0, len);

  return true;
}

bool Adafruit_FlashTransport_RP2040::writeCommand(uint8_t command,
                                                  uint8_t const *data,
                                                  uint32_t len) {
  // mostly is Write Status, do nothing
  (void)command;
  (void)data;
  (void)len;

  return true;
}

bool Adafruit_FlashTransport_RP2040::eraseCommand(uint8_t command,
                                                  uint32_t addr) {
  uint32_t erase_sz;

  if (command == SFLASH_CMD_ERASE_SECTOR) {
    erase_sz = SFLASH_SECTOR_SIZE;
  } else if (command == SFLASH_CMD_ERASE_BLOCK) {
    erase_sz = SFLASH_BLOCK_SIZE;
  } else {
    return false;
  }

  fl_lock();
  flash_range_erase(_start_addr + addr, erase_sz);
  fl_unlock();

  return true;
}

bool Adafruit_FlashTransport_RP2040::readMemory(uint32_t addr, uint8_t *data,
                                                uint32_t len) {
  memcpy(data, (void *)(XIP_BASE + _start_addr + addr), len);
  return true;
}

bool Adafruit_FlashTransport_RP2040::writeMemory(uint32_t addr,
                                                 uint8_t const *data,
                                                 uint32_t len) {
  fl_lock();
  flash_range_program(_start_addr + addr, data, len);
  fl_unlock();
  return true;
}

#endif
