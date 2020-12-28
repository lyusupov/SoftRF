/*
  EEPROM.cpp - TI CC13X0 and TI CC13X2 EEPROM emulation

  Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.

  Copyright (c) 2020-2021 Linar Yusupov. All rights reserved.
  Ported onto TI CC13X0 and TI CC13X2 targets as a part of SoftRF project.

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

#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)

#include "Arduino.h"
#include "EEPROM_CC13XX.h"
#include "debug.h"

#ifdef DEBUG_ASR_CORE
#define DEBUGV(fmt, ...) ::printf((PGM_P)PSTR(fmt), ## __VA_ARGS__)
#endif

#ifndef DEBUGV
#define DEBUGV(...) do { (void)0; } while (0)
#endif

#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSCC26XX.h>

#if defined(ENERGIA_ARCH_CC13XX)

#define NVS_REGIONS_BASE 0x1E000
#define SECTORSIZE       4096
#define REGIONSIZE       (SECTORSIZE * 1)

#define CONFIG_NVS_COUNT 1

__attribute__ ((section (".nvs"))) static char flashBuf[REGIONSIZE];

/*

    This patch is to be applied onto Energia GNU ld script

--- ./system/energia/linker.cmd.orig    2017-09-13 18:50:07.000000000 +0300
+++ ./system/energia/linker.cmd 2020-05-04 15:39:22.000000000 +0300
@@ -50,6 +50,10 @@
     _vtable_base_address = 536870912;
 ti_sysbios_family_arm_m3_Hwi_nvic = 0xe000e000;

+    .nvs (0x1e000) (NOLOAD) : AT (0x1e000) {
+        *(.nvs)
+    } > REGION_TEXT
+

     __TI_STACK_BASE = __stack;
 }

*/

NVSCC26XX_Object nvsCC26xxObjects[CONFIG_NVS_COUNT];

const NVSCC26XX_HWAttrs nvsCC26xxHWAttrs[CONFIG_NVS_COUNT] = {
  {
    .block = (void *)flashBuf,      /*!< Address of flash block to manage */
    .blockSize = REGIONSIZE,        /*!< The size of block */
    .copyBlock = NULL,              /*!< A RAM buffer or flash block to use */
    .isRam = false,                 /*!< TRUE if copyBlock is a RAM buffer */
  },
};

extern "C" const NVS_Config NVS_config[CONFIG_NVS_COUNT] = {
  {
     .fxnTablePtr = &NVSCC26XX_fxnTable,
     .object = &nvsCC26xxObjects[0],
     .hwAttrs = &nvsCC26xxHWAttrs[0],
  },
};

extern "C" const uint8_t NVS_count = CONFIG_NVS_COUNT;

#elif defined(ENERGIA_ARCH_CC13X2)

extern uint8_t __NVS_BASE__;
extern uint8_t __NVS_SIZE__;

#define NVS_REGIONS_BASE (void *)&__NVS_BASE__
#define SECTORSIZE       8192
#define REGIONSIZE       (size_t)(&__NVS_SIZE__)

//const NVSCC26XX_HWAttrs nvsCC26xxHWAttrs[CONFIG_NVS_COUNT] = {
//  {
//    .regionBase = (void *)flashBuf,
//    .regionSize = REGIONSIZE,
//  },
//};
#else
#error "This hardware platform is not supported!"
#endif

#define _EEPROM_SIZE     (REGIONSIZE > SECTORSIZE ? SECTORSIZE : REGIONSIZE)

EEPROMClass::EEPROMClass(void)
: _data(0)
, _size(0)
, _dirty(false)
, _nvsHandle(NULL)
{
}

void EEPROMClass::begin(size_t size) {
  NVS_Params nvsParams;
  uint16_t status;

  if (_nvsHandle != NULL) {
    DEBUGV("EEPROMClass::begin error, NVS is busy\n");
    return;
  }

  if (size <= 0) {
    DEBUGV("EEPROMClass::begin error, size == 0\n");
    return;
  }
  if (size > _EEPROM_SIZE) {
    DEBUGV("EEPROMClass::begin error, %d > %d\n", size, _EEPROM_SIZE);
    size = _EEPROM_SIZE;
  }

  size = (size + 3) & (~3);

  //In case begin() is called a 2nd+ time, don't reallocate if size is the same
  if(_data && size != _size) {
    delete[] _data;
    _data = new uint8_t[size];
  } else if(!_data) {
    _data = new uint8_t[size];
  }

  _size = size;

  NVS_init();

  /* Region 0 is internal flash */
  _nvsHandle = NVS_open(0, NULL);

  // confirm that the NVS region opened properly
  if (_nvsHandle == NULL) {
    // Error opening NVS driver
    DEBUGV("EEPROMClass::begin flash open failed\n");
    return;
  }

  status = NVS_read(_nvsHandle, 0, _data, _size);
  if (status != NVS_STATUS_SUCCESS) {
    DEBUGV("EEPROMClass::begin flash read failed\n");
  }

  _dirty = false; //make sure dirty is cleared in case begin() is called 2nd+ time
}

void EEPROMClass::end() {
  if (_nvsHandle == NULL) {
    return;
  }

  if (!_size)
    return;

  commit();
  if(_data) {
    delete[] _data;
  }
  _data = 0;
  _size = 0;
  _dirty = false;

  NVS_close(_nvsHandle);
  _nvsHandle = NULL;
}

uint8_t EEPROMClass::read(int const address) {
  if (address < 0 || (size_t)address >= _size) {
    DEBUGV("EEPROMClass::read error, address %d > %d or %d < 0\n", address, _size, address);
    return 0;
  }
  if (!_data) {
    DEBUGV("EEPROMClass::read without ::begin\n");
    return 0;
  }

  return _data[address];
}

void EEPROMClass::write(int const address, uint8_t const value) {
  if (address < 0 || (size_t)address >= _size) {
    DEBUGV("EEPROMClass::write error, address %d > %d or %d < 0\n", address, _size, address);
    return;
  }
  if(!_data) {
    DEBUGV("EEPROMClass::read without ::begin\n");
    return;
  }

  // Optimise _dirty. Only flagged if data written is different.
  uint8_t* pData = &_data[address];
  if (*pData != value)
  {
    *pData = value;
    _dirty = true;
  }
}

bool EEPROMClass::commit() {
  NVS_Attrs regionAttrs;
  uint16_t status;

  if (_nvsHandle == NULL) {
    return false;
  }

  if (!_size)
    return false;
  if(!_dirty)
    return true;
  if(!_data)
    return false;

  // fetch the generic NVS region attributes
  NVS_getAttrs(_nvsHandle, &regionAttrs);

#if defined(ENERGIA_ARCH_CC13XX)
  size_t blockSize = regionAttrs.blockSize;
  unsigned int flags = NVS_WRITE_VALIDATE;
  #define NVS_ERASE_ARGS
#elif defined(ENERGIA_ARCH_CC13X2)
  size_t blockSize = regionAttrs.sectorSize;
  uint16_t flags = NVS_WRITE_POST_VERIFY;
  #define NVS_ERASE_ARGS  , 0, blockSize
#else
#error "This hardware platform is not supported!"
#endif

  // erase the first sector of the NVS region
  status = NVS_erase(_nvsHandle NVS_ERASE_ARGS);
  if (status == NVS_STATUS_SUCCESS) {
    status = NVS_write(_nvsHandle, 0, _data, _size, flags);
    if (status == NVS_STATUS_SUCCESS) {
      _dirty = false;
      return true;
    }
  }

  DEBUGV("EEPROMClass::commit failed\n");
  return false;
}

uint8_t * EEPROMClass::getDataPtr() {
  _dirty = true;
  return &_data[0];
}

uint8_t const * EEPROMClass::getConstDataPtr() const {
  return &_data[0];
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_EEPROM)
EEPROMClass EEPROM;
#endif

#endif /* CC13XX or CC13X2 */
