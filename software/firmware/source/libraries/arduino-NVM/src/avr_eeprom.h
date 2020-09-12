/*
   avr_eeprom.cpp - Emulate some avr/eeprom.h functionality.
   Original Copyright (c) 2017 Frank Holtz.  All right reserved.

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

#include <NVRAM.h>

#define E2END (NVRAM.length() - 1)
#define eeprom_write_byte(idx, val) NVRAM.write(idx, val)
#define eeprom_read_byte(idx) NVRAM.read(idx)
#define eeprom_write_block(src, idx, n)                                        \
  NVRAM.write_block((uint8_t *)src, (size_t)idx, n)
#define eeprom_read_block(dst, idx, n)                                         \
  NVRAM.read_block((uint8_t *)dst, (size_t)idx, n)
