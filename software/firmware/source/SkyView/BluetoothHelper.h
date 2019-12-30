/*
 * BluetoothHelper.h
 * Copyright (C) 2019-2020 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BLUETOOTHHELPER_H
#define BLUETOOTHHELPER_H

#include <stddef.h>

typedef struct Bluetooth_ops_struct {
  const char name[16];
  void (*setup)();
  void (*loop)();
  int (*available)(void);
  int (*read)(void);
  size_t (*write)(const uint8_t *buffer, size_t size);
} Bluetooth_ops_t;

#if defined(ESP32)

extern Bluetooth_ops_t ESP32_Bluetooth_ops;

#endif /* ESP32 */

#endif /* BLUETOOTHHELPER_H */