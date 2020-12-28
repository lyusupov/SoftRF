/*
 * BluetoothHelper.h
 * Copyright (C) 2019-2021 Linar Yusupov
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
  void (*fini)();
  int (*available)(void);
  int (*read)(void);
  size_t (*write)(const uint8_t *buffer, size_t size);
} Bluetooth_ops_t;

#if defined(ESP32)

#define SERVICE_UUID              "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID       "0000ffe1-0000-1000-8000-00805f9b34fb"

#define BT_NODATA_TIMEOUT         30000

/* (FLAA x MAX_TRACKING_OBJECTS + GNGGA + GNRMC + FLAU) x 80 symbols */
#define BLE_FIFO_TX_SIZE          256
#define BLE_FIFO_RX_SIZE          1024

#define BLE_MAX_WRITE_CHUNK_SIZE  20

typedef struct Bluetooth_ctl_struct {
  portMUX_TYPE mutex;
  int command;
  int status;
} Bluetooth_ctl_t;

enum
{
	BT_CMD_NONE,
	BT_CMD_CONNECT,
	BT_CMD_DISCONNECT,
	BT_CMD_SHUTDOWN
};

enum
{
	BT_STATUS_NC,
	BT_STATUS_CON
};

extern Bluetooth_ops_t ESP32_Bluetooth_ops;
extern Bluetooth_ctl_t ESP32_BT_ctl;

#endif /* ESP32 */

#endif /* BLUETOOTHHELPER_H */
