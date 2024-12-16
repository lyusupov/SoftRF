/*
 * BluetoothHelper.h
 * Copyright (C) 2019-2025 Linar Yusupov
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

#if defined(ESP32)

#define SERVICE_UUID16            ((uint16_t) 0xFFE0)
#define CHARACTERISTIC_UUID16     ((uint16_t) 0xFFE1)
#define SERVICE_UUID128           "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID128    "0000ffe1-0000-1000-8000-00805f9b34fb"

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

extern IODev_ops_t ESP32_Bluetooth_ops;
extern Bluetooth_ctl_t ESP32_BT_ctl;

#elif defined(ARDUINO_ARCH_RP2040) && defined(ARDUINO_RASPBERRY_PI_PICO_W)

#define UART_SERVICE_UUID         "0000ffe0-0000-1000-8000-00805f9b34fb"
#define UART_CHARACTERISTIC_UUID  "0000ffe1-0000-1000-8000-00805f9b34fb"

/* (FLAA x MAX_TRACKING_OBJECTS + GNGGA + GNRMC + FLAU) x 80 symbols */
#define BLE_FIFO_TX_SIZE          1024
#define BLE_FIFO_RX_SIZE          256

#define BLE_MAX_WRITE_CHUNK_SIZE  20

extern IODev_ops_t CYW43_Bluetooth_ops;

#endif /* ESP32 or ARDUINO_ARCH_RP2040 */

#endif /* BLUETOOTHHELPER_H */
