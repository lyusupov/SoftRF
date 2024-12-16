/*
 * BluetoothHelper.h
 * Copyright (C) 2018-2025 Linar Yusupov
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

#define UART_SERVICE_UUID16                 ((uint16_t) 0xFFE0)
#define UART_CHARACTERISTIC_UUID16          ((uint16_t) 0xFFE1)
#define UART_SERVICE_UUID128            "0000ffe0-0000-1000-8000-00805f9b34fb"
#define UART_CHARACTERISTIC_UUID128     "0000ffe1-0000-1000-8000-00805f9b34fb"

#define MIDI_SERVICE_UUID               "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define MIDI_CHARACTERISTIC_UUID        "7772e5db-3868-4112-a1a9-f2669d106bf3"

#define UUID16_SVC_BATTERY                  ((uint16_t) 0x180F)
#define UUID16_CHR_BATTERY_LEVEL            ((uint16_t) 0x2A19)

#define UUID16_SVC_DEVICE_INFORMATION       ((uint16_t) 0x180A)
#define UUID16_CHR_MODEL_NUMBER_STRING      ((uint16_t) 0x2A24)
#define UUID16_CHR_SERIAL_NUMBER_STRING     ((uint16_t) 0x2A25)
#define UUID16_CHR_FIRMWARE_REVISION_STRING ((uint16_t) 0x2A26)
#define UUID16_CHR_HARDWARE_REVISION_STRING ((uint16_t) 0x2A27)
#define UUID16_CHR_SOFTWARE_REVISION_STRING ((uint16_t) 0x2A28)
#define UUID16_CHR_MANUFACTURER_NAME_STRING ((uint16_t) 0x2A29)

#define SENSBOX_SERVICE_UUID            "aba27100-143b-4b81-a444-edcd0000f020"
#define NAVIGATION_CHARACTERISTIC_UUID  "aba27100-143b-4b81-a444-edcd0000f022"
#define MOVEMENT_CHARACTERISTIC_UUID    "aba27100-143b-4b81-a444-edcd0000f023"
#define GPS2_CHARACTERISTIC_UUID        "aba27100-143b-4b81-a444-edcd0000f024"
#define SYSTEM_CHARACTERISTIC_UUID      "aba27100-143b-4b81-a444-edcd0000f025"

/* (FLAA x MAX_TRACKING_OBJECTS + GNGGA + GNRMC + FLAU) x 80 symbols */
#define BLE_FIFO_TX_SIZE          1024
#define BLE_FIFO_RX_SIZE          256

#define BLE_MAX_WRITE_CHUNK_SIZE  20

extern IODev_ops_t ESP32_Bluetooth_ops;
