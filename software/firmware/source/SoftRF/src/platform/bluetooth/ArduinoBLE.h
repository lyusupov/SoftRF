/*
 * ArduinoBLE.h
 * Copyright (C) 2024-2025 Linar Yusupov
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

#ifndef ARDUINOBLE_H
#define ARDUINOBLE_H

#define UART_SERVICE_UUID16                 "FFE0"
#define UART_CHARACTERISTIC_UUID16          "FFE1"

#define UUID16_SVC_BATTERY                  "180F"
#define UUID16_CHR_BATTERY_LEVEL            "2A19"

#define UUID16_SVC_DEVICE_INFORMATION       "180A"
#define UUID16_CHR_MODEL_NUMBER_STRING      "2A24"
#define UUID16_CHR_SERIAL_NUMBER_STRING     "2A25"
#define UUID16_CHR_FIRMWARE_REVISION_STRING "2A26"
#define UUID16_CHR_HARDWARE_REVISION_STRING "2A27"
#define UUID16_CHR_SOFTWARE_REVISION_STRING "2A28"
#define UUID16_CHR_MANUFACTURER_NAME_STRING "2A29"

/* (FLAA x MAX_TRACKING_OBJECTS + GNGGA + GNRMC + FLAU) x 80 symbols */
#define BLE_FIFO_TX_SIZE          256 /* TBD */
#define BLE_FIFO_RX_SIZE          128 /* TBD */

#define BLE_MAX_WRITE_CHUNK_SIZE  20

extern IODev_ops_t ArdBLE_Bluetooth_ops;

#endif /* ARDUINOBLE_H */
