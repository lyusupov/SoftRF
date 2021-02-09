/*
 * BluetoothHelper.h
 * Copyright (C) 2018-2021 Linar Yusupov
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

enum
{
	BLUETOOTH_OFF,
	BLUETOOTH_SPP,
	BLUETOOTH_LE_HM10_SERIAL,
	BLUETOOTH_A2DP_SOURCE
};

#if defined(ESP32)

#define UART_SERVICE_UUID         "0000ffe0-0000-1000-8000-00805f9b34fb"
#define UART_CHARACTERISTIC_UUID  "0000ffe1-0000-1000-8000-00805f9b34fb"

#define MIDI_SERVICE_UUID         "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define MIDI_CHARACTERISTIC_UUID  "7772e5db-3868-4112-a1a9-f2669d106bf3"

#define UUID16_SVC_BATTERY        ((uint16_t) 0x180F)
#define UUID16_CHR_BATTERY_LEVEL  ((uint16_t) 0x2A19)

/* (FLAA x MAX_TRACKING_OBJECTS + GNGGA + GNRMC + FLAU) x 80 symbols */
#define BLE_FIFO_TX_SIZE          1024
#define BLE_FIFO_RX_SIZE          256

#define BLE_MAX_WRITE_CHUNK_SIZE  20

extern IODev_ops_t ESP32_Bluetooth_ops;
extern void ESP32_BLEMIDI_test(void);

#if defined(ENABLE_BT_VOICE)

/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef __BT_APP_CORE_H__
#define __BT_APP_CORE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define BT_APP_CORE_TAG                   "BT_APP_CORE"

#define BT_APP_SIG_WORK_DISPATCH          (0x01)

/**
 * @brief     handler for the dispatched work
 */
typedef void (* bt_app_cb_t) (uint16_t event, void *param);

/* message to be sent */
typedef struct {
    uint16_t             sig;      /*!< signal to bt_app_task */
    uint16_t             event;    /*!< message event id */
    bt_app_cb_t          cb;       /*!< context switch callback */
    void                 *param;   /*!< parameter area needs to be last */
} bt_app_msg_t;

/**
 * @brief     parameter deep-copy function to be customized
 */
typedef void (* bt_app_copy_cb_t) (bt_app_msg_t *msg, void *p_dest, void *p_src);

/**
 * @brief     work dispatcher for the application task
 */
bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_t p_copy_cback);

void bt_app_task_start_up(void);

void bt_app_task_shut_down(void);

#endif /* __BT_APP_CORE_H__ */

#endif /* ENABLE_BT_VOICE */

#elif defined(ARDUINO_ARCH_NRF52)

extern IODev_ops_t nRF52_Bluetooth_ops;
extern void nRF52_BLEMIDI_test(void);

#endif /* ESP32 or ARDUINO_ARCH_NRF52 */

#endif /* BLUETOOTHHELPER_H */
