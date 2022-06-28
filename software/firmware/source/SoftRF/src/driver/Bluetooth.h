/*
 * BluetoothHelper.h
 * Copyright (C) 2018-2022 Linar Yusupov
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
#include "sdkconfig.h"
#endif

#if defined(ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S2)

#define UART_SERVICE_UUID         "0000ffe0-0000-1000-8000-00805f9b34fb"
#define UART_CHARACTERISTIC_UUID  "0000ffe1-0000-1000-8000-00805f9b34fb"

#define MIDI_SERVICE_UUID         "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define MIDI_CHARACTERISTIC_UUID  "7772e5db-3868-4112-a1a9-f2669d106bf3"

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

#include "bluefruit_common.h"

#include "BLECharacteristic.h"
#include "BLEService.h"

class BLESensBox : public BLEService
{
  protected:
    BLECharacteristic _sensbox_nav;
    BLECharacteristic _sensbox_move;
    BLECharacteristic _sensbox_gps2;
    BLECharacteristic _sensbox_sys;

  public:
    BLESensBox(void);

    virtual err_t begin(void);

    bool notify_nav (uint8_t);
    bool notify_move(uint8_t);
    bool notify_gps2(uint8_t);
    bool notify_sys (uint8_t);
};

/*
 * Source:
 * https://github.com/flytec/SensBoxLib_iOS/blob/master/_SensBox%20Documentation/SensorBox%20BLE%20Protocol.pdf
 */
typedef struct {
    uint32_t  timestamp;  /* Date/Time (UTC), UnixTime */
    int32_t   lat;        /* deg * 10^7 */
    int32_t   lon;        /* deg * 10^7 */
    int16_t   gnss_alt;   /* GPS Hight MSL, m */
    int16_t   pres_alt;   /* Pressure Altitude, m */
    int16_t   vario;      /* Vario 1 Hz, cm/s */
    uint8_t   status;     /* Status: 0..2 - GNSS, 3 - time, 4 - charge, 5 - Bat, 6 - Log */
} __attribute__((packed)) sensbox_navigation_t;

typedef struct {
    int32_t   pres_alt;   /* Pressure Altitude, cm */
    int16_t   vario;      /* Vario 8 Hz, cm/s */
    int16_t   gs;         /* Ground Speed, dm/s */
    int16_t   cog;        /* GPS Heading, deg * 10 */
    int16_t   pitch;      /* deg * 10 */
    int16_t   yaw;        /* deg * 10 */
    int16_t   roll;       /* deg * 10 */
    uint16_t  accel;      /* Acceleration, 'g' * 10 */
    uint8_t   status;     /* Status: same as above */
} __attribute__((packed)) sensbox_movement_t;

typedef struct {
    uint16_t  accuracy_h; /* Horizontal Accuracy, dm */
    uint16_t  accuracy_v; /* Vertical   Accuracy, dm */
    int16_t   geo_separ;  /* GPS Height Ellipsoid, m */
    uint8_t   sats;       /* Number of satellites */
    uint8_t   status;     /* Status: same as above */
} __attribute__((packed)) sensbox_gps2_t;

typedef struct {
    uint32_t  timestamp;  /* Date/Time (UTC), UnixTime */
    uint8_t   battery;    /* Battery Level, % */
    uint8_t   log;        /* Logging level, % */
    int16_t   temp;       /* Temperature, °C * 10 */
    uint8_t   status;     /* Status: same as above */
    uint8_t   status2;
    uint16_t  qnh;        /* QNH, Pa * 10 */
    int32_t   pressure;   /* mPa  */
} __attribute__((packed)) sensbox_system_t;

#define isTimeToSensBox() (millis() - BLE_SensBox_TimeMarker > 500) /* 2 Hz */

extern IODev_ops_t nRF52_Bluetooth_ops;

#endif /* ESP32 or ARDUINO_ARCH_NRF52 */

#endif /* BLUETOOTHHELPER_H */
