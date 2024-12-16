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

class BLEDfuSecure : public BLEService
{
  protected:
    BLECharacteristic _chr_control;

  public:
    BLEDfuSecure(void);

    virtual err_t begin(void);
};

extern IODev_ops_t nRF52_Bluetooth_ops;
