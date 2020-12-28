/*
 * EEPROMHelper.h
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

#ifndef EEPROMHELPER_H
#define EEPROMHELPER_H

#if defined(ARDUINO) && !defined(ENERGIA_ARCH_CC13XX) && !defined(RASPBERRY_PI)
#include <EEPROM.h>
#endif /* ARDUINO */


#define SKYWATCH_EEPROM_MAGIC   0xC0BAFACE
#define SKYWATCH_EEPROM_VERSION 0x0000001B

typedef struct SoftRF_Settings {
    uint8_t  mode;
    uint8_t  rf_protocol;
    uint8_t  band;
    uint8_t  aircraft_type;
    uint8_t  txpower;
    uint8_t  volume;
    uint8_t  led_num;
    uint8_t  pointer;

    bool     nmea_g:1;
    bool     nmea_p:1;
    bool     nmea_l:1;
    bool     nmea_s:1;
    bool     resvd1:1;
    uint8_t  nmea_out:3;

    uint8_t  bluetooth:3; /* ESP32 built-in Bluetooth */
    uint8_t  alarm:3;
    bool     stealth:1;
    bool     no_track:1;

    uint8_t  gdl90:3;
    uint8_t  d1090:3;
    uint8_t  json:2;

    uint8_t  power_save;
    int8_t   freq_corr; /* +/-, kHz */
    uint8_t  resvd23456;
    uint8_t  resvd7;
    uint8_t  resvd8;
    uint8_t  resvd9;
    uint8_t  resvd10;
    uint8_t  resvd11;
    uint8_t  resvd12;
    uint8_t  resvd13;
    uint8_t  resvd14;
    uint8_t  resvd15;
    uint8_t  resvd16;
} softrf_settings_t;

typedef struct SkyView_Settings {
    uint8_t  adapter;

    uint8_t  connection:4;
    uint8_t  units:2;
    uint8_t  zoom:2;

    uint8_t  protocol;
    uint8_t  baudrate;
    char     ssid    [16];
    char     psk     [16];

    uint8_t  resvd1:3;
    uint8_t  orientation:1;
    uint8_t  adb:2;
    uint8_t  idpref:2;

    uint8_t  bluetooth; /* ESP32 built-in Bluetooth */
    char     bt_name [16];
    char     bt_key  [16];

    uint8_t  vmode:2;
    uint8_t  voice:3;
    uint8_t  aghost:3;

    uint8_t  resvd2;
    uint8_t  resvd3;
    uint8_t  resvd4;
    uint8_t  resvd5;
    uint8_t  resvd6;
    uint8_t  resvd7;
    uint8_t  resvd8;
    uint8_t  resvd9;
} skyview_settings_t;

typedef struct Settings {
    softrf_settings_t  s; /* slave  */
    skyview_settings_t m; /* master */
} settings_t;

typedef struct EEPROM_S {
    uint32_t   magic;
    uint32_t   version;
    settings_t settings;
} eeprom_struct_t;

typedef union EEPROM_U {
   eeprom_struct_t field;
   uint8_t raw[sizeof(eeprom_struct_t)];
} eeprom_t;

void EEPROM_setup(void);
void EEPROM_defaults(void);
void EEPROM_store(void);
extern settings_t *settings;

#endif /* EEPROMHELPER_H */
