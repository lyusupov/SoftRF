/*
 * EEPROMHelper.h
 * Copyright (C) 2019 Linar Yusupov
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


#define SKYVIEW_EEPROM_MAGIC   0xABBAFACE
#define SKYVIEW_EEPROM_VERSION 0x0000001B

typedef struct Settings {
    uint8_t  adapter;
    uint8_t  connection;
    uint8_t  protocol;
    uint8_t  baudrate;
    char     ssid    [16];
    char     psk     [16];

    uint8_t  units:2;
    uint8_t  vmode:1;
    uint8_t  orientation:1;
    uint8_t  zoom:2;
    uint8_t  idpref:2;

    uint8_t  bluetooth; /* ESP32 built-in Bluetooth */
    char     bt_name [16];
    char     bt_key  [16];

    uint8_t  voice;
    uint8_t  resvd12;
    uint8_t  resvd3;
    uint8_t  resvd4;
    uint8_t  resvd5;
    uint8_t  resvd6;
    uint8_t  resvd7;
    uint8_t  resvd8;
    uint8_t  resvd9;
} settings_t;

typedef struct EEPROM_S {
    uint32_t  magic;
    uint32_t  version;
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