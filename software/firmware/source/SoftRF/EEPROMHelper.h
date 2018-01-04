/*
 * EEPROMHelper.h
 * Copyright (C) 2016-2018 Linar Yusupov
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

#include "SoftRF.h"

#define SOFTRF_EEPROM_MAGIC 0xBABAABBA
#define SOFTRF_EEPROM_VERSION 0x00000099

typedef struct Settings {
    uint8_t  mode;
    uint8_t  band;
    uint8_t  txpower;
    uint8_t  volume;
    uint8_t  led_num;
    uint8_t  nmea_g;
    uint8_t  nmea_p;
    uint8_t  nmea_l;
    uint8_t  pointer;
    uint8_t  rf_protocol;
    uint8_t  gdl90;
    uint8_t  d1090;
    uint8_t  aircraft_type;
    uint8_t  resvd5;
    uint8_t  resvd6;
    uint8_t  resvd7;
    uint8_t  resvd8;
    uint8_t  resvd9;
    uint8_t  resvd10;
    uint8_t  resvd11;
    uint8_t  resvd12;
    uint8_t  resvd13;
    uint8_t  resvd14;
    uint8_t  resvd15;
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