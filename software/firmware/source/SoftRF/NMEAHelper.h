/*
 * NMEAHelper.h
 * Copyright (C) 2017-2018 Linar Yusupov
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

#ifndef NMEAHELPER_H
#define NMEAHELPER_H

#include "SoCHelper.h"
#include "SoftRF.h"

void NMEA_setup(void);
void NMEA_loop(void);
void NMEA_Export(void);
void NMEA_Position(void);

#if defined(AIRCONNECT_IS_ACTIVE)
#define MAX_AIRCONNECT_CLIENTS    2
extern WiFiClient AirConnectClient[MAX_AIRCONNECT_CLIENTS];
#endif

#endif /* NMEAHELPER_H */