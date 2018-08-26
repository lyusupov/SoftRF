/*
 * GNSSHelper.h
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

#ifndef GNSSHELPER_H
#define GNSSHELPER_H

#include <TinyGPS++.h>

#include "SoftRF.h"

void GNSS_setup();

void GNSSTimeSync(void);
void PickGNSSFix(void);
int LookupSeparation(float, float);

uint8_t makeUBXCFG(uint8_t, uint8_t, uint8_t, const uint8_t *);
void sendUBX(const uint8_t *, uint8_t);
boolean getUBX_ACK(uint8_t, uint8_t);

extern TinyGPSPlus gnss;
extern volatile unsigned long PPS_TimeMarker;
extern uint8_t UBXbuf[48];

#endif /* GNSSHELPER_H */