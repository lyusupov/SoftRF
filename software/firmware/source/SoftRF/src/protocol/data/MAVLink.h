/*
 * MAVLinkHelper.h
 * Copyright (C) 2016-2021 Linar Yusupov
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

#ifndef MAVLINKHELPER_H
#define MAVLINKHELPER_H

#if defined(ARDUINO) && !defined(RASPBERRY_PI)
#include <Arduino.h>  // uint32_t definition is required for mavlink.h
#endif /* ARDUINO */

#define MAVLINK10

#include <mavlink.h>        // Mavlink interface
#include <aircraft.h>

#define isValidMAVFix() (the_aircraft.gps.fix_type == 3 /* 3D fix */ )

void MAVLink_setup();
void PickMAVLinkFix();
void MAVLinkTimeSync();
void MAVLinkShareTraffic();
void MAVLinkSetWiFiPower();

#endif /* MAVLINKHELPER_H */