/*
 * WiFiHelper.h
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

#ifndef WIFIHELPER_H
#define WIFIHELPER_H

#include <WiFiUdp.h>

#include "SoftRF.h"

#define HOSTNAME "SoftRF-"

void WiFi_setup(void);
void WiFi_loop(void);
IPAddress WiFi_get_broadcast(void);
void WiFi_transmit_UDP(int, byte *, size_t);

extern String host_name;
extern WiFiUDP Uni_Udp;

#endif /* WIFIHELPER_H */