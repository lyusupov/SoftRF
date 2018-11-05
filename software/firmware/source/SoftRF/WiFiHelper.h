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

#if defined(ARDUINO)
#include <WiFiUdp.h>
#endif

#include "SoftRF.h"

#define HOSTNAME "SoftRF-"

enum
{
    WIFI_TX_POWER_MIN = 0,  /* 0  dBm */
    WIFI_TX_POWER_MED = 10, /* 10 dBm */
    WIFI_TX_POWER_MAX = 18  /* 18 dBm */
};

void WiFi_setup(void);
void WiFi_loop(void);
size_t Raw_Receive_UDP(uint8_t *);
void Raw_Transmit_UDP(void);

extern String host_name;
#if defined(ARDUINO)
extern WiFiUDP Uni_Udp;
#endif

extern char UDPpacketBuffer[256];

#endif /* WIFIHELPER_H */
