/*
 * WiFiHelper.h
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

#ifndef WIFIHELPER_H
#define WIFIHELPER_H

#if defined(ARDUINO) && !defined(ENERGIA_ARCH_CC13XX)
#include <WiFiUdp.h>
#endif

#include "SoCHelper.h"

#define UDP_PACKET_BUFSIZE  256
#define WIFI_DHCP_LEASE_HRS 8

enum
{
    WIFI_PARAM_TX_POWER,
    WIFI_PARAM_DHCP_LEASE_TIME
};

enum
{
    WIFI_TX_POWER_MIN = 0,  /* 0  dBm */
    WIFI_TX_POWER_MED = 10, /* 10 dBm */
    WIFI_TX_POWER_MAX = 18  /* 18 dBm */
};

void WiFi_setup(void);
void WiFi_loop(void);
void WiFi_fini(void);

#if defined(ARDUINO) && !defined(ENERGIA_ARCH_CC13XX)
extern WiFiUDP Uni_Udp;
#endif

extern char UDPpacketBuffer[UDP_PACKET_BUFSIZE];

#endif /* WIFIHELPER_H */
