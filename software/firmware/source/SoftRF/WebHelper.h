/*
 * WebHelper.h
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

#ifndef WEBHELPER_H
#define WEBHELPER_H

#include <TinyGPS++.h>
#include <WiFiClient.h>

#include "SoftRF.h"
#include "EEPROMHelper.h"
#include "RFHelper.h"

void Web_setup(void);
void Web_loop(void);

void Hex2Bin(String, byte *);
String Bin2Hex(byte *);

extern uint32_t tx_packets_counter, rx_packets_counter;
extern byte TxBuffer[PKT_SIZE];
extern String TxDataTemplate;
extern WiFiClient client;
extern TinyGPSPlus gps;

#endif /* WEBHELPER_H */