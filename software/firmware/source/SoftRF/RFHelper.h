/*
 * RFHelper.h
 * Copyright (C) 2016-2017 Linar Yusupov
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

#ifndef RFHELPER_H
#define RFHELPER_H

#include <nRF905.h>
#include <TimeLib.h>
#include <ESP8266TrueRandom.h>

#include "SoftRF.h"
#include "GNSSHelper.h"
#include "WebHelper.h"
#include "legacy_codec.h"

enum
{
	RF_BAND_EU,	/* 868.4 MHz band */
	RF_BAND_RU1,	/* 868.2 MHz band */
	RF_BAND_RU2,	/* 868.8 MHz band */
	RF_BAND_CN,	/* 433 MHz band */
	RF_BAND_US, 	/* 915 MHz band */
	RF_BAND_NZ,	/* 869.250 MHz band */
	RF_BAND_AU 	/* 921 MHz band */
};

void RF_setup(void);
void RF_Transmit(void);
bool RF_Receive(void);

extern byte TxBuffer[PKT_SIZE], RxBuffer[PKT_SIZE];
//extern tx_state txready;
extern unsigned long TxTimeMarker;
//extern uint32_t Device_Id;
extern legacy_packet TxPkt;
extern ufo_t ThisAircraft;

#endif /* RFHELPER_H */