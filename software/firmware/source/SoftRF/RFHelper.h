/*
 * RFHelper.h
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

#ifndef RFHELPER_H
#define RFHELPER_H

#include <nRF905.h>
#include <TimeLib.h>

#include <lmic.h>
#include <hal/hal.h>
#include <lib_crc.h>
#include <protocol.h>

#include "SoftRF.h"
#include "GNSSHelper.h"
#include "Protocol_Legacy.h"
#include "Protocol_P3I.h"

enum
{
	RF_BAND_AUTO,
	RF_BAND_EU,	 /* 868.4 MHz band */
	RF_BAND_US,	 /* 915 MHz band */
	RF_BAND_AU,	 /* 921 MHz band */
	RF_BAND_NZ,	 /* 869.250 MHz band */
	RF_BAND_RU,	 /* 868.8 MHz band */
	RF_BAND_CN,	 /* 433 MHz band */
	RF_BAND_UK 	 /* 869.52 MHz band */
};

enum
{
  RF_IC_NRF905,
  RF_IC_SX1276
};

enum
{
  RF_TX_POWER_FULL,
  RF_TX_POWER_LOW,
  RF_TX_POWER_OFF
};

#define RXADDR {0x31, 0xfa , 0xb6} // Address of this device (4 bytes)
#define TXADDR {0x31, 0xfa , 0xb6} // Address of device to send to (4 bytes)

typedef struct rfchip_ops_struct {
  byte type;
  const char name[8];
  bool (*probe)();
  void (*setup)();
  void (*channel)(uint8_t);
  bool (*receive)();
  void (*transmit)();
} rfchip_ops_t;

uint8_t parity(uint32_t);

void RF_setup(void);
void RF_SetChannel(void);
void RF_loop(void);
size_t RF_Encode(void);
void RF_Transmit(size_t);
bool RF_Receive(void);

bool nrf905_probe(void);
void nrf905_setup(void);
void nrf905_channel(uint8_t);
bool nrf905_receive(void);
void nrf905_transmit(void);

bool sx1276_probe(void);
void sx1276_setup(void);
void sx1276_channel(uint8_t);
bool sx1276_receive(void);
void sx1276_transmit(void);

extern byte TxBuffer[PKT_SIZE], RxBuffer[PKT_SIZE];
extern unsigned long TxTimeMarker;
extern byte TxPkt[MAX_PKT_SIZE];
extern ufo_t ThisAircraft;

extern rfchip_ops_t *rf_chip;
extern bool RF_SX1276_RST_is_connected;
extern size_t (*protocol_encode)(void *, ufo_t *);
extern bool (*protocol_decode)(void *, ufo_t *, ufo_t *);

#endif /* RFHELPER_H */
