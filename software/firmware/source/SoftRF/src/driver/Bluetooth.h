/*
 * BluetoothHelper.h
 * Copyright (C) 2018-2024 Linar Yusupov
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

#ifndef BLUETOOTHHELPER_H
#define BLUETOOTHHELPER_H

enum
{
	BLUETOOTH_NONE,
	BLUETOOTH_SPP,
	BLUETOOTH_LE_HM10_SERIAL,
	BLUETOOTH_A2DP_SOURCE,
};

#if defined(ESP32)
#include "sdkconfig.h"
#endif

#if defined(ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S2)
#include "../system/SoC.h"
#if defined(USE_NIMBLE)
#include "../platform/bluetooth/NimBLE.h"
#else
#include "../platform/bluetooth/Bluedroid.h"
#endif /* USE_NIMBLE */
#elif defined(ARDUINO_ARCH_NRF52)
#include "../platform/bluetooth/Bluefruit.h"
#elif defined(ARDUINO_ARCH_RP2040) && defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include "../platform/bluetooth/BTstack.h"
#elif defined(ARDUINO_ARCH_RENESAS)
#include "../platform/bluetooth/ArduinoBLE.h"
#endif /* ESP32 or NRF52 or RP2040 or RENESAS */

#endif /* BLUETOOTHHELPER_H */
