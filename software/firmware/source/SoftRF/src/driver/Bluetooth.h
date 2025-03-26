/*
 * BluetoothHelper.h
 * Copyright (C) 2018-2025 Linar Yusupov
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
#include "../system/SoC.h"
#if !defined(EXCLUDE_BLUETOOTH)
#if defined(USE_NIMBLE)
#include "../platform/bluetooth/NimBLE.h"
#elif defined(USE_ARDUINOBLE)
#include "../platform/bluetooth/ArduinoBLE.h"
#else
#include "../platform/bluetooth/Bluedroid.h"
#endif /* USE_NIMBLE */
#endif /* EXCLUDE_BLUETOOTH */
#elif defined(ARDUINO_ARCH_NRF52)
#include "../platform/bluetooth/Bluefruit.h"
#elif defined(ARDUINO_ARCH_NRF52840)
#include "../system/SoC.h"
#if defined(USE_ARDUINOBLE)
#include "../platform/bluetooth/ArduinoBLE.h"
#endif /* USE_ARDUINOBLE */
#elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
#if defined(ARDUINO_RASPBERRY_PI_PICO_W) || defined(ARDUINO_RASPBERRY_PI_PICO_2W)
#include "../platform/bluetooth/BTstack.h"
#elif defined(ARDUINO_NANO_RP2040_CONNECT)
#if defined(USE_ARDUINOBLE)
#include "../platform/bluetooth/ArduinoBLE.h"
#endif /* USE_ARDUINOBLE */
#endif /* PI_PICO_W NANO_RP2040_CONNECT */
#elif defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_SILABS)
#include "../platform/bluetooth/ArduinoBLE.h"
#endif /* ESP32 or NRF52 or RP2040 or RENESAS or SILABS */

#endif /* BLUETOOTHHELPER_H */
