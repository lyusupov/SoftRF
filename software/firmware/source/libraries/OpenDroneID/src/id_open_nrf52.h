/* -*- tab-width: 2; mode: c; -*-
 * 
 * C++ class for Arduino to function as a wrapper around opendroneid.
 * This file has nRF52 specific code.
 *
 * Copyright (c) 2022, Steve Jack.
 *
 * NOTES
 *
 */

#if defined(ARDUINO_ARCH_NRF52)

#pragma GCC diagnostic warning "-Wunused-variable"

#include <Arduino.h>

#if ID_OD_BT

#include <ble.h>

/*
 *
 */

#if BLE_OPTION == 1 || BLE_OPTION == 3

#include <bluefruit.h>
#include <BLEAdvertising.h>

#elif BLE_OPTION == 2

#endif

/*
 *
 */

#endif // ID_OD_BT

#endif // nRF52
