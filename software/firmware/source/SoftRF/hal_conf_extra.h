/*
 * hal_conf_extra.h
 * Copyright (C) 2020-2021 Linar Yusupov
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

#ifndef HAL_CONF_EXTRA_H
#define HAL_CONF_EXTRA_H

/*
 * Override NUCLEO-L073RZ default Serial (2) assignment with S76G one's (1)
 */
#if defined(ARDUINO_NUCLEO_L073RZ)
#if SERIAL_UART_INSTANCE == 2

#undef  SERIAL_UART_INSTANCE
#undef  PIN_SERIAL_RX
#undef  PIN_SERIAL_TX

#define SERIAL_UART_INSTANCE    1
#define PIN_SERIAL_RX           PA10
#define PIN_SERIAL_TX           PA9

// (Few) pre-production SoftRF Dongles had 1.5k D+ external pullup resistor
// If you have one of these, then either:
// - break down or un-solder the D+ pullup resistor, or
// - keep using Arduino Core 1.8.0 for STM32, or
// - uncomment #define USBD_FIXED_PULLUP line below then re-build and re-install SoftRF firmware, or
// - unplug then plug in the Dongle again into a USB slot every time after you've done a settings change
//
// This indicates that there is an external and fixed 1.5k pullup
// on the D+ line. This define is only needed on boards that have
// internal pullups *and* an external pullup. Note that it would have
// been better to omit the pullup and exclusively use the internal
// pullups instead.
//#define USBD_FIXED_PULLUP

#endif /* SERIAL_UART_INSTANCE */
#endif /* ARDUINO_NUCLEO_L073RZ */

#if defined(ENERGIA_ARCH_CC13XX)

/*
 * Built-in 128K flash memory of the CC1310F128 (7x7)
 * does fit for either:
 * - RECEIVER & BRIDGE modes, or
 * - NORMAL mode
 * but not both at the same time.
 */
#define ENABLE_NORMAL_MODE
#define EXCLUDE_TEST_MODE

#define EXCLUDE_SX12XX
#define EXCLUDE_OGLEP3

#elif defined(ENERGIA_ARCH_CC13X2)

#define USE_SERIAL_DEEP_SLEEP

#define ENABLE_NORMAL_MODE
#define EXCLUDE_TEST_MODE

//#define EXCLUDE_SX12XX

#endif /* ENERGIA_ARCH_CC13X0 & ENERGIA_ARCH_CC13X2 */

#endif /* HAL_CONF_EXTRA_H */
