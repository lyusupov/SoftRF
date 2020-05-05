/*
 * hal_conf_extra.h
 * Copyright (C) 2020 Linar Yusupov
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

#elif defined(ENERGIA_ARCH_CC13X2)

#define ENABLE_NORMAL_MODE
#define EXCLUDE_TEST_MODE

#define EXCLUDE_SX12XX

#endif /* ENERGIA_ARCH_CC13X0 & ENERGIA_ARCH_CC13X2 */

#endif /* HAL_CONF_EXTRA_H */
