/*
 * Platform_CC13XX.h
 * Copyright (C) 2019 Linar Yusupov
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
#if defined(ENERGIA_ARCH_CC13XX)

#ifndef PLATFORM_CC13XX_H
#define PLATFORM_CC13XX_H

#include "IPAddress.h"

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define swSer       Serial
#define UATSerial   Serial /* TBD */
#define yield() ({ })

#define SOC_GPIO_PIN_SS       18               // GPIO 11
#define SOC_GPIO_PIN_RST      LMIC_UNUSED_PIN

/* 
 *  UART pins
 *
 * Board_UART_TX               GPIO 3
 * Board_UART_RX               GPIO 2
 * BootLoader                  GPIO 1
 */

#define SOC_GPIO_PIN_MODE_PULLDOWN INPUT_PULLDOWN
#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN

#endif /* PLATFORM_CC13XX_H */

#endif /* ENERGIA_ARCH_CC13XX */
