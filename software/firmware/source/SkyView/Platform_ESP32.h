/*
 * Platform_ESP32.h
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
#if defined(ESP32)

#ifndef PLATFORM_ESP32_H
#define PLATFORM_ESP32_H

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

#define SerialInput           Serial1

/* Peripherals */
#define SOC_GPIO_PIN_GNSS_RX  21 /* TBD */
#define SOC_GPIO_PIN_GNSS_TX  22 /* TBD */

#define SOC_GPIO_PIN_MOSI     23 /* TBD */
#define SOC_GPIO_PIN_MISO     19 /* TBD */
#define SOC_GPIO_PIN_SCK      18 /* TBD */
#define SOC_GPIO_PIN_SS       5  /* TBD */

#endif /* PLATFORM_ESP32_H */

#endif /* ESP32 */
