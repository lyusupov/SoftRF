/*
 * Platform_RPi.h
 * Copyright (C) 2018 Linar Yusupov
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

#if defined(RASPBERRY_PI)

#ifndef RPIHELPER_H
#define RPIHELPER_H

/* Dragino LoRa/GPS HAT */
#define SOC_GPIO_PIN_MOSI     12
#define SOC_GPIO_PIN_MISO     13
#define SOC_GPIO_PIN_SCK      14
#define SOC_GPIO_PIN_SS       6
#define SOC_GPIO_PIN_RST      0
#define SOC_GPIO_PIN_DIO0     7

#endif /* RPIHELPER_H */

#endif /* RASPBERRY_PI */
