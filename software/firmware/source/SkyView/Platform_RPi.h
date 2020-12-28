/*
 * Platform_RPi.h
 * Copyright (C) 2018-2021 Linar Yusupov
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

#ifndef PLATFORM_RPI_H
#define PLATFORM_RPI_H

#include <raspi/TTYSerial.h>
#include <raspi/raspi.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    9

#define PCM_DEVICE              "default"
#define WAV_FILE_PREFIX         "Audio/"

/* Waveshare Pi HAT 2.7" buttons mapping */
#define SOC_GPIO_BUTTON_MODE    RPI_V2_GPIO_P1_29
#define SOC_GPIO_BUTTON_UP      RPI_V2_GPIO_P1_31
#define SOC_GPIO_BUTTON_DOWN    RPI_V2_GPIO_P1_33
#define SOC_GPIO_BUTTON_4       RPI_V2_GPIO_P1_35

extern TTYSerial SerialInput;

#endif /* PLATFORM_RPI_H */

#endif /* RASPBERRY_PI */
