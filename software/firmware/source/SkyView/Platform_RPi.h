/*
 * Platform_RPi.h
 * Copyright (C) 2018-2020 Linar Yusupov
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
#define SOC_GPIO_BUTTON_MODE    5
#define SOC_GPIO_BUTTON_UP      6
#define SOC_GPIO_BUTTON_DOWN    13
#define SOC_GPIO_BUTTON_4       19   /* not assigned yet */

extern TTYSerial SerialInput;

#endif /* PLATFORM_RPI_H */

#endif /* RASPBERRY_PI */
