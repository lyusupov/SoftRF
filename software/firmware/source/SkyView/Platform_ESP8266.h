/*
 * Platform_ESP8266.h
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
#if defined(ESP8266)

#ifndef PLATFORM_ESP8266_H
#define PLATFORM_ESP8266_H

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#include <Exp_SoftwareSerial.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS    8

/* Peripherals */
#define SOC_GPIO_PIN_SWSER_RX D3
#define SOC_GPIO_PIN_SWSER_TX D1 // 9  /* not in use */

extern "C" {
#include <user_interface.h>
}

extern Exp_SoftwareSerial SerialInput;

#endif /* PLATFORM_ESP8266_H */

#endif /* ESP8266 */
