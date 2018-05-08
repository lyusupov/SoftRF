/*
 * LEDHelper.h
 * Copyright (C) 2016-2018 Linar Yusupov
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

#ifndef LEDHELPER_H
#define LEDHELPER_H

#include <Adafruit_NeoPixel.h>
#include <TimeLib.h>

#include "SoftRF.h"
#include "GNSSHelper.h"
#include "EEPROMHelper.h"

#define STATUS_LED_NUM  4
//#define RING_LED_NUM (ESP.getChipId() == 0xeacdd ? 12 : 8)
#define RING_LED_NUM  8
//#define PIX_NUM 8 /* 12 */
#define PIX_NUM (RING_LED_NUM + STATUS_LED_NUM)

#define LED_STATUS_POWER (RING_LED_NUM + 0)
#define LED_STATUS_SAT   (LED_STATUS_POWER + 1)
#define LED_STATUS_TX    (LED_STATUS_SAT + 1)
#define LED_STATUS_RX    (LED_STATUS_TX + 1)

#define LED_COLOR_BLACK     strip.Color(0, 0, 0)
#define LED_COLOR_BACKLIT   strip.Color(1, 1, 1)
#define LED_COLOR_RED       strip.Color(255, 0, 0)
#define LED_COLOR_YELLOW    strip.Color(255, 200, 0)
#define LED_COLOR_GREEN     strip.Color(0, 255, 0)
#define LED_COLOR_BLUE      strip.Color(0, 0, 255)
#define LED_COLOR_MI_RED    strip.Color(7, 0, 0)
#define LED_COLOR_MI_YELLOW strip.Color(7, 5, 0)
#define LED_COLOR_MI_GREEN  strip.Color(0, 5, 0)

#define ZERO_BEARING_LED_NUM (RING_LED_NUM / 2) 
#define SECTOR_PER_LED (360 / RING_LED_NUM)
#define LED_ROTATE_ANGLE (ZERO_BEARING_LED_NUM * SECTOR_PER_LED) 

#define LED_DISTANCE_CLOSE  500
#define LED_DISTANCE_NEAR   1500
#define LED_DISTANCE_FAR    10000

enum
{
	DIRECTION_TRACK_UP,
	DIRECTION_NORTH_UP,
	LED_OFF
};

void LED_setup();
void LED_test();
void LED_DisplayTraffic();
void LED_Clear();

extern Adafruit_NeoPixel strip;
extern ufo_t Container[MAX_TRACKING_OBJECTS];
extern uint32_t tx_packets_counter, rx_packets_counter;
extern ufo_t ThisAircraft;

#endif /* LEDHELPER_H */
