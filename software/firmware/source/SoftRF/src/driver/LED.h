/*
 * LEDHelper.h
 * Copyright (C) 2016-2025 Linar Yusupov
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

#include "../system/SoC.h"
#include "GNSS.h"
#include "EEPROM.h"

#define STATUS_LED_NUM  4
//#define RING_LED_NUM (ESP.getChipId() == 0xeacdd ? 12 : 8)
#define RING_LED_NUM  8
//#define PIX_NUM 8 /* 12 */
#define PIX_NUM (RING_LED_NUM + STATUS_LED_NUM)

#define LED_STATUS_POWER (RING_LED_NUM + 0)
#define LED_STATUS_SAT   (LED_STATUS_POWER + 1)
#define LED_STATUS_TX    (LED_STATUS_SAT + 1)
#define LED_STATUS_RX    (LED_STATUS_TX + 1)

#define LED_COLOR_BLACK     uni_Color(0, 0, 0)
#define LED_COLOR_BACKLIT   uni_Color(1, 1, 1)
#define LED_COLOR_RED       uni_Color(255, 0, 0)
#define LED_COLOR_YELLOW    uni_Color(255, 200, 0)
#define LED_COLOR_GREEN     uni_Color(0, 255, 0)
#define LED_COLOR_BLUE      uni_Color(0, 0, 255)
#define LED_COLOR_MI_RED    uni_Color(7, 0, 0)
#define LED_COLOR_MI_YELLOW uni_Color(7, 5, 0)
#define LED_COLOR_MI_GREEN  uni_Color(0, 5, 0)

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

enum
{
	DISPLAY_NONE,
	DISPLAY_OLED_HELTEC, /* 0.96", 128X64, SSD1306, RST */
	DISPLAY_OLED_TTGO,   /* 0.96", 128X64, SSD1306 */
	DISPLAY_OLED_0_49,   /* 0.49",  64X32, SSD1306 */
	DISPLAY_OLED_1_3,    /* 1.3" , 128X64, SH1106  */
	DISPLAY_TFT_TTGO_240,
	DISPLAY_TFT_TTGO_135,
	DISPLAY_TFT_HELTEC_80,
	DISPLAY_TFT_PORTAPACK,
	DISPLAY_TFT_WIRELESSTAG_7, /* 7.0", 1024x600, EK79007 */
	DISPLAY_EPD_2_7,
	DISPLAY_EPD_1_54,
	DISPLAY_EPD_2_13,
	DISPLAY_EPD_3_71,
};

void LED_setup();
void LED_test();
void LED_DisplayTraffic();
void LED_Clear();
void LED_loop();

#endif /* LEDHELPER_H */
