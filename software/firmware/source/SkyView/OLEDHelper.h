/*
 * OLEDHelper.h
 * Copyright (C) 2019-2021 Linar Yusupov
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

#ifndef OLEDHELPER_H
#define OLEDHELPER_H

#if defined(RASPBERRY_PI)

#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define maxof2(a,b)   (a > b ? a : b)

#define OLED_EXPIRATION_TIME     5 /* seconds */

#define COLUMN1_TITLE "  ID  "
#define COLUMN2_TITLE " DIST "
#define COLUMN3_TITLE " BRG  "
#define COLUMN4_TITLE " ELEV "

#define OLED_LINES_PER_PAGE  5

void OLED_setup();
void OLED_loop();

extern Adafruit_SSD1306 odisplay;

#endif /* RASPBERRY_PI */

#endif /* OLEDHELPER_H */
